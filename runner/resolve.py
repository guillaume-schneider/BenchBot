from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import copy
import datetime as dt
import hashlib
import os
import yaml


def deep_merge(a: dict, b: dict) -> dict:
    """Merge b into a (recursive). Returns new dict."""
    out = copy.deepcopy(a)
    for k, v in (b or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def iso_now_local() -> str:
    # Europe/Zurich implied by user; use local tz via system. Good enough.
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def stable_run_id(dataset_id: str, slam_id: str, seed: int, repeat_index: int) -> str:
    ts = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return f"{ts}__{dataset_id}__{slam_id}__seed{seed}__r{repeat_index}"


def flatten_topics(topics: list[str]) -> list[str]:
    # Keep order, remove duplicates.
    seen = set()
    out = []
    for t in topics or []:
        if t not in seen:
            out.append(t)
            seen.add(t)
    return out


def recursive_substitute(obj, mapping):
    if isinstance(obj, dict):
        return {k: recursive_substitute(v, mapping) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [recursive_substitute(x, mapping) for x in obj]
    elif isinstance(obj, str):
        for k, v in mapping.items():
            obj = obj.replace(k, v)
        return obj
    else:
        return obj

def load_yaml(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    
    mapping = {
        "${PROJECT_ROOT}": str(Path.cwd()),
        "${HOME}": str(Path.home())
    }
    return recursive_substitute(data, mapping)


def resolve_run_config(
    matrix: dict,
    dataset_obj: dict,
    slam_entry: dict,
    slam_profile: dict,
    combo_overrides: dict | None,
    slam_overrides: dict | None,
    dataset_overrides: dict | None,
    seed: int,
    repeat_index: int,
    run_id: str,
    output_root: str,
) -> dict:
    defaults = matrix.get("defaults", {})
    workspace = matrix.get("workspace", {})

    # Start from a base “resolved” skeleton
    resolved = {
        "schema_version": 1,
        "run_id": run_id,
        "created_at": iso_now_local(),
        "tags": (combo_overrides or {}).get("tags", []) or [],
        "seed": int(seed),
        "repeat_index": int(repeat_index),
        "workspace": {
            "ros_distro": workspace.get("ros_distro", ""),
            "use_sim_time": bool(workspace.get("use_sim_time", False)),
            "repo_state": {
                "orchestrator_git_sha": "UNKNOWN",
                "mexplore_git_sha": "UNKNOWN",
                "slam_repo_git_sha": None,
                "dirty": False,
            },
            "system": {
                "hostname": os.uname().nodename,
                "os": "UNKNOWN",
                "kernel": os.uname().release,
                "cpu_model": "UNKNOWN",
                "ram_gb": 0.0,
            },
        },
        "dataset": copy.deepcopy(dataset_obj),
        "slam": {
            "id": slam_entry["id"],
            "profile_path": slam_entry["profile"],
            "launch": copy.deepcopy(slam_profile.get("launch", {})),
            "params": copy.deepcopy(slam_profile.get("params", {})),
            "remaps": copy.deepcopy(slam_profile.get("remaps", {})),
            "io_contract": copy.deepcopy(slam_profile.get("io_contract", {})),
        },
        "run_control": copy.deepcopy(defaults.get("run", {})),
        "recording": {
            "enabled": bool(defaults.get("rosbag", {}).get("enabled", True)),
            "rosbag2": copy.deepcopy(defaults.get("rosbag", {})),
            "extra_artifacts": copy.deepcopy(matrix.get("output", {}).get("artifacts", {})),
        },
        "probes": {
            "ready": copy.deepcopy(defaults.get("probes", {}).get("required", [])),
            "health": copy.deepcopy(slam_profile.get("probes", {}).get("health", [])),
            "stop": copy.deepcopy(slam_profile.get("probes", {}).get("stop", [])) or [],
        },
        "execution_plan": {
            "states": [
                {"name": "SETUP"},
                {"name": "START_SCENARIO"},
                {"name": "START_SLAM"},
                {"name": "WAIT_READY"},
                {"name": "WARMUP", "duration_s": float(defaults.get("run", {}).get("warmup_s", 2.0))},
                {"name": "RUN", "timeout_s": float(defaults.get("run", {}).get("timeout_s", 240.0))},
                {"name": "DRAIN", "duration_s": float(defaults.get("run", {}).get("drain_s", 1.0))},
                {"name": "STOP"},
                {"name": "EVALUATE"},
                {"name": "INDEX"},
            ]
        },
        "paths": {
            "run_dir": str(Path(output_root) / run_id),
            "logs_dir": str(Path(output_root) / run_id / "logs"),
            "bags_dir": str(Path(output_root) / run_id / "bags"),
            "metrics_json": str(Path(output_root) / run_id / "metrics.json"),
            "index_file": matrix.get("output", {}).get("index_file", "results/index.jsonl"),
        },
        "reproduce": {
            "command": ["python3", "tools/reproduce_run.py", "--config", str(Path(output_root) / run_id / "config_resolved.yaml")],
            "requires": [],
        },
    }

    # Apply dataset overrides
    if dataset_overrides:
        if "run" in dataset_overrides:
            resolved["run_control"] = deep_merge(resolved["run_control"], dataset_overrides["run"])
        if "rosbag" in dataset_overrides:
            resolved["recording"]["rosbag2"] = deep_merge(resolved["recording"]["rosbag2"], dataset_overrides["rosbag"])
        if "probes" in dataset_overrides:
            # allow replacing/adding required/optional, keep simple:
            if "required" in dataset_overrides["probes"]:
                resolved["probes"]["ready"] = dataset_overrides["probes"]["required"]

    # Apply slam-specific overrides (from matrix slams list)
    if slam_overrides:
        if "params" in slam_overrides:
            resolved["slam"]["params"]["inline"] = deep_merge(resolved["slam"]["params"].get("inline", {}), slam_overrides["params"])
        if "remaps" in slam_overrides:
            resolved["slam"]["remaps"] = deep_merge(resolved["slam"]["remaps"], slam_overrides["remaps"])
        if "launch" in slam_overrides and "args" in slam_overrides["launch"]:
            resolved["slam"]["launch"]["args"] = deep_merge(resolved["slam"]["launch"].get("args", {}), slam_overrides["launch"]["args"])
        if "probes" in slam_overrides and "required" in slam_overrides["probes"]:
            resolved["probes"]["ready"] = slam_overrides["probes"]["required"]

    # Apply combo overrides (include entry overrides)
    if combo_overrides:
        # tags might be in the include entry; add them
        if "tags" in combo_overrides:
            resolved["tags"] = list(combo_overrides["tags"])
        if "run" in combo_overrides:
            resolved["run_control"] = deep_merge(resolved["run_control"], combo_overrides["run"])
        if "rosbag" in combo_overrides:
            resolved["recording"]["rosbag2"] = deep_merge(resolved["recording"]["rosbag2"], combo_overrides["rosbag"])
        if "probes" in combo_overrides and "required" in combo_overrides["probes"]:
            resolved["probes"]["ready"] = combo_overrides["probes"]["required"]

    # Finalize rosbag topics: defaults topics + slam recording_overrides
    slam_extra_topics = (slam_profile.get("recording_overrides", {}) or {}).get("topics", []) or []
    topics = (resolved["recording"]["rosbag2"].get("topics") or []) + slam_extra_topics
    resolved["recording"]["rosbag2"]["topics"] = flatten_topics(topics)

    # ensure warmup/drain/timeouts reflected in plan
    for s in resolved["execution_plan"]["states"]:
        if s["name"] == "WARMUP":
            s["duration_s"] = float(resolved["run_control"].get("warmup_s", s.get("duration_s", 2.0)))
        if s["name"] == "DRAIN":
            s["duration_s"] = float(resolved["run_control"].get("drain_s", s.get("duration_s", 1.0)))
        if s["name"] == "RUN":
            s["timeout_s"] = float(resolved["run_control"].get("timeout_s", s.get("timeout_s", 240.0)))

    return resolved


def write_yaml(path: str | Path, data: dict) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
