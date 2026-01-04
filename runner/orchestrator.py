from __future__ import annotations

import time
from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from runner.process_manager import ProcessManager
from runner.bag_recorder import RosbagConfig, build_rosbag_cmd
from runner.probes.ros_probes import (
    ProbeContext,
    TfAvailableProbe,
    TopicPublishTypedProbe,
    TopicHzTypedProbe,
    ServiceAvailableProbe,
    NodePresentProbe,
)


def load_run_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def ensure_dirs(cfg: dict) -> None:
    for k in ["run_dir", "logs_dir", "bags_dir"]:
        Path(cfg["paths"][k]).mkdir(parents=True, exist_ok=True)


def build_probe(p: dict):
    t = p["type"]
    if t == "tf_available":
        return TfAvailableProbe(p["from_frame"], p["to_frame"], p.get("timeout_s", 10))
    if t == "topic_publish":
        return TopicPublishTypedProbe(
            p["topic"], p["msg_type"], p.get("min_messages", 1), p.get("timeout_s", 10)
        )
    if t == "topic_hz":
        return TopicHzTypedProbe(
            p["topic"], p["msg_type"], p["min_hz"], p.get("window_s", 5), p.get("timeout_s", 20)
        )
    if t == "service_available":
        return ServiceAvailableProbe(p["service"], p["srv_type"], p.get("timeout_s", 10))
    if t == "node_present":
        return NodePresentProbe(p["node"], p.get("timeout_s", 10))
    raise ValueError(f"Unknown probe type: {t}")


def run_once(config_path: str) -> int:
    cfg = load_run_config(config_path)
    ensure_dirs(cfg)

    logs_dir = cfg["paths"]["logs_dir"]
    bags_dir = cfg["paths"]["bags_dir"]

    pm = ProcessManager(logs_dir=logs_dir)

    # Start ROS probe node
    rclpy.init()
    node = Node("slam_bench_probe_node")
    ctx = ProbeContext(node=node)

    # --------- Mode A additions: explore pause/resume ----------
    explore_resume_topic = "/explore/resume"
    explore_pub = node.create_publisher(Bool, explore_resume_topic, 10)

    def set_explore(enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        explore_pub.publish(msg)
        # ensure the message is actually sent
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.05)

    # --------- Scenario: multi-process support ----------
    scenario = cfg.get("dataset", {}).get("scenario", {}) or {}
    scenario_processes = scenario.get("processes", None)

    # Backward compatible: old config had scenario.launch.cmd
    if not scenario_processes:
        launch = scenario.get("launch", {}) or {}
        cmd = launch.get("cmd", None)
        if not cmd:
            raise RuntimeError(
                "No scenario processes found. Expected dataset.scenario.processes "
                "or legacy dataset.scenario.launch.cmd."
            )
        scenario_processes = [{
            "name": "scenario",
            "cmd": cmd,
            "env": launch.get("env", {}) or {},
            "cwd": launch.get("cwd", None),
        }]

    # SLAM process (Mode A can be noop)
    slam_cmd = cfg.get("slam", {}).get("launch", {}).get("cmd", None)
    slam_env = cfg.get("slam", {}).get("launch", {}).get("env", {}) or {}
    slam_cwd = cfg.get("slam", {}).get("launch", {}).get("cwd", None)
    slam_id = (cfg.get("slam", {}) or {}).get("id", "")

    # rosbag config
    bag_cfg = cfg["recording"]["rosbag2"]
    rosbag_cfg = RosbagConfig(
        storage_id=bag_cfg.get("storage_id", "sqlite3"),
        compression=bag_cfg.get("compression", None),
        max_bag_size_mb=bag_cfg.get("max_bag_size_mb", None),
        include_hidden_topics=bag_cfg.get("include_hidden_topics", False),
        qos_overrides_path=bag_cfg.get("qos_overrides_path", None),
        topics=bag_cfg.get("topics", []),
    )

    bag_out_dir = str(Path(bags_dir) / "output")
    bag_cmd = build_rosbag_cmd(bag_out_dir, rosbag_cfg)

    warmup_s = float(cfg["run_control"]["warmup_s"])
    drain_s = float(cfg["run_control"]["drain_s"])
    timeout_s = float(cfg["run_control"]["timeout_s"])

    try:
        # START_SCENARIO (multi-process)
        for proc in scenario_processes:
            pm.start(
                proc["name"],
                proc["cmd"],
                env=proc.get("env", {}) or {},
                cwd=proc.get("cwd", None),
            )

        # Pause exploration for stable probes/warmup
        set_explore(False)

        # START_SLAM (Mode A: may be noop or omitted)
        # If you want to allow "no slam", keep slam_cmd optional.
        if slam_cmd and slam_id != "noop":
            pm.start("slam", slam_cmd, env=slam_env, cwd=slam_cwd)
        elif slam_cmd and slam_id == "noop":
            # still launch noop for logging consistency (optional)
            pm.start("slam", slam_cmd, env=slam_env, cwd=slam_cwd)

        # START_ROSBAG
        if cfg["recording"]["enabled"]:
            pm.start("rosbag", bag_cmd, env={}, cwd=None)

        # WAIT_READY (probes)
        for p in cfg["probes"]["ready"]:
            probe = build_probe(p)
            res = probe.run(ctx)
            if not res.ok:
                node.get_logger().error(f"[PROBE FAIL] {res.message}")
                return 2
            node.get_logger().info(f"[PROBE OK] {res.message}")

        # WARMUP (still paused)
        t0 = time.time()
        while time.time() - t0 < warmup_s:
            rclpy.spin_once(node, timeout_sec=0.1)

        # RUN: resume exploration now
        set_explore(True)

        t1 = time.time()
        while time.time() - t1 < timeout_s:
            # Health probes optional (light)
            for p in (cfg.get("probes", {}).get("health") or []):
                probe = build_probe(p)
                res = probe.run(ctx)
                if not res.ok:
                    node.get_logger().warn(f"[HEALTH WARN] {res.message}")
            rclpy.spin_once(node, timeout_sec=0.2)

        # DRAIN: pause and wait a bit so last map/TF can flush
        set_explore(False)
        t2 = time.time()
        return 0

    finally:
        # STOP (reverse-ish)
        try:
            set_explore(False)
        except Exception:
            pass

        try:
            pm.stop("rosbag")
        except Exception:
            pass

        try:
            pm.stop("slam")
        except Exception:
            pass

        # Stop scenario processes in reverse order (explore then nav2_sim, etc.)
        try:
            for proc in reversed(scenario_processes):
                pm.stop(proc["name"])
        except Exception:
            pass

        # EVALUATE (after stop)
        try:
            node.get_logger().info("Starting post-run evaluation...")
            from tools.benchmark import run_benchmark
            import json
            
            bag_path = cfg["paths"]["bags_dir"] + "/output"
            metrics_path = cfg["paths"]["metrics_json"]
            
            rmse = run_benchmark(bag_path)
            
            # Save to metrics.json
            metrics = {"ate_rmse": rmse}
            with open(metrics_path, "w") as f:
                json.dump(metrics, f, indent=4)
            node.get_logger().info(f"Evaluation finished. RMSE: {rmse}")
        except Exception as e:
            node.get_logger().error(f"Evaluation failed: {e}")

        node.destroy_node()
        rclpy.shutdown()
