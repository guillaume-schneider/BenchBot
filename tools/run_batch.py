#!/usr/bin/env python3
import os
import sys
import yaml
import subprocess
from pathlib import Path

# Add project root to sys.path
sys.path.append(str(Path(__file__).parent.parent))

from runner.resolve import load_yaml, resolve_run_config, stable_run_id, write_yaml

import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("matrix", help="Path to config_matrix.yaml")
    parser.add_argument("--gui", action="store_true", help="Enable Gazebo GUI")
    args = parser.parse_args()

    matrix_path = args.matrix
    matrix = load_yaml(matrix_path)
    
    # Setup output
    output_root = matrix.get("output", {}).get("root_dir", "results/runs")
    Path(output_root).mkdir(parents=True, exist_ok=True)

    # Gather jobs
    jobs = []
    
    slams_map = {s["id"]: s for s in matrix.get("slams", [])}
    datasets_map = {d["id"]: d for d in matrix.get("datasets", [])}

    for inc in matrix.get("matrix", {}).get("include", []):
        d_id = inc["dataset"]
        dataset_def = datasets_map.get(d_id)
        if not dataset_def:
            print(f"Dataset {d_id} not found, skipping")
            continue
            
        for s_id in inc.get("slams", []):
            slam_entry = slams_map.get(s_id)
            if not slam_entry:
                print(f"SLAM {s_id} not found, skipping")
                continue
                
            project_root = Path(__file__).parent.parent
            profile_path = project_root / slam_entry["profile"]
            slam_profile = load_yaml(profile_path)
            
            for seed in inc.get("seeds", [0]):
                for r in range(inc.get("repeats", 1)):
                    run_id = stable_run_id(d_id, s_id, seed, r)
                    
                    # Resolve config
                    resolved = resolve_run_config(
                        matrix=matrix,
                        dataset_obj=dataset_def,
                        slam_entry=slam_entry,
                        slam_profile=slam_profile,
                        combo_overrides=inc.get("overrides"),
                        slam_overrides=slam_entry.get("overrides"),
                        dataset_overrides=dataset_def.get("overrides"),
                        seed=seed,
                        repeat_index=r,
                        run_id=run_id,
                        output_root=output_root
                    )
                    
                    if args.gui:
                        # Override GUI settings for Gazebo in nav2_sim
                        for proc in resolved.get("dataset", {}).get("scenario", {}).get("processes", []):
                            if proc.get("name") == "nav2_sim":
                                cmd = proc.get("cmd", [])
                                new_cmd = []
                                for arg in cmd:
                                    if arg.startswith("headless:="):
                                        new_cmd.append("headless:=False")
                                    elif arg.startswith("gui:="):
                                        new_cmd.append("gui:=True")
                                    else:
                                        new_cmd.append(arg)
                                proc["cmd"] = new_cmd

                    config_path = Path(output_root) / run_id / "config_resolved.yaml"
                    jobs.append((run_id, config_path, resolved))

    print(f"Found {len(jobs)} runs to execute.")

    # Execute
    for i, (run_id, config_path, resolved) in enumerate(jobs):
        print(f"[{i+1}/{len(jobs)}] Preparing {run_id}...")
        write_yaml(config_path, resolved)
        
        print(f"   -> Starting run...")
        # Call runner/run_one.py as a module
        # We assume runner is in python path or strictly relative
        project_root = Path(__file__).parent.parent
        cmd = [sys.executable, "-m", "runner.run_one", str(config_path)]
        
        try:
            subprocess.check_call(cmd, cwd=project_root)
            print(f"   -> [SUCCESS] {run_id}")
        except subprocess.CalledProcessError as e:
            print(f"   -> [FAILURE] {run_id} (code {e.returncode})")
            # Continue to next or stop? Usually continue.

if __name__ == "__main__":
    main()
