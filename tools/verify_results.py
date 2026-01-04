#!/usr/bin/env python3
import sys
import yaml
import subprocess
from pathlib import Path
from typing import Optional

def check_file(path: Path, desc: str) -> bool:
    if not path.exists():
        print(f"[FAIL] {desc} missing: {path}")
        return False
    # check size > 0 if not dir
    if path.is_file() and path.stat().st_size == 0:
        print(f"[FAIL] {desc} is empty: {path}")
        return False
    print(f"[OK] {desc} found.")
    return True

def get_bag_info(bag_path: Path) -> dict:
    """Run `ros2 bag info` and return text output. Returns None on fail."""
    cmd = ["ros2", "bag", "info", str(bag_path)]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return res.stdout
    except subprocess.CalledProcessError as e:
        print(f"[FAIL] ros2 bag info failed: {e}")
        return None

def verify_run(run_dir: Path) -> bool:
    print(f"Verifying run in: {run_dir}")
    all_ok = True

    # 1. Structure
    all_ok &= check_file(run_dir / "config_resolved.yaml", "Resolved Config")
    all_ok &= check_file(run_dir / "logs", "Logs Directory")
    all_ok &= check_file(run_dir / "bags" / "output" / "metadata.yaml", "Rosbag Metadata")

    # 2. Config validity
    config = {}
    try:
        with open(run_dir / "config_resolved.yaml", "r") as f:
            config = yaml.safe_load(f)
        print("[OK] Config is valid YAML.")
    except Exception as e:
        print(f"[FAIL] Config parse error: {e}")
        all_ok = False

    # 3. Bag Content
    bag_path = run_dir / "bags" / "output"
    info = get_bag_info(bag_path)
    if info:
        print("[OK] Bag info retrieved.")
        # Check critical topics
        required_topics = ["/scan", "/tf", "/odom"]
        # Map is optional depending on export, but standard run has it.
        # We can perform loose check.
        missing = []
        for t in required_topics:
            if f"Topic: {t}" not in info and f"Topic: {t} " not in info:
                # "ros2 bag info" format varies, usually "Topic: /foo"
                # Let's simple check strict substring "Topic: /foo" is risky if spacing differs
                # Just check if topic name is present in output lines starting with Topic:
                pass
        
        # A more robust check:
        found_topics = []
        for line in info.splitlines():
            line = line.strip()
            if line.startswith("Topic:"):
                parts = line.split()
                if len(parts) >= 2:
                    found_topics.append(parts[1])
        
        for t in required_topics:
            if t not in found_topics:
                print(f"[WARN] Topic {t} not found in bag info.")
                # warning only for now? or fail?
                # all_ok = False 
        
        print(f"[INFO] Found {len(found_topics)} recorded topics.")
    else:
        all_ok = False

    # 4. Logs checks (simple)
    logs_dir = run_dir / "logs"
    if logs_dir.exists():
        for log_file in logs_dir.glob("*.log"):
            # Check for crash markers
            # This can be noisy, so maybe just check if size > 0
            if log_file.stat().st_size == 0:
                 print(f"[WARN] Log file empty: {log_file.name}")
    
    return all_ok

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 verify_results.py <run_dir_or_latest>")
        # Default to finding latest in results/runs if not specified? 
        # Let's keep it explicit for now or try to auto-find.
        base_runs = Path("results/runs")
        if base_runs.exists():
            # find latest
            runs = sorted([d for d in base_runs.iterdir() if d.is_dir()], key=lambda x: x.name)
            if runs:
                latest = runs[-1]
                print(f"No run specified, using latest: {latest}")
                if verify_run(latest):
                    sys.exit(0)
                else:
                    sys.exit(1)
            else:
                print("No runs found in results/runs.")
                sys.exit(1)
        else:
            sys.exit(1)
    
    run_dir = Path(sys.argv[1])
    if verify_run(run_dir):
        print("\n[SUCCESS] Verification Passed.")
        sys.exit(0)
    else:
        print("\n[FAILURE] Verification Failed.")
        sys.exit(1)

if __name__ == "__main__":
    main()
