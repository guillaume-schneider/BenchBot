#!/usr/bin/env python3
import sys
import yaml
import argparse
from pathlib import Path

# Add project root to sys.path so we can import 'runner' package
root_dir = Path(__file__).resolve().parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from runner.resolve import resolve_run_config, load_yaml, write_yaml, stable_run_id

def main():
    parser = argparse.ArgumentParser(description="Resolve and run a full benchmark matrix via CLI (Headless).")
    parser.add_argument("matrix_path", help="Path to matrix config yaml (e.g. configs/matrices/test_headless_ci.yaml)")
    args = parser.parse_args()
    
    matrix = load_yaml(args.matrix_path)
    output_jobs_dir = Path("results/jobs")
    output_jobs_dir.mkdir(parents=True, exist_ok=True)
    
    runs_to_execute = []
    
    print(f"Resolving matrix: {args.matrix_path}")
    
    # Very simplified resolution logic similar to GUI's RunWorker but synchronous
    # NOTE: This duplicates logic from gui/main.py. Ideally should be shared in runner/resolve.py
    
    for inc in matrix.get("matrix", {}).get("include", []):
        d_id = inc["dataset"]
        slams = inc["slams"]
        seeds = inc.get("seeds", [0])
        repeats = inc.get("repeats", 1)
        overrides = inc  # simplistic
        
        # Find dataset object
        d_obj = next((d for d in matrix["datasets"] if d["id"] == d_id), None)
        if not d_obj:
            print(f"Dataset {d_id} not found")
            continue
            
        for s_id in slams:
            s_entry = next((s for s in matrix["slams"] if s["id"] == s_id), None)
            if not s_entry:
                print(f"SLAM {s_id} not found")
                continue
                
            s_profile = load_yaml(s_entry["profile"])
            
            for seed in seeds:
                for r in range(repeats):
                    run_id = stable_run_id(d_id, s_id, seed, r)
                    print(f"  Job: {run_id}")
                    
                    resolved = resolve_run_config(
                        matrix=matrix,
                        dataset_obj=d_obj,
                        slam_entry=s_entry,
                        slam_profile=s_profile,
                        combo_overrides=overrides,
                        slam_overrides=None,
                        dataset_overrides=None,
                        seed=seed,
                        repeat_index=r,
                        run_id=run_id,
                        output_root=matrix["output"]["root_dir"]
                    )
                    
                    job_path = output_jobs_dir / f"{run_id}.yaml"
                    write_yaml(job_path, resolved)
                    runs_to_execute.append(job_path)
                    
    print(f"Generated {len(runs_to_execute)} jobs.")
    
    from runner.orchestrator import run_once
    
    for i, job_path in enumerate(runs_to_execute):
        print(f"--- Running Job {i+1}/{len(runs_to_execute)}: {job_path.name} ---")
        ret = run_once(str(job_path))
        if ret != 0:
            print(f"JOB FAILED with code {ret}")
            # Continue or stop? Benchmark usually continues
        else:
            print("JOB SUCCESS")

if __name__ == "__main__":
    main()
