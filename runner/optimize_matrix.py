#!/usr/bin/env python3
import sys
import yaml
import argparse
import json
from pathlib import Path

# Add project root to sys.path
root_dir = Path(__file__).resolve().parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from runner.resolve import resolve_run_config, load_yaml, write_yaml, stable_run_id
from tools.optimizer import SLAMOptimizer

def main():
    parser = argparse.ArgumentParser(description="Automate multiple SLAM optimizations based on a matrix config.")
    parser.add_argument("matrix_path", help="Path to matrix config yaml containing optimization blocks")
    args = parser.parse_args()
    
    matrix = load_yaml(args.matrix_path)
    output_jobs_dir = Path("results/jobs")
    output_jobs_dir.mkdir(parents=True, exist_ok=True)
    
    # We look for a new top-level key: "optimization_studios"
    studios = matrix.get("optimization_studios", [])
    if not studios:
        print("No 'optimization_studios' found in the matrix file.")
        print("Example structure:")
        print("optimization_studios:")
        print("  - name: 'my_tune'")
        print("    dataset: 'ds_id'")
        print("    slam: 'slam_id'")
        print("    trials: 10")
        print("    params: { 'path.to.param': { 'min': 0, 'max': 1 } }")
        return

    print(f"Found {len(studios)} optimization studio(s) to process.")

    for studio in studios:
        name = studio.get("name", "unnamed_opt")
        d_id = studio.get("dataset")
        s_id = studio.get("slam")
        trials = studio.get("trials", 10)
        params_spec = studio.get("params", {})

        print(f"\n>>> Starting Studio: {name} (SLAM: {s_id}, Dataset: {d_id})")

        # 1. Resolve the "Template" config (Trial 0 baseline)
        d_obj = next((d for d in matrix.get("datasets", []) if d["id"] == d_id), None)
        s_entry = next((s for s in matrix.get("slams", []) if s["id"] == s_id), None)
        
        if not d_obj or not s_entry:
            print(f"Error: Dataset {d_id} or SLAM {s_id} not found in matrix.")
            continue

        s_profile = load_yaml(s_entry["profile"])
        run_id = f"opt_base_{name}"
        
        resolved = resolve_run_config(
            matrix=matrix,
            dataset_obj=d_obj,
            slam_entry=s_entry,
            slam_profile=s_profile,
            combo_overrides={},
            slam_overrides=None,
            dataset_overrides=None,
            seed=0,
            repeat_index=0,
            run_id=run_id,
            output_root="results/optimization/temp"
        )
        
        base_job_path = output_jobs_dir / f"{run_id}.yaml"
        write_yaml(base_job_path, resolved)

        # 2. Create tuning spec file
        tuning_spec = {"params": params_spec}
        tuning_path = Path("results/optimization") / name / "search_space.yaml"
        tuning_path.parent.mkdir(parents=True, exist_ok=True)
        write_yaml(tuning_path, tuning_spec)

        # 3. Launch Optimizer
        optimizer = SLAMOptimizer(base_job_path, tuning_path, name)
        study = optimizer.run(n_trials=trials)
        
        print(f"Studio {name} Finished! Best Value: {study.best_value:.4f}")

if __name__ == "__main__":
    main()
