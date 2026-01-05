#!/usr/bin/env python3
import sys
import os
import json
import yaml
import optuna
import shutil
import time
from pathlib import Path
from runner.resolve import load_yaml, write_yaml

def set_nested_value(dic, keys, value):
    for key in keys[:-1]:
        dic = dic.setdefault(key, {})
    dic[keys[-1]] = value

def get_nested_value(dic, keys):
    for key in keys:
        dic = dic.get(key)
        if dic is None: return None
    return dic

class SLAMOptimizer:
    def __init__(self, base_job_path, tuning_config_path, study_name="slam_optimization"):
        self.base_job_path = Path(base_job_path)
        self.tuning_config_path = Path(tuning_config_path)
        self.base_config = load_yaml(self.base_job_path)
        self.tuning_config = load_yaml(self.tuning_config_path)
        self.study_name = study_name
        
        self.output_dir = Path("results/optimization") / study_name
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Add project root to path for runner
        root_dir = Path(__file__).resolve().parent.parent
        if str(root_dir) not in sys.path:
            sys.path.insert(0, str(root_dir))

    def objective(self, trial):
        # 1. Suggest parameters
        trial_config = self.base_config.copy()
        
        # Deep copy to avoid mutation of shared objects in dict
        import copy
        trial_config = copy.deepcopy(self.base_config)
        
        suggested_params = {}
        for param_name, spec in self.tuning_config.get("params", {}).items():
            ptype = spec.get("type", "float")
            if ptype == "float":
                val = trial.suggest_float(param_name, spec["min"], spec["max"], log=spec.get("log", False))
            elif ptype == "int":
                val = trial.suggest_int(param_name, spec["min"], spec["max"])
            elif ptype == "categorical":
                val = trial.suggest_categorical(param_name, spec["choices"])
            else:
                continue
            
            suggested_params[param_name] = val
            
            # Param name is dot-separated path, e.g., "slam.parameters.slam_gmapping.maxUrange"
            keys = param_name.split(".")
            set_nested_value(trial_config, keys, val)
        
        # 2. Prepare temp run
        run_name = f"trial_{trial.number}"
        temp_job_path = self.output_dir / f"job_{run_name}.yaml"
        
        # Override run_id and paths to avoid collision
        trial_config["run_id"] = f"{self.study_name}_{run_name}"
        trial_config["paths"]["root_dir"] = str(self.output_dir / "runs" / run_name)
        trial_config["paths"]["bags_dir"] = str(self.output_dir / "runs" / run_name / "bags")
        trial_config["paths"]["logs_dir"] = str(self.output_dir / "runs" / run_name / "logs")
        trial_config["paths"]["metrics_json"] = str(self.output_dir / "runs" / run_name / "metrics.json")
        
        Path(trial_config["paths"]["root_dir"]).mkdir(parents=True, exist_ok=True)
        write_yaml(temp_job_path, trial_config)
        
        # 3. Execute
        from runner.orchestrator import run_once
        print(f"\n[OPTIMIZER] Starting Trial {trial.number} with {suggested_params}")
        
        try:
            # We must isolate rclpy shutdown/init if possible or hope it handles it
            # run_once calls rclpy.init/shutdown. 
            # In a loop, we might need a separate process to be safe
            import subprocess
            cmd = [sys.executable, "runner/run_one.py", str(temp_job_path)]
            result = subprocess.run(cmd, capture_output=False) # allow prints to stdout
            
            if result.returncode != 0:
                print(f"[OPTIMIZER] Trial {trial.number} failed with code {result.returncode}")
                return float('inf') # Return high error on crash
        except Exception as e:
            print(f"[OPTIMIZER] Error during trial {trial.number}: {e}")
            return float('inf')

        # 4. Gather Result
        metrics_file = Path(trial_config["paths"]["metrics_json"])
        if metrics_file.exists():
            with open(metrics_file, "r") as f:
                res = json.load(f)
                rmse = res.get("ate_rmse")
                if rmse is None:
                    print(f"[OPTIMIZER] Trial {trial.number} produced no RMSE metric")
                    return float('inf')
                print(f"[OPTIMIZER] Trial {trial.number} Result: RMSE={rmse:.4f}")
                return rmse
        else:
            print(f"[OPTIMIZER] Trial {trial.number} produced no metrics.json")
            return float('inf')

    def run(self, n_trials=20):
        study = optuna.create_study(
            study_name=self.study_name,
            direction="minimize",
            storage=f"sqlite:///{self.output_dir}/study.db",
            load_if_exists=True
        )
        study.optimize(self.objective, n_trials=n_trials)
        
        print("\n" + "="*40)
        print("OPTIMIZATION COMPLETE")
        print(f"Best Trial: {study.best_trial.number}")
        print(f"Best Value (RMSE): {study.best_value}")
        print("Best Params:")
        for k, v in study.best_params.items():
            print(f"  {k}: {v}")
        print("="*40)
        
        # Save results to a report file
        report = {
            "best_value": study.best_value,
            "best_params": study.best_params,
            "best_trial": study.best_trial.number
        }
        with open(self.output_dir / "best_result.json", "w") as f:
            json.dump(report, f, indent=4)
        
        return study

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("base_job", help="Resolved job YAML")
    parser.add_argument("tuning_config", help="Search space YAML")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--name", default="optimizer_run")
    args = parser.parse_args()
    
    opt = SLAMOptimizer(args.base_job, args.tuning_config, args.name)
    opt.run(n_trials=args.trials)
