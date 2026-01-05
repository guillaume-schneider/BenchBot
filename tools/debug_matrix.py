import sys
import yaml
from pathlib import Path
from runner.resolve import load_yaml, resolve_run_config, stable_run_id
from runner.orchestrator import run_once

PROJECT_ROOT = Path(__file__).parent.parent

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 debug_matrix.py <config_path>")
        sys.exit(1)

    config_path = sys.argv[1]
    matrix = load_yaml(config_path)
    
    # Minimal resolution logic targeting the first job found
    slams_map = {s["id"]: s for s in matrix.get("slams", [])}
    datasets_map = {d["id"]: d for d in matrix.get("datasets", [])}
    
    for inc in matrix.get("matrix", {}).get("include", []):
        d_id = inc["dataset"]
        dataset_def = datasets_map.get(d_id)
        
        for s_id in inc.get("slams", []):
            slam_entry = slams_map.get(s_id)
            profile_path = PROJECT_ROOT / slam_entry["profile"]
            slam_profile = load_yaml(profile_path)
            
            # Resolve first seed/repeat
            run_id = stable_run_id(d_id, s_id, 0, 0)
            output_root = "results/runs_debug" # Use separate dir for safety
            
            resolved = resolve_run_config(
                matrix=matrix, dataset_obj=dataset_def,
                slam_entry=slam_entry, slam_profile=slam_profile,
                combo_overrides=inc.get("overrides"),
                slam_overrides=slam_entry.get("overrides"),
                dataset_overrides=dataset_def.get("overrides"),
                seed=0, repeat_index=0, run_id=run_id, output_root=output_root
            )
            
            # Inject Headless for debug
            print("Force Headless for CLI debug...")
            # Modify cmd in scenario
            sc = resolved.get("dataset", {}).get("scenario", {})
            if "processes" in sc:
                for p in sc["processes"]:
                    if "cmd" in p:
                         # Append - use_gazebo:=False if not present
                         # Simplified logic: convert list to string/vice versa handled by orchestrator?
                         # Orchestrator expects list usually for cmd.
                         cmd = p["cmd"]
                         if isinstance(cmd, list):
                             p["cmd"] = cmd + ["use_gazebo:=False", "use_rviz:=False"] # HEADLESS TOTAL
            
            debug_conf_path = "debug_resolved.yaml"
            with open(debug_conf_path, "w") as f:
                yaml.dump(resolved, f)
                
            print(f"Running job: {run_id}")
            try:
                run_once(debug_conf_path)
            except Exception as e:
                import traceback
                traceback.print_exc()
            return # Run only one

if __name__ == "__main__":
    main()
