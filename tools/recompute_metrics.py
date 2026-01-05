#!/usr/bin/env python3
import sys
import json
import os
import argparse
import rclpy
from pathlib import Path
from evaluation.metrics import (
    load_gt_map, 
    read_messages_by_topic, 
    occupancy_arrays_from_msgs, 
    compute_coverage, 
    compute_iou,
    compute_accessible_coverage,
    save_map_image,
    compute_ssim,
    compute_wall_thickness
)

def reevaluate_run(run_dir):
    run_path = Path(run_dir)
    print(f"Checking run: {run_path.name}")
    
    metrics_path = run_path / "metrics.json"
    bags_dir = run_path / "bags" / "output"
    
    if not metrics_path.exists():
        print(f" [SKIP] No metrics.json found in {run_path}")
        return False
        
    try:
        with open(metrics_path, 'r') as f:
            metrics = json.load(f)
    except Exception as e:
        print(f" [ERROR] Could not read metrics.json: {e}")
        return False
        
    has_iou = "occupancy_iou" in metrics
    has_ssim = "map_ssim" in metrics
    has_wall = "wall_thickness_m" in metrics
    has_noise = "lidar_noise" in metrics
    has_images = (run_path / "final_map.png").exists() and (run_path / "gt_map.png").exists()
    
    # Force recompute if requested or missing
    if has_iou and has_ssim and has_wall and has_noise and has_images:
        print(" [SKIP] Run already evaluated with new metrics.")
        return True

    print(" [INFO] Re-evaluating run metrics...")
    
    gt_map_path = None
    # Try to recover degradation settings from config
    degradation = None
    config_candidates = [run_path / "config_resolved.yaml", run_path / "run_config.yaml"]
    for cp in config_candidates:
        if cp.exists():
            try:
                import yaml
                with open(cp) as f:
                    cfg = yaml.safe_load(f)
                    if "degradation" in cfg:
                        degradation = cfg["degradation"]
                        # Also use this loop to find GT map if needed later
                        gt_def = cfg.get("dataset", {}).get("ground_truth", {})
                        if gt_def:
                             project_root = Path(__file__).resolve().parent.parent
                             if not gt_map_path: 
                                 gt_map_path = project_root / gt_def.get("map_path")
                    # If we found degradation, keep it
                    if degradation: break
            except: pass

    if degradation:
        metrics["degradation"] = degradation
        metrics["lidar_noise"] = degradation.get("noise_std")
        metrics["lidar_range"] = degradation.get("max_range")
        metrics["speed_scale"] = degradation.get("speed_scale")
    
    # 1. Identify config to find GT map
    # We need to find the executed config copy or infer from original config
    # Orchestrator saves run config usually?
    # run_config.yaml is usually saved in run_dir? 
    # Let's look for a yaml file that entails the config.
    # The `gui/pages/benchmark.py` parses ID like `2024...__dataset__slam`.
    # Let's try to find a saved config. If not, we have to look into `configs/matrices/...`
    # But wait, orchestrator copies `config.yaml` to run_dir usually? 
    # Let's check a run folder structure in a future step if needed, but for now assuming we can load a config.
    # Actually, orchestrator saves `metrics.json` but doesn't explicitly save `run_config` in `orchestrator.py`? 
    # Wait, `run_once` takes `config_path`. `run_batch` likely copies it?
    
    # Heuristic: try to find the GT map from the `dataset` name in run ID.
    parts = run_path.name.split("__")
    dataset_name = ""
    if len(parts) >= 2:
        dataset_name = parts[1]
    
    # Load the matrix/dataset config to find GT (this is tricky without the original config file)
    # However, `orchestrator.py` logic was: `gt_def = cfg.get("dataset", {}).get("ground_truth", {})`
    # We can try to grep the GT path from the log file? Or just search for standard GT maps.
    
    # Better approach: We can look at `tools/benchmark.py` which was used for ATE. ATE uses `read_bag_data`.
    # We need the GT Map YAML.
    
    # Let's assume standard locations or try to find a `config.yaml` in the run dir.
    # If not, we default to a known map if dataset matches?
    # Let's try to locate the file `run_config.yaml` inside the run dir.
    
    run_config_path = run_path / "run_config.yaml"
    gt_map_path = None
    gt_res = 0.05
    gt_origin = [0,0,0]
    
    if run_config_path.exists():
        import yaml
        with open(run_config_path) as f:
            cfg = yaml.safe_load(f)
            gt_def = cfg.get("dataset", {}).get("ground_truth", {})
            if gt_def:
                # Resolve relative to project root
                # Config saved in run_dir might have preserved relative paths from project root
                # We need project root.
                project_root = Path(__file__).resolve().parent.parent
                gt_map_path = project_root / gt_def.get("map_path")

    if not gt_map_path or not gt_map_path.exists():

        # Fallback: Check if we can find it via the logs or just guess
        # If dataset is 'tb3_sim_explore_modeA', map is usually 'maps/turtlebot3_world/gt_map.yaml'
        # Let's verify commonly used maps
        project_root = Path(__file__).resolve().parent.parent
        
        candidates = [
            project_root / "maps/turtlebot3_world/gt_map.yaml",
            project_root / "maps/gt/model.yaml",
            project_root / "gt_map/model.yaml"
        ]
        
        for cand in candidates:
             if cand.exists():
                 gt_map_path = cand
                 print(f" [WARN] Using inferred GT map: {gt_map_path}")
                 break

    if not gt_map_path or not gt_map_path.exists():
        print(f" [ERROR] Could not locate Ground Truth map for {run_path.name}")
        return False
        
    # Load GT Map
    try:
        gt_map, gt_res, gt_origin = load_gt_map(str(gt_map_path))
    except Exception as e:
        print(f" [ERROR] Failed to load GT map: {e}")
        return False
        
    # Read Bag
    if not bags_dir.exists():
        print(f" [ERROR] Bag directory not found: {bags_dir}")
        return False
        
    try:
        msgs = read_messages_by_topic(str(bags_dir), ["/map"])
        map_msgs = msgs.get("/map", [])
    except Exception as e:
        print(f" [ERROR] Error reading bag: {e}")
        return False
        
    if not map_msgs:
        print(" [WARN] No map messages found in bag.")
        return False
        
    # Compute Metrics
    try:
        est_on_gt = occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin)
        
        iou = compute_iou(gt_map, est_on_gt)
        # Recalculate coverage just in case
        coverage = compute_coverage(gt_map, est_on_gt)
        
        # Accessible
        _, last_map_msg = map_msgs[-1]
        est_origin_pose = [
            last_map_msg.info.origin.position.x, 
            last_map_msg.info.origin.position.y, 
            0
        ]
        accessible_cov = compute_accessible_coverage(
            gt_map, est_on_gt, gt_res, gt_origin,
            est_origin_pose, last_map_msg.info.width, 
            last_map_msg.info.height, last_map_msg.info.resolution
        )
        
        ssim_val = compute_ssim(gt_map, est_on_gt)
        wall_thick = compute_wall_thickness(est_on_gt, gt_res)
        
        # Save Images
        map_img_path = run_path / "final_map.png"
        gt_img_path = run_path / "gt_map.png"
        
        save_map_image(est_on_gt, map_img_path)
        save_map_image(gt_map, gt_img_path)
        
        print(f" [OK] Metrics computed: IoU={iou:.3f}, AccCov={accessible_cov:.1f}%, SSIM={ssim_val:.3f}")
        
        # Update metrics.json
        metrics["occupancy_iou"] = iou
        metrics["accessible_coverage"] = accessible_cov
        metrics["coverage"] = coverage # Update just in case
        metrics["map_ssim"] = ssim_val
        metrics["wall_thickness_m"] = wall_thick
        
        # Save absolute paths or relative? GUI parser handles filename if we follow convention?
        # The GUI parser currently does NOT look for image paths. 
        # The Report Generator needs paths.
        # Let's save absolute paths
        metrics["map_image_path"] = str(map_img_path)
        metrics["gt_map_image_path"] = str(gt_img_path)
        
        with open(metrics_path, 'w') as f:
            json.dump(metrics, f, indent=4)
            
        return True
        
    except Exception as e:
        print(f" [ERROR] Computation failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    parser = argparse.ArgumentParser(description="Re-evaluate runs to add IoU/Coverage/Images")
    parser.add_argument("--runs_dir", default="results/runs", help="Directory containing run folders")
    args = parser.parse_args()
    
    rclpy.init()
    
    root = Path.cwd() / args.runs_dir
    if not root.exists():
        print(f"Runs directory not found: {root}")
        return
        
    print(f"Scanning {root} for runs to update...")
    
    count = 0
    updated = 0
    
    for run_dir in root.iterdir():
        if run_dir.is_dir():
            count += 1
            if reevaluate_run(run_dir):
                updated += 1
                
    print(f"\nDone. Processed {count} runs, updated {updated}.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
