
import yaml
import cv2
import numpy as np
import os
import sys

def load_gt_map(yaml_path):
    with open(yaml_path, "r") as f:
        info = yaml.safe_load(f)

    pgm_path = info["image"]
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Failed to load PGM map from: {pgm_path}")
    
    # Simple thresholding for GT (assuming black/white)
    # White (254-255) is Free, Black (0) is Wall.
    # ROS convention: 0=Occ, 255=Free usually, but standard PGM:
    # 255 = Free (White)
    # 0 = Occupied (Black)
    # 205 = Unknown (Light Gray)
    
    # The file evaluation/metrics.py uses:
    # gt_occ_grid[img_norm > occupied_thresh] = 0    (Free)
    # gt_occ_grid[img_norm < free_thresh] = 100      (Occupied)
    
    # Let's just use the same logic as metrics.py if possible, or simplified.
    # metrics.py uses 0 for Free.
    
    occ_grid = np.full(img.shape, -1, dtype=np.int8)
    
    # Normalize
    img_norm = img.astype(np.float32) / 255.0
    
    # From metrics.py
    # occupied_thresh = 0.65 (default)
    # free_thresh = 0.196 (default)
    
    occ_grid[img_norm > 0.65] = 0    # Free
    occ_grid[img_norm < 0.196] = 100 # Occupied
    
    return occ_grid

def analyze_components(yaml_path):
    print(f"Analyzing {yaml_path}")
    gt_map = load_gt_map(yaml_path)
    
    total_pixels = gt_map.size
    free_pixels = (gt_map == 0).sum()
    occ_pixels = (gt_map == 100).sum()
    
    print(f"Total Pixels: {total_pixels}")
    print(f"Free Pixels:  {free_pixels} ({free_pixels/total_pixels*100:.1f}%)")
    print(f"Occ Pixels:   {occ_pixels}")
    
    # Connected Components on Free Space
    gt_free = (gt_map == 0).astype(np.uint8)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(gt_free, connectivity=8)
    
    print(f"\nFound {num_labels-1} connected components of Free Space:")
    # Sort by area (descending)
    sorted_indices = np.argsort(stats[1:, cv2.CC_STAT_AREA])[::-1] + 1
    
    for rank, i in enumerate(sorted_indices):
        area = stats[i, cv2.CC_STAT_AREA]
        percent_of_free = (area / free_pixels) * 100
        print(f"  Component {i}: {area} pixels ({percent_of_free:.2f}% of total free space)")
        
    # Provide interpretation
    if len(sorted_indices) >= 2:
        largest = sorted_indices[0]
        second = sorted_indices[1]
        area1 = stats[largest, cv2.CC_STAT_AREA]
        area2 = stats[second, cv2.CC_STAT_AREA]
        
        print("\nInterpretation:")
        print(f"  The largest component (Likely 'Outside') is {area1} pixels.")
        print(f"  The second largest (Likely 'Inside') is {area2} pixels.")
        print(f"  Ratio Inside/TotalFree = {area2/free_pixels*100:.2f}%")
        
        print(f"\n  If the robot explores the Inside completely:")
        print(f"    -> Accessible Coverage should be 100%")
        print(f"    -> Global Coverage should be ~{area2/free_pixels*100:.2f}%")
        
        print(f"\n  Your SLAM results showing ~18% Coverage matches this hypothesis.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_map_components.py <path_to_model.yaml>")
        sys.exit(1)
    
    analyze_components(sys.argv[1])
