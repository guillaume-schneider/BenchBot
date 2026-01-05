#!/usr/bin/env python3
"""
Skeleton evaluation script for frontier-based exploration runs.

What it does:
- Loads a GT occupancy grid map from a YAML + PGM.
- Reads a ros2 bag (rosbag2) with topics:
    - /map  (nav_msgs/OccupancyGrid)
    - /odom (nav_msgs/Odometry)
- Computes:
    - time_to_50, time_to_80, time_to_90 percent coverage
    - final occupancy IoU vs GT
    - total path length from /odom

Requirements:
    - ROS 2 Python environment (source your ROS 2 + workspace)
    - rosbag2_py
    - numpy
    - opencv-python
    - pyyaml

This is a skeleton: adjust topic names, thresholds,
and map alignment to your setup.
"""

import os
import math
import yaml
import cv2
import numpy as np

import rclpy              
from rclpy.serialization import deserialize_message# just for time helpers (optional)
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from rosidl_runtime_py.utilities import get_message
from builtin_interfaces.msg import Time as RosTime


# ========= USER CONFIGURATION =========
BAG_PATH = "/home/guillaume/Documents/Projects/benchmark/res/fbe_run2"  # e.g. "/home/guillaume/fbe_run1"
MAP_TOPIC = "/map"
ODOM_TOPIC = "/odom"

GT_MAP_YAML = "/home/guillaume/gt_turtlebot3_world.yaml"           # e.g. "/home/guillaume/gt_turtlebot3_world.yaml"

COVERAGE_THRESHOLDS = [0.5, 0.8, 0.9]   # 50%, 80%, 90%
# =====================================


# ---------- Helpers ----------

def ros_time_to_sec(t: RosTime) -> float:
    """Convert builtin_interfaces/Time to seconds (float)."""
    return float(t.sec) + float(t.nanosec) * 1e-9


def load_gt_map(yaml_path: str):
    """
    Load ground-truth map from <name>.yaml + <name>.pgm.
    Returns:
        gt_occ_grid: np.ndarray of shape (H, W) with int8 data (0..100, -1 for unknown)
        resolution: float (m/cell)
        origin: (x, y, yaw) of map in world frame
    """
    with open(yaml_path, "r") as f:
        info = yaml.safe_load(f)

    pgm_path = info["image"]
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    # load pgm as grayscale
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Failed to load PGM map from: {pgm_path}")

    # YAML fields (nav2/slam_toolbox convention)
    resolution = float(info["resolution"])
    origin = info["origin"]   # [x, y, yaw]
    negate = int(info.get("negate", 0))
    occupied_thresh = float(info.get("occupied_thresh", 0.65))
    free_thresh = float(info.get("free_thresh", 0.196))

    # Convert PGM to occupancy values [0..100] + -1 for unknown
    # Here's a simple heuristic; you may want to tune it to your pipeline.
    img_norm = img.astype(np.float32) / 255.0
    if negate:
        img_norm = 1.0 - img_norm

    gt_occ_grid = np.full(img_norm.shape, -1, dtype=np.int8)
    gt_occ_grid[img_norm > occupied_thresh] = 100
    gt_occ_grid[img_norm < free_thresh] = 0
    
    # Keep the map in its original orientation for calculations
    # We'll handle the flip in visualization if needed

    return gt_occ_grid, resolution, origin


def open_bag_reader(bag_path: str) -> SequentialReader:
    """Open a ros2 bag for reading, auto-detecting storage format."""
    # Try different storage formats
    storage_formats = ["sqlite3", "mcap"]
    
    for storage_id in storage_formats:
        try:
            storage_options = StorageOptions(
                uri=bag_path,
                storage_id=storage_id
            )
            converter_options = ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            )
            reader = SequentialReader()
            reader.open(storage_options, converter_options)
            return reader
        except Exception as e:
            if storage_id == storage_formats[-1]:
                # Last format failed, raise error
                raise RuntimeError(
                    f"Failed to open rosbag at '{bag_path}'. "
                    f"Tried formats: {storage_formats}. "
                    f"Make sure the path points to a valid ROS2 bag directory. "
                    f"Last error: {str(e)}"
                )
            # Try next format
            continue


def read_messages_by_topic(bag_path: str, topics_of_interest):
    """
    Read all messages from a ros2 bag for given topics.

    Returns:
        dict: { topic_name: [(timestamp_sec, msg), ...] }
    """
    try:
        reader = open_bag_reader(bag_path)
    except Exception as e:
        raise RuntimeError(f"Cannot open rosbag: {str(e)}")

    # Map topic name -> type string
    topic_types = {}
    for topic in reader.get_all_topics_and_types():
        topic_types[topic.name] = topic.type

    # Prepare message classes only for the topics we care about
    msg_classes = {}
    for tname in topics_of_interest:
        if tname in topic_types:
            msg_classes[tname] = get_message(topic_types[tname])

    messages = {t: [] for t in topics_of_interest}

    while reader.has_next():
        topic_name, data, t = reader.read_next()
        if topic_name not in topics_of_interest:
            continue

        msg_cls = msg_classes.get(topic_name)
        if msg_cls is None:
            # topic not in our interest list or type unknown
            continue

        # ✅ correct way: use rclpy.serialization
        msg = deserialize_message(data, msg_cls)

        # t is nanoseconds since bag start (int) in rosbag2; convert to seconds
        timestamp_sec = float(t) * 1e-9
        messages[topic_name].append((timestamp_sec, msg))

    # sort by time
    for t in messages:
        messages[t].sort(key=lambda x: x[0])

    return messages



# -- Helper --

def align_est_map_to_gt(gt_map, gt_res, gt_origin, est_msg):
    """
    Projette une OccupancyGrid 'est_msg' dans le repère de la carte GT
    en utilisant les origines + résolution (on suppose pas de rotation).

    Retourne une grille de même taille que gt_map, remplie avec:
        - valeurs de la carte estimée là où c'est défini
        - -1 ailleurs
    """
    gt_h, gt_w = gt_map.shape

    est_res = est_msg.info.resolution
    est_w = est_msg.info.width
    est_h = est_msg.info.height

    # on suppose résolutions identiques (slam_toolbox + map_saver)
    if abs(est_res - gt_res) > 1e-6:
        print(f"[WARN] est_res={est_res} != gt_res={gt_res}, on applique un facteur d'échelle.")
    scale = est_res / gt_res

    gt_origin_x, gt_origin_y, _ = gt_origin
    est_origin_x = est_msg.info.origin.position.x
    est_origin_y = est_msg.info.origin.position.y

    print(f"[DEBUG] GT map: {gt_w}x{gt_h}, res={gt_res}, origin=({gt_origin_x}, {gt_origin_y})")
    print(f"[DEBUG] Est map: {est_w}x{est_h}, res={est_res}, origin=({est_origin_x}, {est_origin_y})")

    est_flat = np.array(est_msg.data, dtype=np.int8)
    est_grid = est_flat.reshape((est_h, est_w))
    
    # Count known cells in estimated map
    known_cells = np.sum(est_grid != -1)
    free_cells = np.sum(est_grid == 0)
    occupied_cells = np.sum(est_grid > 50)
    print(f"[DEBUG] Est map stats: {known_cells} known ({free_cells} free, {occupied_cells} occupied)")

    # carte de sortie dans le repère GT
    est_on_gt = np.full_like(gt_map, -1, dtype=np.int8)
    
    cells_mapped = 0

    # IMPORTANT: The GT map generator uses np.flipud() when saving the PGM
    # This means when we load it with cv2.imread, it's ALREADY in ROS convention
    # (row 0 = bottom, increasing row = increasing Y in world coordinates)
    # So we DON'T need to flip the Y coordinate - use same logic as X
    
    # boucle sur les cellules de la carte estimée
    for i in range(est_h):
        for j in range(est_w):
            val = est_grid[i, j]
            if val == -1:
                continue  # inconnue, on ignore

            # coord monde (centre de cellule)
            # In ROS OccupancyGrid, row i corresponds to y = origin_y + i * resolution
            world_x = est_origin_x + (j + 0.5) * est_res
            world_y = est_origin_y + (i + 0.5) * est_res

            # Transform to GT map indices
            # Both X and Y use the same logic since GT is already in ROS convention
            gt_j = int((world_x - gt_origin_x) / gt_res)
            gt_i = int((world_y - gt_origin_y) / gt_res)

            if 0 <= gt_i < gt_h and 0 <= gt_j < gt_w:
                est_on_gt[gt_i, gt_j] = val
                cells_mapped += 1
    
    print(f"[DEBUG] Cells successfully mapped to GT: {cells_mapped}")

    return est_on_gt


# ---------- Metrics ----------

def occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin):
    """
    Utilise le dernier /map et l'aligne correctement sur la GT
    en utilisant les origines + résolution.
    Returns: (est_on_gt, last_map_msg)
    """
    if not map_msgs:
        raise RuntimeError("No /map messages found in bag.")

    _, last_map = map_msgs[-1]
    est_on_gt = align_est_map_to_gt(gt_map, gt_res, gt_origin, last_map)
    return est_on_gt



def compute_coverage(gt_map, est_map):
    """
    Coverage: fraction of free GT cells that are known (free or occupied) in est_map.
    This measures coverage of the ENTIRE GT map.
    """
    gt_free = (gt_map == 0)
    est_known = (est_map != -1)

    if gt_free.sum() == 0:
        return 0.0

    covered = (gt_free & est_known).sum()
    return covered / gt_free.sum()


def compute_accessible_coverage(gt_map, est_map, gt_res, gt_origin, est_origin, est_width, est_height, est_res):
    """
    Coverage of the accessible area only (bounding box of the estimated map).
    This gives a more meaningful metric for exploration performance.
    """
    # Calculate the bounding box of the estimated map in world coordinates
    est_min_x = est_origin[0]
    est_max_x = est_origin[0] + est_width * est_res
    est_min_y = est_origin[1]
    est_max_y = est_origin[1] + est_height * est_res
    
    # Convert to GT map indices
    gt_h, gt_w = gt_map.shape
    gt_origin_x, gt_origin_y, _ = gt_origin
    
    # Calculate bounding box in GT map indices
    # Since GT is already in ROS convention (flipud in generator), use same logic as alignment
    gt_j_min = max(0, int((est_min_x - gt_origin_x) / gt_res))
    gt_j_max = min(gt_w, int((est_max_x - gt_origin_x) / gt_res) + 1)
    gt_i_min = max(0, int((est_min_y - gt_origin_y) / gt_res))
    gt_i_max = min(gt_h, int((est_max_y - gt_origin_y) / gt_res) + 1)
    
    # Extract the accessible region from both maps
    gt_accessible = gt_map[gt_i_min:gt_i_max, gt_j_min:gt_j_max]
    est_accessible = est_map[gt_i_min:gt_i_max, gt_j_min:gt_j_max]
    
    # Compute coverage in this region
    gt_free = (gt_accessible == 0)
    est_known = (est_accessible != -1)
    
    if gt_free.sum() == 0:
        return 0.0
    
    covered = (gt_free & est_known).sum()
    return covered / gt_free.sum()


def compute_iou(gt_map, est_map):
    """
    IoU over OCCUPIED cells (100 vs >50 threshold).
    """
    gt_occ = (gt_map > 50)
    est_occ = (est_map > 50)

    intersection = (gt_occ & est_occ).sum()
    union = (gt_occ | est_occ).sum()
    return float(intersection) / float(union) if union > 0 else 0.0


def compute_time_to_coverage(gt_map, gt_res, gt_origin, map_msgs, thresholds):
    """
    Compute time to reach each coverage threshold using
    un alignement correct est->GT pour chaque carte.
    """
    times_to_cov = {th: None for th in thresholds}
    reached = {th: False for th in thresholds}

    for t, msg in map_msgs:
        if msg.info.width == 0 or msg.info.height == 0:
            continue

        est_on_gt = align_est_map_to_gt(gt_map, gt_res, gt_origin, msg)
        cov = compute_coverage(gt_map, est_on_gt)

        for th in thresholds:
            if not reached[th] and cov >= th:
                times_to_cov[th] = t
                reached[th] = True

    return times_to_cov


def compute_path_length(odom_msgs):
    """
    Integrate path length from /odom messages.
    """
    if not odom_msgs:
        return 0.0

    prev_x = None
    prev_y = None
    length = 0.0

    for t, msg in odom_msgs:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if prev_x is not None:
            dx = x - prev_x
            dy = y - prev_y
            length += math.sqrt(dx*dx + dy*dy)

        prev_x, prev_y = x, y

    return length


def get_trajectory(odom_msgs):
    """
    Extract x, y points from a list of (timestamp, msg) odom messages.
    Returns: (list_x, list_y)
    """
    tx, ty = [], []
    for _, msg in odom_msgs:
        tx.append(msg.pose.pose.position.x)
        ty.append(msg.pose.pose.position.y)
    return tx, ty


def detect_anomalies(odom_msgs, ate_rmse=None):
    """
    Heuristic-based failure detection.
    Returns: (list of warning strings, is_failure boolean)
    """
    warnings = []
    is_failure = False
    
    if not odom_msgs:
        return (["No odometry data found."], True)
        
    # 1. Minimum Movement Check
    total_dist = compute_path_length(odom_msgs)
    if total_dist < 0.2: # Less than 20cm
        warnings.append("Robot barely moved (Stuck/Process fail)")
        is_failure = True
        
    # 2. Velocity / Jump Check
    # TB3 max speed is ~0.26 m/s. Let's flag anything > 1.5 m/s as a TF Jump
    max_jump = 0.0
    for i in range(1, len(odom_msgs)):
        t1, p1 = odom_msgs[i-1]
        t2, p2 = odom_msgs[i]
        dt = t2 - t1
        if dt <= 0: continue
        
        dx = p2.pose.pose.position.x - p1.pose.pose.position.x
        dy = p2.pose.pose.position.y - p1.pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        vel = dist / dt
        
        if vel > 1.5: 
            max_jump = max(max_jump, vel)
            
    if max_jump > 0:
        warnings.append(f"Major TF Jump detected! (Max speed pulse: {max_jump:.2f} m/s)")
        is_failure = True

    # 3. Accuracy Fail
    if ate_rmse is not None and ate_rmse > 1.0:
        warnings.append(f"Massive drift detected! (ATE RMSE: {ate_rmse:.3f} m)")
        is_failure = True

    return warnings, is_failure


# ---------- Main ----------

def main():
    print("=== Frontier-based exploration evaluation ===")
    print(f"Bag: {BAG_PATH}")
    print(f"GT map: {GT_MAP_YAML}")

    # Load GT
    gt_map, gt_res, gt_origin = load_gt_map(GT_MAP_YAML)
    print(f"Loaded GT map: shape={gt_map.shape}, res={gt_res}, origin={gt_origin}")

    # Read messages
    topics = [MAP_TOPIC, ODOM_TOPIC]
    msgs_by_topic = read_messages_by_topic(BAG_PATH, topics)

    map_msgs = msgs_by_topic.get(MAP_TOPIC, [])
    odom_msgs = msgs_by_topic.get(ODOM_TOPIC, [])

    if not map_msgs:
        raise RuntimeError(f"No messages on {MAP_TOPIC} in bag.")
    if not odom_msgs:
        print(f"Warning: no messages on {ODOM_TOPIC}; path length will be 0.")

    # Final map -> IoU, final coverage
    est_map_final = occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin)
    final_cov = compute_coverage(gt_map, est_map_final)
    final_iou = compute_iou(gt_map, est_map_final)

    times_to_cov = compute_time_to_coverage(
        gt_map, gt_res, gt_origin, map_msgs, COVERAGE_THRESHOLDS
    )
    # Path length
    path_len = compute_path_length(odom_msgs)

    # Results
    print("\n--- Results ---")
    print(f"Final coverage         : {final_cov*100:.1f} %")
    print(f"Final occupancy IoU    : {final_iou:.3f}")
    print(f"Total path length      : {path_len:.2f} m")

    # Anomalies
    warnings, is_fail = detect_anomalies(odom_msgs, final_iou) # Use IoU as proxy for ATE if needed
    if warnings:
        print("\n--- Warnings/Anomalies ---")
        for w in warnings:
            print(f" [!] {w}")

    # bag time is relative; we just print seconds since first message
    t0 = map_msgs[0][0]
    for th, t in times_to_cov.items():
        if t is None:
            print(f"Time to {int(th*100)}% coverage: not reached")
        else:
            print(f"Time to {int(th*100)}% coverage: {t - t0:.2f} s (bag-relative)")

    print("\nDone.")


if __name__ == "__main__":
    main()
