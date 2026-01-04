#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sqlite3
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import tf_transformations

# Import message types (dynamically or statically)
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def read_bag_data(bag_path):
    """
    Reads a rosbag and extracts:
    - GT Poses: (timestamp, x, y) from /gazebo/model_states
    - TF Updates: list of (timestamp, TransformStamped)
    """
    bag_path = str(bag_path)
    if not Path(bag_path).exists():
        raise FileNotFoundError(f"Bag not found: {bag_path}")

    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        # Try appending .db3 if it's a file path
        if Path(bag_path).is_dir():
            # Find metadata.yaml? rosbag2 opens folder.
            pass
        return [], [], []

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    gt_poses = []
    tf_data = []
    odom_data = []

    msg_count = 0
    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        msg_type_str = type_map[topic]
        
        try:
            msg_type = get_message(msg_type_str)
            msg = deserialize_message(data, msg_type)
        except Exception as e:
            # print(f"Deserialization error for {topic}: {e}")
            continue

        if topic == '/gazebo/model_states':
            # msg is ModelStates
            try:
                idx = -1
                for i, name in enumerate(msg.name):
                    if 'turtlebot3' in name:
                        idx = i
                        break
                
                if idx != -1:
                    pose = msg.pose[idx]
                    gt_poses.append((t_ns, pose.position.x, pose.position.y))
            except Exception:
                pass

        elif topic == '/tf' or topic == '/tf_static':
            for transform in msg.transforms:
                # Normalize frame names
                transform.header.frame_id = transform.header.frame_id.strip().lstrip('/')
                transform.child_frame_id = transform.child_frame_id.strip().lstrip('/')
                tf_data.append((t_ns, transform))
        
        elif topic == '/odom':
            # msg is Odometry
            # Normalize frame names
            msg.header.frame_id = msg.header.frame_id.strip().lstrip('/')
            msg.child_frame_id = msg.child_frame_id.strip().lstrip('/')
            
            # Store as translation and quaternion
            trans = msg.pose.pose.position
            rot = msg.pose.pose.orientation
            odom_data.append((t_ns, np.array([trans.x, trans.y, trans.z]), np.array([rot.x, rot.y, rot.z, rot.w])))
        
        msg_count += 1
        
    print(f"Read: {len(gt_poses)} GT, {len(tf_data)} TF, {len(odom_data)} Odom.")
    return gt_poses, tf_data, odom_data

class TFBuffer:
    def __init__(self):
        # (parent, child) -> (timestamp, translation, rotation)
        self.transforms = {}

    def update(self, t_ns, transform):
        parent = transform.header.frame_id
        child = transform.child_frame_id
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        self.transforms[(parent, child)] = (
            t_ns,
            np.array([trans.x, trans.y, trans.z]),
            np.array([rot.x, rot.y, rot.z, rot.w])
        )

    def lookup_transform_map_odom(self):
        if ('map', 'odom') in self.transforms:
            return self.transforms[('map', 'odom')]
        return None

def calculate_ate(gt_poses, tf_data, odom_data):
    # Sort by time
    tf_data.sort(key=lambda x: x[0])
    odom_data.sort(key=lambda x: x[0])
    
    tf_buffer = TFBuffer()
    tf_idx = 0
    odom_idx = 0
    
    errors = []
    est_traj = []
    gt_traj_synced = []
    
    # Store initial offset to align trajectories
    offset_x = 0
    offset_y = 0
    first_aligned = False
    
    for t_ns, x_gt, y_gt in gt_poses:
        # Update TF buffer up to this time
        while tf_idx < len(tf_data) and tf_data[tf_idx][0] <= t_ns:
            tf_buffer.update(tf_data[tf_idx][0], tf_data[tf_idx][1])
            tf_idx += 1
        
        # Update Odom index to nearest time (or last known)
        while odom_idx < len(odom_data) and odom_data[odom_idx][0] <= t_ns:
            odom_idx += 1
        
        if odom_idx == 0:
            continue
            
        # Use the latest odom pose
        _, trans_ob, rot_ob = odom_data[odom_idx - 1]
        
        # Get map->odom from TF
        tf_mo = tf_buffer.lookup_transform_map_odom()
        if tf_mo is None:
            continue
            
        _, trans_mo, rot_mo = tf_mo
        
        # Compose T_map_base = T_map_odom * T_odom_base
        mat_mo = tf_transformations.quaternion_matrix(rot_mo)
        mat_mo[:3, 3] = trans_mo
        
        mat_ob = tf_transformations.quaternion_matrix(rot_ob)
        mat_ob[:3, 3] = trans_ob
        
        mat_mb = np.dot(mat_mo, mat_ob)
        pos = mat_mb[:3, 3]
            
        x_est_raw, y_est_raw = pos[0], pos[1]

        if not first_aligned:
            # First point alignment: align estimate to GT
            offset_x = x_gt - x_est_raw
            offset_y = y_gt - y_est_raw
            first_aligned = True
            print(f"[INFO] Trajectory alignment offset: x={offset_x:.3f}, y={offset_y:.3f}")

        x_est = x_est_raw + offset_x
        y_est = y_est_raw + offset_y

        error = np.sqrt((x_gt - x_est)**2 + (y_gt - y_est)**2)
        errors.append(error)
        est_traj.append((x_est, y_est))
        gt_traj_synced.append((x_gt, y_gt))
            
    if not errors:
        return None, [], [], []
        
    rmse = np.sqrt(np.mean(np.array(errors)**2))
    return rmse, errors, gt_traj_synced, est_traj

def run_benchmark(bag_path, plot_path=None):
    print(f"Reading bag: {bag_path}")
    gt_poses, tf_data, odom_data = read_bag_data(bag_path)
    
    if not gt_poses:
        print("[ERROR] No Ground Truth poses found!")
        return None
        
    rmse, errors, gt_traj, est_traj = calculate_ate(gt_poses, tf_data, odom_data)
    
    if rmse is None:
        print("[ERROR] Could not compute ATE (maybe missing TFs?)")
        return None
        
    print(f"ATE RMSE: {rmse:.4f} m")
    
    # Plot
    gt_x = [p[0] for p in gt_traj]
    gt_y = [p[1] for p in gt_traj]
    est_x = [p[0] for p in est_traj]
    est_y = [p[1] for p in est_traj]
    
    plt.figure(figsize=(10,6))
    plt.plot(gt_x, gt_y, 'g-', label='Ground Truth')
    plt.plot(est_x, est_y, 'b--', label='SLAM Estimate')
    plt.title(f"Trajectory Comparison (ATE RMSE: {rmse:.3f}m)")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    if plot_path is None:
        plot_path = Path(bag_path) / "ate_plot.png" if Path(bag_path).is_dir() else Path(bag_path).parent / "ate_plot.png"
    
    plt.savefig(str(plot_path))
    plt.close()
    print(f"Plot saved to {plot_path}")
    return rmse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', help="Path to rosbag folder or file")
    args = parser.parse_args()
    
    run_benchmark(args.bag_path)

if __name__ == '__main__':
    main()
