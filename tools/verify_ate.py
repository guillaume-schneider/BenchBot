
import numpy as np
import argparse
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def quaternion_matrix(quaternion):
    """
    Return homogeneous rotation matrix from quaternion.
    [x, y, z, w]
    """
    x, y, z, w = quaternion
    _2x = x + x; _2y = y + y; _2z = z + z
    _2w = w + w
    nx = _2x * x; ny = _2y * y; nz = _2z * z
    nw = _2w * w
    
    _2xy = _2x * y; _2xz = _2x * z; _2xw = _2x * w
    _2yz = _2y * z; _2yw = _2y * w; _2zw = _2z * w
    
    return np.array([
        [1.0 - (ny + nz), _2xy - _2zw,      _2xz + _2yw,      0.0],
        [_2xy + _2zw,      1.0 - (nx + nz), _2yz - _2xw,      0.0],
        [_2xz - _2yw,      _2yz + _2xw,      1.0 - (nx + ny), 0.0],
        [0.0,              0.0,              0.0,              1.0]
    ])

def read_bag_data(bag_path):
    bag_path = str(bag_path)
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    try:
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return [], [], []

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    gt_poses = []
    tf_data = []
    odom_data = []

    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        msg_type_str = type_map[topic]
        
        try:
            msg_type = get_message(msg_type_str)
            msg = deserialize_message(data, msg_type)
        except:
            continue

        if topic == '/gazebo/model_states':
            try:
                idx = -1
                for i, name in enumerate(msg.name):
                    if 'turtlebot3' in name:
                        idx = i
                        break
                if idx != -1:
                    pose = msg.pose[idx]
                    gt_poses.append((t_ns, pose.position.x, pose.position.y))
            except:
                pass

        elif topic == '/tf' or topic == '/tf_static':
            for transform in msg.transforms:
                transform.header.frame_id = transform.header.frame_id.strip().lstrip('/')
                transform.child_frame_id = transform.child_frame_id.strip().lstrip('/')
                tf_data.append((t_ns, transform))
        
        elif topic == '/odom':
            msg.header.frame_id = msg.header.frame_id.strip().lstrip('/')
            msg.child_frame_id = msg.child_frame_id.strip().lstrip('/')
            trans = msg.pose.pose.position
            rot = msg.pose.pose.orientation
            odom_data.append((t_ns, np.array([trans.x, trans.y, trans.z]), np.array([rot.x, rot.y, rot.z, rot.w])))
        
    return gt_poses, tf_data, odom_data

class TFBuffer:
    def __init__(self):
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

def align_trajectories(gt_traj, est_traj):
    """
    Align est_traj to gt_traj using Umeyama (SVD) for rigid transformation (R, t).
    """
    gt = np.array(gt_traj).T
    est = np.array(est_traj).T
    
    n = gt.shape[1]
    if n < 3:
        return 0, 0, 0
    
    # Centre centroids
    gt_mean = np.mean(gt, axis=1).reshape(2,1)
    est_mean = np.mean(est, axis=1).reshape(2,1)
    
    gt_centered = gt - gt_mean
    est_centered = est - est_mean
    
    # Covariance
    H = est_centered @ gt_centered.T
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Determinent should be 1 for rotation (not reflection)
    if np.linalg.det(R) < 0:
        # print("Reflection detected, fixing...")
        Vt[1,:] *= -1
        R = Vt.T @ U.T
        
    t = gt_mean - R @ est_mean
    
    est_aligned = R @ est + t
    
    error = gt - est_aligned
    rmse = np.sqrt(np.mean(np.sum(error**2, axis=0)))
    
    return rmse

def compute_metrics(gt_poses, tf_data, odom_data):
    tf_data.sort(key=lambda x: x[0])
    odom_data.sort(key=lambda x: x[0])
    
    tf_buffer = TFBuffer()
    tf_idx = 0
    odom_idx = 0
    
    est_traj = []
    gt_traj_synced = []
    
    est_traj_raw = [] # Without any alignment

    for t_ns, x_gt, y_gt in gt_poses:
        while tf_idx < len(tf_data) and tf_data[tf_idx][0] <= t_ns:
            tf_buffer.update(tf_data[tf_idx][0], tf_data[tf_idx][1])
            tf_idx += 1
        
        while odom_idx < len(odom_data) and odom_data[odom_idx][0] <= t_ns:
            odom_idx += 1
        
        if odom_idx == 0: continue
        
        _, trans_ob, rot_ob = odom_data[odom_idx - 1]
        tf_mo = tf_buffer.lookup_transform_map_odom()
        if tf_mo is None: continue
            
        _, trans_mo, rot_mo = tf_mo
        
        mat_mo = quaternion_matrix(rot_mo)
        mat_mo[:3, 3] = trans_mo
        
        mat_ob = quaternion_matrix(rot_ob)
        mat_ob[:3, 3] = trans_ob
        
        mat_mb = np.dot(mat_mo, mat_ob)
        pos = mat_mb[:3, 3]
        
        est_traj_raw.append((pos[0], pos[1]))
        gt_traj_synced.append((x_gt, y_gt))

    if not est_traj_raw:
        print("No synced trajectory found.")
        return

    # 1. Current Metric (Translation alignment on first point only)
    offset_x = gt_traj_synced[0][0] - est_traj_raw[0][0]
    offset_y = gt_traj_synced[0][1] - est_traj_raw[0][1]
    
    errors_naive = []
    for i in range(len(est_traj_raw)):
        ex = est_traj_raw[i][0] + offset_x
        ey = est_traj_raw[i][1] + offset_y
        err = np.sqrt((gt_traj_synced[i][0] - ex)**2 + (gt_traj_synced[i][1] - ey)**2)
        errors_naive.append(err)
        
    rmse_naive = np.sqrt(np.mean(np.array(errors_naive)**2))
    
    # 2. Optimal Rigid Alignment (Rotation + Translation)
    rmse_optimal = align_trajectories(gt_traj_synced, est_traj_raw)
    
    print(f"  Current ATE (Trans only): {rmse_naive:.4f} m")
    print(f"  Optimal ATE (Rot+Trans) : {rmse_optimal:.4f} m")
    if rmse_naive > 0.5 and rmse_optimal < 0.2:
        print("  -> DIAGNOSIS: Alignment Issue (Rotation missing)")
    elif rmse_naive > 0.5 and rmse_optimal > 0.5:
         print("  -> DIAGNOSIS: Real SLAM Drift / Distortion")
    else:
        print("  -> DIAGNOSIS: Good")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bags', nargs='+')
    args = parser.parse_args()
    
    for bag in args.bags:
        print(f"\nAnalyzing: {bag}")
        gt, tf, odom = read_bag_data(bag)
        compute_metrics(gt, tf, odom)

if __name__ == '__main__':
    main()
