
import numpy as np
import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def quaternion_to_matrix(quaternion):
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

def get_sim_time(msg_header):
    return msg_header.stamp.sec + msg_header.stamp.nanosec * 1e-9

def read_bag_data_synced(bag_path):
    print(f"Reading {bag_path}...")
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    gt_poses = []     # (sim_time, x, y)
    tf_data = []      # (sim_time, transform)
    odom_data = []    # (sim_time, pos, rot)
    
    # Clock synchronization (bag_time -> sim_time)
    # We'll use a simple "latest clock" approach or interpolation
    # Since model_states comes from gazebo, it corresponds to the CURRENT simulation time.
    # The safest bet: map the ModelStates bag_time to the specific clock value associated with it? 
    # Usually /clock is published, then /model_states using that clock. Or vice-versa.
    # We will assume piecewise linear mapping or just use nearest clock.
    
    # First pass: read all clocks? No, too slow.
    # Single pass: maintain current_sim_time from /clock
    
    current_sim_time = None
    
    # Store GT with potential bag_time for debug, but we need sim coordinate
    # Problem: if /clock comes AFTER /model_states in the bag for the same tick?
    # We can perform a small buffer reordering or just use the "nearest" clock found so far.
    
    gt_temp = [] # (bag_time, x, y)
    clock_map = [] # (bag_time, sim_time)
    
    count = 0
    while reader.has_next():
        (topic, data, t_ns_bag) = reader.read_next()
        t_bag = t_ns_bag * 1e-9
        
        if topic == '/clock':
            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9
                current_sim_time = sim_t
                clock_map.append((t_bag, sim_t))
            except:
                pass

        elif topic == '/gazebo/model_states':
            # Assign CURRENT Sim Time to this ground truth
            # If we haven't seen a clock yet, skip
            if current_sim_time is not None:
                try:
                    msg_type = get_message(type_map[topic])
                    msg = deserialize_message(data, msg_type)
                    idx = -1
                    for i, name in enumerate(msg.name):
                        if 'turtlebot3' in name:
                            idx = i
                            break
                    if idx != -1:
                        pose = msg.pose[idx]
                        # We use the current_sim_time from the last /clock message
                        # This assumes /clock is published before /model_states for a given tick
                        gt_poses.append((current_sim_time, pose.position.x, pose.position.y))
                except:
                    pass

        elif topic == '/tf' or topic == '/tf_static':
            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                for transform in msg.transforms:
                    t_header = get_sim_time(transform.header)
                    transform.header.frame_id = transform.header.frame_id.strip().lstrip('/')
                    transform.child_frame_id = transform.child_frame_id.strip().lstrip('/')
                    tf_data.append((t_header, transform))
            except:
                pass
        
        elif topic == '/odom':
            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                t_header = get_sim_time(msg.header)
                msg.header.frame_id = msg.header.frame_id.strip().lstrip('/')
                msg.child_frame_id = msg.child_frame_id.strip().lstrip('/')
                trans = msg.pose.pose.position
                rot = msg.pose.pose.orientation
                odom_data.append((t_header, np.array([trans.x, trans.y, trans.z]), np.array([rot.x, rot.y, rot.z, rot.w])))
            except:
                pass
        
        count += 1
        
    return gt_poses, tf_data, odom_data

class TFBuffer:
    def __init__(self):
        self.transforms = {}

    def update(self, t_sim, transform):
        parent = transform.header.frame_id
        child = transform.child_frame_id
        trans = transform.transform.translation
        rot = transform.transform.rotation
        self.transforms[(parent, child)] = (
            t_sim,
            np.array([trans.x, trans.y, trans.z]),
            np.array([rot.x, rot.y, rot.z, rot.w])
        )

    def lookup_transform_map_odom(self):
        if ('map', 'odom') in self.transforms:
            return self.transforms[('map', 'odom')]
        return None

def align_trajectories(gt_traj, est_traj):
    gt = np.array(gt_traj).T
    est = np.array(est_traj).T
    n = gt.shape[1]
    if n < 3: return 0.0
    
    gt_mean = np.mean(gt, axis=1).reshape(2,1)
    est_mean = np.mean(est, axis=1).reshape(2,1)
    gt_c = gt - gt_mean
    est_c = est - est_mean
    
    H = est_c @ gt_c.T
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = Vt.T @ U.T
        
    t = gt_mean - R @ est_mean
    est_aligned = R @ est + t
    error = gt - est_aligned
    rmse = np.sqrt(np.mean(np.sum(error**2, axis=0)))
    return rmse

def compute_metrics(gt_poses, tf_data, odom_data):
    # Sort by SIM TIME
    tf_data.sort(key=lambda x: x[0])
    odom_data.sort(key=lambda x: x[0])
    gt_poses.sort(key=lambda x: x[0])
    
    if not gt_poses:
        print("No GT poses found.")
        return

    print(f"Time Range (Sim): {gt_poses[0][0]:.2f} to {gt_poses[-1][0]:.2f}s")
    
    tf_buffer = TFBuffer()
    tf_idx = 0
    odom_idx = 0
    
    est_traj = []
    gt_traj_synced = []
    
    est_traj_raw = []

    for t_sim, x_gt, y_gt in gt_poses:
        # Update TF buffer to current sim time
        while tf_idx < len(tf_data) and tf_data[tf_idx][0] <= t_sim:
            tf_buffer.update(tf_data[tf_idx][0], tf_data[tf_idx][1])
            tf_idx += 1
        
        # Update Odom to current sim time
        while odom_idx < len(odom_data) and odom_data[odom_idx][0] <= t_sim:
            odom_idx += 1
        
        if odom_idx == 0: continue
        
        # Get interpolated odom? For now just nearest/latest
        _, trans_ob, rot_ob = odom_data[odom_idx - 1]
        
        tf_mo = tf_buffer.lookup_transform_map_odom()
        if tf_mo is None: continue
            
        _, trans_mo, rot_mo = tf_mo
        
        # T_map_base = T_map_odom * T_odom_base
        mat_mo = quaternion_to_matrix(rot_mo)
        mat_mo[:3, 3] = trans_mo
        
        mat_ob = quaternion_to_matrix(rot_ob)
        mat_ob[:3, 3] = trans_ob
        
        mat_mb = np.dot(mat_mo, mat_ob)
        pos = mat_mb[:3, 3]
        
        est_traj_raw.append((pos[0], pos[1]))
        gt_traj_synced.append((x_gt, y_gt))

    if not est_traj_raw:
        print("No synced trajectory found.")
        return

    # Metrics
    # 1. Naive Translation only (current approach in dashboard)
    offset_x = gt_traj_synced[0][0] - est_traj_raw[0][0]
    offset_y = gt_traj_synced[0][1] - est_traj_raw[0][1]
    errors_naive = []
    for i in range(len(est_traj_raw)):
        ex = est_traj_raw[i][0] + offset_x
        ey = est_traj_raw[i][1] + offset_y
        err = np.sqrt((gt_traj_synced[i][0] - ex)**2 + (gt_traj_synced[i][1] - ey)**2)
        errors_naive.append(err)
    rmse_naive = np.sqrt(np.mean(np.array(errors_naive)**2))
    
    # 2. Optimal Rigid
    rmse_optimal = align_trajectories(gt_traj_synced, est_traj_raw)
    
    print(f"  [SimTime Sync] ATE (Trans only): {rmse_naive:.4f} m")
    print(f"  [SimTime Sync] ATE (Rot+Trans) : {rmse_optimal:.4f} m")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bags', nargs='+')
    args = parser.parse_args()
    
    for bag in args.bags:
        gt, tf, odom = read_bag_data_synced(bag)
        compute_metrics(gt, tf, odom)

if __name__ == '__main__':
    main()
