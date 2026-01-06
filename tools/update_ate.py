
import argparse
import json
import numpy as np
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Patch for numpy
if not hasattr(np, 'float'): np.float = np.float64
if not hasattr(np, 'int'): np.int = np.int_

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

class TFBuffer:
    def __init__(self):
        self.transforms = {}
    def update(self, t_ns, transform):
        parent = transform.header.frame_id.strip().lstrip('/')
        child = transform.child_frame_id.strip().lstrip('/')
        trans = transform.transform.translation
        rot = transform.transform.rotation
        self.transforms[(parent, child)] = (
            t_ns,
            np.array([trans.x, trans.y, trans.z]),
            np.array([rot.x, rot.y, rot.z, rot.w])
        )
    def lookup_transform_map_odom(self):
        return self.transforms.get(('map', 'odom'))

def calculate_ate_robust(bag_path):
    bag_path = str(bag_path)
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    gt_poses = []
    tf_data = []
    odom_data = []
    
    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        if topic not in type_map: continue
        
        try:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
        except: continue

        if topic == '/gazebo/model_states':
            idx = -1
            for i, name in enumerate(msg.name):
                # FIXED LOGIC HERE
                if ('turtlebot3' in name or 'waffle' in name or 'burger' in name):
                    if 'house' in name or 'world' in name or 'ground' in name: continue
                    idx = i
                    break
            if idx != -1:
                pose = msg.pose[idx]
                gt_poses.append((t_ns, pose.position.x, pose.position.y))

        elif topic in ['/tf', '/tf_static']:
            for t in msg.transforms:
                tf_data.append((t_ns, t))
        
        elif topic == '/odom':
            trans = msg.pose.pose.position
            rot = msg.pose.pose.orientation
            odom_data.append((t_ns, np.array([trans.x, trans.y, trans.z]), np.array([rot.x, rot.y, rot.z, rot.w])))

    # Compute ATE
    tf_data.sort(key=lambda x: x[0])
    odom_data.sort(key=lambda x: x[0])
    
    tf_buffer = TFBuffer()
    tf_idx = 0; odom_idx = 0
    errors = []
    
    # Simple alignment on first point
    offset_x = 0; offset_y = 0; first = False
    
    for t_ns, x_gt, y_gt in gt_poses:
        while tf_idx < len(tf_data) and tf_data[tf_idx][0] <= t_ns:
            tf_buffer.update(tf_data[tf_idx][0], tf_data[tf_idx][1])
            tf_idx += 1
        while odom_idx < len(odom_data) and odom_data[odom_idx][0] <= t_ns:
            odom_idx += 1
            
        if odom_idx == 0: continue
        _, trans_ob, rot_ob = odom_data[odom_idx-1]
        
        tf_mo = tf_buffer.lookup_transform_map_odom()
        if tf_mo is None: continue
        _, trans_mo, rot_mo = tf_mo
        
        mat_mo = quaternion_matrix(rot_mo)
        mat_mo[:3,3] = trans_mo
        mat_ob = quaternion_matrix(rot_ob)
        mat_ob[:3,3] = trans_ob
        mat_mb = np.dot(mat_mo, mat_ob)
        
        x_est, y_est = mat_mb[0,3], mat_mb[1,3]
        
        if not first:
            offset_x = x_gt - x_est
            offset_y = y_gt - y_est
            first = True
            
        err = np.sqrt((x_gt - (x_est+offset_x))**2 + (y_gt - (y_est+offset_y))**2)
        errors.append(err)
        
    if not errors: return None
    return np.sqrt(np.mean(np.array(errors)**2))

def update_run(run_dir):
    run_path = Path(run_dir)
    bag_path = run_path / "bags" / "output"
    json_path = run_path / "metrics.json"
    
    if not bag_path.exists() or not json_path.exists():
        print(f"Skipping {run_dir} (missing bag or json)")
        return

    print(f"Processing {run_path.name}...")
    rmse = calculate_ate_robust(bag_path)
    
    if rmse is not None:
        print(f"  -> New ATE: {rmse:.4f} m")
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        data['ate_rmse'] = rmse
        # Also clean up failure flag if it was false positive due to ATE
        if data.get('is_failure') and "Massive drift" in str(data.get('failure_reasons', [])):
             # Only clear if ATE is now good
             if rmse < 0.5:
                print("  -> Clearing false positive failure flag")
                data['is_failure'] = False
                data['failure_reasons'] = [r for r in data.get('failure_reasons', []) if "Massive drift" not in r]

        with open(json_path, 'w') as f:
            json.dump(data, f, indent=4)
        print("  -> Updated metrics.json")
        
        # Also trigger plot regeneration
        import subprocess, sys
        cmd = [sys.executable, "tools/benchmark.py", str(bag_path)]
        subprocess.run(cmd, check=True)
    else:
        print("  -> Could not calculate ATE")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+')
    args = parser.parse_args()
    for r in args.runs:
        update_run(r)
