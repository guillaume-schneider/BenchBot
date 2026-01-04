import xml.etree.ElementTree as ET
import numpy as np
import math
import yaml
import os

def parse_pose(pose_str):
    if not pose_str:
        return np.zeros(6)
    return np.fromstring(pose_str.strip(), sep=' ')

def get_transform_matrix(pose):
    x, y, z, roll, pitch, yaw = pose
    
    # Rotation matrix (standard Rz * Ry * Rx)
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    
    R = np.eye(4)
    R[0, 0] = cy*cp
    R[0, 1] = cy*sp*sr - sy*cr
    R[0, 2] = cy*sp*cr + sy*sr
    R[1, 0] = sy*cp
    R[1, 1] = sy*sp*sr + cy*cr
    R[1, 2] = sy*sp*cr - cy*sr
    R[2, 0] = -sp
    R[2, 1] = cp*sr
    R[2, 2] = cp*cr
    
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    
    # In SDF, pose = translation then rotation relative to parent
    # The translation is in the parent frame.
    # So M = T_translation * R_rotation
    return T @ R

def process_model(model_elem, current_transform, obstacles):
    # Process links
    for link in model_elem.findall('link'):
        link_pose_raw = link.find('pose')
        link_pose = parse_pose(link_pose_raw.text) if link_pose_raw is not None else np.zeros(6)
        link_transform = current_transform @ get_transform_matrix(link_pose)
        
        for collision in link.findall('collision'):
            coll_pose_raw = collision.find('pose')
            coll_pose = parse_pose(coll_pose_raw.text) if coll_pose_raw is not None else np.zeros(6)
            coll_transform = link_transform @ get_transform_matrix(coll_pose)
            
            # Extract world position and yaw
            # x = coll_transform[0, 3]
            # y = coll_transform[1, 3]
            # z = coll_transform[2, 3]
            # yaw = math.atan2(coll_transform[1, 0], coll_transform[0, 0])
             
            geometry = collision.find('geometry')
            box = geometry.find('box')
            cylinder = geometry.find('cylinder')
            mesh = geometry.find('mesh')
            
            if box is not None:
                size = np.fromstring(box.find('size').text, sep=' ')
                obstacles.append({
                    'type': 'box',
                    'transform': coll_transform,
                    'size': size
                })
            elif cylinder is not None:
                radius = float(cylinder.find('radius').text)
                length = float(cylinder.find('length').text)
                obstacles.append({
                    'type': 'cylinder',
                    'transform': coll_transform,
                    'r': radius,
                    'l': length
                })
            elif mesh is not None:
                # Approximate mesh with a box or circle based on model name
                # or just use a default bounding box if we can't parse it
                # For trash can: radius ~0.2, height 0.7
                # For table: we'll use a bounding box
                model_name = model_elem.get('name', '')
                if 'trash_can' in model_name:
                    obstacles.append({
                        'type': 'cylinder',
                        'transform': coll_transform @ get_transform_matrix([0,0,0.35,0,0,0]),
                        'r': 0.22,
                        'l': 0.7
                    })
                elif 'table_marble' in model_name:
                    obstacles.append({
                        'type': 'box',
                        'transform': coll_transform,
                        'size': [1.5, 0.8, 1.0] # Approximate
                    })

    # Process nested models
    for nested_model in model_elem.findall('model'):
        model_pose_raw = nested_model.find('pose')
        model_pose = parse_pose(model_pose_raw.text) if model_pose_raw is not None else np.zeros(6)
        process_model(nested_model, current_transform @ get_transform_matrix(model_pose), obstacles)

def generate_png(pgm_path):
    try:
        from PIL import Image
        img = Image.open(pgm_path)
        png_path = pgm_path.replace('.pgm', '.png')
        img.save(png_path)
        return True, png_path
    except Exception as e:
        return False, str(e)

def generate_debug_plot(obstacles, laser_z, output_name):
    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(12, 12))
        for obs in obstacles:
            if obs['type'] == 'box':
                w, h = obs['w'], obs['h']
                c, s = math.cos(obs['yaw']), math.sin(obs['yaw'])
                corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2), (-w/2, -h/2)]
                tx = [obs['x'] + x*c - y*s for x, y in corners]
                ty = [obs['y'] + x*s + y*c for x, y in corners]
                plt.plot(tx, ty, 'b-')
                if 'Wall' not in obs['name']:
                    plt.text(obs['x'], obs['y'], obs['name'].split('/')[-1], fontsize=8, ha='center')
            else:
                circle = plt.Circle((obs['x'], obs['y']), obs['r'], color='r', fill=False)
                plt.gca().add_artist(circle)
                plt.text(obs['x'], obs['y'], obs['name'].split('/')[-1], fontsize=8, ha='center')

        plt.axis('equal')
        plt.grid(True)
        plt.title(f"SDF Geometry (Laser height = {laser_z}m)")
        plt.xlabel("World X (m)")
        plt.ylabel("World Y (m)")
        debug_path = f"{output_name}_debug.png"
        plt.savefig(debug_path)
        plt.close()
        return True, debug_path
    except Exception as e:
        return False, str(e)

def process_model_detailed(model_elem, current_transform, laser_z, depth=0):
    if depth > 50:
        print("DEBUG: Max recursion depth reached in process_model_detailed")
        return []
        
    obstacles = []
    if model_elem is None:
        print("DEBUG: process_model_detailed called with None model_elem")
        return []

    model_name = model_elem.get('name', 'unknown')
    # print(f"DEBUG: Processing model {model_name} at depth {depth}")

    for link in model_elem.findall('link'):
        link_name = link.get('name')
        link_pose_raw = link.find('pose')
        link_pose = parse_pose(link_pose_raw.text) if link_pose_raw is not None else np.zeros(6)
        link_transform = current_transform @ get_transform_matrix(link_pose)
        
        for collision in link.findall('collision'):
            coll_name = collision.get('name')
            coll_pose_raw = collision.find('pose')
            coll_pose = parse_pose(coll_pose_raw.text) if coll_pose_raw is not None else np.zeros(6)
            coll_transform = link_transform @ get_transform_matrix(coll_pose)
            
            geom = collision.find('geometry')
            if geom is None: continue
            
            box = geom.find('box')
            cylinder = geom.find('cylinder')
            
            pos = coll_transform[:3, 3]
            yaw = math.atan2(coll_transform[1, 0], coll_transform[0, 0])
            
            if box is not None:
                size_elem = box.find('size')
                if size_elem is not None:
                    size = np.fromstring(size_elem.text, sep=' ')
                    h_half = size[2] / 2
                    if pos[2] - h_half <= laser_z <= pos[2] + h_half:
                        obstacles.append({
                            'name': f"{link_name}/{coll_name}",
                            'type': 'box', 'x': pos[0], 'y': pos[1], 'yaw': yaw,
                            'w': size[0], 'h': size[1]
                        })
            elif cylinder is not None:
                rad_elem = cylinder.find('radius')
                len_elem = cylinder.find('length')
                if rad_elem is not None and len_elem is not None:
                    radius = float(rad_elem.text)
                    length = float(len_elem.text)
                    h_half = length / 2
                    if pos[2] - h_half <= laser_z <= pos[2] + h_half:
                        obstacles.append({
                            'name': f"{link_name}/{coll_name}",
                            'type': 'cylinder', 'x': pos[0], 'y': pos[1], 'r': radius
                        })
    
    for nested in model_elem.findall('model'):
        model_pose_raw = nested.find('pose')
        model_pose = parse_pose(model_pose_raw.text) if model_pose_raw is not None else np.zeros(6)
        obstacles.extend(process_model_detailed(nested, current_transform @ get_transform_matrix(model_pose), laser_z, depth+1))
    return obstacles

def generate_map(sdf_path='world/model.sdf', resolution=0.05, laser_z=0.17, padding=1.0, output_name='map_gt', gen_png=True, gen_debug=True):
    print(f"DEBUG: generate_map called for {sdf_path}")
    if not os.path.exists(sdf_path):
        return False, f"Error: {sdf_path} not found."

    try:
        print("DEBUG: Parsing XML...")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        print("DEBUG: XML Parsed. Finding top model...")
        
        # Determine if root is 'sdf' or 'model'
        if root.tag == 'sdf':
             top_model = root.find('model')
             if root.find('world'):
                 # It's a world file, look for models in world?
                 # For now, let's assume one main model or panic.
                 print(f"DEBUG: Root is SDF. First model found: {top_model is not None}")
        elif root.tag == 'model':
             top_model = root
        else:
             top_model = None

        if top_model is None:
            return False, f"Could not find <model> in {sdf_path}"

        print(f"DEBUG: Processing top model: {top_model.get('name')}")
        projected_obstacles = process_model_detailed(top_model, np.eye(4), laser_z)

        if not projected_obstacles:
            return False, "No obstacles found at the specified laser height."

        # Calculate bounds
        all_x = []
        all_y = []
        for obs in projected_obstacles:
            if obs['type'] == 'box':
                d = math.sqrt(obs['w']**2 + obs['h']**2) / 2
                all_x.extend([obs['x'] - d, obs['x'] + d])
                all_y.extend([obs['y'] - d, obs['y'] + d])
            elif obs['type'] == 'cylinder':
                all_x.extend([obs['x'] - obs['r'], obs['x'] + obs['r']])
                all_y.extend([obs['y'] - obs['r'], obs['y'] + obs['r']])
                
        min_x = min(all_x) - padding
        max_x = max(all_x) + padding
        min_y = min(all_y) - padding
        max_y = max(all_y) + padding
        
        width = int((max_x - min_x) / resolution)
        height = int((max_y - min_y) / resolution)
        
        grid = np.full((height, width), 255, dtype=np.uint8)
        
        for obs in projected_obstacles:
            if obs['type'] == 'box':
                hw, hh = obs['w']/2, obs['h']/2
                yaw = obs['yaw']
                cos_y = math.cos(-yaw)
                sin_y = math.sin(-yaw)
                d = math.sqrt(hw**2 + hh**2)
                min_px = int((obs['x'] - d - min_x) / resolution)
                max_px = int((obs['x'] + d - min_x) / resolution)
                min_py = int((obs['y'] - d - min_y) / resolution)
                max_py = int((obs['y'] + d - min_y) / resolution)
                for y_idx in range(max(0, min_py), min(height, max_py + 1)):
                    for x_idx in range(max(0, min_px), min(width, max_px + 1)):
                        lx = (x_idx * resolution + min_x) - obs['x']
                        ly = (y_idx * resolution + min_y) - obs['y']
                        ux = lx * cos_y - ly * sin_y
                        uy = lx * sin_y + ly * cos_y
                        if -hw <= ux <= hw and -hh <= uy <= hh:
                            grid[y_idx, x_idx] = 0
            elif obs['type'] == 'cylinder':
                cx = int((obs['x'] - min_x) / resolution)
                cy = int((obs['y'] - min_y) / resolution)
                r_px = int(obs['r'] / resolution)
                for y_idx in range(max(0, cy - r_px - 1), min(height, cy + r_px + 2)):
                    for x_idx in range(max(0, cx - r_px - 1), min(width, cx + r_px + 2)):
                        dx = (x_idx * resolution + min_x) - obs['x']
                        dy = (y_idx * resolution + min_y) - obs['y']
                        if dx*dx + dy*dy <= obs['r']**2:
                            grid[y_idx, x_idx] = 0

        pgm_path = f"{output_name}.pgm"
        yaml_path = f"{output_name}.yaml"

        # Ensure parent directory exists
        os.makedirs(os.path.dirname(pgm_path) if os.path.dirname(pgm_path) else '.', exist_ok=True)

        with open(pgm_path, 'wb') as f:
            f.write(f"P5\n{width} {height}\n255\n".encode())
            f.write(np.flipud(grid).tobytes())

        yaml_data = {
            'image': os.path.basename(pgm_path),  # Use basename for relative path
            'resolution': float(resolution),
            'origin': [float(min_x), float(min_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

        messages = [f"Success: {pgm_path} and {yaml_path} generated."]
        
        if gen_png:
            p_success, p_path = generate_png(pgm_path)
            if p_success:
                messages.append(f"PNG image generated: {p_path}")
            else:
                messages.append(f"Warning: PNG generation failed: {p_path}")

        if gen_debug:
            d_success, d_path = generate_debug_plot(projected_obstacles, laser_z, output_name)
            if d_success:
                messages.append(f"Debug plot generated: {d_path}")
            else:
                messages.append(f"Warning: Debug plot failed: {d_path}")

        return True, "\n".join(messages)
    except Exception as e:
        import traceback
        return False, f"Error: {str(e)}\n{traceback.format_exc()}"

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Generate 2D map from Gazebo SDF.')
    parser.add_argument('--sdf', default='world/model.sdf', help='Path to SDF file')
    parser.add_argument('--res', type=float, default=0.05, help='Resolution (m/px)')
    parser.add_argument('--z', type=float, default=0.17, help='Laser height (m)')
    parser.add_argument('--padding', type=float, default=1.0, help='Map padding (m)')
    parser.add_argument('--out', default='generated_maps/map_gt', help='Output base name')
    parser.add_argument('--no-png', action='store_true', help='Disable PNG generation')
    parser.add_argument('--no-debug', action='store_true', help='Disable debug plot generation')
    
    args = parser.parse_args()
    success, msg = generate_map(args.sdf, args.res, args.z, args.padding, args.out, not args.no_png, not args.no_debug)
    print(msg)

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()
