import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os

def show_map(yaml_path):
    if not os.path.exists(yaml_path):
        print(f"Error: {yaml_path} not found.")
        return

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    image_path = data['image']
    # If image path is relative to yaml
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    resolution = data['resolution']
    origin = data['origin'] # [x, y, yaw]

    # Load image
    img = mpimg.imread(image_path)
    
    # Calculate extent for matplotlib [left, right, bottom, top]
    # In ROS, image (0,0) is bottom-left of map if we use origin correctly.
    # However, PGM images are usually top-down. 
    # My generator uses np.flipud(grid) so the first row of PGM is the highest Y.
    
    height, width = img.shape[:2]
    
    left = origin[0]
    right = origin[0] + width * resolution
    bottom = origin[1]
    top = origin[1] + height * resolution
    
    extent = [left, right, bottom, top]

    plt.figure(figsize=(10, 8))
    plt.imshow(img, cmap='gray', extent=extent, origin='lower')
    plt.colorbar(label='Grayscale Value')
    plt.title(f"Map: {image_path}\nResolution: {resolution}m/px, Origin: {origin[:2]}")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    
    print("Opening map viewer... (Close the window to continue)")
    plt.show()

if __name__ == "__main__":
    import sys
    yaml_file = 'map_gt.yaml'
    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    
    show_map(yaml_file)
