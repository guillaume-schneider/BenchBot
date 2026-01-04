#!/usr/bin/env python3

import argparse
import yaml
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from pathlib import Path

def view_map(yaml_path, output_file=None):
    yaml_path = Path(yaml_path).resolve()
    if not yaml_path.exists():
        print(f"[ERROR] YAML file not found: {yaml_path}")
        return

    # Load Metadata
    with open(yaml_path, 'r') as f:
        metadata = yaml.safe_load(f)

    # Resolve image path
    image_filename = metadata.get('image', '')
    image_path = yaml_path.parent / image_filename

    if not image_path.exists():
        print(f"[ERROR] Map image file not found: {image_path}")
        return
    
    # Load Image
    try:
        img = Image.open(image_path)
        img_data = np.array(img)
    except Exception as e:
        print(f"[ERROR] Failed to load image: {e}")
        return

    # Parse metadata
    resolution = metadata.get('resolution', 0.05)
    origin = metadata.get('origin', [0.0, 0.0, 0.0])
    origin_x, origin_y = origin[0], origin[1]
    
    height, width = img_data.shape
    
    # Calculate real-world dimensions
    real_width = width * resolution
    real_height = height * resolution
    
    # Extent for matplotlib [left, right, bottom, top]
    # NOTE: ROS maps usually have origin at bottom-left
    # origin_x, origin_y is the position of the bottom-left pixel in the map frame
    
    extent = [
        origin_x,
        origin_x + real_width,
        origin_y,
        origin_y + real_height
    ]

    print(f"[INFO] Map: {yaml_path.name}")
    print(f"[INFO] Resolution: {resolution} m/px")
    print(f"[INFO] Size: {width}x{height} px ({real_width:.2f}x{real_height:.2f} m)")
    print(f"[INFO] Origin: ({origin_x}, {origin_y})")

    # Plot
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # ROS maps: 0 (black) = occupied, 255 (white) = free, 205 (gray) = unknown
    # We use 'gray' colormap: 0=Black, 255=White
    # But usually verification is easier if we invert: Obstacles=Black, Free=White
    # Standard PGM is typically: 0=Black(Occupied), 254=White(Free), 205=Unknown
    # Let's just display as is with 'gray' colormap which maps 0->Black, 255->White
    
    # If the map is upside down (common in image coords vs map coords), we might need origin='lower'
    # ROS map server usually treats the image bottom-left as origin if mode is not specified/default.
    
    ax.imshow(img_data, cmap='gray', extent=extent, origin='lower')
    
    ax.set_title(f"GT Map: {yaml_path.stem}")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, which='both', linestyle='--', alpha=0.9, color='cyan')
    
    # Identify center (0,0) with a red cross
    ax.plot(0, 0, 'r+', markersize=15, markeredgewidth=2, label='Origin (0,0)')
    ax.legend()

    if output_file:
        plt.savefig(output_file)
        print(f"[SUCCESS] Saved visualization to {output_file}")
    else:
        # Save to same directory with _preview.png
        out_path = yaml_path.with_name(yaml_path.stem + "_preview.png")
        plt.savefig(out_path)
        print(f"[SUCCESS] Saved visualization to {out_path}")

def main():
    parser = argparse.ArgumentParser(description="Visualize ROS 2 Map from YAML")
    parser.add_argument("yaml_file", help="Path to map.yaml")
    parser.add_argument("-o", "--output", help="Output PNG file")
    
    args = parser.parse_args()
    view_map(args.yaml_file, args.output)

if __name__ == "__main__":
    main()
