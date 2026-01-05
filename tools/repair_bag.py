#!/usr/bin/env python3
import sys
import yaml
from pathlib import Path
import sqlite3

def repair_bag(bag_dir):
    bag_dir = Path(bag_dir)
    metadata_path = bag_dir / "metadata.yaml"
    
    if metadata_path.exists():
        print("Metadata exists, no repair needed.")
        return

    print("Metadata missing! Attempting to reconstruct...")
    
    db_file = list(bag_dir.glob("*.db3"))[0]
    if not db_file:
        print("No .db3 file found!")
        return
        
    # Connect to DB to get info
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()
    
    # Get total count
    cursor.execute("SELECT count(*) FROM messages")
    count = cursor.fetchone()[0]
    
    # Get duration
    cursor.execute("SELECT min(timestamp), max(timestamp) FROM messages")
    t_min, t_max = cursor.fetchone()
    duration_ns = t_max - t_min
    
    # Get topics
    cursor.execute("SELECT name, type, serialization_format FROM topics")
    topics = cursor.fetchall()
    
    conn.close()
    
    meta = {
        "rosbag2_bagfile_information": {
            "version": 4,
            "storage_identifier": "sqlite3",
            "duration": {
                "nanoseconds": duration_ns
            },
            "starting_time": {
                "nanoseconds_since_epoch": t_min
            },
            "message_count": count,
            "topics_with_message_count": [],
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [db_file.name],
            "files": [
                {
                    "path": db_file.name,
                    "starting_time": {"nanoseconds_since_epoch": t_min},
                    "duration": {"nanoseconds": duration_ns},
                    "message_count": count
                }
            ]
        }
    }
    
    for name, type_name, fmt in topics:
        meta["rosbag2_bagfile_information"]["topics_with_message_count"].append({
            "topic_metadata": {
                "name": name,
                "type": type_name,
                "serialization_format": fmt,
                "offered_qos_profiles": "" 
            },
            "message_count": 0 # Unknown
        })

    with open(metadata_path, 'w') as f:
        yaml.dump(meta, f)
        
    print(f"Repaired! Created {metadata_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: ./repair_bag.py <bag_dir>")
        sys.exit(1)
    repair_bag(sys.argv[1])
