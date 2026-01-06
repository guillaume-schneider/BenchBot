
import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def check_timestamps(bag_path):
    print(f"Checking timestamps in {bag_path}")
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    first_stamps = {}
    
    count = 0
    while reader.has_next() and count < 1000:
        (topic, data, t_ns_bag) = reader.read_next()
        
        if topic not in first_stamps:
            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                
                # Try to get header stamp
                if hasattr(msg, 'header'):
                    t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    first_stamps[topic] = (t_ns_bag * 1e-9, t_msg)
                elif topic == '/tf':
                    # TF message has transforms list
                    if msg.transforms:
                        t_msg = msg.transforms[0].header.stamp.sec + msg.transforms[0].header.stamp.nanosec * 1e-9
                        first_stamps[topic] = (t_ns_bag * 1e-9, t_msg)
            except:
                pass
        
        if len(first_stamps) > 5:
            break
        count += 1
        
    print(f"{'Topic':<30} | {'Bag Time (s)':<15} | {'Header Time (s)':<15} | {'Delta (s)':<10}")
    print("-" * 80)
    for topic, (t_bag, t_header) in first_stamps.items():
        delta = t_bag - t_header
        print(f"{topic:<30} | {t_bag:<15.3f} | {t_header:<15.3f} | {delta:<10.3f}")

if __name__ == "__main__":
    import sys
    check_timestamps(sys.argv[1])
