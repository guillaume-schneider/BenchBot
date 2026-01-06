
import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def check_model_names(bag_path):
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        if topic == '/gazebo/model_states':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            print("Found models in /gazebo/model_states:")
            for i, name in enumerate(msg.name):
                # Also print position to identify which one moves
                pos = msg.pose[i].position
                print(f"  [{i}] {name} (Pos: x={pos.x:.2f}, y={pos.y:.2f})")
            return

if __name__ == "__main__":
    import sys
    check_model_names(sys.argv[1])
