
import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def check_model_names_deep(bag_path):
    print(f"Deep check in {bag_path}...")
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    count = 0
    names_seen = set()
    
    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        if topic == '/gazebo/model_states':
            count += 1
            # Check every 100 messages to cover time duration
            if count % 100 == 0:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                
                current_names = set(msg.name)
                # If we see a NEW name we haven't seen before, print it!
                new_names = current_names - names_seen
                if new_names:
                    print(f"[@ Msg {count}] Found NEW models: {new_names}")
                    names_seen.update(new_names)
                    
                    # Print details of the new models
                    for i, name in enumerate(msg.name):
                        if name in new_names:
                            pos = msg.pose[i].position
                            print(f"  -> {name} Pos: ({pos.x:.2f}, {pos.y:.2f})")
    
    if not names_seen:
        print("No models found at all??")
    else:
        print(f"Final list of all models seen over {count} messages: {names_seen}")

if __name__ == "__main__":
    import sys
    check_model_names_deep(sys.argv[1])
