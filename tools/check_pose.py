import sqlite3
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from gazebo_msgs.msg import ModelStates

def get_first_model_states(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        if topic == '/gazebo/model_states':
            msg = deserialize_message(data, ModelStates)
            print(f"Models at t={t_ns}:")
            for i, name in enumerate(msg.name):
                print(f"  - {name}: position={msg.pose[i].position}")
            return
    print("No robot found in ModelStates")

if __name__ == "__main__":
    import sys
    get_first_model_states(sys.argv[1])
