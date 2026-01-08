
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from gazebo_msgs.msg import ModelStates

bag_path = sys.argv[1]
storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader = SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topic_types}

found_models = set()
count = 0
while reader.has_next():
    (topic, data, t_ns) = reader.read_next()
    if topic == '/gazebo/model_states':
        msg = deserialize_message(data, ModelStates)
        current_models = set(msg.name)
        if hasattr(msg, 'pose'):
            # Check positions of turtlebot3_house (assuming idx 1 based on previous list)
            pass 
            
        if current_models != found_models:
            print(f"Time {t_ns}: Models changed to: {msg.name}")
            found_models = current_models
        count += 1
        if count > 5000: break # limite pour pas lire tout le bag

