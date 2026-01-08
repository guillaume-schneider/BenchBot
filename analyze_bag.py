
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

bag_path = sys.argv[1]
storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader = SequentialReader()
reader.open(storage_options, converter_options)

start_time = None
end_time = None
odom_count = 0
min_x, max_x = float('inf'), float('-inf')
min_y, max_y = float('inf'), float('-inf')

while reader.has_next():
    (topic, data, t_ns) = reader.read_next()
    if start_time is None: start_time = t_ns
    end_time = t_ns
    
    if topic == '/odom':
        msg = deserialize_message(data, Odometry)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)
        odom_count += 1

print(f"Bag Duration: {(end_time - start_time)/1e9:.2f}s")
print(f"Odom messages: {odom_count}")
print(f"Odom Bounds: X[{min_x:.2f}, {max_x:.2f}], Y[{min_y:.2f}, {max_y:.2f}]")
