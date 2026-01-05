import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('turtlebot3_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'turtlebot3_waffle.urdf.xacro')
    
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
            }]
        )
    ])
