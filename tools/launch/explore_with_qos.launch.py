#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default='')
    
    # Create node with QoS overrides
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('move_base', '/navigate_to_pose')
        ],
        # QoS overrides for /map subscription
        arguments=['--ros-args', '--log-level', 'info'],
        # Use parameter overrides to force transient_local
        ros_arguments=[
            '--param', 'qos_overrides./map.subscription.durability:=transient_local',
            '--param', 'qos_overrides./map.subscription.reliability:=reliable'
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=''),
        explore_node
    ])
