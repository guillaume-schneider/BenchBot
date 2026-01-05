
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Path to standard URDF/XACRO
    xacro_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )
    
    # Process Xacro via CLI
    robot_desc = Command(['xacro ', xacro_file, ' namespace:=', '""'])

    # 1. Start Gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_gazebo'))
    )

    # 2. Robot State Publisher (needs parsed URDF)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': robot_desc}],
        arguments=[] 
    )
    
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_waffle', 
                   '-file', robot_sdf,
                   '-x', x_pose, '-y', y_pose, '-z', '0.01'],
        output='screen'
    )

    
    # 3. Nav2 (Navigation Only)
    params_file_arg = LaunchConfiguration('params_file')
    
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file_arg,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False'
        }.items()
    )

    # 4. RViz (Conditional & Delayed)
    use_rviz = LaunchConfiguration('use_rviz', default='False')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_cmd]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('robot_sdf'),
        DeclareLaunchArgument('x_pose'),
        DeclareLaunchArgument('y_pose'),
        DeclareLaunchArgument('use_sim_time'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('use_gazebo', default_value='True'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')),
        
        gzserver_cmd,
        gzclient_cmd,
        start_robot_state_publisher_cmd,
        spawn_entity_cmd,
        nav2_launch_cmd,
        delayed_rviz
    ])
