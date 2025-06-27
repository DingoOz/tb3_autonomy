#!/usr/bin/env python3
#
# SLAM launch file for TurtleBot3 using slam_toolbox
# 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('turtlebot3_bringup')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dir, 'param', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # Include robot launch (robot hardware + lidar)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'dingo.robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Start the slam_toolbox node
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Configure slam_toolbox after 3 seconds
    configure_slam_toolbox = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 3 && ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"'],
        output='screen'
    )

    # Activate slam_toolbox after 4 seconds
    activate_slam_toolbox = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 4 && ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"'],
        output='screen'
    )

    # Static transform from base_scan to laser frame (fixes frame_id mismatch)
    static_transform_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_scan', 'laser'],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add the actions to launch
    ld.add_action(robot_launch)
    ld.add_action(static_transform_laser)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_slam_toolbox)
    ld.add_action(activate_slam_toolbox)

    return ld