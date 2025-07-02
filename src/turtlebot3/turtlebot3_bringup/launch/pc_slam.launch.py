#!/usr/bin/env python3
#
# PC-optimized SLAM launch file for TurtleBot3 using slam_toolbox
# Enhanced with loop closing and higher resolution mapping for powerful PCs
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

    # PC SLAM - assumes robot is running remotely
    # Include robot state publisher for URDF/static transforms (no hardware needed)
    
    # Include robot state publisher to load burger URDF and provide static transforms
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Start the slam_toolbox node with PC-optimized settings
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

    # Configure slam_toolbox after 8 seconds (wait for lidar to initialize)
    configure_slam_toolbox = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 8 && ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"'],
        output='screen'
    )

    # Activate slam_toolbox after 10 seconds
    activate_slam_toolbox = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 10 && ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"'],
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

    # Launch RViz2 with SLAM configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'rviz',
        'model.rviz'
    )

    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add the actions to launch
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(static_transform_laser)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_slam_toolbox)
    ld.add_action(activate_slam_toolbox)
    ld.add_action(start_rviz2)

    return ld