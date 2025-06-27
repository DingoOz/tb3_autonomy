#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

# This is for the dingo variant of the turtlebot3 robot
# rplidar a1
# ROS2 Jazzy

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    # default to burger
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # default to the ROS_DOMAIN_ID
    ros_domain_id = "30"
    
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    #LDS_MODEL = os.environ.get('LDS_MODEL', "A1")  # Fixed: use .get() method
    LDS_MODEL = "A1"  # kludge to get it to work with the dingo robot
    # LDS_LAUNCH_FILE = '/hlds_laser.launch.py'
    LDS_LAUNCH_FILE = "/hlds_laser.launch.py"   # kludge to get it to work with the dingo robot
    

    namespace = LaunchConfiguration('namespace', default='')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    if ROS_DISTRO == 'jazzy':
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                TURTLEBOT3_MODEL + '.yaml'))  # Fixed: use the string variable
    else:
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                TURTLEBOT3_MODEL + '.yaml'))  # Fixed: use the string variable

    if LDS_MODEL == "A1":
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory("rplidar_ros"), "launch"),
            )
        LDS_LAUNCH_FILE = "/rplidar.launch.py"
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
        )
        LDS_LAUNCH_FILE = "/rplidar.launch.py"

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),

        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan',
                              'namespace': namespace}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[
                tb3_param_dir,
                {
                    'namespace': namespace,
                    'enable_stamped_cmd_vel': True,
                    'opencr.id': 200,
                    'opencr.baud_rate': 1000000,
                    'opencr.protocol_version': 2.0,
                    'wheels.separation': 0.160,
                    'wheels.radius': 0.033,
                    'motors.profile_acceleration_time': 0.0,
                    'motors.profile_time': 0.0,
                    'motors.torque': True,
                    'motors.left.id': 1,
                    'motors.right.id': 2,
                    'sensors.bumper_1': 0,
                    'sensors.bumper_2': 0, 
                    'sensors.illumination': 0,
                    'sensors.ir': 0,
                    'sensors.sonar': 0,
                    'diff_drive_controller.publish_rate': 30.0,
                    'diff_drive_controller.left_wheel_name': 'wheel_left_joint',
                    'diff_drive_controller.right_wheel_name': 'wheel_right_joint'
                }],
            arguments=['-i', usb_port],
            output='screen'),
    ])