#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', 
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Joy node to read Xbox controller input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Xbox teleop node to convert joy messages to TwistStamped cmd_vel
    xbox_teleop_node = Node(
        package='xbox_teleop_stamped',
        executable='xbox_teleop_node',
        name='xbox_teleop_stamped_node',
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        xbox_teleop_node,
    ])