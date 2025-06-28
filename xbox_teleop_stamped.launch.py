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
        description='Joystick device'
    )
    
    # Joy node to read controller input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Custom teleop node that outputs TwistStamped
    teleop_stamped_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_joy_stamped',
        name='teleop_joy_stamped_node',
        parameters=[{
            'axis_linear.x': 1,      # Left stick vertical
            'axis_angular.yaw': 0,   # Left stick horizontal  
            'scale_linear.x': 0.7,   # Max linear speed
            'scale_angular.yaw': 1.0, # Max angular speed
            'enable_button': 7,      # Right bumper (RB)
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        teleop_stamped_node,
    ])
