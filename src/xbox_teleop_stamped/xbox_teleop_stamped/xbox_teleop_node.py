#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
import math

class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        # Changed to publish TwistStamped instead of Twist
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, joy_msg):
        # Create TwistStamped message instead of Twist
        twist_stamped = TwistStamped()
        
        # Set the timestamp
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'  # Adjust frame_id as needed
        
        # Initialize inputs
        forward_input = 0.0
        sideways_input = 0.0
        direction_reversed = False
        
        # Check if Axis 5 (right trigger) is fully pressed (-1.0)
        dead_man_switch = joy_msg.axes[5] == -1.0
        
        if dead_man_switch:
            # Axis 1 (left stick Y-axis) for forward/backward movement
            forward_input = joy_msg.axes[1]
            
            # Axis 2 (left stick X-axis) for left/right movement
            sideways_input = joy_msg.axes[2]
            
            # Reverse left/right direction only when moving forward
            if forward_input >= 0:
                sideways_input = -sideways_input
                direction_reversed = True
            
            # Calculate linear velocity
            linear_velocity = math.sqrt(forward_input**2 + sideways_input**2)
            # Preserve the sign of forward_input to allow backward motion
            twist_stamped.twist.linear.x = math.copysign(linear_velocity, forward_input) * 0.5  # Scale down to 0.5 m/s max
            
            # Calculate angular velocity using atan2
            if linear_velocity != 0:
                # Use -sideways_input to get correct turning direction
                angular_velocity = math.atan2(-sideways_input, abs(forward_input))
                twist_stamped.twist.angular.z = angular_velocity * 1.5  # Scale to 1.5 rad/s max
            else:
                twist_stamped.twist.angular.z = 0.0

        # Display Xbox controller axes data
        self.get_logger().info('Xbox Controller Axes:')
        for i, axis in enumerate(joy_msg.axes):
            self.get_logger().info(f'  Axis {i}: {axis:.2f}')
        
        # Display Twist output
        self.get_logger().info('TwistStamped Output:')
        self.get_logger().info(f'  Linear X: {twist_stamped.twist.linear.x:.2f} m/s')
        self.get_logger().info(f'  Angular Z: {twist_stamped.twist.angular.z:.2f} rad/s')
        self.get_logger().info(f'  Dead Man\'s Switch: {"Engaged" if dead_man_switch else "Disengaged"}')
        self.get_logger().info(f'  Direction Reversed: {"Yes" if direction_reversed else "No"}')
        self.get_logger().info(f'  Timestamp: {twist_stamped.header.stamp.sec}.{twist_stamped.header.stamp.nanosec}')
        
        self.get_logger().info('------------------------')
        
        self.publisher_.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    xbox_teleop_node = XboxTeleopNode()
    rclpy.spin(xbox_teleop_node)
    xbox_teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()