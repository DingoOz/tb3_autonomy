#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile


class TeleopJoyStamped(Node):
    def __init__(self):
        super().__init__('teleop_joy_stamped')
        
        # Parameters
        self.declare_parameter('axis_linear.x', 1)
        self.declare_parameter('axis_angular.yaw', 0)
        self.declare_parameter('scale_linear.x', 0.7)
        self.declare_parameter('scale_angular.yaw', 1.0)
        self.declare_parameter('enable_button', 5)  # Right bumper
        
        self.axis_linear_x = self.get_parameter('axis_linear.x').value
        self.axis_angular_yaw = self.get_parameter('axis_angular.yaw').value
        self.scale_linear_x = self.get_parameter('scale_linear.x').value
        self.scale_angular_yaw = self.get_parameter('scale_angular.yaw').value
        self.enable_button = self.get_parameter('enable_button').value
        
        # Publishers and subscribers
        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos)
        
        self.get_logger().info('Xbox Controller Teleop (TwistStamped) started')
        self.get_logger().info(f'Controls:')
        self.get_logger().info(f'  Left stick vertical: Forward/Backward')
        self.get_logger().info(f'  Left stick horizontal: Turn Left/Right')
        self.get_logger().info(f'  Right bumper (RB): Enable movement')
        
    def joy_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        
        # Check if enable button is pressed
        if len(msg.buttons) > self.enable_button and msg.buttons[self.enable_button]:
            # Get joystick values
            if len(msg.axes) > max(self.axis_linear_x, self.axis_angular_yaw):
                linear_x = msg.axes[self.axis_linear_x] * self.scale_linear_x
                angular_z = msg.axes[self.axis_angular_yaw] * self.scale_angular_yaw
                
                twist_stamped.twist.linear.x = linear_x
                twist_stamped.twist.angular.z = angular_z
        
        # Always publish (even zeros when button not pressed)
        self.cmd_vel_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    teleop_joy_stamped = TeleopJoyStamped()
    
    try:
        rclpy.spin(teleop_joy_stamped)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = teleop_joy_stamped.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        teleop_joy_stamped.cmd_vel_pub.publish(twist_stamped)
        
        teleop_joy_stamped.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()