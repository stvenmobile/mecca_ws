#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
import math
import time

class SimpleWheelController(Node):
    def __init__(self):
        super().__init__('simple_wheel_controller')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.145)
        
        # Get parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        # Create publishers and subscribers
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Store current wheel velocities
        self.wheel_velocities = {
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'rear_left_wheel_joint': 0.0,
            'rear_right_wheel_joint': 0.0
        }
        
        # Create a timer to publish joint states
        self.timer = self.create_timer(0.01, self.publish_joint_states)
        
        self.get_logger().info('Simple wheel controller started')
    
    def twist_callback(self, msg):
        # Extract linear and angular velocity
        linear_x = msg.linear.x
        linear_y = msg.linear.y  # For mecanum motion
        angular_z = msg.angular.z
        
        # Calculate wheel velocities for mecanum drive
        # For a simple visualization, we'll use a differential drive approximation
        left_velocity = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        right_velocity = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        
        # For mecanum drive, we could handle strafing with linear_y
        # But for simplicity in visualization, we're ignoring it for now
        
        # Update wheel velocities
        self.wheel_velocities['front_left_wheel_joint'] = left_velocity
        self.wheel_velocities['front_right_wheel_joint'] = right_velocity
        self.wheel_velocities['rear_left_wheel_joint'] = left_velocity
        self.wheel_velocities['rear_right_wheel_joint'] = right_velocity
    
    def publish_joint_states(self):
        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Add joint names and velocities
        for joint_name, velocity in self.wheel_velocities.items():
            joint_state.name.append(joint_name)
            joint_state.position.append(0.0)  # We don't track positions for continuous joints
            joint_state.velocity.append(velocity)
            joint_state.effort.append(0.0)  # No effort information
        
        # Publish joint state
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleWheelController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()