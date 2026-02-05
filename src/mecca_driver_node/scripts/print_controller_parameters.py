#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ParameterPrinter(Node):
    def __init__(self):
        super().__init__('parameter_printer')

        params = [
            "mecanum_drive_controller.base_frame_id",
            "mecanum_drive_controller.enable_odom_tf",
            "mecanum_drive_controller.front_left_wheel_command_joint_name",
            "mecanum_drive_controller.front_left_wheel_name",
            "mecanum_drive_controller.front_right_wheel_command_joint_name",
            "mecanum_drive_controller.front_right_wheel_name",
            "mecanum_drive_controller.rear_left_wheel_command_joint_name",
            "mecanum_drive_controller.rear_left_wheel_name",
            "mecanum_drive_controller.rear_right_wheel_command_joint_name",
            "mecanum_drive_controller.rear_right_wheel_name",
            "mecanum_drive_controller.type",
            "mecanum_drive_controller.wheel_radius",
            "mecanum_drive_controller.wheel_separation_x",
            "mecanum_drive_controller.wheel_separation_y",
            "update_rate",
            "use_sim_time"
        ]

        self.get_logger().info("Fetching parameter values from /controller_manager")

        for param in params:
            value = self.get_param(param)
            if value is not None:
                self.get_logger().info(f"{param}: {value}")
            else:
                self.get_logger().warn(f"Parameter '{param}' not found or not set.")

    def get_param(self, param_name):
        return self.get_parameter(param_name).value

def main(args=None):
    rclpy.init(args=args)
    node = ParameterPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
