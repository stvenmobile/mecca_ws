import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import re

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("!!! STAMPED VELOCITY & PID MONITORING ACTIVE !!!")
        
        # Mapping for the 4-motor serial protocol
        # Note: Adjust these indices if the physical wheel order differs
        self.WHEEL_MAP = {'rear_right': 1, 'front_right': 2, 'rear_left': 3, 'front_left': 4}

        # Subscriptions
        # Subscribing to TwistStamped to match the controller output requirements
        self.subscription = self.create_subscription(
            TwistStamped, 
            '/mecanum_drive_controller/reference', 
            self.cmd_vel_callback, 
            10)
            
        self.serial_feedback_sub = self.create_subscription(
            UInt8MultiArray, 
            '/serial_read', 
            self.serial_output_callback, 
            10)
        
        # Publishers
        self.serial_publisher = self.create_publisher(UInt8MultiArray, '/serial_write', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Main 50Hz Loop (20ms)
        self.main_timer = self.create_timer(0.02, self.unified_heartbeat_callback)
        self.heartbeat_counter = 0
        
        # State
        self.target_v = [0.0, 0.0, 0.0]
        self.VEL_SCALING = 2500.0  
        self.line_buffer = ""
        self.encoder_positions = [0, 0, 0, 0]
        self.prev_encoder_positions = [0, 0, 0, 0]
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.last_odom_time = self.get_clock().now()

    def _send_command(self, text):
        """Helper to send strings over serial bridge topic."""
        msg = UInt8MultiArray()
        msg.data = [ord(c) for c in text]
        self.serial_publisher.publish(msg)

    def cmd_vel_callback(self, msg):
        """Extracts linear and angular velocity from the stamped message."""
        self.target_v = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]

    def unified_heartbeat_callback(self):
        """Periodic task to send velocity heartbeats and poll for encoders."""
        vx, vy, vt = self.target_v
        
        # Scale and Clamp for STM32 protocol
        vx_i = max(min(int(vx * self.VEL_SCALING), 1000), -1000)
        vy_i = max(min(int(vy * self.VEL_SCALING), 1000), -1000)
        vt_i = max(min(int(vt * self.VEL_SCALING * 0.6), 1000), -1000)

        # 1. Send Velocity command to serial
        self._send_command(f"V {vx_i} {vy_i} {vt_i}\r\n")

        # 2. Every 5th cycle (10Hz), poll encoders for odometry
        self.heartbeat_counter += 1
        if self.heartbeat_counter >= 5:
            self._send_command("I ENC\r\n")
            self.heartbeat_counter = 0

    def serial_output_callback(self, msg):
        """Reassembles incoming serial bytes into lines for parsing."""
        self.line_buffer += "".join(chr(b) for b in msg.data)
        while "\n" in self.line_buffer:
            line, self.line_buffer = self.line_buffer.split("\n", 1)
            line = line.strip()
            if "I ENC" in line:
                self.parse_encoders(line)
            elif "I PID" in line:
                self.parse_pid_debug(line)

    def parse_pid_debug(self, line):
        """Parses Target and Actual speeds for PID tuning feedback."""
        # Expected Format: I PID <T1> <A1> <T2> <A2> <T3> <A3> <T4> <A4>
        vals = re.findall(r'-?\d+', line)
        if len(vals) >= 8:
            v = [int(x) for x in vals]
            # Log throttled to 2Hz to prevent console flood
            self.get_logger().info(
                f"PID Feedback [M1: {v[1]}/{v[0]}] [M2: {v[3]}/{v[2]}] [M3: {v[5]}/{v[4]}] [M4: {v[7]}/{v[6]}]",
                throttle_duration_sec=0.5 
            )

    def parse_encoders(self, line):
        """Parses raw encoder strings and updates position tracking."""
        vals = re.findall(r'-?\d+', line)
        if len(vals) >= 4:
            self.prev_encoder_positions = list(self.encoder_positions)
            self.encoder_positions = [int(v) for v in vals]
            self.publish_odometry()

    def publish_odometry(self):
        """Performs kinematics integration and publishes Odom + TF."""
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0: return
        
        # Constants: 2420 ticks per revolution, 0.050m wheel radius
        ticks_to_m = ((2 * math.pi) / 2420.0) * 0.050
        d_wheels = [(self.encoder_positions[i] - self.prev_encoder_positions[i]) * ticks_to_m for i in range(4)]
                   
        # Mecanum Kinematics
        vx = sum(d_wheels) / 4.0
        vy = (-d_wheels[0] + d_wheels[1] + d_wheels[2] - d_wheels[3]) / 4.0
        v_alpha = 0.175 # (Base Width + Base Length) / 2
        vth = (-d_wheels[0] - d_wheels[1] + d_wheels[2] + d_wheels[3]) / (4.0 * v_alpha)
        
        # Integration of pose
        self.robot_x += (vx * math.cos(self.robot_theta) - vy * math.sin(self.robot_theta))
        self.robot_y += (vx * math.sin(self.robot_theta) + vy * math.cos(self.robot_theta))
        self.robot_theta += vth
        
        # Create and publish TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.rotation.z = math.sin(self.robot_theta / 2.0)
        t.transform.rotation.w = math.cos(self.robot_theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation.z = t.transform.rotation.z
        odom.pose.pose.orientation.w = t.transform.rotation.w
        self.odom_pub.publish(odom)
        
        self.last_odom_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt: Stopping motors...")
        for _ in range(5):
            node._send_command("V 0 0 0\r\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()