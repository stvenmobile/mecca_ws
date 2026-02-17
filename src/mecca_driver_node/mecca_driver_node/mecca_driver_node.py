import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import re

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("!!! DIAGNOSTIC MODE ACTIVE !!!")
        
        self.WHEEL_MAP = {'rear_right': 1, 'front_right': 2, 'rear_left': 3, 'front_left': 4}

        # Subscriptions
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.serial_feedback_sub = self.create_subscription(
            UInt8MultiArray, '/serial_read', self.serial_output_callback, 10)
        
        # Publishers
        self.serial_publisher = self.create_publisher(UInt8MultiArray, '/serial_write', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Main 50Hz Loop
        self.main_timer = self.create_timer(0.02, self.unified_heartbeat_callback)
        self.heartbeat_counter = 0
        
        # State
        self.target_v = [0.0, 0.0, 0.0]
        self.VEL_SCALING = 2500.0
        self.line_buffer = ""
        self.encoder_positions = [0, 0, 0, 0]
        self.prev_encoder_positions = [0, 0, 0, 0]
        self.robot_x = 0.0; self.robot_y = 0.0; self.robot_theta = 0.0
        self.last_odom_time = self.get_clock().now()

    def _send_command(self, text):
        msg = UInt8MultiArray()
        msg.data = [ord(c) for c in text]
        self.serial_publisher.publish(msg)

    def cmd_vel_callback(self, msg):
        # DEBUG: Print to console when a twist message arrives
        self.get_logger().info(f"INCOMING CMD_VEL: x={msg.linear.x}")
        self.target_v = [msg.linear.x, msg.linear.y, msg.angular.z]

    def unified_heartbeat_callback(self):
        vx, vy, vt = self.target_v
        
        # Scale and Clamp
        vx_i = max(min(int(vx * self.VEL_SCALING), 1000), -1000)
        vy_i = max(min(int(vy * self.VEL_SCALING), 1000), -1000)
        vt_i = max(min(int(vt * self.VEL_SCALING * 0.6), 1000), -1000)

        # 1. Send Velocity
        self._send_command(f"V {vx_i} {vy_i} {vt_i}\r\n")

        # 2. Every 5th cycle, poll encoders
        self.heartbeat_counter += 1
        if self.heartbeat_counter >= 5:
            # DEBUG: Occasional print to verify timer is alive
            if vx_i != 0:
                self.get_logger().info(f"TIMER ACTIVE: Sending V {vx_i}")
            self._send_command("I ENC\r\n")
            self.heartbeat_counter = 0

    def serial_output_callback(self, msg):
        self.line_buffer += "".join(chr(b) for b in msg.data)
        while "\n" in self.line_buffer:
            line, self.line_buffer = self.line_buffer.split("\n", 1)
            if "Encoder Values" in line:
                self.parse_encoders(line.strip())

    def parse_encoders(self, line):
        matches = re.findall(r'M(\d+)=(-?\d+)', line)
        if len(matches) >= 4:
            m_data = {int(m): int(v) for m, v in matches}
            self.prev_encoder_positions = list(self.encoder_positions)
            self.encoder_positions = [
                m_data[self.WHEEL_MAP['front_left']], m_data[self.WHEEL_MAP['rear_left']],
                m_data[self.WHEEL_MAP['front_right']], m_data[self.WHEEL_MAP['rear_right']]
            ]
            self.publish_odometry()

    def publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0: return
        d_wheels = [(self.encoder_positions[i] - self.prev_encoder_positions[i]) * ((2 * math.pi) / 2420.0) * 0.050 for i in range(4)]
        vx = sum(d_wheels) / 4.0
        vy = (-d_wheels[0] + d_wheels[1] + d_wheels[2] - d_wheels[3]) / 4.0
        v_alpha = 0.175 # (base+track)/2
        vth = (-d_wheels[0] - d_wheels[1] + d_wheels[2] + d_wheels[3]) / (4.0 * v_alpha)
        self.robot_x += (vx * math.cos(self.robot_theta) - vy * math.sin(self.robot_theta))
        self.robot_y += (vx * math.sin(self.robot_theta) + vy * math.cos(self.robot_theta))
        self.robot_theta += vth
        t = TransformStamped()
        t.header.stamp = now.to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_x; t.transform.translation.y = self.robot_y
        t.transform.rotation.z = math.sin(self.robot_theta / 2.0)
        t.transform.rotation.w = math.cos(self.robot_theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        self.last_odom_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt: Emergency Stop initiated...")
        # Send multiple stop commands to ensure at least one gets through the buffer
        for _ in range(5):
            node._send_command("V 0 0 0\r\n")
    finally:
        # Give it a tiny moment to actually push those bytes before killing the node
        node.destroy_node()
        rclpy.shutdown()