import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import re

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("Mecca Driver Node Initialized")
        
        # --- Hardware Mapping (Verified) ---
        # Maps Physical Position to low-level Motor ID (M1-M4)
        self.WHEEL_MAP = {
            'rear_right':  1,  # M1
            'front_right': 2,  # M2
            'rear_left':   3,  # M3
            'front_left':  4   # M4
        }

        # Subscriptions
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.serial_feedback_sub = self.create_subscription(String, '/serial_driver/output_data', self.serial_output_callback, 10)
        
        # Publishers
        self.serial_publisher = self.create_publisher(String, '/serial_driver/input', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.encoder_poll_timer = self.create_timer(0.1, self.poll_encoders) # 10Hz polling
        
        # Parameters (Units: Meters)
        self.wheel_radius = 0.050 
        self.wheel_base = 0.175   
        self.track_width = 0.175  
        self.ticks_per_rev = 2420.0
        self.radians_per_tick = (2 * math.pi) / self.ticks_per_rev
        
        # Speed Scaling
        self.MAX_SPEED_INT = 1100.0  # STM32 max integer
        self.SPEED_SCALE = 1.0       # Direct scaling for testing
        
        # State Tracking
        self.encoder_positions = [0, 0, 0, 0] # Order: FL, RL, FR, RR
        self.prev_encoder_positions = [0, 0, 0, 0]
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        """Translate Twist (m/s) to STM32 'V' command (integer units)."""
        # Linear velocity scaling
        vx_out = int(msg.linear.x * self.MAX_SPEED_INT * self.SPEED_SCALE)
        vy_out = int(msg.linear.y * self.MAX_SPEED_INT * self.SPEED_SCALE)
        
        # Angular velocity scaling (Z-rotation)
        vt_out = int(msg.angular.z * 100) 

        # The \r\n is often required for STM32 to 'process' the buffer
        command = f"V {vx_out} {vy_out} {vt_out}\r\n"
        
        self.serial_publisher.publish(String(data=command))
        # Log exactly what we are sending for debugging
        self.get_logger().info(f"OUT: {command.strip()}")

    def serial_output_callback(self, msg):
        """Parse 'Encoder Values M1=... M4=...' from STM32."""
        if "Encoder Values" in msg.data:
            matches = re.findall(r'M(\d+)=(-?\d+)', msg.data)
            if len(matches) >= 4:
                m_data = {int(m): int(v) for m, v in matches}
                
                self.prev_encoder_positions = list(self.encoder_positions)
                
                # Re-index based on physical layout for ROS Kinematics
                # Order for math: [0: FL, 1: RL, 2: FR, 3: RR]
                self.encoder_positions = [
                    m_data[self.WHEEL_MAP['front_left']],
                    m_data[self.WHEEL_MAP['rear_left']],
                    m_data[self.WHEEL_MAP['front_right']],
                    m_data[self.WHEEL_MAP['rear_right']]
                ]
                self.publish_odometry()

    def publish_odometry(self):
        """Mecanum Kinematics calculation."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # Delta position in meters per wheel
        d_wheels = [(self.encoder_positions[i] - self.prev_encoder_positions[i]) * self.radians_per_tick * self.wheel_radius for i in range(4)]

        # Kinematics: FL(0), RL(1), FR(2), RR(3)
        vx = (d_wheels[0] + d_wheels[1] + d_wheels[2] + d_wheels[3]) / 4.0
        vy = (-d_wheels[0] + d_wheels[1] + d_wheels[2] - d_wheels[3]) / 4.0
        vth = (-d_wheels[0] - d_wheels[1] + d_wheels[2] + d_wheels[3]) / (4.0 * ((self.wheel_base + self.track_width)/2.0))

        # Update Pose
        self.robot_x += (vx * math.cos(self.robot_theta) - vy * math.sin(self.robot_theta))
        self.robot_y += (vx * math.sin(self.robot_theta) + vy * math.cos(self.robot_theta))
        self.robot_theta += vth

        # Broadcast TF and Odom Message
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.rotation.z = math.sin(self.robot_theta / 2.0)
        t.transform.rotation.w = math.cos(self.robot_theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        self.last_time = now

    def poll_encoders(self):
        self.serial_publisher.publish(String(data="I ENC"))

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()