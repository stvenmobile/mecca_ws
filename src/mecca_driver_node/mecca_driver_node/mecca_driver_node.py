import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time
import re


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        self.get_logger().info("Started node")
        
        # Subscribe to "safe" velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'safe_cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer for encoder polling
        self.encoder_poll_timer = self.create_timer(0.2, self.poll_encoders)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Joint names (must match your URDF exactly)
        self.joint_names = [
            'front_left_wheel_joint',
            'rear_left_wheel_joint', 
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]

        # Encoder and odometry data
        self.encoder_positions = [0, 0, 0, 0]  # ticks
        self.previous_encoder_positions = [0, 0, 0, 0]  # for velocity calculation
        self.encoder_last_published = self.get_clock().now()
        
        # Robot pose tracking for odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Robot physical parameters (adjust to match your robot)
        self.wheel_radius = 0.050  # 50mm wheel radius
        self.wheel_base = 0.175    # 175mm wheelbase (front to rear)
        self.track_width = 0.175   # 175mm track width (left to right)
        
        # Encoder parameters
        self.ticks_per_rev = 2420
        self.radians_per_tick = (2 * math.pi) / self.ticks_per_rev
        
        # Serial communication
        self.create_subscription(
            String,
            '/serial_driver/output_data',
            self.serial_output_callback,
            10
        )

        # Publisher to send velocity commands to serial communication node
        self.serial_publisher = self.create_publisher(String, '/serial_driver/input', 10)

        # Speed scaling factors
        self.MAX_SPEED = 1100  # Maximum possible speed
        self.NORMAL_SCALE = 0.4  # Default scaling (40% of MAX_SPEED)
        self.TURBO_SCALE = 0.6   # Turbo mode scaling (60% of MAX_SPEED)

        self.NORMAL_ROTATE_SCALE = 1.4  # Boosted rotation scale
        self.TURBO_ROTATE_SCALE = 2.0   # 30% more for turbo rotation

        self.speed_scale = self.NORMAL_SCALE  # Default to normal mode

    def cmd_vel_callback(self, msg):
        """Convert velocity message into serial command and publish."""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        scaled_x = int(linear_x * self.MAX_SPEED * self.speed_scale)
        scaled_y = int(linear_y * self.MAX_SPEED * self.speed_scale)

        if self.speed_scale == self.TURBO_SCALE:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.TURBO_ROTATE_SCALE)
        else:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.NORMAL_ROTATE_SCALE)

        # Construct command string
        command = f"V {scaled_x} {scaled_y} {scaled_rot}"

        # Publish to serial node
        msg_out = String()
        msg_out.data = command
        self.serial_publisher.publish(msg_out)

        # Log for debugging
        self.get_logger().debug(f"Published: {command}")

    def serial_output_callback(self, msg):
        if "Encoder Values" in msg.data:
            try:
                # More robust parsing - handle malformed data
                data = msg.data.strip()
                
                # Find the "Encoder Values" part and extract everything after it
                encoder_start = data.find("Encoder Values")
                if encoder_start != -1:
                    encoder_part = data[encoder_start:]
                    
                    # Look for M1=, M2=, M3=, M4= patterns
                    matches = re.findall(r'M(\d+)=(\d+)', encoder_part)
                    
                    if len(matches) >= 4:
                        # Extract motor values in order
                        motor_values = {}
                        for motor_num, value in matches:
                            motor_values[int(motor_num)] = int(value)
                        
                        # Ensure we have M1, M2, M3, M4
                        if all(i in motor_values for i in [1, 2, 3, 4]):
                            # Store previous positions for velocity calculation
                            self.previous_encoder_positions = self.encoder_positions.copy()
                            
                            self.encoder_positions = [
                                motor_values[1], motor_values[2], 
                                motor_values[3], motor_values[4]
                            ]
                            self.publish_joint_states()
                            self.publish_odometry()
                            # self.get_logger().info(f"✅ Encoders: {self.encoder_positions}")
                        else:
                            self.get_logger().warn(f"❌ Missing motor data in: {encoder_part}")
                    else:
                        self.get_logger().warn(f"❌ Could not find motor values in: {encoder_part}")
                else:
                    self.get_logger().warn(f"❌ 'Encoder Values' not found in: {data}")
                    
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to parse encoder data: {e} - Raw data: {msg.data}")

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names

        # Convert encoder ticks to radians
        js.position = [ticks * self.radians_per_tick for ticks in self.encoder_positions]

        js.velocity = []
        js.effort = []
        self.joint_state_pub.publish(js)

    def publish_odometry(self):
        """Calculate and publish odometry based on wheel encoders"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Calculate wheel velocities from encoder differences
        encoder_diff = [
            self.encoder_positions[i] - self.previous_encoder_positions[i] 
            for i in range(4)
        ]
        
        # Convert encoder differences to wheel velocities (m/s)
        wheel_velocities = [
            (diff * self.radians_per_tick * self.wheel_radius) / dt
            for diff in encoder_diff
        ]
        
        # Mecanum wheel kinematics (assuming standard X configuration)
        # FL, RL, FR, RR = Front Left, Rear Left, Front Right, Rear Right
        vx = (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]) / 4.0
        vy = (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]) / 4.0
        vth = (-wheel_velocities[0] - wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]) / (4.0 * (self.wheel_base + self.track_width) / 2.0)
        
        # Update robot pose
        delta_x = (vx * math.cos(self.robot_theta) - vy * math.sin(self.robot_theta)) * dt
        delta_y = (vx * math.sin(self.robot_theta) + vy * math.cos(self.robot_theta)) * dt
        delta_th = vth * dt
        
        self.robot_x += delta_x
        self.robot_y += delta_y
        self.robot_theta += delta_th
        
        # Publish transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        
        odom_trans.transform.translation.x = self.robot_x
        odom_trans.transform.translation.y = self.robot_y
        odom_trans.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        odom_trans.transform.rotation.x = 0.0
        odom_trans.transform.rotation.y = 0.0
        odom_trans.transform.rotation.z = math.sin(self.robot_theta / 2.0)
        odom_trans.transform.rotation.w = math.cos(self.robot_theta / 2.0)
        
        self.tf_broadcaster.sendTransform(odom_trans)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_trans.transform.rotation
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

    def poll_encoders(self):
        msg = String()
        msg.data = "I ENC"
        self.serial_publisher.publish(msg)
        # self.get_logger().info("🟢 Sent I ENC to STM32")


def main(args=None):
    print("🟢 Entered main() in mecca_driver_node")
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
