import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

# Send V commands at this rate regardless of incoming message rate.
# 20 Hz keeps the STM32 watchdog (1 s) fed and avoids STOP_BRAKE glitches.
SEND_RATE_HZ = 20
CMD_TIMEOUT_S = 0.5   # zero out if no /cmd_vel received within this window

# Velocity thresholds for LED state classification (same units as Twist: m/s, rad/s)
_LIN_THRESH = 0.02
_ROT_THRESH = 0.05
_K          = 0.175   # (wheel_sep_x + wheel_sep_y) / 2

_LED_MAP = {
    'forward':      'fwd',
    'reverse':      'bwd',
    'strafe_left':  'left',
    'strafe_right': 'right',
    'rotate_left':  'left',
    'rotate_right': 'right',
    'stop':         'stop',
}


class TeleopSerialBridgeNode(Node):

    def __init__(self):
        super().__init__('teleop_serial_bridge')

        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._last_cmd_time = 0.0   # epoch seconds
        self._led_state = 'stop'

        self._serial_pub = self.create_publisher(String, '/serial_driver/input', 10)
        self._led_pub    = self.create_publisher(String, 'led_commands', 10)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_timer(1.0 / SEND_RATE_HZ, self._send_cmd)

        self.get_logger().info(
            f'Teleop Serial Bridge started — sending V at {SEND_RATE_HZ} Hz, '
            f'timeout {CMD_TIMEOUT_S} s')

    # ── Incoming twist ──────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._vz = msg.angular.z
        self._last_cmd_time = time.time()

        new_state = self._classify(msg)
        if new_state != self._led_state:
            self._led_state = new_state
            led = String()
            led.data = _LED_MAP[new_state]
            self._led_pub.publish(led)

    # ── 20 Hz heartbeat ─────────────────────────────────────────────────────────

    def _send_cmd(self):
        # Zero out if no fresh command arrived within the timeout window
        if self._last_cmd_time > 0 and (time.time() - self._last_cmd_time) > CMD_TIMEOUT_S:
            self._vx = self._vy = self._vz = 0.0
            if self._led_state != 'stop':
                self._led_state = 'stop'
                led = String()
                led.data = 'stop'
                self._led_pub.publish(led)

        vx_mm = int(self._vx * 1000)
        vy_mm = int(self._vy * 1000)
        vz_mr = int(self._vz * 1000)

        msg = String()
        msg.data = f'V {vx_mm} {vy_mm} {vz_mr}'
        self._serial_pub.publish(msg)

    # ── Motion classifier (for LED) ─────────────────────────────────────────────

    def _classify(self, twist: Twist) -> str:
        lx, ly, az = twist.linear.x, twist.linear.y, twist.angular.z
        if abs(lx) < _LIN_THRESH and abs(ly) < _LIN_THRESH and abs(az) < _ROT_THRESH:
            return 'stop'
        az_equiv = abs(az) * _K
        if abs(lx) >= abs(ly) and abs(lx) >= az_equiv:
            return 'forward' if lx > 0 else 'reverse'
        if abs(ly) >= abs(lx) and abs(ly) >= az_equiv:
            return 'strafe_left' if ly > 0 else 'strafe_right'
        return 'rotate_left' if az > 0 else 'rotate_right'

    # ── Shutdown ─────────────────────────────────────────────────────────────────

    def destroy_node(self):
        stop = String()
        stop.data = 'V 0 0 0'
        self._serial_pub.publish(stop)
        self.get_logger().info('Teleop Serial Bridge stopped.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
