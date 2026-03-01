import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from mecca_driver_node import ws2812
import spidev
import time
import numpy as np
from numpy import sin, pi
import threading
import queue

LED_COUNT  = 7
SPI_BUS    = 0
SPI_DEVICE = 0

# Velocity thresholds for state classification
_LIN_THRESH = 0.02   # m/s   — ignore linear velocities below this
_ROT_THRESH = 0.05   # rad/s — ignore rotation below this
_K          = 0.175  # m     — (wheel_sep_x + wheel_sep_y) / 2; scales angular to linear for comparison


class LEDControllerNode(Node):

    def __init__(self):
        super().__init__('led_controller_node')
        self.num_leds = LED_COUNT

        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 6400000

        self._state          = 'stop'
        self._stop_event     = threading.Event()
        self._current_thread: threading.Thread | None = None

        # State changes are handled in a worker thread so _reference_cb
        # never blocks the ROS spin loop (thread.join in _start_animation
        # can take up to 600 ms, which delays message delivery and triggers
        # the mecanum controller's cmd_vel_timeout → V 0 0 0 → motors stop).
        self._state_queue  = queue.Queue(maxsize=1)
        self._state_worker = threading.Thread(target=self._state_worker_fn,
                                              daemon=True, name='led_state_worker')
        self._state_worker.start()

        self.get_logger().info('LED Controller Node Started')

        self.startup_sequence()
        self._set_state('stop')

        # Automatic state-driven LEDs from motion commands
        self.create_subscription(
            TwistStamped,
            '/mecanum_drive_controller/reference',
            self._reference_cb,
            10)

        # Manual override topic (testing / external control)
        self.create_subscription(String, 'led_commands', self._led_command_cb, 10)

    # ── State machine ──────────────────────────────────────────────────────────

    def _classify(self, twist):
        """Classify a Twist into a motion-state string."""
        lx = twist.linear.x
        ly = twist.linear.y
        az = twist.angular.z

        if abs(lx) < _LIN_THRESH and abs(ly) < _LIN_THRESH and abs(az) < _ROT_THRESH:
            return 'stop'

        # Scale angular to equivalent wheel-rim speed for fair comparison
        az_equiv = abs(az) * _K

        if abs(lx) >= abs(ly) and abs(lx) >= az_equiv:
            return 'forward' if lx > 0 else 'reverse'
        if abs(ly) >= abs(lx) and abs(ly) >= az_equiv:
            return 'strafe_left' if ly > 0 else 'strafe_right'
        return 'rotate_left' if az > 0 else 'rotate_right'

    def _state_worker_fn(self):
        """Consume state changes off the queue without blocking the ROS spin thread."""
        while True:
            try:
                state = self._state_queue.get(timeout=0.5)
                self._set_state(state)
            except queue.Empty:
                pass

    def _reference_cb(self, msg: TwistStamped):
        new_state = self._classify(msg.twist)
        if new_state != self._state:
            self._state = new_state          # optimistic update — deduplicates rapid changes
            try:
                self._state_queue.put_nowait(new_state)
            except queue.Full:
                pass                         # a state change is already queued; drop this one

    def _set_state(self, state: str):
        self._state = state
        self.get_logger().debug(f'LED state → {state}')
        effects = {
            'forward':      self._anim_forward,
            'reverse':      self._anim_reverse,
            'strafe_left':  lambda: self._anim_scroll((0, 255, 0),    forward=True),
            'strafe_right': lambda: self._anim_scroll((0, 255, 0),    forward=False),
            'rotate_left':  lambda: self._anim_scroll((255, 165, 0),  forward=True),
            'rotate_right': lambda: self._anim_scroll((255, 165, 0),  forward=False),
            'stop':         self._anim_stop,
        }
        self._start_animation(effects.get(state, self._anim_stop))

    def _start_animation(self, fn):
        """Stop the running animation, wait for it to finish, then start fn."""
        self._stop_event.set()
        if self._current_thread is not None and self._current_thread.is_alive():
            self._current_thread.join(timeout=0.6)   # wait for thread to see the event
        self._stop_event.clear()
        self._current_thread = threading.Thread(target=fn, daemon=True)
        self._current_thread.start()

    # ── Animations (all non-blocking; check _stop_event in loops) ─────────────

    def _anim_forward(self):
        """Blue ping-pong sweep — indicates forward motion."""
        sequence = [
            (1, 0, 0, 0, 0, 0, 1),
            (0, 1, 0, 0, 0, 1, 0),
            (0, 0, 1, 0, 1, 0, 0),
            (0, 0, 0, 1, 0, 0, 0),
            (0, 0, 1, 0, 1, 0, 0),
            (0, 1, 0, 0, 0, 1, 0),
            (1, 0, 0, 0, 0, 0, 1),
        ]
        while not self._stop_event.is_set():
            for step in sequence:
                if self._stop_event.is_set():
                    break
                self.update_strip([(0, 0, 255) if v else (0, 0, 0) for v in step])
                time.sleep(0.05)

    def _anim_reverse(self):
        """Red flash — indicates reverse motion."""
        while not self._stop_event.is_set():
            self.update_strip([(255, 0, 0)] * self.num_leds)
            time.sleep(0.4)
            if self._stop_event.is_set():
                break
            self.update_strip([(0, 0, 0)] * self.num_leds)
            time.sleep(0.2)

    def _anim_scroll(self, color, forward=True):
        """Sequential fill scrolling left or right.
        Green  = strafe.
        Orange = rotate.
        """
        order = range(self.num_leds) if forward else range(self.num_leds - 1, -1, -1)
        while not self._stop_event.is_set():
            leds = [(0, 0, 0)] * self.num_leds
            for i in order:
                if self._stop_event.is_set():
                    break
                leds[i] = color
                self.update_strip(leds)
                time.sleep(0.08)
            if not self._stop_event.is_set():
                self.update_strip([(0, 0, 0)] * self.num_leds)
                time.sleep(0.05)

    def _anim_stop(self):
        """First and last LED white — robot idle."""
        leds        = [(0, 0, 0)] * self.num_leds
        leds[0]     = (255, 255, 255)
        leds[-1]    = (255, 255, 255)
        self.update_strip(leds)
        # Static pattern — no loop needed

    # ── Manual override (led_commands topic) ──────────────────────────────────

    _CMD_MAP = {
        'fwd':   'forward',
        'bwd':   'reverse',
        'left':  'strafe_left',
        'right': 'strafe_right',
        'stop':  'stop',
    }

    def _led_command_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'rainbow':
            self._start_animation(self.set_rainbow_wave_effect)
        elif cmd == 'test':
            threading.Thread(target=self.run_test_sequence, daemon=True).start()
        elif cmd in self._CMD_MAP:
            self._set_state(self._CMD_MAP[cmd])
        else:
            self.get_logger().warn(f'Unknown LED command: {cmd}')

    # ── Startup / rainbow / test ───────────────────────────────────────────────

    def startup_sequence(self):
        self.set_rainbow_wave_effect(duration=2)
        time.sleep(2.5)

    def set_rainbow_wave_effect(self, duration=6):
        self._stop_event.set()
        if self._current_thread is not None and self._current_thread.is_alive():
            self._current_thread.join(timeout=0.6)
        self._stop_event.clear()
        tStart  = time.time()
        indices = 4 * np.array(range(self.num_leds), dtype=np.uint32) * pi / self.num_leds
        stop    = self._stop_event   # capture reference for the thread

        def _run():
            period0, period1, period2 = 2.0, 2.1, 2.2
            while not stop.is_set() and (time.time() - tStart < duration):
                t = time.time() - tStart
                f = np.zeros((self.num_leds, 3))
                f[:, 0] = sin(2 * pi * t / period0 + indices)
                f[:, 1] = sin(2 * pi * t / period1 + indices)
                f[:, 2] = sin(2 * pi * t / period2 + indices)
                f = (255 * ((f + 1.0) / 2.0)).astype(np.uint8)
                self.update_strip([tuple(c) for c in f])
                time.sleep(0.05)
            self.update_strip([(0, 0, 0)] * self.num_leds)
            self.get_logger().info('Rainbow effect complete.')

        self._current_thread = threading.Thread(target=_run, daemon=True)
        self._current_thread.start()

    def run_test_sequence(self):
        self.get_logger().info('Starting LED test sequence')
        for state in ['forward', 'reverse', 'strafe_left', 'strafe_right',
                      'rotate_left', 'rotate_right', 'stop']:
            self.get_logger().info(f'  Testing: {state}')
            self._set_state(state)
            time.sleep(3)
        self.get_logger().info('LED test sequence complete')

    # ── Low-level strip control ───────────────────────────────────────────────

    def update_strip(self, led_array):
        if len(led_array) != self.num_leds:
            self.get_logger().warn(
                f'LED array size mismatch: expected {self.num_leds}, got {len(led_array)}')
            return
        corrected = [(g, r, b) for (r, g, b) in led_array]   # WS2812 GRB order
        ws2812.write2812(self.spi, np.array(corrected, dtype=np.uint8).flatten())

    def set_leds(self, color):
        data = [val for c in [self.encode_color(color)] * LED_COUNT for val in c]
        self.spi.xfer2(data)

    def encode_color(self, color):
        return [
            (color[1] >> 1) | 0x80,   # Green
            (color[0] >> 1) | 0x80,   # Red
            (color[2] >> 1) | 0x80,   # Blue
        ]

    def destroy_node(self):
        self._stop_event.set()
        time.sleep(0.1)
        self.update_strip([(0, 0, 0)] * self.num_leds)
        self.spi.close()
        self.get_logger().info('LED Controller Node Stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LEDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LED Controller Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
