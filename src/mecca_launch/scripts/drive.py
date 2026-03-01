#!/usr/bin/env python3
"""
drive.py — Timestamped velocity publisher with live wheel feedback for PID tuning.

Usage:
    python3 drive.py [vx] [vy] [vz] [duration]

Arguments:
    vx        Forward/backward velocity in m/s  (default 0.0)
    vy        Strafe left/right velocity in m/s  (default 0.0)
    vz        Rotation in rad/s, CCW positive    (default 0.0)
    duration  Run time in seconds; 0 = until Ctrl+C  (default 3)

Examples:
    python3 drive.py 0.4              # forward 0.4 m/s for 3 s
    python3 drive.py 0.4 0 0 5       # forward 0.4 m/s for 5 s
    python3 drive.py 0 0.3           # strafe left 0.3 m/s for 3 s
    python3 drive.py 0 0 0.5         # rotate CCW 0.5 rad/s for 3 s
    python3 drive.py 0 0 0 0         # stop and hold (Ctrl+C to exit)

NOTE: PID gains are set by reflashing the STM32 (bsp_pid.h — PID_K*_DEFAULT).
NOTE: kill joy/teleop nodes before running to avoid interference:
    ros2 node kill /joy_node /teleop_twist_joy_node
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

# Must match mecca_hardware.hpp and controllers.yaml
WHEEL_RADIUS   = 0.0485  # m  (97 mm diameter marked on wheel)
WHEEL_SEP_X    = 0.175   # m  wheelbase
WHEEL_SEP_Y    = 0.175   # m  track width
K              = (WHEEL_SEP_X + WHEEL_SEP_Y) / 2.0   # 0.175 m

# Joint names as exported by the hardware interface (URDF order)
JOINT_NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]
LABELS = ['FL', 'FR', 'RL', 'RR']


def inverse_kinematics(vx, vy, vz):
    """Robot body velocity → per-wheel surface speed (m/s)."""
    fl = vx - vy - vz * K
    fr = vx + vy + vz * K
    rl = vx + vy - vz * K
    rr = vx - vy + vz * K
    return [fl, fr, rl, rr]


class DriveNode(Node):

    def __init__(self, vx, vy, vz, duration):
        super().__init__('drive_pub')

        self.vx       = vx
        self.vy       = vy
        self.vz       = vz
        self.duration = duration   # 0 = indefinite

        # Per-wheel speed targets in mm/s (what the STM32 PID is aiming for)
        targets_ms       = inverse_kinematics(vx, vy, vz)
        self.targets_mms = [v * 1000.0 for v in targets_ms]

        # V command the hardware interface sends to STM32.
        # write() does IK→FK round-trip; result equals the original body twist.
        self.vcmd_x   = int(vx * 1000.0)    # mm/s
        self.vcmd_y   = int(vy * 1000.0)    # mm/s
        self.vcmd_rot = int(vz * 1000.0)    # mrad/s

        self.actual_mms  = [0.0] * 4   # mm/s, indexed FL FR RL RR
        self.name_to_idx = {name: i for i, name in enumerate(JOINT_NAMES)}

        self.start_time    = None
        self.print_counter = 0
        self.done          = False

        self.pub = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/reference', 10)

        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        self.create_timer(0.033, self._loop)   # 30 Hz publish + feedback

        # ── Header ──────────────────────────────────────────────────────────
        sep = '-' * 88
        print(f'\nCmd: vx={vx:+.3f} m/s  vy={vy:+.3f} m/s  vz={vz:+.3f} rad/s  '
              f'duration={"inf" if duration == 0 else f"{duration}s"}')
        print(f'V-cmd to STM32 (est.): V {self.vcmd_x:+d} {self.vcmd_y:+d} {self.vcmd_rot:+d}'
              f'  (mm/s  mm/s  mrad/s)')
        tgt = '  '.join(f'{l}={v:+.0f}' for l, v in zip(LABELS, self.targets_mms))
        print(f'Per-wheel target (mm/s):  {tgt}')
        print(sep)
        print(f'{"t(s)":>5}  '
              f'{"Act FL":>7} {"Act FR":>7} {"Act RL":>7} {"Act RR":>7}  '
              f'{"Err FL":>7} {"Err FR":>7} {"Err RL":>7} {"Err RR":>7}')
        print(sep)

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            idx = self.name_to_idx.get(name)
            if idx is not None and i < len(msg.velocity):
                # rad/s → mm/s
                self.actual_mms[idx] = msg.velocity[i] * WHEEL_RADIUS * 1000.0

    def _loop(self):
        if self.done:
            return

        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.duration > 0 and elapsed >= self.duration:
            self._stop()
            return

        # Publish stamped command
        msg = TwistStamped()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = self.vx
        msg.twist.linear.y  = self.vy
        msg.twist.angular.z = self.vz
        self.pub.publish(msg)

        # Print feedback every 15 cycles ≈ 500 ms (at 30 Hz)
        self.print_counter += 1
        if self.print_counter >= 15:
            self.print_counter = 0
            a   = self.actual_mms
            err = [a[i] - self.targets_mms[i] for i in range(4)]
            print(f'{elapsed:>5.1f}  '
                  f'{a[0]:>+7.0f} {a[1]:>+7.0f} {a[2]:>+7.0f} {a[3]:>+7.0f}  '
                  f'{err[0]:>+7.0f} {err[1]:>+7.0f} {err[2]:>+7.0f} {err[3]:>+7.0f}')

    def _stop(self):
        self.done = True
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # All zeros = stop
        self.pub.publish(msg)
        print('-' * 88)
        print('Stopped. (STM32 watchdog will also brake within 1 s)')
        rclpy.shutdown()


def main():
    vx       = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    vy       = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    vz       = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    duration = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0

    rclpy.init()
    node = DriveNode(vx, vy, vz, duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.done:
            node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
