This is a Ros2 Jazzy project supporting a Mecanum Wheels robot. 
The hardware for the robot consists of:
  # Yahboom YB-ERF01-V3.0 STM32 robot controller board 
  # Raspberry Pi 5 running Ubuntu 24.04 and ROS2 Jazzy
  # VL53L1X Range Sensor
  # WS2812 Strip RGB (7 LED's) 
  # 11.1v 3S LiPo Battery.
  # 12v Volt Meter for battery status indication
The STM32 controller board listens on Serial port for motor commands coming from the Raspberry Pi.
The Raspberry Pi accepts basic mecanum wheel movement commands (including left/right strafing)
from Joystick, but also filters user commands with safety overrides base don data from the
VL53L1X ranges sensor for obstacle avoidance.
