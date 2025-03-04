This is a Ros2 Jazzy project supporting a Mecanum Wheels robot. 
The hardware for the robot consists of:
	| Quantity    | Item
	| 1 					| Yahboom YB-ERF01-V3.0 STM32 robot controller board 
	| 1						| Raspberry Pi 5 running Ubuntu 24.04 and ROS2 Jazzy
	| 1						| VL53L1X Range Sensor
	| 1						| WS2812 Strip RGB (7 LED's)
	| 1						| 12v Volt Meter for battery status indication
	| 1						| 11.1v 3S LiPo Battery

The STM32 controller board listens on Serial port for motor commands coming from the Raspberry Pi.
The Raspberry Pi accepts basic mecanum wheel movement commands (including left/right strafing)
from Joystick, but also filters user commands with safety overrides base don data from the
VL53L1X ranges sensor for obstacle avoidance.

The aluminum chassis and mecanum wheels  were obtained from HiWonder in this  kit:
Hiwonder Large Metal 4WD Vehicle Chassis for Arduino/Raspberry Pi/ROS Robot with 12V Encoder Geared Motor
However I later replaced the motors with ones from Yahboom to more easily integrate with the 
STM32 robot controller board. I added additional 3-d printed components to extend the chassis and
simplify mounting for the electronics.

The ROS Nodes included in the project are:
	|  Node										|  Function
	|  Mecca_Driver_Node			|  Publishes Motor_Commands
	|  Serial_Comm						|  Serial COmmunications
	|  LED_Controller_Node		|  Controls LED Strip with different patterns based on motor movements
	|  VL53L1X_Sensor					|  Publishes Distance to Objects (in front of the robot)
	|  Navigator_Node					|  Overrides motor commands when an obstacle is detected 
