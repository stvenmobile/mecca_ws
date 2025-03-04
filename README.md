# Mecca_WS - ROS2 Jazzy Mecanum Wheels Robot

This is a **ROS2 Jazzy** project supporting a **Mecanum Wheels** robot.

## Hardware Components

The hardware for the robot consists of:

| #  | Item                                              |
|----|--------------------------------------------------|
| 1  | Yahboom YB-ERF01-V3.0 STM32 robot controller board |
| 1  | Raspberry Pi 5 running Ubuntu 24.04 and ROS2 Jazzy |
| 1  | VL53L1X Range Sensor                             |
| 1  | WS2812 Strip RGB (7 LEDs)                        |
| 1  | 12V Volt Meter for battery status indication     |
| 1  | 11.1V 3S LiPo Battery                            |
| 4  | JGB37-520 12V 205RPM Motors with Encoders       |

## System Overview

- The **STM32 controller board** listens on the serial port for motor commands from the Raspberry Pi.
- The STM32 also uses a **PID algorithm** to maintain precise speeds for accurate navigation.
- The **Raspberry Pi** accepts joystick commands for movement but **filters user inputs** with safety overrides based on data from the VL53L1X range sensor for **obstacle avoidance**.

## Chassis and Customizations

- The **aluminum chassis and mecanum wheels** were obtained from HiWonder as part of this kit:
  - *Hiwonder Large Metal 4WD Vehicle Chassis for Arduino/Raspberry Pi/ROS Robot with 12V Encoder Geared Motor*
- The motors were later replaced with **Yahboom motors** for better integration with the STM32 board.
- Additional **3D-printed components** were added to extend the chassis and simplify electronics mounting.

## ROS Nodes

| Node Name              | Function                                               |
|------------------------|-------------------------------------------------------|
| `Mecca_Driver_Node`    | Publishes motor commands                              |
| `Serial_Comm`          | Handles serial communication between Raspberry Pi & STM32 |
| `LED_Controller_Node`  | Controls the LED strip with different patterns based on movement |
| `VL53L1X_Sensor`       | Publishes distance to objects in front of the robot  |
| `Navigator_Node`       | Overrides motor commands when an obstacle is detected |

---

## Future Enhancements

- **SLAM Navigation:** Implementing mapping and autonomous movement.
- **Voice Commands:** Exploring ESP32-based voice control for movement.
- **Camera Integration:** Adding real-time video streaming.

---

### How to Use

1. **Power on the robot** and ensure the STM32 controller is connected to the Raspberry Pi.
2. **Run the ROS2 launch file** to start all necessary nodes:
   ```bash
   ros2 launch mecca_ws mecca_bringup.launch.py
**Use a joystick to control the robot, or allow it to navigate autonomously.
**The LED strip will reflect motion state changes with different light effects.
