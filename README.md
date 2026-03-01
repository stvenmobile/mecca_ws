##  <a name='Mecca_WS-ROS2JazzyMecanumWheelsRobot'></a> Mecca_WS - ROS2 Jazzy Mecanum Wheels Robot <!-- omit in toc -->

This is a **ROS2 Jazzy** project supporting a **Mecanum Wheels** robot.

![  Mecca the Robot  ](images/mecca_1.png)

<p align="center">Mecca the Robot - fully assembled!</p>

---
### **Table of Contents**
<!-- TOC -->
- [System Overview](#system-overview)
- [Hardware Components](#hardware-components)
- [Chassis and Construction](#chassis-and-construction)
- [Wiring](#wiring)
- [Configuration Settings](#configuration-settings)
- [Software Notes](#software-notes)
- [How to Use](#how-to-use)
- [Future Enhancements](#future-enhancements)

---
## <a name='SystemOverview'></a>System Overview

Mecca operates in two distinct modes depending on the use case:

**Joystick Teleop Mode** — Direct serial control, no ros2_control overhead:
- Joystick input → `teleop_twist_joy` → `teleop_serial_bridge_node` → `simple_serial_node` → STM32
- Responsive, no cmd_vel timeout issues, LED strip reflects motion state

**SLAM / Autonomous Nav Mode** — Full ros2_control stack for Nav2 integration:
- `ros2_control` + `mecanum_drive_controller` manages wheel velocity commands
- Hardware interface (`mecca_hardware_interface`) handles serial I/O at 30 Hz
- Alternates velocity commands (`V vx vy vz`) with encoder polls (`I ENC`) each cycle
- `open_loop: true` — velocity commanded from reference, odometry integration planned

The **STM32 controller board** listens on the serial port for motor commands from the Raspberry Pi and uses a **PID algorithm** to maintain precise wheel speeds.

---
## <a name='HardwareComponents'></a>Hardware Components <!-- omit in toc -->

| #  | Item                                                         |
|----|--------------------------------------------------------------|
| 1  | HiWonder Large Metal 4WD Vehicle Chassis with Mecanum Wheels |
| 1  | Yahboom YB-ERF01-V3.0 STM32 robot controller                |
| 1  | Raspberry Pi 5 running Ubuntu 24.04 and ROS2 Jazzy           |
| 4  | JGB37-520 12V 205RPM Motors with Encoders                    |
| 1  | SLLIDAR A1 (LIDAR for SLAM)                                  |
| 1  | VL53L1X Range Sensor (ToF, future obstacle avoidance)        |
| 1  | WS2812 Strip RGB (7 LEDs)                                    |
| 1  | 8BitDo Ultimate C 2.4G Wireless Controller with USB dongle   |
| 1  | 12V Volt Meter for battery status indication                 |
| 1  | 11.1V 3S LiPo Battery                                        |
| 1  | Double Pole Double Throw rocker switch                       |
| 1  | Panel mount banana jack for bench power supply input         |
| 1  | USB-A to USB-C cable (STM32 to Raspberry Pi)                 |

---
## <a name='ChassisandConstruction'></a>Chassis and Construction
- The **aluminum chassis and mecanum wheels** were obtained from HiWonder as part of this kit:
  - [Hiwonder Large Metal 4WD Vehicle Chassis](https://www.hiwonder.com/products/large-metal-4wd-vehicle-chassis-green)
- The motors were later replaced with **Yahboom JGB37-520 motors** for better integration with the STM32 board.
  - [Yahboom JGB37-520 Motors](https://category.yahboom.net/products/md520)
- Additional **3D-printed components** were added to extend the chassis to look like a truck cab and provide mounting for additional components.

---
## <a name='Wiring'></a>Wiring

### <a name='YB-ERF01-V3-0-Controller-Wiring'></a>YB-ERF01-V3.0 Controller Wiring <!-- omit in toc -->

![Controller Board ](images/YB-ERF01-V3.0.png)

| Reference | Connection |
|-----------|------------------------------------------------|
| 1. | The Deans connector is connected to the center poles of the DPDT Rocker Switch. |
| 4. | The USB-C connector is connected to one of the USB-A ports on the Raspberry Pi. |
| 5. | The USB-C connector on the controller board is connected to the USB-C power input jack on the Raspberry Pi 5. |
| 6. | The 5V barrel jack output is connected to the Vin and GND for the WS2812 RGB LED strip. |
| 10. | The power switch on the controller board is left in the ON position. |
| 20. | The four motor/encoders are connected via 6-conductor cables to the four motor jacks as shown below. |

![Motor Orientation](images/motors.png)

---

###  <a name='VL53L1XTime-of-FlightToFSensorWiringI2C'></a> VL53L1X Time-of-Flight (ToF) Sensor Wiring (I2C) <!-- omit in toc -->
The **VL53L1X ToF sensor** communicates via **I2C** and is connected as follows:

| **VL53L1X Pin** | **Raspberry Pi 5 Pin** | **Function** |
|---------------|------------------|------------|
| **VCC** | Pin **1** (3.3V) or Pin **2/4** (5V) | Power Supply |
| **GND** | Pin **6** (GND) | Ground |
| **SDA** | Pin **3** (GPIO2 - I2C SDA) | I2C Data |
| **SCL** | Pin **5** (GPIO3 - I2C SCL) | I2C Clock |
| **XSHUT** *(optional)* | Any GPIO (e.g., GPIO17 - Pin 11) | Enable/Disable Sensor |
| **GPIO1 (Interrupt)** *(optional)* | Any GPIO (e.g., GPIO27 - Pin 13) | Interrupt Signal |

---
###  <a name='WS28127-LightRGBLEDStripWiringSPI'></a> WS2812 7-Light RGB LED Strip Wiring (SPI)  <!-- omit in toc -->
The **WS2812 LED strip** is controlled via **SPI Bus 0**:

| **WS2812 Pin** | **Raspberry Pi 5 Pin** | **Function** |
|---------------|------------------|------------|
| **VCC** | Pin **2** or **4** (5V) | Power Supply |
| **GND** | Pin **6** (GND) | Ground |
| **DIN** (Data In) | Pin **19** (GPIO10 - SPI0 MOSI) | SPI Data Line |

---

Here is a pictorial diagram of the wiring to the Raspberry Pi 5:
![Wiring Diagram](images/fritzing.png)

---

## <a name='ConfigurationSettings'></a>Configuration Settings

### I2C/SPI
If I2C or SPI is disabled, enable them via:
```bash
sudo raspi-config
```
Navigate to Interface Options > I2C or SPI > Enable.

### USB Device Naming
The robot uses two USB serial devices. Udev rules create stable symlinks so device order doesn't matter:

```bash
cd ~/mecca_ws/src/mecca_launch/utilities/udev
./install_udev_rules.sh
sudo reboot
```

| **Physical Device** | **Stable Symlink**  | **Purpose**                        |
|---------------------|---------------------|------------------------------------|
| STM32 Controller    | `/dev/stm32_serial` | Motor control and encoder feedback |
| SLLIDAR A1          | `/dev/lidar_serial` | Laser range scanning               |

### 8BitDo Controller USB Autosuspend Fix
The 8BitDo USB dongle uses 2.4GHz radio between the controller and dongle. Linux USB autosuspend can cause brief disconnects that interrupt driving. Disable it permanently:

```bash
sudo tee /etc/udev/rules.d/99-8bitdo-no-autosuspend.rules << 'EOF'
# Disable USB autosuspend for 8BitDo Ultimate C 2.4G controller
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2dc8", ATTR{idProduct}=="3106", TEST=="power/control", ATTR{power/control}="on"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---
## <a name='SoftwareNotes'></a>Software Notes

The STM32 firmware is maintained in a companion repository:
**https://github.com/stvenmobile/Car_Motion.git**

### ROS2 Nodes

| Node                        | Package                    | Function                                                        |
|-----------------------------|----------------------------|-----------------------------------------------------------------|
| `robot_state_publisher`     | robot_state_publisher      | Publishes URDF transforms                                       |
| `ros2_control_node`         | controller_manager         | Manages hardware interface (SLAM/Nav mode only)                 |
| `mecanum_drive_controller`  | mecanum_drive_controller   | Converts twist commands to wheel velocities (SLAM/Nav mode)     |
| `joint_broad`               | joint_state_broadcaster    | Publishes joint states (SLAM/Nav mode)                          |
| `simple_serial_node`        | mecca_driver_node          | Serial bridge — String commands to/from `/dev/stm32_serial`     |
| `teleop_serial_bridge`      | mecca_driver_node          | Converts `/cmd_vel` Twist to V serial commands at 20 Hz         |
| `led_controller`            | mecca_driver_node          | WS2812 LED animations driven by motion state                    |
| `joy_node`                  | joy                        | Reads joystick hardware                                         |
| `teleop_twist_joy_node`     | teleop_twist_joy           | Maps joystick axes to Twist velocity commands                   |

### Serial Protocol (ROS → STM32)

| Command         | Format                              | Notes                          |
|-----------------|-------------------------------------|--------------------------------|
| Velocity        | `V <vx> <vy> <vz>\r\n`             | mm/s, mm/s, mrad/s             |
| Encoder poll    | `I ENC\r\n`                         | STM32 replies with tick counts |
| Encoder reply   | `I ENC <m1> <m2> <m3> <m4>`        | M1=FL, M2=RL, M3=FR, M4=RR    |
| PID tune        | `P <Kp> <Ki> <Kd>\r\n`             | Live tuning via drive.py       |

### PID Live Tuning (no firmware reflash needed)
Edit `PID_KP`, `PID_KI`, `PID_KD` at the top of `src/mecca_launch/scripts/drive.py`. Values are sent to the STM32 at startup of each run. Once confirmed, update defaults in `bsp_pid.h` and reflash.

---
## <a name='HowtoUse'></a>How to Use

### Joystick Teleop Mode
Direct serial control — use this for manual driving. No ros2_control overhead.

```bash
ros2 launch mecca_launch joystick_teleop.launch.py
# Without LEDs:
ros2 launch mecca_launch joystick_teleop.launch.py with_led:=false
```

![Joystick Controls](images/joystick_controls.png)

| Input                        | Action                    |
|------------------------------|---------------------------|
| Hold **RB** (Button 5)       | Enable movement (dead-man switch) |
| Left Stick Up/Down           | Forward / Reverse         |
| Left Stick Left/Right        | Strafe Left / Right       |
| Right Stick Left/Right       | Rotate Left / Right       |
| Hold **RB + A** (Button 0)   | Turbo speed               |

### SLAM / Autonomous Navigation Mode
Full ros2_control stack. Use this for Nav2, SLAM, or headless velocity testing with `drive.py`.

```bash
ros2 launch mecca_launch mecca_bringup.launch.py with_joy:=false
# With LED controller:
ros2 launch mecca_launch mecca_bringup.launch.py with_joy:=false with_led:=true
```

### Headless Velocity Testing (drive.py)
Use with SLAM mode running. Sends direct velocity commands and reports encoder feedback.

```bash
python3 src/mecca_launch/scripts/drive.py [vx] [vy] [vz] [duration]
# Examples:
python3 drive.py 0.4 0 0 5    # Forward 0.4 m/s for 5 seconds
python3 drive.py 0 0.4 0 5    # Strafe left 0.4 m/s for 5 seconds
```

---
## <a name='FutureEnhancements'></a>Future Enhancements
- **SLAM Navigation:** Implementing mapping and autonomous movement with Nav2 (in progress)
- **Closed-loop Odometry:** Enable encoder feedback into mecanum_drive_controller
- **Obstacle Avoidance:** VL53L1X ToF sensor integration with navigator_node
- **Voice Commands:** Exploring ESP32-based voice control for movement
- **Camera Integration:** Adding real-time video streaming

---
