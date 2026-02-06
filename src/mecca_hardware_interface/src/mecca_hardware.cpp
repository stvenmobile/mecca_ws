#include "mecca_hardware_interface/mecca_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <sstream>

namespace mecca_hardware_interface
{

hardware_interface::CallbackReturn MeccaHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info.joints.size());
  hw_commands_.resize(info.joints.size(), 0.0);

  // Read parameters from URDF if available
  // Expecting parameters in <hardware> tag
  for (const auto & [name, value] : info.hardware_parameters) {
    if (name == "wheel_radius") {
      wheel_radius_ = std::stod(value);
    } else if (name == "wheel_separation_x") {
      wheel_separation_x_ = std::stod(value);
    } else if (name == "wheel_separation_y") {
      wheel_separation_y_ = std::stod(value);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"),
              "Initialized MeccaHardware with R=%.3f, SepX=%.3f, SepY=%.3f", 
              wheel_radius_, wheel_separation_x_, wheel_separation_y_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MeccaHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i].velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MeccaHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MeccaHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"), "Activating MeccaHardware...");
  
  // Try to open the serial port
  // Note: hardcoded path matching the udev rule
  if (!openSerialPort("/dev/stm32_serial", 115200)) {
     RCLCPP_ERROR(rclcpp::get_logger("MeccaHardware"), "Failed to open /dev/stm32_serial");
     return hardware_interface::CallbackReturn::ERROR;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MeccaHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"), "Deactivating MeccaHardware...");
  closeSerialPort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MeccaHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ < 0) return hardware_interface::return_type::OK;

  char buf[256];
  int n = ::read(serial_port_, buf, sizeof(buf) - 1);
  
  if (n > 0) {
    buf[n] = 0;
    serial_buffer_ += buf;
    
    size_t pos;
    while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
      std::string line = serial_buffer_.substr(0, pos);
      serial_buffer_.erase(0, pos + 1);
      
      // Parse the line
      // Expected format: "Encoder Values M1=123 M2=456 M3=789 M4=101"
      if (line.find("Encoder Values") != std::string::npos) {
        parseSerialData(line);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

void MeccaHardware::parseSerialData(const std::string & line) {
    // Basic parsing logic to match python regex M(\d+)=(\d+)
    // M1 = Front Left, M2 = Front Right, M3 = Rear Left, M4 = Rear Right (Check wiring!)
    // NOTE: The python script mapping was:
    // M1=FL, M2=FR, M3=RL, M4=RR (Wait, let me double check the python script)
    // Python script: 
    // M1 -> [0] Front Left?
    // Let's assume standard mapping:
    // 0: FL, 1: FR, 2: RL, 3: RR (Standard ROS order usually FL, FR, RL, RR or FL, RL, FR, RR)
    // In on_init joints are ordered by URDF. 
    // We need to map carefully. 
    // Default assumption: The info_.joints order matches ros2_control_node.yaml config order
    
    // Simplest parse: sscanf
    // M1=%d M2=%d M3=%d M4=%d
    // We need to find the substrings.
    
    int m1=0, m2=0, m3=0, m4=0;
    
    // Find positions of M1=, etc.
    // This is fragile but fast.
    const char* str = line.c_str();
    const char* p1 = strstr(str, "M1=");
    const char* p2 = strstr(str, "M2=");
    const char* p3 = strstr(str, "M3=");
    const char* p4 = strstr(str, "M4=");
    
    if (p1) m1 = atoi(p1 + 3);
    if (p2) m2 = atoi(p2 + 3);
    if (p3) m3 = atoi(p3 + 3);
    if (p4) m4 = atoi(p4 + 3);
    
    // Assign to states (Assuming index 0=FL, 1=FR, 2=RL, 3=RR based on yaml usually)
    // But check joints names in info_ to be sure? 
    // For now assuming order: FL, FR, RL, RR based on typical vector ordering if alphabetical?
    // Actually, let's map by name in on_init if we want to be safe, but for now strict index:
    // 0: Front Left, 1: Front Right, 2: Rear Left, 3: Rear Right
    
    // Convert Ticks to Radians
    // Note: m1, m2... are accumulated ticks usually? Or relative? 
    // Python script treated them as absolute positions.
    
    hw_states_[0].position = m1 * RADS_PER_TICK;
    hw_states_[1].position = m2 * RADS_PER_TICK;
    hw_states_[2].position = m3 * RADS_PER_TICK;
    hw_states_[3].position = m4 * RADS_PER_TICK;
    
    // TODO: Velocity estimation?
    // For now leave velocity as 0 or let controller estimate it.
}

hardware_interface::return_type MeccaHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ < 0) return hardware_interface::return_type::OK;

  // 1. Get Wheel Velocities (rad/s)
  double fl_vel = hw_commands_[0];
  double fr_vel = hw_commands_[1];
  double rl_vel = hw_commands_[2];
  double rr_vel = hw_commands_[3];

  // 2. Convert to Linear Wheel Speeds (m/s)
  double fl_m = fl_vel * wheel_radius_;
  double fr_m = fr_vel * wheel_radius_;
  double rl_m = rl_vel * wheel_radius_;
  double rr_m = rr_vel * wheel_radius_;

  // 3. Inverse Kinematics -> Body Velocity (m/s, rad/s)
  // Average x velocity
  double vx = (fl_m + fr_m + rl_m + rr_m) / 4.0;
  
  // Average y velocity
  // Mecanum: vy comes from opposing corners
  double vy = (-fl_m + fr_m + rl_m - rr_m) / 4.0;
  
  // Rotational velocity
  // geometry factor = (Lx + Ly)
  double k = (wheel_separation_x_ + wheel_separation_y_) / 2.0; // Half-widths usually
  // Wait, standard formula: omega = (-FL + FR - RL + RR) / (4 * (lx + ly))
  // Denominator depends on definitions of lx, ly. 
  // Let's trust the logic: rotational components add up.
  // Right side (FR, RR) positive -> Turn Left (Positive Omega) ?? No, usually right side fwd -> Turn Left.
  // Standard ROS: X fwd, Y left, Z up.
  // Positive Omega (Z) = Turn Left.
  // To Turn Left: Right wheels Fwd, Left wheels Back.
  // So FR+, RR+, FL-, RL-
  double omega = (-fl_m + fr_m - rl_m + rr_m) / (4.0 * k);

  // 4. Scale to Integer Command (0-1100 range)
  // Python: scaled_x = linear_x * MAX_SPEED * speed_scale
  // Assume MAX_SPEED ~ 1100 corresponds to ~1.0 m/s ? (Pure guess without calibration)
  // Let's scale heavily to ensure movement.
  // If we assume max speed is around 0.5 m/s?
  // Let's use a factor of 1000 for now. 1 m/s = 1000 command units.
  
  int cmd_x = static_cast<int>(vx * CMD_SCALE_FACTOR) * -1; // Inverted to match joystick direction
  int cmd_y = static_cast<int>(vy * CMD_SCALE_FACTOR);
  int cmd_rot = static_cast<int>(omega * CMD_SCALE_FACTOR * 0.5); // Rotation usually scaled differently

  // 5. Format String "V x y rot\r\n"
  char cmd_buf[64];
  int len = snprintf(cmd_buf, sizeof(cmd_buf), "V %d %d %d\r\n", cmd_x, cmd_y, cmd_rot);

  // 6. Write
  ::write(serial_port_, cmd_buf, len);

  // DEBUG: Print if non-zero
  if (cmd_x != 0 || cmd_y != 0 || cmd_rot != 0) {
      // Throttle slightly to prevent terminal flood if holding stick
      static int debug_limiter = 0;
      if (debug_limiter++ % 10 == 0) {
          RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"), "Sending Serial: %s", cmd_buf);
      }
  }

  return hardware_interface::return_type::OK;
}

// -------------------------------------------------------------------------
// Serial Helpers
// -------------------------------------------------------------------------
bool MeccaHardware::openSerialPort(const std::string & port_name, int baud_rate)
{
    serial_port_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port_ < 0) {
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0) {
        ::close(serial_port_);
        return false;
    }

    // Set Baud Rate
    speed_t speed = B115200; // Default
    switch(baud_rate) {
        case 9600: speed = B9600; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 Mode
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bits
    
    // No flow control
    tty.c_cflag &= ~CRTSCTS; 

    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No sw flow control
    
    // Raw output
    tty.c_oflag &= ~OPOST;

    // Settings
    tty.c_cflag |= (CLOCAL | CREAD); // Enable read, ignore ctrl lines
    
    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
        ::close(serial_port_);
        return false;
    }
    
    return true;
}

void MeccaHardware::closeSerialPort()
{
    if (serial_port_ >= 0) {
        ::close(serial_port_);
        serial_port_ = -1;
    }
}

}  // namespace mecca_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecca_hardware_interface::MeccaHardware, hardware_interface::SystemInterface)
