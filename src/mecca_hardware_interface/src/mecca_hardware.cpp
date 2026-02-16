#include "mecca_hardware_interface/mecca_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <errno.h> 
#include <string.h> 
#include <iomanip>

namespace mecca_hardware_interface
{

const std::string HARDWARE_VERSION = "3.0.0";

hardware_interface::CallbackReturn MeccaHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // In newer Jazzy, we don't call the base class init manually. 
  // The framework handles it. We just store the info and proceed.
  this->info_ = info; 

  hw_states_.resize(info_.joints.size());
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const auto & [name, value] : info_.hardware_parameters) {
    if (name == "wheel_radius") wheel_radius_ = std::stod(value);
    else if (name == "wheel_separation_x") wheel_separation_x_ = std::stod(value);
    else if (name == "wheel_separation_y") wheel_separation_y_ = std::stod(value);
  }

  RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"), "MeccaHardware Version: %s (Jazzy Framework Init)", HARDWARE_VERSION.c_str());
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
  RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"), "Activating hardware version %s...", HARDWARE_VERSION.c_str());
  if (!openSerialPort("/dev/stm32_serial", 115200)) return hardware_interface::CallbackReturn::ERROR;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MeccaHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  closeSerialPort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MeccaHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ < 0) return hardware_interface::return_type::OK;

  static int req_cnt = 0;
  if (req_cnt++ % 10 == 0) {
    const char* req = "I ENC\r\n";
    ::write(serial_port_, req, strlen(req));
  }

  char buf[512];
  int n = ::read(serial_port_, buf, sizeof(buf) - 1);
  
  if (n > 0) {
    buf[n] = 0;
    serial_buffer_ += buf;
    
    size_t pos;
    while ((pos = serial_buffer_.find_first_of("\r\n")) != std::string::npos) {
      std::string line = serial_buffer_.substr(0, pos);
      serial_buffer_.erase(0, pos + 1);

      if (!line.empty() && line.find("M1=") != std::string::npos) {
        parseSerialData(line);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

void MeccaHardware::parseSerialData(const std::string & line) {
    int m1=0, m2=0, m3=0, m4=0;
    const char* str = line.c_str();
    
    const char *p1 = strstr(str, "M1="), *p2 = strstr(str, "M2="), *p3 = strstr(str, "M3="), *p4 = strstr(str, "M4=");
    
    if (p1) m1 = atoi(p1 + 3);
    if (p2) m2 = atoi(p2 + 3);
    if (p3) m3 = atoi(p3 + 3);
    if (p4) m4 = atoi(p4 + 3);
    
    rclcpp::Time current_time = rclcpp::Clock().now();
    
    if (first_read_) {
        last_positions_.resize(4, 0.0);
        last_timestamp_ = current_time;
        first_read_ = false;
    }

    double dt = (current_time - last_timestamp_).seconds();
    
    double current_pos[4];
    current_pos[0] = m4 * RADS_PER_TICK; // FL
    current_pos[1] = m2 * RADS_PER_TICK; // FR
    current_pos[2] = m3 * RADS_PER_TICK; // RL
    current_pos[3] = m1 * RADS_PER_TICK; // RR

    if (dt > 0.0) {
        for (size_t i = 0; i < 4; i++) {
            hw_states_[i].velocity = (current_pos[i] - last_positions_[i]) / dt;
            hw_states_[i].position = current_pos[i];
            last_positions_[i] = current_pos[i];
        }
    }

    last_timestamp_ = current_time;
}

hardware_interface::return_type MeccaHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ < 0) return hardware_interface::return_type::OK;

  double fl_m = hw_commands_[0] * wheel_radius_;
  double fr_m = hw_commands_[1] * wheel_radius_;
  double rl_m = hw_commands_[2] * wheel_radius_;
  double rr_m = hw_commands_[3] * wheel_radius_;

  double vx = (fl_m + fr_m + rl_m + rr_m) / 4.0;
  double vy = (-fl_m + fr_m + rl_m - rr_m) / 4.0;
  double k = (wheel_separation_x_ + wheel_separation_y_) / 2.0; 
  double omega = (-fl_m + fr_m - rl_m + rr_m) / (4.0 * k);

  int cmd_x = static_cast<int>(vx * CMD_SCALE_FACTOR) * -1; 
  int cmd_y = static_cast<int>(vy * CMD_SCALE_FACTOR);
  int cmd_rot = static_cast<int>(omega * CMD_SCALE_FACTOR * 0.5) * -1;

  char cmd_buf[64];
  int len = snprintf(cmd_buf, sizeof(cmd_buf), "V %d %d %d\r\n", cmd_x, cmd_y, cmd_rot);
  ::write(serial_port_, cmd_buf, len);

  return hardware_interface::return_type::OK;
}

bool MeccaHardware::openSerialPort(const std::string & port_name, int baud_rate)
{
    serial_port_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); 
    if (serial_port_ < 0) return false;

    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0) {
        ::close(serial_port_);
        return false;
    }

    cfmakeraw(&tty);
    
    speed_t speed = B115200;
    if (baud_rate == 9600) speed = B9600;
    else if (baud_rate == 57600) speed = B57600;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);
    
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0; 

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
        ::close(serial_port_);
        return false;
    }

    tcflush(serial_port_, TCIOFLUSH);
    return true;
}

void MeccaHardware::closeSerialPort() { if (serial_port_ >= 0) ::close(serial_port_); serial_port_ = -1; }

}  // namespace mecca_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecca_hardware_interface::MeccaHardware, hardware_interface::SystemInterface)