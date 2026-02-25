#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <vector>
#include <string>
#include <cmath>

namespace mecca_hardware_interface
{

class MeccaHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial Port Methods
  bool openSerialPort(const std::string & port_name, int baud_rate);
  void closeSerialPort();
  void parseSerialData(const std::string & data);

  // Data Members
  struct JointState {
    double position = 0.0;
    double velocity = 0.0;
  };

  // Needed for velocity calcs
  rclcpp::Time last_timestamp_;
  std::vector<double> last_positions_;
  bool first_read_ = true;
    

  std::vector<JointState> hw_states_;
  std::vector<double> hw_commands_;

  // Serial Port Handle
  int serial_port_ = -1;
  std::string serial_buffer_;
  
  // Robot Parameters (Defaults, overwritten by info)
  // NOTE: These need to match the python script for consistency
  // wheel_radius = 0.048, wheel_base = 0.175, track_width = 0.175
  double wheel_radius_ = 0.048;
  double wheel_separation_x_ = 0.175; // Wheel base
  double wheel_separation_y_ = 0.175; // Track width
  
  // Encoder Params
  const double TICKS_PER_REV = 650.0;
  const double RADS_PER_TICK = (2.0 * M_PI) / TICKS_PER_REV;
  
  // Command Scaling (From Python Script)
  // The STM32 expects integers in mm/s for PID control.
  // ROS uses m/s.
  // 1 m/s = 1000 mm/s.
  const double CMD_SCALE_FACTOR = 1000.0; // Corrected to 1000 to match mm/s
};

}  // namespace mecca_hardware_interface
