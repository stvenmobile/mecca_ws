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

  std::vector<JointState> hw_states_;
  std::vector<double> hw_commands_;

  // Serial Port Handle
  int serial_port_ = -1;
  std::string serial_buffer_;
  
  // Robot Parameters (Defaults, overwritten by info)
  // NOTE: These need to match the python script for consistency
  // wheel_radius = 0.05, wheel_base = 0.175, track_width = 0.175
  double wheel_radius_ = 0.05;
  double wheel_separation_x_ = 0.175; // Wheel base
  double wheel_separation_y_ = 0.175; // Track width
  
  // Encoder Params
  const double TICKS_PER_REV = 2420.0;
  const double RADS_PER_TICK = (2.0 * M_PI) / TICKS_PER_REV;
  
  // Command Scaling (From Python Script)
  // The STM32 expects integers around 0-1100 range roughly corresponding to mm/s or raw PWM?
  // Python script: scaled_x = linear_x * MAX_SPEED * speed_scale
  // If we assume linear_x is m/s, and MAX_SPEED=1100.
  // Let's deduce the conversion factor.
  const double CMD_SCALE_FACTOR = 1500.0; // Boosted power to overcome friction (was 500.0)
};

}  // namespace mecca_hardware_interface
