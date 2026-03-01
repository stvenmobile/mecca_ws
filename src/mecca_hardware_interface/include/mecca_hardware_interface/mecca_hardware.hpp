#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>
#include <cmath>

namespace mecca_hardware_interface
{

class MeccaHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Serial helpers ──────────────────────────────────────────────────────────
  bool openSerialPort();
  void closeSerialPort();
  void parseEncoderData(const std::string & line);

  // ── Joint state storage ─────────────────────────────────────────────────────
  struct JointState {
    double position = 0.0;
    double velocity = 0.0;
  };
  std::vector<JointState> hw_states_;
  std::vector<double>     hw_commands_;

  // ── Serial port ─────────────────────────────────────────────────────────────
  int         serial_port_ = -1;
  std::string port_name_   = "/dev/stm32_serial";
  std::string serial_buffer_;

  // ── Encoder state ───────────────────────────────────────────────────────────
  bool         first_read_      = true;
  double       last_enc_pos_[4] = {0.0, 0.0, 0.0, 0.0};
  rclcpp::Time last_enc_time_;

  // ── Cycle counter — alternates write() between V commands and I ENC polls ──
  int write_cycle_ = 0;

  // ── Robot geometry (overridden by URDF params) ──────────────────────────────
  double wheel_radius_       = 0.048;   // m
  double wheel_separation_x_ = 0.175;   // m  wheelbase (front-to-rear)
  double wheel_separation_y_ = 0.175;   // m  track width (left-to-right)

  // ── Encoder constants ───────────────────────────────────────────────────────
  static constexpr double TICKS_PER_REV = 2474.0;   // empirically measured
  static constexpr double RADS_PER_TICK = (2.0 * M_PI) / TICKS_PER_REV;

  // ── Debug flag ──────────────────────────────────────────────────────────────
  bool hw_debug_    = false;
  int  log_counter_ = 0;
};

}  // namespace mecca_hardware_interface
