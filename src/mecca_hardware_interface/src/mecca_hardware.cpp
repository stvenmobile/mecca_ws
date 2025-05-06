#include "mecca_hardware_interface/mecca_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecca_hardware_interface
{

hardware_interface::CallbackReturn MeccaHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info.joints.size());  // Each is a JointState struct
  hw_commands_.resize(info.joints.size(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"),
              "Initialized MeccaHardware interface with %zu joints", info.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MeccaHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
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
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MeccaHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MeccaHardware"), "Deactivating MeccaHardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MeccaHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MeccaHardware"), "Reading hardware (simulated)");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MeccaHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MeccaHardware"), "Writing hardware commands (simulated)");
  return hardware_interface::return_type::OK;
}

}  // namespace mecca_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecca_hardware_interface::MeccaHardware, hardware_interface::SystemInterface)
