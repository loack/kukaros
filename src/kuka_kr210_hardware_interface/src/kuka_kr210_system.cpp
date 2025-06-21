#include "kuka_kr210_hardware_interface/kuka_kr210_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace kuka_kr210_hardware_interface
{

hardware_interface::CallbackReturn KUKAKR210System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t num_joints = info_.joints.size();

  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KUKAKR210System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KUKAKR210System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type KUKAKR210System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Mock read: simulate robot following commands exactly
  hw_positions_ = hw_commands_;
  hw_velocities_.assign(hw_velocities_.size(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("KUKAKR210System"), "Reading joint commands...");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KUKAKR210System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // In a real interface, this would send `hw_commands_` to the robot via KUKAVARProxy
  // For now, just log
  RCLCPP_INFO(rclcpp::get_logger("KUKAKR210System"), "Writing joint commands...");
  return hardware_interface::return_type::OK;
}

}  // namespace kuka_kr210_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kuka_kr210_hardware_interface::KUKAKR210System, hardware_interface::SystemInterface)
