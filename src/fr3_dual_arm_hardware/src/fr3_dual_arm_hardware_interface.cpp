#include "fr3_dual_arm_hardware/fr3_dual_arm_hardware_interface.hpp"

#include <algorithm>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace fr3_dual_arm_hardware
{

hardware_interface::CallbackReturn Fr3DualArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_position_state_.assign(info_.joints.size(), 0.0);
  joint_position_command_.assign(info_.joints.size(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Fr3DualArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      "position",
      &joint_position_state_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
Fr3DualArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      "position",
      &joint_position_command_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn Fr3DualArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_position_command_ = joint_position_state_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Fr3DualArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Fr3DualArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Replace with reads from FAIRINO SDK and HKV serial drivers.
  joint_position_state_ = joint_position_command_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Fr3DualArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Send joint position commands through vendor SDK/serial interfaces.
  return hardware_interface::return_type::OK;
}

}  // namespace fr3_dual_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  fr3_dual_arm_hardware::Fr3DualArmHardwareInterface,
  hardware_interface::SystemInterface)
