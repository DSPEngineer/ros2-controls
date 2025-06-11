#include "turtlebot3_hardware/turtlebot3_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace turtlebot3_hardware
{
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBot3Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Initialize hardware, read parameters, etc.
  RCLCPP_INFO(rclcpp::get_logger("TurtleBot3Hardware"), "on_init was called");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TurtleBot3Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Export state interfaces for each joint, e.g., position and velocity

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TurtleBot3Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // Export command interfaces for each joint, e.g., velocity

  return command_interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBot3Hardware::on_activate(const rclcpp_lifecycle::State &)
{
  // Start hardware communication
  RCLCPP_INFO(rclcpp::get_logger("TurtleBot3Hardware"), "on_activate was called");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBot3Hardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop hardware communication
  RCLCPP_INFO(rclcpp::get_logger("TurtleBot3Hardware"), "on_deactivate was called");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TurtleBot3Hardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // Read from hardware
  RCLCPP_INFO(rclcpp::get_logger("TurtleBot3Hardware"), "read was called");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3Hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Write to hardware
  RCLCPP_INFO(rclcpp::get_logger("TurtleBot3Hardware"), "write was called");

  return hardware_interface::return_type::OK;
}

}  // namespace turtlebot3_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  turtlebot3_hardware::TurtleBot3Hardware,
  hardware_interface::SystemInterface
)
