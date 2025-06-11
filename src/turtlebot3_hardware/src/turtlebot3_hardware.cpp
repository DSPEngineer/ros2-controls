#include "turtlebot3_hardware/turtlebot3_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace turtlebot3_hardware
{
hardware_interface::CallbackReturn Turtlebot3Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "on_init() successful");

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Turtlebot3Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "on_configure() successful");
  // TODO(anderson): setup communications
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Turtlebot3Hardware::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "export_state_interfaces() successful");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Turtlebot3Hardware::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "export_command_interfaces() successful");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Turtlebot3Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "on_activate() successful");
  // TODO(anderson): enable motors
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Turtlebot3Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "on_deactivate() successful");
  // TODO(anderson): disable motors
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Turtlebot3Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "read() successful");
  // TODO(anderson): read from robot
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Turtlebot3Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "write() successful");
  // TODO(anderson): write to robot
  return hardware_interface::return_type::OK;
}
}  // namespace turtlebot3_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  turtlebot3_hardware::Turtlebot3Hardware, hardware_interface::SystemInterface)
