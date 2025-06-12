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

  // Get hardware parameters
  usb_port_ = info_.hardware_parameters["usb_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  left_wheel_id_ = std::stoi(info_.hardware_parameters["left_wheel_id"]);
  right_wheel_id_ = std::stoi(info_.hardware_parameters["right_wheel_id"]);

  // Initialize Dynamixel SDK handlers
  port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  // Initialize SyncWrite and SyncRead objects
  group_sync_write_goal_velocity_ = new dynamixel::GroupSyncWrite(port_handler_, packet_handler_, ADDR_GOAL_VELOCITY, 4);
  group_sync_read_present_position_ = new dynamixel::GroupSyncRead(port_handler_, packet_handler_, ADDR_PRESENT_POSITION, 4);
  group_sync_read_present_velocity_ = new dynamixel::GroupSyncRead(port_handler_, packet_handler_, ADDR_PRESENT_VELOCITY, 4);

  hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Turtlebot3Hardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Turtlebot3Hardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Turtlebot3Hardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Turtlebot3Hardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Turtlebot3Hardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Turtlebot3Hardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Turtlebot3Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "on_configure() successful");

  if (!port_handler_->openPort()) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to open port %s", usb_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!port_handler_->setBaudRate(baud_rate_)) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to set baud rate %d", baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  uint8_t dxl_error;
  if (packet_handler_->ping(port_handler_, left_wheel_id_, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to ping left wheel motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (packet_handler_->ping(port_handler_, right_wheel_id_, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to ping right wheel motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "Successfully pinged motors");

  // Add motor IDs to the SyncRead objects
  if (group_sync_read_present_position_->addParam(left_wheel_id_) != true) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to add param for left wheel position");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (group_sync_read_present_position_->addParam(right_wheel_id_) != true) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to add param for right wheel position");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (group_sync_read_present_velocity_->addParam(left_wheel_id_) != true) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to add param for left wheel velocity");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (group_sync_read_present_velocity_->addParam(right_wheel_id_) != true) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to add param for right wheel velocity");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Turtlebot3Hardware::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "export_state_interfaces() successful");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Turtlebot3Hardware::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "export_command_interfaces() successful");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Turtlebot3Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "on_activate() successful");

  uint8_t dxl_error;
  // Set operating mode to velocity control for both motors (address 11, value 1)
  if (packet_handler_->write1ByteTxRx(port_handler_, left_wheel_id_, ADDR_OPERATING_MODE, 1, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to set operating mode to velocity on left wheel motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (packet_handler_->write1ByteTxRx(port_handler_, right_wheel_id_, ADDR_OPERATING_MODE, 1, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to set operating mode to velocity on right wheel motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Enable torque on both motors
  if (packet_handler_->write1ByteTxRx(port_handler_, left_wheel_id_, ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to enable torque on left wheel motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (packet_handler_->write1ByteTxRx(port_handler_, right_wheel_id_, ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to enable torque on right wheel motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "Successfully enabled torque on motors");

  // Set goal velocity to 0 for both motors to be safe
  uint8_t dxl_goal_velocity[4];
  *(int32_t*)dxl_goal_velocity = 0;

  // Add parameter storage for left wheel goal velocity
  if (group_sync_write_goal_velocity_->addParam(left_wheel_id_, dxl_goal_velocity) != true) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to add param for left wheel goal velocity");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Add parameter storage for right wheel goal velocity
  if (group_sync_write_goal_velocity_->addParam(right_wheel_id_, dxl_goal_velocity) != true) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to add param for right wheel goal velocity");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Transmit SyncWrite packet
  if (group_sync_write_goal_velocity_->txPacket() != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to transmit SyncWrite packet");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Clear SyncWrite parameter storage
  group_sync_write_goal_velocity_->clearParam();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Turtlebot3Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "on_deactivate() successful");

  uint8_t dxl_error;
  // Disable torque on both motors
  if (packet_handler_->write1ByteTxRx(port_handler_, left_wheel_id_, ADDR_TORQUE_ENABLE, 0, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to disable torque on left wheel motor");
  }

  if (packet_handler_->write1ByteTxRx(port_handler_, right_wheel_id_, ADDR_TORQUE_ENABLE, 0, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("Turtlebot3Hardware"), "Failed to disable torque on right wheel motor");
  }

  // The Dynamixel SDK's PortHandler API does not have a method to check if the port is open.
  // We will unconditionally call closePort() here. It is safe to call on an already closed port.
  port_handler_->closePort();

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
