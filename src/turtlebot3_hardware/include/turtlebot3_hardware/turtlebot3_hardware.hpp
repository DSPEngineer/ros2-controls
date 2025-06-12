#ifndef TURTLEBOT3_HARDWARE__TURTLEBOT3_HARDWARE_HPP_
#define TURTLEBOT3_HARDWARE__TURTLEBOT3_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace turtlebot3_hardware
{
class Turtlebot3Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Turtlebot3Hardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

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
  // Store the command for the wheels
  std::vector<double> hw_commands_velocity_;

  // Store the state for the wheels
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Parameters for the Dynamixel SDK
  std::string usb_port_;
  uint32_t baud_rate_;

  // Dynamixel SDK objects
  dynamixel::PortHandler * port_handler_;
  dynamixel::PacketHandler * packet_handler_;
};
}  // namespace turtlebot3_hardware

#endif  // TURTLEBOT3_HARDWARE__TURTLEBOT3_HARDWARE_HPP_
