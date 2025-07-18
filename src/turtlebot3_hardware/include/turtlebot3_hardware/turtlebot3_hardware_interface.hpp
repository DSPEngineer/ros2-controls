#ifndef TURTLEBOT3_HARDWARE_INTERFACE_HPP
#define TURTLEBOT3_HARDWARE_INTERFACE_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace turtlebot3_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TurtlebotSystemHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(TurtlebotSystemHardware);

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    rclcpp::Logger get_logger() const { return *logger_; }

private:
    // Logger
    std::shared_ptr<rclcpp::Logger> logger_;

    // Dynamixel SDK interface to motor port
    dynamixel::PortHandler * port_handler_;

    // Dynamixel message packet handler
    dynamixel::PacketHandler * packet_handler_;

    // Reader for synchronous position reads from all motors
    std::unique_ptr<dynamixel::GroupSyncRead> present_position_reader_;

    // Reader for synchronous velocity reads from all motors
    std::unique_ptr<dynamixel::GroupSyncRead> present_velocity_reader_;

    // Writer for synchronous velocity writes to all motors
    std::unique_ptr<dynamixel::GroupSyncWrite> goal_velocity_writer_;
};

} // namespace turtlebot3_hardware

#endif // TURTLEBOT3_HARDWARE_INTERFACE_HPP