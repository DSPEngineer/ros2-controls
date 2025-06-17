#ifndef DIFF_CONTROLLER_HPP
#define DIFF_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>

using hardware_interface::CallbackReturn;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;

namespace turtlebot3_hardware
{
    class TurtleBot3Hardware : public hardware_interface::SystemInterface
    {
    private:
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;
        std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite;
        std::unique_ptr<dynamixel::GroupSyncRead> groupSyncReadPosition;
        std::unique_ptr<dynamixel::GroupSyncRead> groupSyncReadVelocity;

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(TurtleBot3Hardware);

        CallbackReturn on_init(const HardwareInfo &info) override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };
}

#endif