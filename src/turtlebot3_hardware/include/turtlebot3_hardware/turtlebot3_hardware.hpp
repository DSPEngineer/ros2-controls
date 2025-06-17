#ifndef TURTLEBOT3_HARDWARE_HPP
#define TURTLEBOT3_HARDWARE_HPP

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

class TurtlebotHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(TurtlebotHardware)

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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
    dynamixel::PortHandler *port_handler_;

    // Dynamixel message packet handler
    dynamixel::PacketHandler *packet_handler_;

    // Reader for synchronous position reads from all motors
    dynamixel::GroupSyncRead *present_position_reader_;

    // Reader for synchronous velocity reads from all motors
    dynamixel::GroupSyncRead *present_velocity_reader_;

    // Writer for synchronous velocity writes to all motors
    dynamixel::GroupSyncWrite *goal_velocity_writer_;

    std::string usb_port_;
    uint32_t baud_rate_;
    uint8_t left_wheel_id_;
    uint8_t right_wheel_id_;
    uint8_t dxl_error_ = 0;   
    int dxl_comm_result_ = COMM_TX_FAIL;       
    uint8_t dxl_goal_velocity_[4];


    std::vector<double> joint_commands_vel_;
    std::vector<double> joint_states_pos_;
    std::vector<double> joint_states_vel_;
    std::vector<uint8_t> joint_ids_;
    std::map<uint8_t,uint8_t*> joint_control_map_;

    int configure_dynamixel(uint8_t id);
    int eval_dxl_result(void);
    int stop_dynamixel(uint8_t id);
    int start_dynamixel(uint8_t id);
    int set_joint_velocities();

    /* 
    CallbackReturn set_joint_positions();
    CallbackReturn set_joint_velocities();
    CallbackReturn set_joint_params();
    */
};

} // namespace turtlebot3_hardware

#endif // TURTLEBOT3_HARDWARE_HPP