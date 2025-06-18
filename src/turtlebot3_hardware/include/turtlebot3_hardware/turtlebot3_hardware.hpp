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

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_OPERTAING_MODE         11
#define ADDR_PRO_GOAL_VELOCITY          104
#define ADDR_PRO_PRESET_VELOCITY        128
#define ADDR_PRO_PROFILE_VELOCITY       112

// operation modes
#define VELOCITY_CONTROL_MODE           1
#define POSITION_CONTROL_MODE           3

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRO_GOAL_VELOCITY           4
#define LEN_PRO_PRESENT_VELOCITY        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_SUCCESS                     0
#define DXL_ERROR                       1


namespace turtlebot3_hardware
{
struct ComConfig {
    std::string usb_port = "";
    uint32_t  baud_rate = 0;
};

struct WheelConfig {
    std::string joint_name = "";
    double state_pos = 0.0;
    double state_vel = 0.0;
    double cmd_vel = 0.0;
    uint8_t id = 0;
    uint32_t vel_profile = 0;
    uint32_t vel_factor = 0;
    double vel_ratio = 0.0;
};

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

    ComConfig com_cfg_;
    WheelConfig wheel_l_;
    WheelConfig wheel_r_;

    uint8_t dxl_error_ = 0;   
    int dxl_comm_result_ = COMM_TX_FAIL;       
    uint8_t dxl_goal_velocity_[4];

    int configure_dynamixel(WheelConfig *wheel);
    int eval_dxl_result(void);
    int stop_dynamixel(uint8_t id);
    int start_dynamixel(uint8_t id);
    int set_joint_cmd(WheelConfig *wheel);
    int set_joint_states(WheelConfig *wheel);

};

} // namespace turtlebot3_hardware

#endif // TURTLEBOT3_HARDWARE_HPP