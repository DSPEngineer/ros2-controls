#include "turtlebot3_hardware/turtlebot3_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_OPERTAING_MODE         11
#define ADDR_PRO_GOAL_VELOCITY          104
#define ADDR_PRO_PRESET_VELOCITY        128

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

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_SUCCESS                     0
#define DXL_ERROR                       1

namespace turtlebot3_hardware
{
    CallbackReturn TurtlebotHardware::on_init(const hardware_interface::HardwareInfo & info) {
        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TurtlebotHardware"));

        RCLCPP_INFO_STREAM(get_logger(), "on_init");

        // Initialize base class with info received from URDF
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Example: Validate the URDF ROS2 control config
        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            // Validate that the robot has two state interfaces - POSITION & VELOCITY
            // and one command interfaces - VELOCITY

            RCLCPP_INFO_STREAM(
                get_logger(), "Joint " << joint.name.c_str() << " has " << 
                joint.command_interfaces.size() << " command interfaces of type " << 
                joint.command_interfaces[0].name << ".");
            
            // On validation error - return CallbackReturn::ERROR
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
        (void) previous_state;
        RCLCPP_INFO_STREAM(get_logger(), "on_configure():");
        
        // Connect to Dynamixel
        port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);

        // Initialize PacketHandler instance
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Initialize GroupSyncWrite instance
        dynamixel::GroupSyncWrite groupSyncWrite(
            port_handler_, packet_handler_, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

        // Initialize Groupsyncread instance for Present Position
        dynamixel::GroupSyncRead groupSyncRead(
            port_handler_, packet_handler_, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        
        bool dxl_addparam_result = false;
        
        RCLCPP_INFO_STREAM(get_logger(), "opening port");

        // Open port
        if (port_handler_->openPort()) {
            RCLCPP_INFO_STREAM(get_logger(), "port open success");
        } else {
            RCLCPP_INFO_STREAM(get_logger(), "could not open port");
            return CallbackReturn::ERROR;
        }
        // Set port baudrate
        if (port_handler_->setBaudRate(BAUDRATE)) {
            RCLCPP_INFO_STREAM(get_logger(), "baudrate set success");
        } else {
            RCLCPP_INFO_STREAM(get_logger(), "baudrate setting failed");
            return CallbackReturn::ERROR;
        }

        if (configure_dynamixel(DXL1_ID) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(get_logger(), "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
            return CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(get_logger(), "Dynamixel#%d setup complet", DXL1_ID);
        }

        if (configure_dynamixel(DXL2_ID) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(get_logger(), "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
            return CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(get_logger(), "Dynamixel#%d setup complet", DXL2_ID);
        } 
        RCLCPP_INFO_STREAM(get_logger(), "Successfully configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
        (void) previous_state;
        RCLCPP_INFO_STREAM(get_logger(), "on_activate");
        // activate motors
        if (start_dynamixel(DXL1_ID) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        if (start_dynamixel(DXL2_ID) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Testing to spin wheels with a goal velocity
        packet_handler_->write4ByteTxRx(port_handler_, DXL1_ID, ADDR_PRO_GOAL_VELOCITY, -100);
        packet_handler_->write4ByteTxRx(port_handler_, DXL2_ID, ADDR_PRO_GOAL_VELOCITY, 100);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        (void) previous_state;
        RCLCPP_INFO_STREAM(get_logger(), "on_deactivate");
        if (stop_dynamixel(DXL1_ID) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        if (stop_dynamixel(DXL2_ID) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type TurtlebotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void) period;
        (void) time;
        RCLCPP_INFO_STREAM(get_logger(), "read()");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TurtlebotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void) period;
        (void) time;
        RCLCPP_INFO_STREAM(get_logger(), "write");
        return hardware_interface::return_type::OK;

    }

    int TurtlebotHardware::configure_dynamixel(uint8_t id) {      
        RCLCPP_INFO(get_logger(), "Setting up Dynamixel#%d :", id);

        // diable to make operation mode change possible 
        (void)stop_dynamixel(id);

        // Set Operating Mode   
        dxl_comm_result_ = packet_handler_->write1ByteTxRx(
            port_handler_, id, ADDR_PRO_OPERTAING_MODE, VELOCITY_CONTROL_MODE, &dxl_error_);
        if (eval_dxl_result() != DXL_SUCCESS) {
            return DXL_ERROR;
        } else {
            RCLCPP_INFO(get_logger(), "Dynamixel#%d operating mode set", id);
        } 

        return DXL_SUCCESS;
    }

    int TurtlebotHardware::eval_dxl_result(void) {
        if (dxl_comm_result_ != COMM_SUCCESS) {
            RCLCPP_ERROR(get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result_));
            return DXL_ERROR;
        } else if (dxl_error_ != 0) {
            RCLCPP_ERROR(get_logger(), "%s", packet_handler_->getRxPacketError(dxl_error_));
            return DXL_ERROR;
        } 
        return DXL_SUCCESS;
    }

    int TurtlebotHardware::stop_dynamixel(uint8_t id) {
        RCLCPP_INFO(get_logger(), "Stop Dynamixel#%d :", id);

        // Disable Torque
        dxl_comm_result_ = packet_handler_->write1ByteTxRx(
            port_handler_, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_);
        if (eval_dxl_result() != DXL_SUCCESS) {
            return DXL_ERROR; 
        } else {
            RCLCPP_INFO(get_logger(), "Dynamixel#%d torque disabled", id);
        }
        return DXL_SUCCESS;
    }

    int TurtlebotHardware::start_dynamixel(uint8_t id) {

        // Enable Torque
        dxl_comm_result_ = packet_handler_->write1ByteTxRx(
            port_handler_, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_);
        if (eval_dxl_result() != DXL_SUCCESS) {
            return DXL_ERROR; 
        } else {
            RCLCPP_INFO(get_logger(), "Dynamixel#%d torque enabled", id);
        }
        return DXL_SUCCESS;
    }

} // namespace turtlebot3_hardware

// Export as a plugin
PLUGINLIB_EXPORT_CLASS(turtlebot3_hardware::TurtlebotHardware, hardware_interface::SystemInterface)