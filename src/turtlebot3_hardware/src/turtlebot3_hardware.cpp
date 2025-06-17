#include "turtlebot3_hardware/turtlebot3_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

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
        // init hardware interface settings  
        usb_port_ = info_.hardware_parameters["usb_port"];
        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
        left_wheel_id_ = std::stoi(info_.hardware_parameters["left_wheel_id"]);
        right_wheel_id_ = std::stoi(info_.hardware_parameters["right_wheel_id"]);
        
        // init joint control buffersize
        joint_commands_vel_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        joint_states_pos_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        joint_states_vel_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        joint_ids_.resize(info_.joints.size(), 0);
        

        // Init communication handlers
        port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Initialize GroupSyncWrite instance
        goal_velocity_writer_ = new dynamixel::GroupSyncWrite(
            port_handler_, packet_handler_, ADDR_PRO_GOAL_VELOCITY, LEN_PRO_GOAL_VELOCITY);

        // Initialize Groupsyncread instance for Present variable reading
        present_position_reader_ = new dynamixel::GroupSyncRead(
            port_handler_, packet_handler_, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        present_velocity_reader_ = new dynamixel::GroupSyncRead(
            port_handler_, packet_handler_, ADDR_PRO_PRESET_VELOCITY, LEN_PRO_PRESENT_VELOCITY);

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            // Turtlebot3Hardware has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' have '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
            }
        }
        
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> TurtlebotHardware::export_state_interfaces() {
        RCLCPP_INFO_STREAM(get_logger(), "export_state_interfaces");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_states_pos_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_states_vel_[i]));
        }
        return state_interfaces;
    }
    std::vector<hardware_interface::CommandInterface> TurtlebotHardware::export_command_interfaces() {
        RCLCPP_INFO_STREAM(get_logger(), "export_command_interfaces");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_commands_vel_[i])); 
        }
        return command_interfaces;
    }

    CallbackReturn TurtlebotHardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
        (void) previous_state;
        RCLCPP_INFO_STREAM(get_logger(), "on_configure():");
        
        // Add motor IDs to the SyncRead objects
        if (present_position_reader_->addParam(left_wheel_id_) != true) {
            RCLCPP_FATAL(get_logger(), "Could not add param for left wheel position");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (present_position_reader_->addParam(right_wheel_id_) != true) {
            RCLCPP_FATAL(get_logger(), "Could not add param for right wheel position");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (present_velocity_reader_->addParam(left_wheel_id_) != true) {
            RCLCPP_FATAL(get_logger(), "Could not add param for left wheel velocity");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (present_velocity_reader_->addParam(right_wheel_id_) != true) {
            RCLCPP_FATAL(get_logger(), "Could not add param for right wheel velocity");
            return hardware_interface::CallbackReturn::ERROR;
        }

        joint_ids_[0] = left_wheel_id_;
        joint_ids_[1] = right_wheel_id_; 
        
        RCLCPP_INFO_STREAM(get_logger(), "opening port");
        // Open port
        if (port_handler_->openPort()) {
            RCLCPP_INFO_STREAM(get_logger(), "port open success");
        } else {
            RCLCPP_INFO_STREAM(get_logger(), "could not open port");
            return CallbackReturn::ERROR;
        }
        // Set port baudrate
        if (port_handler_->setBaudRate(baud_rate_)) {
            RCLCPP_INFO_STREAM(get_logger(), "baudrate set success");
        } else {
            RCLCPP_INFO_STREAM(get_logger(), "baudrate setting failed");
            return CallbackReturn::ERROR;
        }

        if (configure_dynamixel(left_wheel_id_) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(get_logger(), "Dynamixel#%d setup complet", left_wheel_id_);
        }

        if (configure_dynamixel(right_wheel_id_) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(get_logger(), "Dynamixel#%d setup complet", right_wheel_id_);
        } 
        RCLCPP_INFO_STREAM(get_logger(), "Successfully configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
        (void) previous_state;
        RCLCPP_INFO_STREAM(get_logger(), "on_activate");
        
        // activate motors
        if (start_dynamixel(left_wheel_id_) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        if (start_dynamixel(right_wheel_id_) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        // reset goal velocity 
        *(int32_t*)dxl_goal_velocity_ = 0;
        // Add parameter storage for left wheel goal velocity
        if (goal_velocity_writer_->addParam(left_wheel_id_, dxl_goal_velocity_) != true) {
            RCLCPP_FATAL(get_logger(), "Could not add param for left wheel goal velocity");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Add parameter storage for right wheel goal velocity
        if (goal_velocity_writer_->addParam(right_wheel_id_, dxl_goal_velocity_) != true) {
            RCLCPP_FATAL(get_logger(), "Failed to add param for right wheel goal velocity");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // deploy the entire package 
        if (goal_velocity_writer_->txPacket() != COMM_SUCCESS) {
            RCLCPP_FATAL(get_logger(), "Could not transmit SyncWrite data");
            return hardware_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        (void) previous_state;
        RCLCPP_INFO_STREAM(get_logger(), "on_deactivate");
        if (stop_dynamixel(left_wheel_id_) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        if (stop_dynamixel(right_wheel_id_) != DXL_SUCCESS) {
            return CallbackReturn::ERROR;
        }
        
        // release port
        port_handler_->closePort();

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type TurtlebotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void) time;
        (void) period;
        RCLCPP_INFO_STREAM(get_logger(), "read()");


        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TurtlebotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void) period;
        (void) time;
        RCLCPP_INFO_STREAM(get_logger(), "write");
        /* */
        for (uint i = 0; i < joint_ids_.size(); i++) {
            *(int32_t*)dxl_goal_velocity_ = (int32_t)joint_commands_vel_[i] / 1000;
            RCLCPP_INFO(get_logger(), "id:%d gloal_vel: %d ",joint_ids_[i], *(int32_t*)dxl_goal_velocity_);  
            goal_velocity_writer_->changeParam(joint_ids_[i], dxl_goal_velocity_);
        }
        goal_velocity_writer_->txPacket();

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

    int TurtlebotHardware::set_joint_velocities() {
        return DXL_SUCCESS;
    }
    
} // namespace turtlebot3_hardware

// Export as a plugin
PLUGINLIB_EXPORT_CLASS(turtlebot3_hardware::TurtlebotHardware, hardware_interface::SystemInterface)