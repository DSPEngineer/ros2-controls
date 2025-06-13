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

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

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
#define DXL_MINIMUM_POSITION_VALUE      0                   // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4096           // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     24                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


namespace turtlebot3_hardware
{
    CallbackReturn TurtlebotHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TurtlebotHardware"));

        RCLCPP_INFO_STREAM(get_logger(), "on_init");

        // Initialize base class with info received from URDF
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Example: Validate the URDF ROS2 control config
        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // Validate that the robot has two state interfaces - POSITION & VELOCITY
            // and one command interfaces - VELOCITY

            RCLCPP_INFO_STREAM(get_logger(), "Joint " << joint.name.c_str() << " has " << joint.command_interfaces.size() << " command interfaces of type " << joint.command_interfaces[0].name << ".");
            
            // On validation error - return CallbackReturn::ERROR
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO_STREAM(get_logger(), "on_configure:");

        // Connect to Dynamixel
        // Initialize PortHandler instance
        // Set the port path
        // Get methods and members of PortHandlerLinux or PortHandlerWindows
        port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);

        // Initialize PacketHandler instance
        // Set the protocol version
        // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Initialize GroupSyncWrite instance
        dynamixel::GroupSyncWrite groupSyncWrite(port_handler_, packet_handler_, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

        // Initialize Groupsyncread instance for Present Position
        dynamixel::GroupSyncRead groupSyncRead(port_handler_, packet_handler_, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

        int index = 0;
        int dxl_comm_result = COMM_TX_FAIL;               // Communication result
        bool dxl_addparam_result = false;                 // addParam result
        bool dxl_getdata_result = false;                  // GetParam result
        int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};  // Goal position

        uint8_t dxl_error = 0;                            // Dynamixel error
        uint8_t param_goal_position[4];
        int32_t dxl1_present_position = 0, dxl2_present_position = 0;                         // Present position
        
        RCLCPP_INFO_STREAM(get_logger(), "opening port");

        // Open port
        if (port_handler_->openPort())
        {
            RCLCPP_INFO_STREAM(get_logger(), "port open success");
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(), "could not open port");
            return CallbackReturn::ERROR;
        }
        // Set port baudrate
        if (port_handler_->setBaudRate(BAUDRATE))
        {
            RCLCPP_INFO_STREAM(get_logger(), "baudrate set success");
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(), "baudrate setting failed");
            return CallbackReturn::ERROR;
        }
        // Enable Dynamixel#1 Torque
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Dynamixel#1 communication error");
            return CallbackReturn::ERROR;
        }
        else if (dxl_error != 0)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Dynamixel#1 internal error");
            return CallbackReturn::ERROR;
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(), "Dynamixel#1 successfully connected");
        }
        // Enable Dynamixel#2 Torque
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Dynamixel#2 communication error");
            return CallbackReturn::ERROR;
        }
        else if (dxl_error != 0)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Dynamixel#2 internal error");
            return CallbackReturn::ERROR;
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(), "Dynamixel#2 successfully connected");
        }
        // Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO_STREAM(get_logger(), "[ID:1] groupSyncRead addparam failed");
            return CallbackReturn::ERROR;
        }
        // Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO_STREAM(get_logger(), "[ID:2] groupSyncRead addparam failed");
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(get_logger(), "Successfully configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO_STREAM(get_logger(), "on_activate");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO_STREAM(get_logger(), "on_deactivate");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type TurtlebotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        RCLCPP_INFO_STREAM(get_logger(), "read()");
        return hardware_interface::return_type::OK;

    }

    hardware_interface::return_type TurtlebotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        RCLCPP_INFO_STREAM(get_logger(), "write");
        return hardware_interface::return_type::OK;

    }

} // namespace turtlebot3_hardware

// Export as a plugin
PLUGINLIB_EXPORT_CLASS(turtlebot3_hardware::TurtlebotHardware, hardware_interface::SystemInterface)