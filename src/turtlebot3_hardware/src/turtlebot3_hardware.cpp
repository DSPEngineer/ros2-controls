#include "turtlebot3_hardware.hpp"

// Control table address
#define ADDR_PRO_TORQUE_ENABLE 54 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_VELOCITY 104
#define ADDR_PRO_PRESENT_POSITION 132
#define ADDR_PRO_PRESENT_VELOCITY 128

// Data Byte Length
#define LEN_PRO_GOAL_VELOCITY 4
#define LEN_PRO_PRESENT_POSITION 4
#define LEN_PRO_PRESENT_VELOCITY 4

// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID 1 // Dynamixel#1 ID: 1
#define DXL2_ID 2 // Dynamixel#2 ID: 2
#define BAUDRATE 57600
#define DEVICENAME "/dev/ttyUSB0" // Check which port is being used on your controller
                                  // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE 1                 // Value for enabling the torque
#define TORQUE_DISABLE 0                // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE 0    // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE 4095 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD 20  // Dynamixel moving status threshold

CallbackReturn turtlebot3_hardware::TurtleBot3Hardware::on_init(const HardwareInfo &info)
{

    CallbackReturn result = hardware_interface::SystemInterface::on_init(info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' has %zu state interfaces found. 2 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn turtlebot3_hardware::TurtleBot3Hardware::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupSyncWrite instance
    groupSyncWrite = std::make_unique<dynamixel::GroupSyncWrite>(dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, LEN_PRO_GOAL_VELOCITY));

    // Initialize Groupsyncread instance for Present Position
    groupSyncReadPosition = std::make_unique<dynamixel::GroupSyncRead>(dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION));
    groupSyncReadVelocity = std::make_unique<dynamixel::GroupSyncRead>(dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY));

    if (portHandler->openPort())
    {
        RCLCPP_INFO(get_logger(), "Succeeded to open Dynamixel port!");
        printf("\n");
    }
    else
    {
        RCLCPP_FATAL(
            get_logger(), "Failed to open the Dynamixel port!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        RCLCPP_INFO(get_logger(), "Succeeded in changing Dynamixel port baudrate!");
    }
    else
    {
        RCLCPP_FATAL(
            get_logger(), "Failed to change the Dynamixel port baudrate!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Add parameter storage for Dynamixel#1 present position value
    bool dxl_addparam_result = false; // addParam result
    dxl_addparam_result = groupSyncReadPosition->addParam(DXL1_ID);
    if (dxl_addparam_result != true)
    {
        RCLCPP_FATAL(
            get_logger(), "[ID:%03d] groupSyncReadPosition addparam failed", DXL1_ID);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncReadPosition->addParam(DXL2_ID);
    if (dxl_addparam_result != true)
    {
        RCLCPP_FATAL(
            get_logger(), "[ID:%03d] groupSyncReadPosition addparam failed", DXL2_ID);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Add parameter storage for Dynamixel#1 present velocity value
    dxl_addparam_result = groupSyncReadVelocity->addParam(DXL1_ID);
    if (dxl_addparam_result != true)
    {
        RCLCPP_FATAL(
            get_logger(), "[ID:%03d] groupSyncReadVelocity addparam failed", DXL1_ID);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Add parameter storage for Dynamixel#2 present velocity value
    dxl_addparam_result = groupSyncReadVelocity->addParam(DXL2_ID);
    if (dxl_addparam_result != true)
    {
        RCLCPP_FATAL(
            get_logger(), "[ID:%03d] groupSyncReadVelocity addparam failed", DXL2_ID);
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto &[name, _] : joint_state_interfaces_)
    {
        set_state(name, 0.0);
    }

    for (const auto &[name, _] : joint_command_interfaces_)
    {
        set_command(name, 0.0);
    }

    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn turtlebot3_hardware::TurtleBot3Hardware::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    // Enable Dynamixel#1 Torque
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_FATAL(
            get_logger(), "Communication Error: %s", packetHandler->getTxRxResult(dxl_comm_result));
        return hardware_interface::CallbackReturn::ERROR;
    }
    else if (dxl_error != 0)
    {
        RCLCPP_FATAL(
            get_logger(), "Dynamixel Error: %s", packetHandler->getRxPacketError(dxl_error));
        return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Dynamixel#%d has been successfully connected", DXL1_ID);
    }

    // Enable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_FATAL(
            get_logger(), "Communication Error: %s", packetHandler->getTxRxResult(dxl_comm_result));
        return hardware_interface::CallbackReturn::ERROR;
    }
    else if (dxl_error != 0)
    {
        RCLCPP_FATAL(
            get_logger(), "Dynamixel Error: %s", packetHandler->getRxPacketError(dxl_error));
        return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Dynamixel#%d has been successfully connected", DXL2_ID);
    }

    for (const auto &[name, _] : joint_state_interfaces_)
    {
        set_command(name, get_state(name));
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn turtlebot3_hardware::TurtleBot3Hardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error

    // Disable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_FATAL(
            get_logger(), "Communication Error: %s", packetHandler->getTxRxResult(dxl_comm_result));
        return hardware_interface::CallbackReturn::ERROR;
    }
    else if (dxl_error != 0)
    {
        RCLCPP_FATAL(
            get_logger(), "Dynamixel Error: %s", packetHandler->getRxPacketError(dxl_error));
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Disable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_FATAL(
            get_logger(), "Communication Error: %s", packetHandler->getTxRxResult(dxl_comm_result));
        return hardware_interface::CallbackReturn::ERROR;
    }
    else if (dxl_error != 0)
    {
        RCLCPP_FATAL(
            get_logger(), "Dynamixel Error: %s", packetHandler->getRxPacketError(dxl_error));
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Close port
    portHandler->closePort();

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

return_type turtlebot3_hardware::TurtleBot3Hardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    std::stringstream ss;
    ss << "Reading states:";

    // Syncread present position
    int dxl_comm_result = groupSyncReadPosition->txRxPacket();
    uint8_t dxl_error = 0;
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (groupSyncReadPosition->getError(DXL1_ID, &dxl_error))
    {
        printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
    }
    else if (groupSyncReadPosition->getError(DXL2_ID, &dxl_error))
    {
        printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
    }

    // Check if groupsyncread data of Dynamixel#1 is available
    bool dxl_getdata_result = groupSyncReadPosition->isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncReadPosition getdata failed", DXL1_ID);
        return hardware_interface::return_type::ERROR;
    }

    // Check if groupsyncread data of Dynamixel#2 is available
    dxl_getdata_result = groupSyncReadPosition->isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncReadPosition getdata failed", DXL2_ID);
        return hardware_interface::return_type::ERROR;
    }

    // Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncReadVelocity->isAvailable(DXL1_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncReadVelocity getdata failed", DXL1_ID);
        return hardware_interface::return_type::ERROR;
    }

    // Check if groupsyncread data of Dynamixel#2 is available
    dxl_getdata_result = groupSyncReadVelocity->isAvailable(DXL2_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncReadVelocity getdata failed", DXL2_ID);
        return hardware_interface::return_type::ERROR;
    }

    // Get Dynamixel#1 present position value
    int32_t dxl1_present_position = 0;
    int32_t dxl2_present_position = 0;
    dxl1_present_position = groupSyncReadPosition->getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    // Get Dynamixel#2 present position value
    dxl2_present_position = groupSyncReadPosition->getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    // set_state(joint_state_interfaces_.at("wheel_left_joint"), dxl1_present_position);
    // set_state(joint_state_interfaces_.at("wheel_right_joint"), dxl2_present_position);

    // Now read the velocity.
    int32_t dxl1_present_velocity = 0;
    int32_t dxl2_present_velocity = 0;
    dxl1_present_velocity = groupSyncReadVelocity->getData(DXL1_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);

    // Get Dynamixel#2 present position value
    dxl2_present_velocity = groupSyncReadVelocity->getData(DXL2_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);
    printf("[ID:%03d] PresVel:%03d  PresPos:%03d\t[ID:%03d] PresVel:%03d  PresPos:%03d\n", DXL1_ID, dxl1_present_velocity, dxl1_present_position, DXL2_ID, dxl2_present_velocity, dxl2_present_position);

    return hardware_interface::return_type::OK;
}

return_type turtlebot3_hardware::TurtleBot3Hardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    std::stringstream ss;
    ss << "Writing commands:";

    uint8_t param_left_goal_position[4];
    uint8_t param_right_goal_position[4];

    int goal_left_position = get_command(joint_command_interfaces_.at("wheel_left_joint").get_name());
    param_left_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_left_position));
    param_left_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_left_position));
    param_left_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_left_position));
    param_left_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_left_position));

    int goal_right_position = get_command(joint_command_interfaces_.at("wheel_right_joint").get_name());
    param_right_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_right_position));
    param_right_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_right_position));
    param_right_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_right_position));
    param_right_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_right_position));

    int dxl_addparam_result = groupSyncWrite->addParam(DXL1_ID, param_left_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return hardware_interface::return_type::ERROR;
    }

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite->addParam(DXL2_ID, param_right_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return hardware_interface::return_type::ERROR;
    }

    // Syncwrite goal position
    int dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite->clearParam();

    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    turtlebot3_hardware::TurtleBot3Hardware, hardware_interface::SystemInterface)