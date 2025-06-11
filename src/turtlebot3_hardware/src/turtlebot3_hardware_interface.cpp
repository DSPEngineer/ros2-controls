#include "turtlebot3_hardware/turtlebot3_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace turtlebot3_hardware
{
    CallbackReturn TurtlebotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Initialize all data structures (apart from hardware) needed here

        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TurtlebotSystemHardware"));
        RCLCPP_INFO_STREAM(get_logger(), "In on_init() method");

        // Initialize base class with info received from URDF
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        //TODO: Try obtaining parameter values from URDF using "info"
        // The port for connecting to the Dynamixel is /dev/u2d2.

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

    CallbackReturn TurtlebotSystemHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        // Connect to Dynamixel

        RCLCPP_INFO_STREAM(get_logger(), "In on_configure() method");

        // Upon any errors (For example: failure to connect to hardware) - return CallbackReturn::ERROR

        RCLCPP_INFO_STREAM(get_logger(), "Successfully configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotSystemHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        // "Turn on the power" / "enable" the actuators
        RCLCPP_INFO_STREAM(get_logger(), "In on_activate() method");


        // Set command to current state to avoid unnecessary motion upon startup
        for (const auto & [name, descr] : joint_command_interfaces_)
        {
            set_command(name, get_state(name));
        }

        RCLCPP_INFO_STREAM(get_logger(), "Successfully activated hardware");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurtlebotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        // Implement shutdown sequence
        // "Turn off the power" / "disable" the actuators.
        RCLCPP_INFO_STREAM(get_logger(), "In on_deactivate() method");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type TurtlebotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Avoid any memory allocation and ideally any blocking calls. Avoid logging
        // Make call to Dynamixel sync reader to obtain position and velocity for both joints
        
        RCLCPP_INFO_STREAM(get_logger(), "In read() method");

        // Assign the obtained positions and velocities. Example below.
        for (const auto & [name, descr] : joint_state_interfaces_)
        {
            if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
            {
                // Set obtained position here
                set_state(name, 0.0);
            }
            else
            {
                // Set obtained velocity here
                set_state(name, 0.1);
            }
        }
        
        // Return Error if read fails.

        return hardware_interface::return_type::OK;

    }

    hardware_interface::return_type TurtlebotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {

        // Assign the commanded velocities. Example below.
        for (const auto & [name, descr] : joint_command_interfaces_)
        {
        
            RCLCPP_INFO_STREAM(get_logger(), "Joint name: " << name << " commanded value: " << get_command(name));
            // Gather into the data structure to send via the Dynamixel SyncWriter
        }

        // Write via syncwriter

        return hardware_interface::return_type::OK;

    }

} // namespace turtlebot3_hardware

// Export as a plugin
PLUGINLIB_EXPORT_CLASS(turtlebot3_hardware::TurtlebotSystemHardware, hardware_interface::SystemInterface)