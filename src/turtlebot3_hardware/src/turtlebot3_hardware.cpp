
#include "turtlebot3_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace turtlebot3_hardware {

    Turtlebot3Hardware::Turtlebot3Hardware() {

    }

    hardware_interface::CallbackReturn Turtlebot3Hardware::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Configuring.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Turtlebot3Hardware::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Cleaning Up.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Turtlebot3Hardware::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Shutting Down.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Turtlebot3Hardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Activating.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Turtlebot3Hardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Deactivating.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Turtlebot3Hardware::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Error Found.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Turtlebot3Hardware::on_init(const hardware_interface::HardwareInfo &hardware_info) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Initialising.");
        return hardware_interface::SystemInterface::on_init(hardware_info);
    }

    hardware_interface::return_type Turtlebot3Hardware::prepare_command_mode_Switch(const std::vector< std::string > &, const std::vector< std::string > &) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Turtlebot3Hardware::perform_command_mode_Switch(const std::vector< std::string > &, const std::vector< std::string > &) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Turtlebot3Hardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Reading.");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Turtlebot3Hardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        RCLCPP_INFO(rclcpp::get_logger("Turtlebot3Hardware"), "TB3: Writing.");
        return hardware_interface::return_type::OK;
    }

}

PLUGINLIB_EXPORT_CLASS(turtlebot3_hardware::Turtlebot3Hardware, hardware_interface::SystemInterface)
