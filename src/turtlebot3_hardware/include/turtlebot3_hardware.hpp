#ifndef TURTLEBOT3_HARDWARE
#define TURTLEBOT3_HARDWARE

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace turtlebot3_hardware {
    class Turtlebot3Hardware : public hardware_interface::SystemInterface {
        public:
        Turtlebot3Hardware();

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        hardware_interface::return_type prepare_command_mode_Switch(const std::vector< std::string > &, const std::vector< std::string > &);
        hardware_interface::return_type perform_command_mode_Switch(const std::vector< std::string > &, const std::vector< std::string > &);

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

    };
}

#endif
