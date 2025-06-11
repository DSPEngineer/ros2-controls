import os
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition

# Packages
CONTROL_PKG: Final[str] = "turtlebot3_control"

# Directories
CONFIG_DIR: Final[str] = "config"

# Files
CONTROLLER_CONFIG_FILE: Final[str] = "control.yaml"

# File Paths
controller_config_path = os.path.join(
    get_package_share_directory(CONTROL_PKG), CONFIG_DIR, CONTROLLER_CONFIG_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = [
    DeclareLaunchArgument(
        name="sim",
        default_value="false",
        choices=["true", "false"],
        description="Enable sim interface",
    )
]


def generate_launch_description() -> LaunchDescription:
    """
    Launches the controller manager and spawns the listed controllers
    """

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        output="both",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("sim"), "false")),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[controller_config_path],
        arguments=["joint_state_broadcaster"],
    )


    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[controller_config_path],
        arguments=["turtlebot_base_controller"],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(base_controller_spawner)
    return ld
