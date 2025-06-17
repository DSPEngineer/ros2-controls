import os
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Packages
DESCRIPTION_PKG: Final[str] = "turtlebot3_description"
TB3_BRINGUP_PKG: Final[str] = "turtlebot3_bringup"
TB3_CONTROL_PKG: Final[str] = "turtlebot3_control"
TELEOP_JOY_PKG: Final[str] = "teleop_twist_joy"

# Directories
LAUNCH_DIR: Final[str] = "launch"
CONFIG_DIR: Final[str] = "config"

# Files
DESCRIPTION_LAUNCH_FILE: Final[str] = "description.launch.py"
HARDWARE_LAUNCH_FILE: Final[str] = "hardware.launch.py"
CONTROL_LAUNCH_FILE: Final[str] = "control.launch.py"
TELEOP_JOY_LAUNCH_FILE: Final[str] = "teleop-launch.py"
JOY_CONFIG_FILE: Final[str] = "8bitdo.yaml"

# File Paths
description_launch_path = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG),
    LAUNCH_DIR,
    DESCRIPTION_LAUNCH_FILE,
)
hardware_launch_path = os.path.join(
    get_package_share_directory(TB3_BRINGUP_PKG),
    LAUNCH_DIR,
    HARDWARE_LAUNCH_FILE,
)
controls_launch_path = os.path.join(
    get_package_share_directory(TB3_CONTROL_PKG),
    LAUNCH_DIR,
    CONTROL_LAUNCH_FILE,
)
teleop_joy_launch_path = os.path.join(
    get_package_share_directory(TELEOP_JOY_PKG),
    LAUNCH_DIR,
    TELEOP_JOY_LAUNCH_FILE
)
joy_config_path = os.path.join(
    get_package_share_directory(TB3_BRINGUP_PKG),
    CONFIG_DIR,
    JOY_CONFIG_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    1. Include description launch
    2. Include hardware launch
    3. Spawn robot in hardware
    4. Launch controls (sim)
    """

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_path]),
        launch_arguments={"sim": "false"}.items(),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([hardware_launch_path]),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "turtlebot3",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-Y",
            "0.0",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    controls_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([controls_launch_path]),
        launch_arguments={"sim":"false"}.items(),
    )

    controls_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                LogInfo(msg="Robot spawned, loading controllers"),
                controls_launch,
            ],
        )
    )

    teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([teleop_joy_launch_path]),
        launch_arguments={
            "joy_vel":"turtlebot_base_controller/cmd_vel", 
            "publish_stamped_twist": "true",
            "config_filepath": joy_config_path}.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description_launch)
    ld.add_action(sim_launch)
    ld.add_action(spawn_robot)
    ld.add_action(controls_handler)
    ld.add_action(teleop_joy)
    return ld
