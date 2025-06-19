import os
from pathlib import Path
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Packages
ROS_GZ_PKG: Final[str] = "ros_gz_sim"
TB3_DESCRIPTION_PKG: Final[str] = "turtlebot3_description"
TB3_GZ_BRINGUP_PKG: Final[str] = "turtlebot3_gz_bringup"

# Directories
LAUNCH_DIR: Final[str] = "launch"
WORLDS_DIR: Final[str] = "worlds"

# Files
GZ_LAUNCH_FILE: Final[str] = "gz_sim.launch.py"
WORLD_FILE: Final[str] = "simple_world.sdf"

# File Paths
gz_launch_path = os.path.join(
    get_package_share_directory(ROS_GZ_PKG), LAUNCH_DIR, GZ_LAUNCH_FILE
)
world_file_path = os.path.join(
    get_package_share_directory(TB3_GZ_BRINGUP_PKG), WORLDS_DIR, WORLD_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    1. Sets the environment variable for GZ Sim Resources
    2. Launch GZ sim
    3. Runs clock bridge node
    """

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join(
            [
                str(
                    Path(
                        get_package_share_directory(TB3_DESCRIPTION_PKG)
                    ).parent.resolve()
                )
            ]
        ),
    )

    # gz_plugin_path = SetEnvironmentVariable(
    #     name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=":".join("/opt/ros/jazzy/lib")
    # )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch_path]),
        launch_arguments=[
            (
                "gz_args",
                [
                    "shapes.sdf",
                    " -r",
                ],
            )
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock"],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    # ld.add_action(gz_plugin_path)
    ld.add_action(gz_sim_launch)
    ld.add_action(clock_bridge)
    return ld
