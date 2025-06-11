import os
from typing import Final, List

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration

# Packages
DESCRIPTION_PKG: Final[str] = "turtlebot3_description"

# Directories
URDF_DIR: Final[str] = "urdf"

# Files
ROBOT_XACRO_FILE: Final[str] = "turtlebot3_waffle.urdf.xacro"

# File Paths
robot_xacro_path = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), URDF_DIR, ROBOT_XACRO_FILE
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


def render_xacro(context: LaunchContext) -> List[SetLaunchConfiguration]:
    """
    Parse robot xacro file to set robot_description launch configuration
    """

    doc = xacro.process_file(
        robot_xacro_path,
        mappings={"sim": context.launch_configurations["sim"]},
    )

    robot_description = doc.toprettyxml(indent="  ")
    return [SetLaunchConfiguration("robot_description", robot_description)]


def generate_launch_description() -> LaunchDescription:
    """
    1. Call render_xacro
    2. Runs robot_state_publisher node
    """

    tf_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": LaunchConfiguration("robot_description")},
        ],
        remappings=tf_remappings,
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=render_xacro))
    ld.add_action(robot_state_publisher_node)
    return ld
