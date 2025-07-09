import launch
from   launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os.path

def  generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='urdf_test').find('urdf_test')
    urdfModePath = os.path.join(pkgPath, 'urdf/model.urdf')
    urdfConfigPath = os.path.join( get_package_share_directory( 'urdf_test' ), 'config/urdf_test.rviz')

    print( urdfModePath )
    print( urdfConfigPath )

    with open(urdfModePath, 'r') as infp:
        robot_desc = infp.read()

    print( robot_desc )
    params = { 'robot_description' : robot_desc }

    robot_state_publisher_node = launch_ros.actions.Node(
                        package = 'robot_state_publisher',
                        executable = 'robot_state_publisher',
                        output = 'screen',
                        parameters = [params]
                    )
 
    joint_state_publisher_node = launch_ros.actions.Node(
                        package = 'joint_state_publisher',
                        executable = 'joint_state_publisher',
                        name = 'joint_state_publisher',
                        parameters = [params],
                        condition = launch.conditions.UnlessCondition( LaunchConfiguration( 'gui' ) )
                    )
    
    joint_state_publisher_gui_node = launch_ros.actions.Node(
                        package = 'joint_state_publisher_gui',
                        executable = 'joint_state_publisher_gui',
                        name = 'joint_state_publisher_gui',
                        condition = launch.conditions.UnlessCondition( LaunchConfiguration( 'gui' ) )
                    )

    rviz_node = launch_ros.actions.Node(
                        package = 'rviz2',
                        executable = 'rviz2',
                        name = 'rviz2',
                        output = 'screen',
                        arguments = [ '-d ' + urdfConfigPath ]
                    )

    return launch.LaunchDescription( 
            [
                launch.actions.DeclareLaunchArgument(
                    name = 'gui',
                    default_value = 'True',
                    description = 'This is a flag for joint_state_published_gui'
                  ),
                launch.actions.DeclareLaunchArgument(
                    name = 'model',
                    default_value = urdfModePath,
                    description = 'Path to urdf mode file'
                  ),
                robot_state_publisher_node,
                joint_state_publisher_node,
                joint_state_publisher_gui_node,
                rviz_node
            ] )



    