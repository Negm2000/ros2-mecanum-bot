import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get the robot description
    urdf_path = os.path.join(get_package_share_directory('mecanumbot_description'), 'urdf', 'mecanumbot.urdf')
    urdf_doc = xacro.parse(open(urdf_path, 'r'))
    xacro.process_doc(urdf_doc)
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    robot_controller_config = os.path.join(get_package_share_directory('mecanumbot_description'), 'config', 'robot_controller_config.yaml')


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('mecanumbot_bringup'),'launch','mecanumbot_state_publisher.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    robot_controller_config]
    )


    mec_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanumbot_drive_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller"],
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        controller_manager,
        joint_broad_spawner,
        mec_drive_spawner,
    ])

