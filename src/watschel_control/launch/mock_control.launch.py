import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

robot_controllers = PathJoinSubstitution(
    [
        FindPackageShare("watschel_control"),
        "config",
        "ros2_control.yaml",
    ]
)

def generate_launch_description():

    pkg_watschel_bringup = get_package_share_directory('watschel_bringup')

    bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_watschel_bringup, 'launch', 'bringup.launch.py')
                ),
                launch_arguments={'use_mock_hardware': 'true'}.items()
            )

    controller_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        bringup,
        controller_node,
        joint_state_broadcaster,
        trajectory_controller
    ])
