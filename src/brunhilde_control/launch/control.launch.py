import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


robot_controllers = PathJoinSubstitution(
    [
        FindPackageShare("brunhilde_control"),
        "config",
        "ros2_control.yaml",
    ]
)

def generate_launch_description():
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

    delay_pos = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[trajectory_controller],
        )
    )


    return LaunchDescription([
        controller_node,
        joint_state_broadcaster,
        trajectory_controller
    ])
