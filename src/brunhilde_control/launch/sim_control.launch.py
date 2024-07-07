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
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    delay_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[trajectory_controller],
        )
    )


    return LaunchDescription([
        joint_state_broadcaster,
        trajectory_controller
    ])
