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

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_controller'],
        output='screen'
    )

    delay_pos = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[position_controller],
        )
    )


    return LaunchDescription([
        joint_state_broadcaster,
        position_controller,
        delay_pos,
    ])
