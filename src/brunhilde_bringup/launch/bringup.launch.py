import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

import xacro


def generate_launch_description():
    urdf_file_name = 'resource/xacro/brunhilde.urdf.xacro'

    urdf = os.path.join(
        get_package_share_directory('brunhilde_description'),
        urdf_file_name)

    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='  ')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('rviz', default='true')

    arg_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

    arg_rviz = DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with this launch file.",
            )

    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
                }],
            arguments=[urdf])

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', os.path.join(get_package_share_directory('brunhilde_description'), 'resource', 'rviz', 'brunhilde.rviz')],
            condition=IfCondition(use_rviz),
            )

    return LaunchDescription([
        arg_sim_time,
        arg_rviz,
        rsp,
        rviz,
    ])