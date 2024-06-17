import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # create urdf description
    pkg_brunhilde_description = get_package_share_directory('brunhilde_description')
    xacro_file = os.path.join(pkg_brunhilde_description, 'resource/xacro/brunhilde.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                                get_package_prefix('brunhilde_description') + '/share'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
                }],
            arguments=[xacro_file]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
            )
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                #'config_file': os.path.join(pkg_project_bringup, 'config','ros_gz_example_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen'
        ),

        Node(package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                        '-entity', 'Brunhilde'],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_brunhilde_description, 'resource', 'rviz', 'brunhilde.rviz')]
        ),
    ])
