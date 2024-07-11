import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

import xacro

def create_robot_description(context):
    urdf_file_name = 'resource/urdf/example_robot.urdf.xacro'

    urdf = os.path.join(get_package_share_directory('watschel_description'), urdf_file_name)

    doc = xacro.process_file(urdf, mappings={'use_sim_hardware': context.launch_configurations['use_sim'],
                                             'use_mock_hardware': context.launch_configurations['use_mock_hardware'],
                                             'use_real_hardware': context.launch_configurations['use_hardware']})
    robot_desc = doc.toprettyxml(indent='  ')
    return [SetLaunchConfiguration('robot_description', robot_desc)]

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='false')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='false')
    use_hardware = LaunchConfiguration('use_hardware', default='true')
    use_rviz = LaunchConfiguration('rviz', default='true')

    robot_desc = OpaqueFunction(function=create_robot_description)

    arg_sim = DeclareLaunchArgument(
                'use_sim',
                default_value='false',
                description='Bringup everything needed for Gazebo simulation.'
            )

    arg_mock_hardware = DeclareLaunchArgument(
                        'use_mock_hardware',
                        default_value='false',
                        description='Use mock hardware interface for testing.'
                        )

    arg_hardware = DeclareLaunchArgument(
                    'use_hardware',
                    default_value='true',
                    description='Use real hardware interface.'
                )


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
                'use_sim_time': use_sim,
                'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)
                }]
            )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            arguments=['-d', os.path.join(get_package_share_directory('watschel_description'), 'resource', 'rviz', 'watschel.rviz')],
            condition=IfCondition(use_rviz),
            )

    return LaunchDescription([
        arg_sim,
        arg_mock_hardware,
        arg_hardware,
        arg_rviz,
        robot_desc,
        rsp,
        rviz,
    ])