import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_joy = get_package_share_directory("joy")
    config_directory = os.path.join(pkg_joy, 'config')
    params = os.path.join(config_directory, 'joy-params.yaml')

    return LaunchDescription([

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            parameters=[params],
        ),
    ])