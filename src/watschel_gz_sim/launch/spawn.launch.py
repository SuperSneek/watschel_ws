import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    model = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                    os.path.join(get_package_prefix('watschel_description'), 'share')
            )

    spawner = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'watschel',
                    '-topic', 'robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', '1000'
                ],
                output='screen',
            )