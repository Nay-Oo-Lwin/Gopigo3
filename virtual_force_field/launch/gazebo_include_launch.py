from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import subprocess 
 
def generate_launch_description():
    pkg_name_world = 'virtual_force_field'
    world_file_subpath = 'worlds/messy_obstacles.world'
    world_file = os.path.join(get_package_share_directory(pkg_name_world),world_file_subpath)
 
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),
 
        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),
 
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={
                'world': world_file
            }.items()
        ),
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
    ])
