import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    simultation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gopigo3_simulation'),'launch','gopigo3_simulation_launch.py')
        )
    )
    rviz_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gopigo3_rviz'),
                         'launch',
                         'gopigo3_rviz.launch.py'
                         )
        )
    )
    wall_follower_node=Node(
        package='wall_follower',
        executable='wall_follower',
        output='screen',
    )
    return LaunchDescription([
        simultation_launch,
        rviz_launch,
        wall_follower_node
    ]
    )