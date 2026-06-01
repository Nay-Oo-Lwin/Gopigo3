import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    simulation_launch=IncludeLaunchDescription(
       PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gopigo3_vff_nav'),'launch','gopigo3_simulation_launch.py')
                     )
                                               
                                               )
    rviz_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gopigo3_rviz'),'launch','gopigo3_rviz.launch.py')
                                      )
    )
    navigation_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gopigo3_vff_nav'),'launch','gopigo3_navigation_sim_launch.py')
                                      )
    )
    frontier_explorer_node=Node(
        package='gopigo3_vff_nav',
        executable='frontier_explorer_node',
        output='screen'
    )
    return LaunchDescription([
        simulation_launch,
        navigation_launch,
        rviz_launch,
       	frontier_explorer_node
    ]
    )
