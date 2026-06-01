import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    simulation_launch=IncludeLaunchDescription(
       PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('virtual_force_field'),'launch','gopigo3_simulation_launch.py')
                     )
                                               
                                               )
    rviz_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gopigo3_rviz'),'launch','gopigo3_rviz.launch.py')
                                      )
    )
    virtual_force_field_node=Node(
        package='virtual_force_field',
        executable='virtual_force_field',
        output='screen'
    )
    return LaunchDescription([
        simulation_launch,
        rviz_launch,
        virtual_force_field_node
    ]
    )
