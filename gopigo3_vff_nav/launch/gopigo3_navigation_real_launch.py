import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():

    slam_params_file = os.path.join(get_package_share_directory('gopigo3_vff_nav'),'config','slam_real_params.yaml')
    slam_toolbox = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
                launch_arguments={'use_sim_time' : 'False',
                                  'slam_params_file': slam_params_file
                                    }.items()
            ),
        ]
    )

    nav2_params_file = os.path.join(get_package_share_directory('gopigo3_vff_nav'),'config','nav2_real_params.yaml')
    nav2_bringup = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
                launch_arguments={'params_file': nav2_params_file,
                                    'headless' : 'False',
                                    'use_sim_time' : 'False'
                                    }.items()
            )
        ]
    )

    return LaunchDescription([
        slam_toolbox,
        nav2_bringup
    ])