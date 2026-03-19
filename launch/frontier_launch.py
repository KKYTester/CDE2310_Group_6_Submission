from launch import LaunchDescription
from launch.actions import TimerAction,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    nav2_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('slam_toolbox')
    explore_dir = get_package_share_directory('explore_lite')
    nav2_params = './param_files/nav2_params.yaml'
    slam_toolbox_params = './param_files/mapper_params_online_async.yaml'

    rviz_config = os.path.join(
        nav2_dir,
        'rviz',
        'nav2_default_view.rviz'
    )

    # Slam_Toolbox launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': slam_toolbox_params,
        }.items()
    )

    # Nav2 bringup launch
    nav2_launch = TimerAction(
        period=5.0,  # seconds delay
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params,
                }.items()
            )
        ]
    )

    # RViz launch
    rviz_node = TimerAction(
        period=10.0,  # seconds delay
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': False}],
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )

    # Frontier Exploration launch
    frontier_launch = TimerAction(
        period=15.0,  # seconds delay
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(explore_dir, 'launch', 'explore.launch.py')
                )
            )
        ]
    ) 
    
    return LaunchDescription([
        slam_launch,
        nav2_launch,
        rviz_node,
        frontier_launch,
    ])
