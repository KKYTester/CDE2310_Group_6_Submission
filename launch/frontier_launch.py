from launch import LaunchDescription
from launch.actions import TimerAction,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Get package directories for nav2_bringup and explore_lite ros2 launch files
    nav2_dir = get_package_share_directory('nav2_bringup')
    explore_dir = get_package_share_directory('explore_lite')
    # Frontier launch directory (for param files)
    frontier_launch_dir = os.path.dirname(__file__)
    # Param file paths
    nav2_params = os.path.join(frontier_launch_dir, 'param_files', 'nav2_params.yaml')
    slam_toolbox_params = os.path.join(frontier_launch_dir, 'param_files', 'mapper_params_online_async.yaml')

    rviz_config = os.path.join(
        nav2_dir,
        'rviz',
        'nav2_default_view.rviz'
    )

    # Scan filterer node
    scan_resampler_node = Node(
        package='scan_filter',
        executable='scan_resampler',
        name='scan_resampler',
        output='screen',
    )

    # Slam_Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {'use_sim_time': False,
             'scan_topic': '/scan_filtered'}
        ]
    )

    docking_node = Node(
        package='docking_pid',
        executable='docking_pid',
        name='docking_pid',
        output='screen',
        prefix=['xterm -e'],
    )

    # nav2aruco node
    nav2aruco_node = Node(
        package='nav2aruco',
        executable='nav2aruco',
        name='nav2aruco',
        output='screen',
        prefix=['xterm -e'],
    )

    # random_nav node launch
    random_nav_node = Node(
        package='random_nav',
        executable='random_nav',
        name='random_nav',
        output='screen',
        prefix=['xterm -e'],
    )

    delayed_random_nav = TimerAction(
        period=10.0,  # seconds
        actions=[random_nav_node]
    )

    # Mission FSM node launch
    mission_fsm_node = Node(
        package='mission_fsm',
        executable='mission_fsm',
        name='mission_fsm',
        output='screen',
        prefix=['xterm -e'], # Runs FSM node in separate terminal for easier debugging and monitoring
    )

    delayed_mission_fsm = TimerAction(
        period=20.0,  # seconds
        actions=[mission_fsm_node]
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
        docking_node,
        scan_resampler_node,
        slam_node,
        nav2aruco_node,
        delayed_random_nav,
        delayed_mission_fsm,
        nav2_launch,
        rviz_node,
        frontier_launch,
    ])
