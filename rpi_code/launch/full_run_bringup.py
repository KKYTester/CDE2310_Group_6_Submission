import os
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to the TurtleBot3 bringup launch files
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    camera_launch_path = os.path.join(turtlebot3_bringup_dir, 'launch', 'camera.launch.py')
    robot_launch_path = os.path.join(turtlebot3_bringup_dir, 'launch', 'robot.launch.py')

    return LaunchDescription([
        # 1. Start the robot bringup (sensors, drivers, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_path)
        ),

        # 2. Start the camera node with custom parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path),
            launch_arguments={
                'format': 'YUYV',
                'width': '327',
                'height': '256',
                'use_image_view': 'True'
            }.items()
        ),

        # 3. Start the ArUco marker detector node
        Node(
            package='aruco_detector',
            executable='aruco_node',
            name='aruco_node',
            output='screen'
        ),

        # 4. Start the ball launcher node
        Node(
            package='ball_launch',
            executable='ball_launch',
            name='ball_launch',
            output='screen'
        ),
    ])
