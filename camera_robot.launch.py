# This file should be in Turtlebot3
# ~/Workspace/ros2_ws/src/my_robot_bringup/launch/camera_robot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 터틀봇3 Bringup 런처 (기존 패키지 파일 가져오기)
    tb3_launch_dir = os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch')
    
    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tb3_launch_dir, '/robot.launch.py'])
    )

    # 2. 카메라 노드 실행 설정
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [320, 240],
            'output_encoding': 'rgb8',
        }]
    )

    return LaunchDescription([
        turtlebot3_bringup,
        camera_node
    ])