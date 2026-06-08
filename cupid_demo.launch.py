#!/usr/bin/env python3
"""
Cupid Demo — 실제 TurtleBot3 연동 런치 파일

[ 터미널 구성 ]
  터틀봇 (로봇에서):
    ros2 launch turtlebot3_bringup robot.launch.py

  노트북 (이 파일):
    source /home/jshim/Desktop/workspace/turtlebot3_ws/install/setup.bash
    ros2 launch /home/jshim/Desktop/Cupid/cupid_demo.launch.py

[ 브라우저 ]
  http://localhost:8080/cupid42_demo.html
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

CUPID       = '/home/jshim/Desktop/Cupid'
MAP_YAML    = os.path.join(CUPID, '2026_0602_1518_2F.yaml')
NAV2_PARAMS = os.path.join(CUPID, 'config', 'nav2_params.yaml')
AMCL_PARAMS = os.path.join(CUPID, 'config', 'amcl_params.yaml')
RVIZ_CFG    = os.path.join(CUPID, 'config', 'nav2.rviz')


def generate_launch_description():
    rviz_args = ['-d', RVIZ_CFG] if os.path.exists(RVIZ_CFG) else []

    return LaunchDescription([

        # 맵 서버 — 고정 맵 로드
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': MAP_YAML, 'use_sim_time': False}],
            output='screen',
        ),

        # AMCL — 라이다 기반 위치 추정
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[AMCL_PARAMS],
            output='screen',
        ),

        # 경로 계획
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[NAV2_PARAMS],
            output='screen',
        ),

        # 경로 추종
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[NAV2_PARAMS],
            output='screen',
        ),

        # 회피 행동 (spin, backup 등)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[NAV2_PARAMS],
            output='screen',
        ),

        # 행동 트리 네비게이터
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[NAV2_PARAMS],
            output='screen',
        ),

        # 라이프사이클 매니저 — 위 노드들 자동 활성화
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
                'autostart': True,
                'use_sim_time': False,
            }],
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),

        # Cupid 웹 브리지 (HTTP :8080 + WebSocket :8765)
        ExecuteProcess(
            cmd=['python3', os.path.join(CUPID, 'web_bridge.py')],
            output='screen',
            name='cupid_web_bridge',
        ),
    ])
