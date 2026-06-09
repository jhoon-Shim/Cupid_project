#!/usr/bin/env python3
"""
Cupid Web Bridge
- HTTP  → http://localhost:8080/cupid42_demo.html
- WS    → ws://localhost:8765
"""
import asyncio
import base64
import http.server
import json
import os
import threading
import time

try:
    import cv2
    import numpy as np
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CompressedImage
import websockets

# 카메라 센서는 BEST_EFFORT QoS 사용 (기본 RELIABLE로 구독하면 프레임 못 받음)
CAM_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

SOFAS = {
    1: {'x': 8.466938655420897,  'y': 1.0469112458221683,  'oz': -0.11496345474109766, 'ow': 0.9933697217421072},
    2: {'x': 8.27739158121794,   'y': 2.6896373844916814,   'oz':  0.13231779127202836, 'ow': 0.9912073456713746},
    3: {'x': 8.294669666639418,  'y': 4.615618591934653,    'oz':  0.13470706405635816, 'ow': 0.9908854660823905},
    4: {'x': 8.198006176707063,  'y': 6.632291138863992,    'oz':  0.758381752062777,   'ow': 0.6518106459227194},
    5: {'x': 6.400106498963835,  'y': 6.8202968391286385,   'oz':  0.7906132023163226,  'ow': 0.6123159023928167},
}

_clients: set = set()
_node = None
_loop = None


class CupidBridgeNode(Node):
    # 압축 이미지 토픽 (빠름, 터틀봇 카메라가 여기 퍼블리시)
    CAM_COMPRESSED_TOPICS = [
        '/camera/image_raw/compressed',
        '/raspicam_node/image/compressed',
        '/camera/rgb/image_raw/compressed',
        '/image_raw/compressed',
    ]
    # 비압축 fallback
    CAM_RAW_TOPICS = [
        '/camera/image_raw',
        '/raspicam_node/image_raw',
        '/camera/rgb/image_raw',
    ]

    def __init__(self):
        super().__init__('cupid_bridge')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._last_cam_t = 0.0
        self._cam_active = False
        for topic in self.CAM_COMPRESSED_TOPICS:
            self.create_subscription(CompressedImage, topic, self._on_compressed, CAM_QOS)
            self.get_logger().info(f'Subscribing compressed: {topic}')
        if _CV2_OK:
            for topic in self.CAM_RAW_TOPICS:
                self.create_subscription(Image, topic, self._on_image, CAM_QOS)
        self.get_logger().info('Cupid bridge node started')

    def _on_compressed(self, msg):
        now = time.time()
        if now - self._last_cam_t < 0.1:
            return
        self._last_cam_t = now
        if not self._cam_active:
            self._cam_active = True
            self.get_logger().info(f'Compressed camera OK: format={msg.format}')
        try:
            b64 = base64.b64encode(bytes(msg.data)).decode()
            if _loop:
                asyncio.run_coroutine_threadsafe(
                    _broadcast({'type': 'camera', 'data': b64}), _loop)
        except Exception as e:
            self.get_logger().warn(f'Compressed camera error: {e}')

    def _on_image(self, msg):
        now = time.time()
        if now - self._last_cam_t < 0.1:   # 10 fps cap
            return
        self._last_cam_t = now
        if not self._cam_active:
            self._cam_active = True
            self.get_logger().info(f'Camera frame received! encoding={msg.encoding} size={msg.width}x{msg.height}')
        try:
            enc = msg.encoding.lower()
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ('rgb8', 'bgr8', 'mono8'):
                ch = 1 if enc == 'mono8' else 3
                arr = arr.reshape(msg.height, msg.width, ch)
                if enc == 'rgb8':
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif enc in ('16uc1', '32fc1'):
                return  # 깊이 이미지 무시
            else:
                self.get_logger().warn(f'Unknown encoding: {enc}')
                return
            _, jpeg = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 60])
            b64 = base64.b64encode(jpeg.tobytes()).decode()
            if _loop:
                asyncio.run_coroutine_threadsafe(
                    _broadcast({'type': 'camera', 'data': b64}), _loop)
        except Exception as e:
            self.get_logger().warn(f'Camera encode error: {e}')

    def navigate(self, seat: int, on_arrive):
        self.get_logger().info(f'navigate() called → seat={seat}')
        pose = SOFAS.get(seat)
        if not pose:
            self.get_logger().warn(f'Unknown seat: {seat}')
            return

        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 action server NOT available (timeout 15s). Nav2가 실행 중인지 확인하세요.')
            return
        self.get_logger().info('Nav2 action server OK → sending goal')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = pose['oz']
        goal.pose.pose.orientation.w = pose['ow']

        def on_feedback(feedback_msg):
            fb = feedback_msg.feedback
            eta_sec = int(fb.estimated_time_remaining.sec)
            dist = round(float(fb.distance_remaining), 2)
            if _loop:
                asyncio.run_coroutine_threadsafe(
                    _broadcast({'type': 'nav_feedback', 'eta_sec': eta_sec, 'distance_remaining': dist}),
                    _loop)

        def on_result(_future):
            self.get_logger().info(f'Arrived at sofa {seat}')
            on_arrive()

        def on_goal_resp(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('Goal was rejected')
                return
            self.get_logger().info(f'Moving to sofa {seat}...')
            handle.get_result_async().add_done_callback(on_result)

        self._client.send_goal_async(
            goal, feedback_callback=on_feedback
        ).add_done_callback(on_goal_resp)


async def _broadcast(msg: dict):
    if not _clients:
        return
    data = json.dumps(msg, ensure_ascii=False)
    await asyncio.gather(*[c.send(data) for c in list(_clients)], return_exceptions=True)


async def _ws_handler(ws):
    _clients.add(ws)
    try:
        async for raw in ws:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if data.get('type') == 'goto':
                seat = int(data.get('seat', 0))
                loop = asyncio.get_running_loop()

                def on_arrive(loop=loop):
                    asyncio.run_coroutine_threadsafe(_broadcast({'type': 'arrived'}), loop)

                threading.Thread(
                    target=_node.navigate, args=(seat, on_arrive), daemon=True
                ).start()
    finally:
        _clients.discard(ws)


def _start_http():
    directory = os.path.dirname(os.path.abspath(__file__))

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=directory, **kwargs)

        def log_message(self, *_):
            pass

    httpd = http.server.HTTPServer(('0.0.0.0', 8080), QuietHandler)
    httpd.serve_forever()


def _ros_spin():
    global _node
    rclpy.init()
    _node = CupidBridgeNode()
    try:
        rclpy.spin(_node)
    finally:
        _node.destroy_node()
        rclpy.shutdown()


async def _amain():
    global _loop
    _loop = asyncio.get_running_loop()

    threading.Thread(target=_start_http, daemon=True).start()

    cam_status = '✓ cv2 사용 가능' if _CV2_OK else '✗ cv2 없음 (pip install opencv-python)'
    print()
    print('╔═══════════════════════════════════════════════════════╗')
    print('║               Cupid Web Bridge Ready                  ║')
    print('╠═══════════════════════════════════════════════════════╣')
    print('║  브라우저 → http://localhost:8080/cupid42_demo.html   ║')
    print('║  WS 연결  → ws://localhost:8765                       ║')
    print(f'║  카메라   → {cam_status:<43}║')
    print('╚═══════════════════════════════════════════════════════╝')
    print()

    async with websockets.serve(_ws_handler, '0.0.0.0', 8765):
        await asyncio.get_running_loop().create_future()


def main():
    threading.Thread(target=_ros_spin, daemon=True).start()
    time.sleep(1.5)
    asyncio.run(_amain())


if __name__ == '__main__':
    main()
