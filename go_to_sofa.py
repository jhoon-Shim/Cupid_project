import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

SOFAS = {
    1: {
        'x': 8.466938655420897,  'y': 1.0469112458221683,
        'z': -0.11496345474109766, 'w': 0.9933697217421072
    },
    2: {
        'x': 8.27739158121794,   'y': 2.6896373844916814,
        'z': 0.13231779127202836, 'w': 0.9912073456713746
    },
    3: {
        'x': 8.294669666639418,  'y': 4.615618591934653,
        'z': 0.13470706405635816, 'w': 0.9908854660823905
    },
    4: {
        'x': 8.198006176707063,  'y': 6.632291138863992,
        'z': 0.758381752062777,   'w': 0.6518106459227194
    },
    5: {
        'x': 6.400106498963835,  'y': 6.8202968391286385,
        'z': 0.7906132023163226,  'w': 0.6123159023928167
    },
}


class GoToSofa(Node):
    def __init__(self):
        super().__init__('go_to_sofa')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, sofa_num):
        pose = SOFAS[sofa_num]

        self.get_logger().info(f'{sofa_num}번 소파로 이동합니다...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose['x']
        goal_msg.pose.pose.position.y = pose['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = pose['z']
        goal_msg.pose.pose.orientation.w = pose['w']

        self._client.wait_for_server()
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다.')
            return

        self.get_logger().info('목표 수락됨. 이동 중...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'{sofa_num}번 소파 도착 완료.')


def main():
    rclpy.init()

    print('=== 소파 이동 프로그램 ===')
    print('이동 가능한 소파: 1 ~ 5번')

    try:
        num = int(input('소파 번호를 입력하세요: '))
    except ValueError:
        print('숫자를 입력해주세요.')
        return

    if num not in SOFAS:
        print(f'올바른 번호가 아닙니다. (1~5 중 입력)')
        return

    node = GoToSofa()
    node.send_goal(num)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
