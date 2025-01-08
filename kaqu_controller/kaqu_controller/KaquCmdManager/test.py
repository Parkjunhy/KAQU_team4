### 조이스틱에서 Joy_topic으로 값 보내는 예제입니다.


import rclpy
from rclpy.node import Node
from kaqu_msgs.msg import JoyMsgs  # Joy_msg 메시지 타입
import time


class JoyPublisherNode(Node):
    def __init__(self, node_name="JoyPublisherNode"):
        super().__init__(node_name)

        # Joy_topic 퍼블리셔 설정
        self.publisher = self.create_publisher(JoyMsgs, 'Joy_topic', 10)

        # 발행 속도 (1초 간격)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # 현재 상태 인덱스
        self.state_index = 0

        # 전송할 states 메시지 리스트 (start, trot, rest)
        self.state_sequence = [
            [True, False, False],  # start
            [False, True, False],  # trot
            [False, False, True],  # rest
        ]

        self.get_logger().info("JoyPublisherNode initialized and started!")

    def timer_callback(self):
        # Joy_msg 메시지 생성
        msg = JoyMsgs()
        msg.states = self.state_sequence[self.state_index]

        # 메시지 발행
        self.publisher.publish(msg)
        self.get_logger().info(f"Published states: {msg.states}")

        # 다음 상태로 이동 (순환)
        self.state_index = (self.state_index + 1) % len(self.state_sequence)


def main(args=None):
    rclpy.init(args=args)

    # 노드 실행
    joy_publisher_node = JoyPublisherNode()
    rclpy.spin(joy_publisher_node)

    # 종료 시 정리
    joy_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
