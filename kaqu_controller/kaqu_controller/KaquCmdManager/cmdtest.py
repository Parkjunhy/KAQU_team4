### 그저 테스트용 코드, 모든 테스트는 해당 노드를 통해 이루어질 것 같음. 삭제금지.

import rclpy
from rclpy.node import Node
from kaqu_msgs.msg import JoyMsgs
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
import time


class KaquCmdManager(Node):
    def __init__(self):
        super().__init__('cmd_manager_node')

        # Joy 상태를 저장할 변수 초기화
        self.CurrentState = {
            "start": False,
            "trot": False,
            "rest": True,  # 기본값으로 'rest' 상태 설정
        }

        # Joy 메시지 구독
        self.sub1_name = 'Joy_topic'
        self.sub1 = self.create_subscription(
            JoyMsgs, self.sub1_name, self._joy_cmd_callback, 10
        )

        self.get_logger().info("KaquCmdManager Node initialized!")

    def _joy_cmd_callback(self, msg):
        if not any(msg.states):  # 기본 상태 로직
            self.CurrentState["start"] = False
            self.CurrentState["trot"] = False
            self.CurrentState["rest"] = True
        else:
            # states 값 업데이트
            self.CurrentState["start"] = msg.states[0]
            self.CurrentState["trot"] = msg.states[1]
            self.CurrentState["rest"] = msg.states[2]

        # 디버깅 출력
        self.get_logger().info(f"Updated CurrentState: {self.CurrentState}")


def main(args=None):
    rclpy.init(args=args)
    node = KaquCmdManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
