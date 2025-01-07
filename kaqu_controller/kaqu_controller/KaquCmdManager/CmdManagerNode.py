import rclpy
from rclpy.node import Node
import threading
from kaqu_msgs.msg import Joy_msg 
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
import time


class KaquCmdManager:
    def __init__(self, set_msgs, send_msgs, node_name='CmdManager_Node'):
        # ROS parameters
        self.node = None
        self.node_name = node_name

        # Joy 상태를 저장할 변수 초기화
        self.CurrentState = {
            "start": False,
            "trot": False,
            "rest": True,  # 기본값으로 'rest' 상태 설정
        }

        # ----- 조이스틱 값 구독 --------
        self.sub1 = None
        self.sub1_name = 'Joy_topic'
        self.sub1_interface = Joy_msg  # 조이스틱 데이터타입
        self.sub1_callback = self._joy_cmd_callback
        self.sub1_queueSize = 30

        # -----------gait 값

        self.stop = True

        # Robot cmds
        self.cmd = set_msgs
        self.pub_msgs = send_msgs

    def _createNode(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.node_name)

    def create_sub1(self):
        self.sub1 = self.node.create_subscription(
            self.sub1_interface,
            self.sub1_name,
            self.sub1_callback,
            self.sub1_queueSize,
        )

    def _joy_cmd_callback(self, msg):
        if not any(msg.states):  # default logic
            self.CurrentState["start"] = False
            self.CurrentState["trot"] = False
            self.CurrentState["rest"] = True
        else:
            # states 값 업데이트
            self.CurrentState["start"] = msg.states[0]
            self.CurrentState["trot"] = msg.states[1]
            self.CurrentState["rest"] = msg.states[2]

# 이렇게 받아오고 robotcontroller에서 currentstate임포트해서 값이 start면 시작하게
# trot이면 trotgaitcontrler 발동! 할예정

        # 디버깅 출력
        self.node.get_logger().info(f"Updated CurrentState: {self.CurrentState}")

    def start(self):
        self._createNode()
        self.create_sub1()
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
        self.stop = False


# 실행 예시
if __name__ == "__main__":
    set_msgs = None  # 실제 값으로 초기화
    send_msgs = None  # 실제 값으로 초기화
    cmd_manager = KaquCmdManager(set_msgs, send_msgs)
    thread1 = threading.Thread(target=cmd_manager.start)
    thread1.start()

    try:
        while thread1.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        cmd_manager.stop()
