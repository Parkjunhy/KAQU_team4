### CmdManager에서는 조이스틱에서 조이토픽으로 보내는 값을 구독함. 그리고 그 값이 각각 어떤 state를 의미하는지 정의함.
### 정의된 state는 robotcontroller에서 어떤 gait을 사용하라 할건지 명령할 예정.
### 또한, IK를 거친 각도값에 대한 publish도 여기서 할 예정.
### imu에 관한 것들은 Gazebosim에서 할지 여기서 받을지 좀더 고민해야함.


import rclpy
from rclpy.node import Node
from kaqu_msgs.msg import JoyMsgs

# 어떤 커맨드일지 총괄 관리하는 클래스임
class Command(object):
    def __init__(self):
        # Joy initialize(기본값 rest로)
        self.current_state = {
            "start": False,
            "trot": False,
            "rest": True,  
        }

    # 이 method에서 state값을 update함.
    def update_state(self, states):
        if not any(states):  # 기본 상태 로직
            self.current_state["start"] = False
            self.current_state["trot"] = False
            self.current_state["rest"] = True
        else:
            self.current_state["start"] = states[0]
            self.current_state["trot"] = states[1]
            self.current_state["rest"] = states[2]

    def get_state(self):
        # 나중에 robotctrler 에서 Command.get_state() ~~하는 방식으로 가져오면 신속한처리가능, if문 달듯
        return self.current_state

# 여기에 각 관절의 값 전체 퍼블리쉬하는거 추가할 예정
class KaquCmdManager(Node):
    def __init__(self):
        super().__init__('cmd_manager_node')

        # Command 객체 생성
        self.command = Command()

        # 조이스틱에서는 Joy_topic으로 값 준다고 가정함. 이름은 변경 가능
        self.sub1_name = 'Joy_topic'
        self.sub1 = self.create_subscription(
            JoyMsgs, self.sub1_name, self._joy_cmd_callback, 10
        )

        self.get_logger().info("KaquCmdManager Node initialized!")



    def _joy_cmd_callback(self, msg):
        # command클래스의 값을 업데이트 해주는 방식
        self.command.update_state(msg.states)

        self.get_logger().info(f"Updated CurrentState: {self.command.get_state()}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = KaquCmdManager()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
