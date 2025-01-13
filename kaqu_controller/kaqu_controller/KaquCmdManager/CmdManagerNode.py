### CmdManager에서는 조이스틱에서 조이토픽으로 보내는 값을 구독함. 그리고 그 값이 각각 어떤 state를 의미하는지 정의함.
### 정의된 state는 robotcontroller에서 어떤 gait을 사용하라 할건지 명령할 예정.
### imu에 관한 것들은 Gazebosim에서 할지 여기서 받을지 좀더 고민해야함.


import rclpy
from rclpy.node import Node
from kaqu_msgs.msg import JoyMsgs  
from enum import Enum


class BehaviorState(Enum):
    START = 0
    TROT = 1
    REST = 2

class RobotState:
    def __init__(self):
        self.behavior_state = BehaviorState.REST  # 기본 상태
        self.velocity = [0.0, 0.0]  # 속도 (x, y)
        self.yaw_rate = 0.0  # Yaw 회전 속도
        self.robot_height = -0.15  # 로봇 기본 높이, 바뀔예정
        self.imu_roll = 0.0  # IMU Roll
        self.imu_pitch = 0.0  # IMU Pitch

        # 각 상태 플래그
        self.trot_event = False
        self.rest_event = False
        self.start_event = False


class StateSubscriber(Node):
    def __init__(self):
        super().__init__('state_subscriber')
        self.state = RobotState()
        self.subscription = self.create_subscription(
            JoyMsgs,
            'Joy_topic',
            self.joystick_callback,
            10
        )

    def joystick_callback(self, msg):
        if msg.States[0]:  # Start
            self.state.behavior_state = BehaviorState.START
            self.state.start_event = True
            self.state.trot_event = False
            self.state.rest_event = False

        elif msg.States[1]:  # Trot
            self.state.behavior_state = BehaviorState.TROT
            self.state.start_event = False
            self.state.trot_event = True
            self.state.rest_event = False

        elif msg.States[3]:  # Rest
            self.state.behavior_state = BehaviorState.REST
            self.state.start_event = False
            self.state.trot_event = False
            self.state.rest_event = True

       # 상태값 출력, imu나 회전 쪽도 출력할 수도 있음
        self.get_logger().info(f'Current State: {self.state.behavior_state.name}')


def main(args=None):
    rclpy.init(args=args)
    node = StateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


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
