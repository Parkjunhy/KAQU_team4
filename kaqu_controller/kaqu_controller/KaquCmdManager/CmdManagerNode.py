### CmdManager에서는 조이스틱에서 조이토픽으로 보내는 값을 구독함. 그리고 그 값이 각각 어떤 state를 의미하는지 정의함. (O)
### 정의된 state는 robotcontroller에서 어떤 gait을 사용하라 할건지 명령할 예정.
### imu에 관한 것들은 Gazebosim에서 할지 여기서 받을지 좀더 고민해야함.


import rclpy
from rclpy.node import Node
from kaqu_msgs.msg import JoyCtrlCmds  
from enum import Enum
import numpy as np


class BehaviorState(Enum):
    START = 0
    TROT = 1
    SIDEMOVE = 2

class RobotState(object):
    def __init__(self, default_height):
        self.velocity = [0.0, 0.0]  # 속도 (x, y)
        self.yaw_rate = 0.0  # Yaw 회전 속도
        self.robot_height = -default_height  # 로봇 기본 높이, 바뀔예정(params에서 가져올듯)
        self.imu_roll = 0.0  # IMU Roll
        self.imu_pitch = 0.0  # IMU Pitch

        self.foot_location = np.zeros((3,4))
        self.body_local_position = np.array([0., 0., 0.])
        self.body_local_orientation = np.array([0., 0., 0.])

        self.ticks = 0

        self.behavior_state = BehaviorState.SIDEMOVE  # 기본 상태

        # 각 상태 플래그
class RobotCommand(object):
    def __init__(self, default_height):
        self.trot_event = False
        self.side_event = False
        self.start_event = False
        
        self.velocity = [0.0, 0.0]  # 속도 (x, y)
        self.yaw_rate = 0.0  # Yaw 회전 속도
        self.robot_height = -default_height  # 로봇 기본 높이, 바뀔예정(params에서 가져올듯)


class StateSubscriber(Node):
    def __init__(self):
        super().__init__('state_subscriber')
        self.state = RobotState(default_height=0.1)
        self.command = RobotCommand(default_height=0.1)
        self.subscription = self.create_subscription(
            JoyCtrlCmds,
            'kaqu_joy_ctrl_cmd',
            self.joystick_callback,
            10
        )

        # 새로운 퍼블리셔 추가
        self.state_pub = self.create_publisher(JoyCtrlCmds, 'robot_state', 10)

    def joystick_callback(self, msg):
        """JoyCtrlCmds 메시지를 수신하고, 상태에 맞는 보행 선택"""
        self.get_logger().info(f"Received JoyCtrlCmds: {msg}")

        # 현재 상태 값을 출력하여 디버깅
        self.get_logger().info(f"Current States in msg: {msg.states}")

        # states에 따라 gait 선택
        if msg.states[0]:  # START 상태
            if self.state.behavior_state != BehaviorState.START:
                self.state.behavior_state = BehaviorState.START
                self.state.start_event = True
                self.state.trot_event = False
                self.state.side_event = False
                self.get_logger().info(f"State changed to: {self.state.behavior_state.name}")
                self.select_gait()  # 상태가 변경될 때마다 gait 선택 호출
        elif msg.states[1]:  # TROT 상태
            if self.state.behavior_state != BehaviorState.TROT:
                self.state.behavior_state = BehaviorState.TROT
                self.state.start_event = False
                self.state.trot_event = True
                self.state.side_event = False
                self.get_logger().info(f"State changed to: {self.state.behavior_state.name}")
                self.select_gait()  # 상태가 변경될 때마다 gait 선택 호출
        elif msg.states[2]:  # SIDEMOVE 상태
            if self.state.behavior_state != BehaviorState.SIDEMOVE:
                self.state.behavior_state = BehaviorState.SIDEMOVE
                self.state.start_event = False
                self.state.trot_event = False
                self.state.side_event = True
                self.get_logger().info(f"State changed to: {self.state.behavior_state.name}")
                self.select_gait()  # 상태가 변경될 때마다 gait 선택 호출

        # 상태 값 출력
        self.get_logger().info(f'Current State: {self.state.behavior_state.name}')

        # 상태 메시지 퍼블리시 (RobotState와 Command)
        self.publish_state()

    def select_gait(self):
        """상태에 따라 로봇의 보행 모드 선택"""
        if self.state.behavior_state == BehaviorState.START:
            # START 상태에 맞는 gait 실행 (예: 초기화 동작)
            self.get_logger().info("Gait selected: START")
            # 로봇 컨트롤러에 START 명령을 전달하는 로직 필요
        elif self.state.behavior_state == BehaviorState.TROT:
            # TROT 상태에 맞는 gait 실행
            self.get_logger().info("Gait selected: TROT")
            # 로봇 컨트롤러에 TROT 명령을 전달하는 로직 필요
        elif self.state.behavior_state == BehaviorState.SIDEMOVE:
            # SIDEMOVE 상태에 맞는 gait 실행
            self.get_logger().info("Gait selected: SIDEMOVE")
            # 로봇 컨트롤러에 SIDEMOVE 명령을 전달하는 로직 필요

    def publish_state(self):
        """로봇 상태를 퍼블리시"""
        msg = JoyCtrlCmds()
        # 상태에 대한 정보를 메시지에 추가
        msg.states = [self.state.start_event, self.state.trot_event, self.state.side_event]
        # 기타 정보도 필요하면 추가 가능

        # 메시지 발행
        self.state_pub.publish(msg)
        self.get_logger().info(f"Published current state: {msg.states}")

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