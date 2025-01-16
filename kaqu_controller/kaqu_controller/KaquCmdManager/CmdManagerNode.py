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
        self.subscription = self.create_subscription(
            JoyCtrlCmds,
            'kaqu_joy_ctrl_cmd',
            self.joystick_callback,
            10
        )

    def joystick_callback(self, msg):

        # Log the raw message received
        self.get_logger().info(f"Received message: {msg}")

        # Log states for debugging
        self.get_logger().info(f"States: {msg.states}, Gait Type: {msg.gait_type}, Pose: {msg.pose}")

        if msg.states[0]:  # Start
            self.state.behavior_state = BehaviorState.START
            self.state.start_event = True
            self.state.trot_event = False
            self.state.side_event = False

        elif msg.states[1]:  # Trot
            self.state.behavior_state = BehaviorState.TROT
            self.state.start_event = False
            self.state.trot_event = True
            self.state.side_event = False

        elif msg.states[2]:  # Rest (sidemove)
            self.state.behavior_state = BehaviorState.SIDEMOVE
            self.state.start_event = False
            self.state.trot_event = False
            self.state.side_event = True
            
       # 상태값 출력, imu나 회전 쪽도 출력할 수도 있음
        self.get_logger().info(f'Current State: {self.state.behavior_state.name}')

        # 각 상태 플래그 출력
        self.get_logger().info(f"Start Event: {self.state.start_event}")
        self.get_logger().info(f"Trot Event: {self.state.trot_event}")
        self.get_logger().info(f"Side Event: {self.state.side_event}")


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