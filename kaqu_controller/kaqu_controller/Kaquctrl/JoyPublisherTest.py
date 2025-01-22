import rclpy
from rclpy.node import Node
from kaqu_msgs.msg import JoyCtrlCmds  # 메시지 유형
from geometry_msgs.msg import Pose, Vector3  # 메시지 내 사용된 타입
from std_msgs.msg import Header  # Header 타입

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        # 퍼블리시할 토픽을 'RobotController2'로 변경
        self.publisher = self.create_publisher(JoyCtrlCmds, 'RobotController2', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # 1초마다 퍼블리시
        self.state_index = 0  # 현재 상태 인덱스

    def publish_message(self):
        # 메시지 생성
        msg = JoyCtrlCmds()

        # Header 초기화
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 상태 값 설정 (순환)
        states_list = [
            [True, False, False],  # START
            [False, True, False],  # TROT
            [False, False, True],  # SIDEMOVE
        ]
        msg.states = states_list[self.state_index]

        # 상태 변경 (0 -> 1 -> 2 -> 0)
        self.state_index = (self.state_index + 1) % len(states_list)

        # Gait Type
        msg.gait_type = self.state_index  # 현재 상태와 같은 값으로 설정

        # Pose
        msg.pose = Pose()
        msg.pose.position.x = 0.1  # slant-x
        msg.pose.position.y = 0.2  # slant-y
        msg.pose.position.z = -0.3  # height
        msg.pose.orientation.x = 0.0  # roll
        msg.pose.orientation.y = 0.0  # pitch
        msg.pose.orientation.z = 0.0  # yaw
        msg.pose.orientation.w = 1.0  # 기본값

        # Gait Step
        msg.gait_step = Vector3()
        msg.gait_step.x = 0.5  # steplen_x
        msg.gait_step.y = 0.0  # steplen_y
        msg.gait_step.z = 0.1  # swing_height

        # 메시지 로그 출력 및 퍼블리시
        self.get_logger().info(f'Publishing: {msg}')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
