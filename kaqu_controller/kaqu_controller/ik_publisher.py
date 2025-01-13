import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node

class IKPublisher(Node):
    def __init__(self, joint_names):
        super().__init__('ik_publisher')

        # JointState 퍼블리셔 생성
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # IK에서 사용하는 발의 이름을 동적으로 설정
        self.joint_names = joint_names

    def publish_joint_angles(self, joint_angles):
        # JointState 메시지 생성
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        
        # 발 위치에 대한 각도를 나타내는 name 필드 추가
        joint_state.name = self.joint_names  # 발 위치나 각도를 설정할 이름들
        
        # IK 계산 후 받은 각도값을 position 배열에 추가
        joint_state.position = joint_angles  # joint_angles는 IK 계산 후 반환된 각도값 배열이어야 합니다.

        # 퍼블리시
        self.joint_pub.publish(joint_state)
        self.get_logger().info(f'Publishing joint angles: {joint_angles}')

def main(args=None):
    rclpy.init(args=args)

    # 예시 발의 위치 이름 (동적으로 설정 가능)
    joint_names = [
        "joy_fr_hip", "joy_fr_knee", "joy_fr_ankle",
        "joy_fl_hip", "joy_fl_knee", "joy_fl_ankle",
        "joy_rr_hip", "joy_rr_knee", "joy_rr_ankle",
        "joy_rl_hip", "joy_rl_knee", "joy_rl_ankle"
    ]
    
    # IKPublisher 노드 생성
    ik_publisher = IKPublisher(joint_names)

    # 예시로 임의의 각도값 (발 위치를 계산한 후 반환된 각도값)
    joint_angles = [0.5, -0.3, 0.7, -0.5, 0.2, -0.7, 0.6, -0.4, 0.8, -0.6, 0.4, -0.2]

    # IK 계산 후 각도값을 퍼블리시
    ik_publisher.publish_joint_angles(joint_angles)

    # rclpy.spin()으로 노드를 계속 실행시킴
    rclpy.spin(ik_publisher)

    ik_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
