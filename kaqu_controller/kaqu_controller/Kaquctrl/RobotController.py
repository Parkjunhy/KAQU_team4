
import rclpy
from rclpy.node import Node
from Kaquctrl import RobotState, BehaviorState  # StateSubscriber가 정의한 상태 클래스
from TrotGaitController import TrotGaitController
from RestController import RestController


class RobotController(Node):
    def __init__(self, state):
        super().__init__('robot_controller')
        self.state = state

        # Gait Controllers 초기화
        self.trot_controller = TrotGaitController()
        self.rest_controller = RestController()
        self.rest_controller = StartController()

        self.current_controller = self.rest_controller

    def select_gait(self):
        """현재 상태에 따라 Gait Controller 선택"""
        if self.state.trot_event:
            self.current_controller = self.trot_controller
            self.state.trot_event = False  

        elif self.state.start_event:
            self.current_controller = self.start_controller
            self.state.start_event = False  
            
        elif self.state.rest_event:
            self.current_controller = self.rest_controller
            self.state.rest_event = False  

    def run(self):
        self.select_gait()
        self.current_controller.run()


def main(args=None):
    rclpy.init(args=args)

    # StateSubscriber에서 상태 가져오기
    state = RobotState()

    # RobotController 초기화 및 실행
    robot_controller = RobotController(state)
    try:
        while rclpy.ok():
            robot_controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
