import numpy as np
import rclpy
from rclpy.node import Node
from ..KaquCmdManager.CmdManagerNode import RobotState, BehaviorState, RobotCommand
  # StateSubscriber가 정의한 상태 클래스
# from TrotGaitController import TrotGaitController
# from RestController import RestController
from std_msgs.msg import String

class TrotGaitController:
    def run(self, state, command):
        # Simulate the trot gait logic
        return "Running Trot Gait"

class SideMoveController:
    def run(self, state, command):
        # Simulate the side move logic
        return "Running Side Move"

class StartController:
    def run(self, state, command):
        # Simulate the start controller logic
        return "Starting Robot"

class RobotController(Node):
    def __init__(self, body, legs):
        super().__init__('robot_controller')  # Node 이름 설정
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.001 # 상수
        self.x_shift_back = -0.001 # 상수
        # 임의로 설정했습니다
        self.default_height = 0.5

        self.state = RobotState(self.default_height)
        self.command = RobotCommand(self.default_height)
        self.state.foot_location = self.default_stance()

        # Gait Controllers 초기화, 각 컨트롤러에 default stance외 변수 넣어야함 3조얼른줘!
        self.trot_controller = TrotGaitController()
        self.sidemove_controller = SideMoveController()
        self.start_controller = StartController()

        self.current_controller = None

        # Test를 위한 퍼블리셔
        self.controller_pub = self.create_publisher(String, '/current_controller', 10)

        # 타이머 생성 (1초마다 현재 컨트롤러 상태를 발행)
        self.timer = self.create_timer(1.0, self.publish_current_controller)

    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])

    def select_gait(self):
        """현재 상태에 따라 Gait Controller 선택"""
        if self.command.trot_event:
            self.current_controller = self.trot_controller
            # self.current_controller.pid_controller.reset() # 3조 형식따라 바뀜
            # self.state.ticks = 0
            self.state.trot_event = False   

        elif self.command.start_event:
            self.current_controller = self.start_controller
            # self.current_controller.pid_controller.reset() # 3조 형식따라 바뀜
            # self.state.ticks = 0
            self.state.start_event = False
            self.get_logger().info("Start controller selected")
            
        elif self.command.side_event:
            self.current_controller = self.sidemove_controller
            # self.current_controller.pid_controller.reset() # 3조 형식따라 바뀜
            # self.state.ticks = 0
            self.state.side_event = False

    def publish_current_controller(self):
        """현재 컨트롤러 상태를 발행"""
        if self.current_controller:
            msg = String()
            msg.data = str(self.current_controller)  # 컨트롤러 이름을 문자열로 변환
            self.get_logger().info(f"Publishing current controller: {msg.data}")
            self.controller_pub.publish(msg)
       
    def run(self):
        if self.current_controller:
            return self.current_controller.run(self.state, self.command)
        else:
            self.get_logger().warn("No controller to run")
            return None

    
def main(args=None):
    rclpy.init(args=args)

    # Body와 Legs의 초기값 설정
    body_dimensions = [0.5, 0.3]  # [length, width]
    leg_dimensions = [0.2, 0.1]  # [length, offset]

    # RobotController 초기화
    robot_controller = RobotController(body_dimensions, leg_dimensions)

    # 이벤트 시뮬레이션
    robot_controller.select_gait()

    try:
        while rclpy.ok():
            rclpy.spin_once(robot_controller, timeout_sec=0.1)

            # Run the active controller logic
            result = robot_controller.run()
            if result is not None:
                robot_controller.get_logger().info(f"Controller result: {result}")
            else:
                robot_controller.get_logger().warn("No controller running.")
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Shutting down RobotController.")

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()