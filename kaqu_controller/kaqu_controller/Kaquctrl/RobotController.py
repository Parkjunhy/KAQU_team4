import numpy as np
import rclpy
from rclpy.node import Node
from KaquCmdManager.CmdManagerNode import RobotState, BehaviorState, RobotCommand  # StateSubscriber가 정의한 상태 클래스
from TrotGaitController import TrotGaitController
from RestController import RestController


class RobotController(object):
    def __init__(self, body, legs):
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.001 # 상수
        self.x_shift_back = -0.001 # 상수
        
        self.state = RobotState(self.default_height)
        self.command = RobotCommand(self.default_height)
        self.state.foot_location = self.default_stance

        # Gait Controllers 초기화, 각 컨트롤러에 default stance외 변수 넣어야함 3조얼른줘!
        self.trot_controller = TrotGaitController(self.default_stance)
        self.rest_controller = RestController(self.default_stance)
        self.start_controller = StartController()

        self.current_controller = self.rest_controller

    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])

    def select_gait(self):
        """현재 상태에 따라 Gait Controller 선택"""
        if self.command.trot_event:
            self.current_controller = self.trot_controller
            self.current_controller.pid_controller.reset() # 3조 형식따라 바뀜
            self.state.ticks = 0
            self.state.trot_event = False  

        elif self.command.start_event:
            self.current_controller = self.start_controller
            self.current_controller.pid_controller.reset() # 3조 형식따라 바뀜
            self.state.ticks = 0
            self.state.start_event = False  
            
        elif self.command.rest_event:
            self.current_controller = self.rest_controller
            self.current_controller.pid_controller.reset() # 3조 형식따라 바뀜
            self.state.ticks = 0
            self.state.rest_event = False  

       

    def run(self):
        return self.current_controller.run(self.state, self.command)

    


# def main(args=None):
#     rclpy.init(args=args)

#     # StateSubscriber에서 상태 가져오기
#     Rs = RobotState()

#     # RobotController 초기화 및 실행
#     robot_controller = RobotController(Rs)
#     try:
#         while rclpy.ok():
#             robot_controller.run()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         robot_controller.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
