import numpy as np
from KaquCmdManager.CmdManagerNode import Command
# gait controller import 하는 부분인데 아직 정확한 이름을 몰라서 임의로 설정했습니다.
from RobotController.RestController import RestController
from RobotController.TrotGaitController import TrotGaitController

# import Trot, Rest, PID etc...


# 이게 여기에 필요한지는 모르겠으나 일단 gpt형님이 필요하다고 해서 넣어봤습니다...ㅎ
# 상태 클래스 정의
class State:
    def __init__(self, default_stance):
        self.body_local_position = np.zeros(3)  # [x, y, z]
        self.body_local_orientation = np.zeros(3)  # [roll, pitch, yaw]
        self.foot_locations = default_stance  # 발 위치 초기화
        self.imu_roll = 0.0  # IMU Roll 값
        self.imu_pitch = 0.0  # IMU Pitch 값


class RobotGait(object):
    def __init__(self):
        
        # Command 클래스 객체 생성
        self.command = Command()
        self.new_state = self.command.get_state()

        # 기본 발 위치 설정
        self.default_stance = np.array([
            [0, 0, 0, 0],  # x 좌표
            [0, 0, 0, 0],  # y 좌표
            [0, 0, 0, 0]  # z 좌표 일단 다 0으로 잡았습니다.
        ])
        self.footlocations = self.default_stance

        # 상태 및 명령 객체 초기화
        self.state = State(self.default_stance)
        self.command_obj = Command(robot_height=0.15)  
        # 로봇 높이 초기화인데...저희 Command 클래스에는 이 부분이 없는 것 같네용...

        # 각 게이트 컨트롤러 초기화
        self.trot_gait_controller = TrotGaitController(self.default_stance)  # Trot 게이트
        self.rest_controller = RestController(self.default_stance)  # Rest 상태
        
    
    def gait_select(self):  
        if self.new_state["start"]:
            print("Starting gait...")
        # 시작시 추가 코드가 필요할까요?
        
        elif self.new_state["trot"]:
            print("Trot gait selected")
            self.footlocations = self.trot_gait_controller.run(self.state, self.command_obj))

        elif self.new_state["rest"]:
            print("Rest selected")
            self.footlocations = self.rest_controller.run(self.state, self.command_obj) 
            # state와 command 부분을 정확히 이해하지는 못했습니다 ㅜ

        else:
            print("Unknown state, switching to Rest mode")
            self.footlocations = self.rest_controller.run(self.state, self.command_obj)

    def get_next_locations(self):
        return self.footlocations
    


            