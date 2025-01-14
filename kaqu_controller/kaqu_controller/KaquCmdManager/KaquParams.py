### 모든 변수들 관리함. 일단 하드웨어 변수들 전부 상수로 입력하기로함. 추후에 필요 시 함수로 변경은 자율.
### 3조에서 필요한거 리스트 추리고 있다고 하니 대기자모드..,!

import numpy as np
from enum import Enum

class Interface():
    RATE = 60
    # USE_IMU = False

class Parameters() :

    class _LegParam():   # Local Coordinates
        def __init__(self):
            self.pose = self.leg_pose()
            self.gait = self._trot_gait_param()
            self.physical = self._local_physical_params()
        class _local_physical_params():   # storing length of each leg segments
            l1 = 1
            l2 = 1
            l3 = 1
            l4 = 1
        class _trot_gait_param:     # variables for trot gait (cycle time, swing time, etc.)
            def __init__(self):
                self.cycle_time = None
                self.stance_time = 0.1
                self.swing_time = 0.1
                self.time_step = 0.01

                self.max_x_vel = 0.01
                self.max_y_vel = 0.01
                self.max_yaw_rate = 0.1 

                self.z_leg_lift = 0.1   # lifted height of leg (might not be used / theta3 would be calculated through IK calculation)
        class leg_pose():
            def_stance = np.array([[0, 0, 0, 0],
                              [0, 0, 0, 0],
                              [0, 0, 0, 0]])
            initial_pose = np.array([[0, 0, 0, 0],
                              [0, 0, 0, 0],
                              [0, 0, 0, 0]])

    class body_params() :   # Global Coordinates
        default_height = 0

        def __init__(self): 
            self.height = None
            self.COM = np.zeros([3])    # center of mass(balancing prob.)
            self.physical = self._physical_params()

            self.default_height = 0
            self.roll = 0
            self.pitch = 0
            self.yaw = 0
            self.ZMP_points = np.zeros([4,3])  # storing the points of each foot

            class _physical_params() :
                _total_length = 1
                _total_width = 1
                _min_height = 1
                _max_height = 1

#class LegParam():
#    def __init__(self):
#        self.FR = Parameters._LegParam()
#        self.FL = Parameters._LegParam()
#        self.RR = Parameters._LegParam()
#        self.RL = Parameters._LegParam()

#class ControlType():
#    USE_JOY = True

class RobotState(Enum):  # Joystick control signal
    START = 0
    TROT = 1
    REST = 2

class CurrentState(object):
    def __init__(self, default_height):
        self.velocity = np([0., 0.])
        self.yaw_state = 0.
        self.robot_height = -default_height

        self.foot_point = np.zeros((3,4))

        self.body_position = np.array([0., 0., 0.])     # x, y ,z coordinates of body
        self.body_orientation = np.array([0., 0., 0.])      # roll, pitch, yaw of body

        self.robot_state = RobotState.REST

class Command(object):
    def __init__(self, default_height):
        self.cmd_vel = np.array([0., 0.])
        self.cmd_yaw_rate = 0.
        self.robot_height = -default_height

        self.start_event = False
        self.trot_event = False
        self.rest_event = False