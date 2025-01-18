### Written by TEAM4

from enum import Enum
import numpy as np


class BehaviorState(Enum):
    START = 0
    TROT = 1
    REST = 2

class RobotState(object):
    def __init__(self, default_height):  # 0.1아님
        self.velocity = [0.0, 0.0]  # 속도 (x, y)
        self.yaw_rate = 0.0  # Yaw 회전 속도
        self.robot_height = -default_height  # 로봇 기본 높이, 바뀔예정(params에서 가져올듯)
        self.imu_roll = 0.0  # IMU Roll
        self.imu_pitch = 0.0  # IMU Pitch

        self.foot_location = np.zeros((3,4))
        self.body_local_position = np.array([0., 0., 0.])
        self.body_local_orientation = np.array([0., 0., 0.])

        self.ticks = 0

        self.behavior_state = BehaviorState.REST  # 기본 상태

        # 각 상태 플래그
class RobotCommand(object):
    def __init__(self, default_height):  # 0.1 아님
        self.trot_event = False
        self.rest_event = False
        self.start_event = False
        
        self.velocity = [0.0, 0.0]  # 속도 (x, y)
        self.yaw_rate = 0.0  # Yaw 회전 속도
        self.robot_height = -default_height  # 로봇 기본 높이, 바뀔예정(params에서 가져올듯)

class LegParameters(object):
    def __init__(self):
        self.pose = self.Leg_Pose()
        self.gait = self.Trot_Gait_Param()
        self.physical = self.Physical_Params()
    
    class Leg_Pose():
        def_stance = np.array([[0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]])
        initial_pose = np.array([[0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]])
    
    class Trot_Gait_Param():
        def __init__(self):
            self.cycle_time = None
            self.stance_time = 0.18
            self.swing_time = 0.24
            self.time_step = 0.02
            self.max_x_vel = 100
            self.max_y_vel = 100
            self.max_yaw_rate = 1.2
            self.z_leg_lift = 0.7
    
    class Physical_Params():
        l1 = 100
        l2 = 100
        l3 = 100
        l4 = 100