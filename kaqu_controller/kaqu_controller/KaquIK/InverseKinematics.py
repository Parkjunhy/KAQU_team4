### IK개발코드, 발끝좌표 12값 받아서 각도 12개 반환.
//입력값이 leg_positions를 받아와야하네 각 발 끝의 위치 얘도 4*3 같은 느낌으로 이해
//(어깨 위치는 base보면 아는 거)
//우리가 알고 싶은 거: 목표 위치에 발끝이 도달하려면 각이 어떻게 되어야 하는지
import numpy as np 
from math import sqrt, atan2, sin, cos, pi 
from KaquIK.KinematicsCalculations import homog_transform_inverse,homog_transform

class InverseKinematics(object):  
    def __init__(self, bodyDimensions, legDimensions): //body, leg 차원 설정

        # Body dimensions  바디랑 레그 전부 params에서 받아올것
        self.bodyLength = bodyDimensions[0] //body 길이
        self.bodyWidth = bodyDimensions[1] //바디 너비

        # Leg dimensions //다리 각 부분의 길이 
        self.l1 = legDimensions[0] //바디  shoulder에서 첫 관절-조인트
        self.l2 = legDimensions[1] //
        self.l3 = legDimensions[2]
        self.l4 = legDimensions[3]

    def get_local_positions(self,leg_positions,dx,dy,dz,roll,pitch,yaw): 
    //로컬 좌표, roll pitch yaw는 축 기준 회전 각도 의미
        """
        Compute the positions of the end points in the shoulder frames.
        """
	//내가 받아오는 게 leg_position, 이를 4*1 행렬 형태로 변환?
        leg_positions = (np.block([[leg_positions],[np.array([1,1,1,1])]])).T

        # Transformation matrix, base_link_world => base_link
        T_blwbl = homog_transform(dx,dy,dz,roll,pitch,yaw)
	//기본 형태 baselinkworld->baselink

	//baselinkworld->다리 기준점  -어깨가 기준이 되는구나!
        # Transformation matrix, base_link_world => FR1
        T_blwFR1 = np.dot(T_blwbl, homog_transform(+0.5*self.bodyLength,
                          -0.5*self.bodyWidth,0,pi /2,-pi/2,0))

        # Transformation matrix, base_link_world => FL1
        T_blwFL1 = np.dot(T_blwbl, homog_transform(+0.5*self.bodyLength,
                          +0.5*self.bodyWidth,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => RR1
        T_blwRR1 = np.dot(T_blwbl, homog_transform(-0.5*self.bodyLength,
                          -0.5*self.bodyWidth,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => RL1
        T_blwRL1 = np.dot(T_blwbl, homog_transform(-0.5*self.bodyLength,
                          +0.5*self.bodyWidth,0,pi/2,-pi/2,0))

        # Local coordinates
        pos_FR = np.dot(homog_transform_inverse(T_blwFR1),leg_positions[0])
        //np.dot:행렬 벡터 계산
        pos_FL = np.dot(homog_transform_inverse(T_blwFL1),leg_positions[1])
        pos_RR = np.dot(homog_transform_inverse(T_blwRR1),leg_positions[2])
        pos_RL = np.dot(homog_transform_inverse(T_blwRL1),leg_positions[3])

        return(np.array([pos_FR[:3],pos_FL[:3],pos_RR[:3],pos_RL[:3]]))
        //이래서 12개를 받아온다고 적혀있구나..pos_FR 3개~pos_FL 도 3개~ 이런식으로 쭉
        
    def inverse_kinematics(self,leg_positions,dx,dy,dz,roll,pitch,yaw):
        """
        Compute the inverse kinematics for all the legs.
        """
        positions = self.get_local_positions(leg_positions,dx,dy,dz,
                                                            roll,pitch,yaw)
        angles = [] //출력값

        for i in range(4): //다리 4개-> range4

            x = positions[i][0]
            y = positions[i][1]
            z = positions[i][2]

            F = sqrt(x**2 + y**2 - self.l2**2) 
            //l2 회전 ㄴㄴ->F가 수평 거리 의미 어깨부터 끝까지
            G = F - self.l1
            //전체 수평거리에서 허벅지 길이 제외
            H = sqrt(G**2 + z**2)
            //어깨부터 끝까지 직선거리

            theta1 = atan2(y,x) + atan2(F,self.l2 * (-1)**i)
            //오른쪽 다리->1 왼쪽다리->-1 방향 보정
            //어깨 관절 회전
 
            D = (H**2 - self.l3**2 - self.l4**2)/(2*self.l3*self.l4)
            //l3, l4 사잇각의 cos값 D

            theta4 = -atan2((sqrt(1-D**2)),D)
            //사잇값에 (-) 붙 (회전 방향)

            theta3 = atan2(z,G) - atan2(self.l4*sin(theta4),
                                          self.l3 + self.l4*cos(theta4))
	//z축에 얼마나 회전-세타4보정
            angles.append(theta1)
            angles.append(theta3)
            angles.append(theta4)
       
        return angles
        // 세타1, 세타3, 세타4 -FR
                              -FL
                              -RR
                              -RL
           이렇게 4바이3 행렬이 결과로 나온다!!
