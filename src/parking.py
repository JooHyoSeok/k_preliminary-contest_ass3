#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
import time
from xycar_msgs.msg import xycar_motor
from astar import AstarPlan,State
import cubic_spline_planner

class Car_State(object):

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(Car_State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx, ry = [], []
cyaw = []
target_idx = 0
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
MAP_WIDTH = 1200
MAP_HEIGHT = 850
STANLEY_K = 1.25
###################################
'''
1200 * 850
'''
#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry, cyaw,target_idx
    # A star 알고리즘을 사용하여 경로 생성하는 코드
    print("Start Position : {} {}".format(sy,sx))
    start_state = State(int(sy),int(sx))
    dest_state = State(P_ENTRY[1],P_ENTRY[0])    
    map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype='uint8')
    astar = AstarPlan(map,h=850,w=1200)
    print("Start Planning!")
    plan = astar.plan(start_state, dest_state)
    print("Planning Done!")
    rx.clear();ry.clear();cyaw.clear();target_idx=0
    tx = [];ty = []
    # 경로를 cubic spline 하기 위한 말단 좌표 추가
    for path in plan[::500]:
        tx.append(path.y); ty.append(path.x)
    else:
        tx.append(1075); ty.append(125)
        tx.append(1130); ty.append(70)
    # 경로 spline 하기
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(tx, ty, ds=1.0) # 곡선으로 이어주는 함수 사용
    for x,y in zip(cx,cy): rx.append((x)); ry.append((y))

    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry, cyaw,target_idx
    state = Car_State(y,x,np.deg2rad(yaw),velocity) # 현재 상태 저장
    di, target_idx ,cte= stanley_control(state, ry, rx, cyaw, target_idx) # 스탠리 컨트롤을 사용해 heading_error , target_idx , cross_track_error 구하기
    print("CTE : ",round(cte,2))
    angle = np.rad2deg(di)
    angle = 50 if angle > 50 else angle
    angle = -50 if angle < -50 else angle
    # angle normalization
    speed = 50
    # 경로와 자동차의 방향의 오차가 클 때 경로에 align 하기 위한 후진
    if ((-cyaw[target_idx] - np.deg2rad(yaw)) > 0.8): 
        speed *= -1; angle = -50
    elif ((-cyaw[target_idx] - np.deg2rad(yaw)) < -0.8):
        speed *= -1; angle = 50
        
    if (target_idx >= len(rx)-1):speed=0
    print("current_angle : {}".format(angle))    
    drive(-angle, speed)


def calc_target_index(state, cx, cy):

    # 앞바퀴 기준으로 계산하기 위한 전진축 좌표 구하기
    fx = state.x - 60 * np.sin(state.yaw)
    fy = state.y + 60 * np.cos(state.yaw)
    # 가장 가까운 좌표 찾기
    dx = [icx - fx for icx in cx]
    dy = [icy - fy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    
    # 가장 가까운 좌표와 현재 x ,y 에러 값을 전진 축 벡터와 수직인 벡터에 내적하여 cte 구하기
    front_axle_vec = [np.cos(state.yaw + np.pi / 2),
                      np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([ dy[target_idx],-dx[target_idx]], front_axle_vec)        

    return target_idx, error_front_axle

def stanley_control(state, cx, cy, cyaw, last_target_idx):

    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)
    # target_idx , cte 구하기
    if last_target_idx >= current_target_idx: current_target_idx = last_target_idx
    # 앞으로 나아가기 위해 current_target_idx 유지
    # heading - error  구하기
    theta_e = normalize_angle(-cyaw[current_target_idx]  - state.yaw)
    #  -180 ~ 180 도 사이로 변경
    theta_d = np.arctan2(STANLEY_K * error_front_axle, abs(state.v))
    # 스탠리 컨트롤 식을 통해 Steering angle 값 계산
    print("angle diff : {}deg".format(round(np.rad2deg(theta_e),2)),end=" ")
    delta = (theta_e * 1.5) + theta_d 
    return delta, current_target_idx , theta_d

def normalize_angle(angle):

    # -180 ~ 180 도 사이로 변경
    while angle > np.pi: angle -= 2.0 * np.pi
    while angle < -np.pi: angle += 2.0 * np.pi
    return angle
