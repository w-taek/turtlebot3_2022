#! /usr/bin/env python
# -*- coding: utf8 -*-

import rospy
#import math as m
from std_msgs.msg import UInt8MultiArray
#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import LaserScan
#from laser_geometry import LaserProjection
#from nav_msgs.msg import Odometry
import numpy as np

import pdb

'''
    arr = uint16[]
    arr[0] : left_red
    arr[1] : left_green
    arr[2] : left_rotten
    arr[3] : right_red
    arr[4] : right_green
    arr[5] : right_rotten
'''


# CONST VAR
a = np.uint8(
 [[0, 0, 0, 0, 0, 0],
 [0, 0, 0, 0, 0, 1],
 [1, 0, 0, 0, 0, 1],
 [0, 1, 0, 0, 1, 0],
 [0, 0, 0, 1, 0, 1],
 [1, 1, 1, 0, 0, 1],
 [0, 0, 0, 1, 1, 1]])


i = 0

def simple_pub():               # simple_pub() 함수 정의 시작
    # 'sample_pub' 노드 초기화 
    rospy.init_node('sample_pub', anonymous=True)
    # String 형식 토픽 'hello'를 발행하는 퍼블리셔 'pub' 선언
    pub = rospy.Publisher('/detect_result_test', UInt8MultiArray, queue_size=10)
    # 1초당 0.2회의 빈도로 토픽을 발행하기 위한 rate 객체 선언 
    rate = rospy.Rate(0.2) # 10hz
    
    # rospy가 종료되지 않았으면 반복할 루프
    while not rospy.is_shutdown(): # roscpp 코드의 "while(ros::ok())"에 해당하는 구문
        # "hello~ " 문자열 뒤에 현재 시간을 덧붙인 문자열을 String 변수 str에 치환 
        # str = "hello~ %s" % rospy.get_time()
        global i
        result = UInt8MultiArray()
        result.data = a[i]
        i += 1
        i = i % len(a)
        # time stamp가 표시되는 화면출력으로 str 출력
        rospy.loginfo(result)
        # 퍼블리셔 'pub'으로 'str'의 내용을 토픽명 'hello'로 발행
        pub.publish(result)
        # 루프 시작 부터 1/10초가 지날 때까지 시간지연(토픽 발행 빈도 10회/초)
        rate.sleep()

if __name__ == '__main__':  # 모듈명이 저장되는 전역변수 __name__에 저장된 값이 '__main__'이면
    try:                    # 뒤에 나오는 예외처리(except ... :)를 고려한 실행 구간 시작
        simple_pub()        # simple_pub() 함수 호출
    except rospy.ROSInterruptException: # ROS 인터럽트 예외 발생시
        print("Program terminated")      # 프로그램 종료 메세지 화면출력