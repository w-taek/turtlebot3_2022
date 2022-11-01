#! /usr/bin/env python

import sys
import rospy
import math as m
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from nav_msgs.msg import Odometry
import numpy as np

import pdb

############# CONST VAR #############
# TO DEBUG
DEBUG = False
TEST = False
LOG = True
LOG_VER = 20
LOG_PATH = '/home/w_taek/log/log_dachshund_2/log_1_0%d.txt' % LOG_VER
# DISTANCE VAR TO RACE
SAFE_DIST_FRONT = 0.05
SAFE_DIST_FOR_ORIENT = 0.1
SAFE_DIST_REAR = 0.36

X_CLOSED_DIST = 0.26
X_FAR_DIST =1.6
Y_CLOSED_DIST = 0.40
Y_MID_DIST = 1.03
Y_FAR_DIST = 1.5

# IMU VAR TO RACE
Y_ORIENT = 87.0
X_ORIENT_REAR = 175.0
X_ORIENT_FRONT = 4.3

# POS VAR TO RACE
X_POS_START = 0.0
X_POS_FAR = 1.55
Y_POS_MID = 0.57

# VELOCITY VAR TO RACE
VEL_LINEAR_X = 0.08
VEL_ANGULAR_Z = 0.18
VEL_ZERO = 0.0

# VAR TO DETECT OBSTACLE
OBS_POINTS_RANGE = 35
OBS_POINTS_RANGE_CENTER = 20
OBS_DETECT_DIST = 0.23
OBS_DEFFERENT_COUNT = 6
STATES_DETEC_OBS = np.array([0, 2, 4, 6])
STATE_AVOID_READY = 9
STATE_AVOID_TO_R1 = 11
STATE_AVOID_TO_R2 = 12
STATE_AVOID_TO_L1 = 21
STATE_AVOID_TO_L2 = 22
STATE_AVOID_TO_R3 = 13
STATE_AVOID_TO_R4 = 14
STATE_AVOID_TO_L3 = 23
STATE_AVOID_TO_L4 = 24


AVOID_YAW = 40
COME_BACK_PRE_YAW = 2.5
AVOID_X_01 = 0.045
AVOID_Z_01 = 0.15

AVOID_YAW_CENTER = 70
#COME_BACK_PRE_YAW = 2
AVOID_X_03 = 0.045
AVOID_Z_03 = 0.25

PRE_NONE = 0
PRE_EXIST_LEFT = 1
PRE_EXIST_RIGHT = 2

############# GLOBAL VAR #############
# race
g_state = 0
odom = Odometry()

# obstacle
g_obs_exist_l = False
g_obs_exist_r = False
g_pre_orient = 0
g_pre_state = 0
g_avoid_state = 0
g_pre_obs_R_or_L = PRE_NONE

def sub_odom():
    odom_sub = rospy.Subscriber('/odom', Odometry, callback_odom)

def sub_laser():
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback)

def callback_odom(data_odom):
    global odom
    odom = data_odom

def quaternion_to_euler_angle_yawing(odom):
    x = odom.pose.pose.orientation.x
    y = odom.pose.pose.orientation.y
    z = odom.pose.pose.orientation.z
    w = odom.pose.pose.orientation.w
    ysqr = y * y
	
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)

    Z = m.degrees(m.atan2(t3, t4))

    return Z

def turtlemove():
    move.linear.x = VEL_LINEAR_X
    move.angular.z = VEL_ZERO
    
def turtlestop():
    move.linear.x = VEL_ZERO
    move.angular.z = VEL_ZERO

def turtleturn_right():
    move.linear.x = VEL_ZERO
    move.angular.z = - VEL_ANGULAR_Z

def turtleturn_left():
    move.linear.x = VEL_ZERO
    move.angular.z = VEL_ANGULAR_Z

def turtle_avoid_right():
    move.linear.x = AVOID_X_01
    move.angular.z = AVOID_Z_01

def turtle_avoid_left():
    move.linear.x = AVOID_X_01
    move.angular.z = - AVOID_Z_01

def turtle_avoid_right_more():
    move.linear.x = AVOID_X_03
    move.angular.z = AVOID_Z_03

def turtle_avoid_left_more():
    move.linear.x = AVOID_X_03
    move.angular.z = - AVOID_Z_03

def refine_ranges(laser):

    laser_arr = np.asarray(laser)
    #import pdb; pdb.set_trace()

    where_zero = np.where(laser_arr == 0)[0]
    st = np.setdiff1d(where_zero, where_zero + 1) - 1
    fi = np.setdiff1d(where_zero, where_zero - 1) + 1

    for i in range(len(st)):
        laser_arr[st[i] + 1 : fi[i]] = laser_arr[st[i]] + laser_arr[fi[i]] / 2

    return laser_arr

def callback(data_laser):
    global g_state
    global g_obs_exist_l
    global g_obs_exist_r
    global g_pre_orient
    global g_pre_state
    global g_pre_obs_R_or_L

    #### DATA FOR RACING ############################################
    refined_laser = refine_ranges(data_laser.ranges)

    ff = np.mean([np.mean(refined_laser[0:1]), refined_laser[359]])

    r = np.mean(refined_laser[268:272])
    # r_r = np.mean(refined_laser[270:275])
    # r_l = np.mean(refined_laser[265:270])
    l = np.mean(refined_laser[88:92])
    # l_r = np.mean(refined_laser[85:90])
    # l_l = np.mean(refined_laser[90:95])

    b = np.mean(refined_laser[178:182])
    # bb = np.mean(refined_laser[179:181])
    b_r = np.mean(refined_laser[175:180])
    b_l = np.mean(refined_laser[180:185])

    yaw = quaternion_to_euler_angle_yawing(odom)
        
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    
    #### DATA FOR DETECTING OBSTACLE ################################
    fL = refined_laser[:OBS_POINTS_RANGE]
    fR = refined_laser[-OBS_POINTS_RANGE:]
    fL[fL > OBS_DETECT_DIST] = 0
    fR[fR > OBS_DETECT_DIST] = 0
    obs_point_count_L = np.sum(fL > 0)
    obs_point_count_R = np.sum(fR > 0)
    g_obs_exist_l = (obs_point_count_L - obs_point_count_R) > OBS_DEFFERENT_COUNT
    g_obs_exist_r = (obs_point_count_R - obs_point_count_L) > OBS_DEFFERENT_COUNT

    #### OBSTACLE IS L OR R #########################################
    if (g_pre_obs_R_or_L == PRE_NONE
    and g_state in STATES_DETEC_OBS
    and (g_obs_exist_l or g_obs_exist_r)):

        g_pre_orient = yaw
        g_pre_state = g_state

        if (g_obs_exist_l):
            g_state = STATE_AVOID_TO_R1
        elif (g_obs_exist_r) :
            g_state = STATE_AVOID_TO_L1
        
        turtlestop()

    if (g_state == STATE_AVOID_TO_R1):

        if (abs(abs(yaw) - abs(g_pre_orient)) > AVOID_YAW):
            turtlestop()
            g_state = STATE_AVOID_TO_R2
        else :
            turtleturn_right()

    elif (g_state == STATE_AVOID_TO_R2):
        if (abs(abs(yaw) - abs(g_pre_orient)) < COME_BACK_PRE_YAW):
            g_pre_obs_R_or_L = PRE_EXIST_LEFT
            turtlestop()
            g_obs_exist_l = g_obs_exist_r = False
            g_state = g_pre_state
        else :
            turtle_avoid_right()

    elif (g_state == STATE_AVOID_TO_L1):

        if (abs(abs(yaw) - abs(g_pre_orient)) > AVOID_YAW):
            turtlestop()
            g_state = STATE_AVOID_TO_L2
        else :
            turtleturn_left()

    elif (g_state == STATE_AVOID_TO_L2):
        
        if (abs(abs(yaw) - abs(g_pre_orient)) < COME_BACK_PRE_YAW):
            g_pre_obs_R_or_L = PRE_EXIST_RIGHT
            turtlestop()
            g_obs_exist_l = g_obs_exist_r = False
            g_state = g_pre_state
        else :
            turtle_avoid_left()
    else :
        pass

    ### OBSTACLE IS CENTER #########################################
    if (g_pre_obs_R_or_L != PRE_NONE
        and g_state in STATES_DETEC_OBS
        and abs(x) < X_POS_FAR
        and (obs_point_count_L + obs_point_count_R) > OBS_DEFFERENT_COUNT):
            g_pre_orient = yaw
            g_pre_state = g_state
            turtlestop()
            if (g_pre_obs_R_or_L == PRE_EXIST_LEFT):
                g_state = STATE_AVOID_TO_L3
            else :
                g_state = STATE_AVOID_TO_R3
    
    if (g_state == STATE_AVOID_TO_R3):

        if (abs(abs(yaw) - abs(g_pre_orient)) > AVOID_YAW_CENTER):
            turtlestop()
            g_state = STATE_AVOID_TO_R4
        else :
            turtleturn_right()

    elif (g_state == STATE_AVOID_TO_R4):
        if (abs(abs(yaw) - abs(g_pre_orient)) < COME_BACK_PRE_YAW):
            turtlestop()
            g_obs_exist_l = g_obs_exist_r = False
            g_state = g_pre_state
            g_pre_obs_R_or_L = PRE_NONE
        else :
            turtle_avoid_right_more()

    elif (g_state == STATE_AVOID_TO_L3):

        if (abs(abs(yaw) - abs(g_pre_orient)) > AVOID_YAW_CENTER):
            turtlestop()
            g_state = STATE_AVOID_TO_L4
        else :
            turtleturn_left()

    elif (g_state == STATE_AVOID_TO_L4):
        
        if (abs(abs(yaw) - abs(g_pre_orient)) < COME_BACK_PRE_YAW):
            turtlestop()
            g_obs_exist_l = g_obs_exist_r = False
            g_pre_obs_R_or_L = PRE_NONE
            g_state = g_pre_state
        else :
            turtle_avoid_left_more()
    else :
        pass
    
    ### GENERAL RACE ######################
    if (g_state == 0):
        if (X_CLOSED_DIST > ff and ff > SAFE_DIST_FRONT
            and x > X_POS_FAR):
            turtlestop()
            g_pre_obs_R_or_L = PRE_NONE
            g_state = 1
        elif (abs(x) > X_POS_FAR and l > Y_FAR_DIST):
            turtlestop()
            g_state = 99
        else:
            turtlemove()

    elif (g_state == 1):
        if (abs(yaw) > Y_ORIENT):
            turtlestop()
            g_state = 2

        else:
            turtleturn_right()

    elif (g_state == 2):
        if (ff < Y_MID_DIST):
            g_pre_obs_R_or_L = PRE_NONE
            g_state = 3

        else:
            turtlemove()

    elif (g_state == 3):
        if (abs(yaw) > X_ORIENT_REAR):
            turtlestop()
            g_state = 4
     
        else:
            turtleturn_right()
    elif (g_state == 4):
        if (X_CLOSED_DIST > ff and ff > SAFE_DIST_FRONT):
            turtlestop()
            g_pre_obs_R_or_L = PRE_NONE
            g_state = 5

        else:
            turtlemove()

    elif (g_state == 5):
        if (abs(yaw) < Y_ORIENT):
            turtlestop()
            g_state = 6

        else:
            turtleturn_left()

    elif (g_state == 6):

        if (ff < Y_CLOSED_DIST) :
            g_pre_obs_R_or_L = PRE_NONE
            g_state = 7

        else:
            turtlemove()

    elif (g_state == 7):
        if (abs(yaw) < X_ORIENT_FRONT):
            g_state = 0
     
        else:
            turtleturn_left()

    elif (g_state == 99):
        turtlestop()
    
    else :
        pass

    ### MOTOR PUBLISH ######################
    motor_pub.publish(move)

    if (LOG):
        f = open(LOG_PATH, 'a')
        f.write("================================================================\n")
        f.write("state      :%d\n" % g_state)
        f.write("***************** IMU *****************\n")
        f.write("yaw        :%f\n" % yaw)
        f.write("yaw_diff   :%f\n" % abs(abs(yaw) - abs(g_pre_orient)))
        f.write("***************** LiDAR *****************\n")
        f.write("ff         :%f\n" % ff)
        f.write("rignt      :%f\n" % r)
        f.write("left       :%f\n" % l)
        f.write("back       :%f\n" % b)
        f.write("***************** POS *****************\n")
        f.write("POS_X      :%f\n" % x)
        f.write("POS_Y      :%f\n" % y)
        f.write("***************** OBSTACLE *****************\n")
        f.write("g_obs_exist_l      :%r\n" % g_obs_exist_l)
        f.write("g_obs_exist_r      :%r\n" % g_obs_exist_l)
        f.write("g_pre_orient       :%r\n" % g_pre_orient)
        f.write("g_pre_state        :%r\n" % g_pre_state)
        f.write("g_avoid_state      :%r\n" % g_avoid_state)
        f.write("g_pre_obs_R_or_L   :%r\n" % g_pre_obs_R_or_L)
        f.write("------------------LEFT-------------------\n")
        f.write("g_obs_exist_l      :%r\n" % g_obs_exist_l)
        f.write("obs_point_count_L  :%d\n" % obs_point_count_L)
        f.write("------------------RIGHT-----------------\n")
        f.write("g_obs_exist_r      :%r\n" % g_obs_exist_r)
        f.write("obs_point_count_R  :%d\n" % obs_point_count_R)
        f.write("---------------------------------------\n")
        f.write("L - R    :\n%d\n" % (obs_point_count_L - obs_point_count_R))
        f.write("R - L    :\n%d\n" % (obs_point_count_R - obs_point_count_L))
        #f.write("fL\n", refined_laser[:OBS_POINTS_RANGE])
        #f.write("fR\n", refined_laser[-OBS_POINTS_RANGE:])
        f.write("================================================================\n\n")

    if (TEST):
        print("=======================================")
        print("state       :%d" % g_state)
        print("---------------------------------------")
        print("ff       :%f" % ff)
        print("b       :%f" % b)
        print("l       :%f" % l)
        print("---------------------------------------")
        print("yaw      :%f" % yaw)
        print("yaw_diff  :%r" % abs(abs(yaw) - abs(g_pre_orient)))
        print("------------------LEFT-------------------")
        print("g_obs_exist_l        :%r" % g_obs_exist_l)
        print("obs_point_count_L    :%d" % obs_point_count_L)
        print("------------------RIGHT-----------------")
        print("g_obs_exist_r        :%r" % g_obs_exist_r)
        print("obs_point_count_R    :%d" % obs_point_count_R)
        print("---------------------------------------")
        print("L - R    :%d" % (obs_point_count_L - obs_point_count_R))
        print("R - L    :%d" % (obs_point_count_R - obs_point_count_L))
        print("---------------------------------------")
        print("fL")
        print(refined_laser[:OBS_POINTS_RANGE])
        print("fR")
        print(refined_laser[-OBS_POINTS_RANGE:])
        # print("g_pre_orient   :%r\n" % g_pre_orient)
        # print("g_pre_state    :%r\n" % g_pre_state)
        # print("g_avoid_state  :%r\n" % g_avoid_state)
        print("=======================================\n")


if __name__ == '__main__':
    rospy.init_node('lidar_driving')

    sub_laser()
    sub_odom()

    move = Twist()
    motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()