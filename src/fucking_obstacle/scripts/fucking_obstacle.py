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

# CONST VAR
#to debug
DEBUG = False
TEST = False
LOG = False
LOG_VER = 1
LOG_PATH = '/home/w_taek/log/log_obstacle/log_2_0%d.txt' % LOG_VER

#to race
SAFE_DIST_FRONT = 0.15
SAFE_DIST_FOR_ORIENT = 0.1
SAFE_DIST_REAR = 0.36

'''
b       :2.101750
l       :1.654625
'''
X_CLOSED_DIST = 0.36
X_FAR_DIST = 2.0
Y_CLOSED_DIST = 0.51
Y_MID_DIST = 1.15
Y_FAR_DIST = 1.5

Y_ORIENT = 87.0
X_ORIENT_REAR = 175.0
X_ORIENT_FRONT = 4.3

Y_POS_MID = 0.57

# to detect obstacle
OBS_POINTS_RANGE = 25
OBS_DETECT_DIST = 0.28          ###
'''
('fL', array([ 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
        0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.]))
('fR', array([ 0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
        0.        ,  0.        ,  0.118     ,  0.15000001,  0.        ,
        0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
        0.26300001,  0.26300001,  0.257     ,  0.25400001,  0.252     ,
        0.252     ,  0.25299999,  0.255     ,  0.25799999,  0.26100001]))
'''
OBS_DEFFERENT_COUNT = 6         ###
STATES_DETEC_OBS = np.array([0, 2, 4, 6])
AVOID_READY = 9
AVOID_TO_R1 = 11
AVOID_TO_R2 = 12
AVOID_TO_L1 = 21
AVOID_TO_L2 = 22

AVOID_YAW = 40
COME_BACK_PRE_YAW = 1

AVOID_X = 0.045
AVOID_Z = 0.17

## GLOBAL VAR
# race
g_state = 0
odom = Odometry()
# obstacle
g_obs_exist_l = False
g_obs_exist_r = False
g_pre_orient = 0
g_pre_state = 0
g_avoid_state = 0


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
    move.linear.x = 0.08
    move.angular.z = 0.0
    
def turtlestop():
    move.linear.x = 0.0
    move.angular.z = 0.0

def turtleturn_right():
    move.linear.x = 0.0
    move.angular.z = - 0.18

def turtleturn_left():
    move.linear.x = 0.0
    move.angular.z = 0.18

def turtle_avoid_right():
    move.linear.x = AVOID_X
    move.angular.z = AVOID_Z

def turtle_avoid_left():
    move.linear.x = AVOID_X
    move.angular.z = - AVOID_Z

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
    
    # To Detect Obstacle
    fL = refined_laser[:OBS_POINTS_RANGE]
    fR = refined_laser[-OBS_POINTS_RANGE:]
    fL[fL > OBS_DETECT_DIST] = 0
    fR[fR > OBS_DETECT_DIST] = 0
    obs_point_count_L = np.sum(fL > 0)
    obs_point_count_R = np.sum(fR > 0)
    g_obs_exist_l = (obs_point_count_L - obs_point_count_R) > OBS_DEFFERENT_COUNT
    g_obs_exist_r = (obs_point_count_R - obs_point_count_L) > OBS_DEFFERENT_COUNT

    if ((g_obs_exist_l or g_obs_exist_r)
    and g_state in STATES_DETEC_OBS):

        g_pre_orient = yaw
        g_pre_state = g_state

        if (g_obs_exist_l):
            g_state = AVOID_TO_R1
        elif (g_obs_exist_r) :
            g_state = AVOID_TO_L1
        
        turtlestop()
        
    if (g_state == AVOID_TO_R1):

        if (abs(abs(yaw) - abs(g_pre_orient)) > AVOID_YAW):
            turtlestop()
            g_state = AVOID_TO_R2
        else :
            turtleturn_right()

    elif (g_state == AVOID_TO_R2):
        if (abs(abs(yaw) - abs(g_pre_orient)) < COME_BACK_PRE_YAW):
            turtlestop()
            g_obs_exist_l = g_obs_exist_r = False
            g_state = g_pre_state
        else :
            turtle_avoid_right()

    elif (g_state == AVOID_TO_L1):

        if (abs(abs(yaw) - abs(g_pre_orient)) > AVOID_YAW):
            turtlestop()
            g_state = AVOID_TO_L2
        else :
            turtleturn_left()

    elif (g_state == AVOID_TO_L2):
        
        if (abs(abs(yaw) - abs(g_pre_orient)) < COME_BACK_PRE_YAW):
            turtlestop()
            g_obs_exist_l = g_obs_exist_r = False
            g_state = g_pre_state
        else :
            turtle_avoid_left()

    if (LOG):
        f = open(LOG_PATH, 'a')
        f.write("=======================================\n")
        f.write("state       :%d\n" % g_state)
        f.write("---------------------------------------\n")
        f.write("yaw      :%f\n" % yaw)
        f.write("yaw_diff  :%f\n" % abs(abs(yaw) - abs(g_pre_orient)))
        f.write("---------------------------------------\n")
        f.write("g_obs_exist_l  :%r\n" % g_obs_exist_l)
        f.write("g_obs_exist_r  :%r\n" % g_obs_exist_l)
        f.write("g_pre_orient   :%r\n" % g_pre_orient)
        f.write("g_pre_state    :%r\n" % g_pre_state)
        f.write("g_avoid_state  :%r\n" % g_avoid_state)
        f.write("---------------------------------------\n")
        f.write("ff       :%f\n" % ff)
        f.write("rignt    :%f\n" % r)
        f.write("left     :%f\n" % l)
        f.write("---------------------------------------\n")
        f.write("back     :%f\n" % b)
        f.write("---------------------------------------\n")
        f.write("pos_y     :%f\n" % y)
        f.write("pos_x     :%f\n" % x)        
        f.write("=======================================\n")

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
        print("fL", refined_laser[:OBS_POINTS_RANGE])
        print("fR", refined_laser[-OBS_POINTS_RANGE:])
        # print("g_pre_orient   :%r\n" % g_pre_orient)
        # print("g_pre_state    :%r\n" % g_pre_state)
        # print("g_avoid_state  :%r\n" % g_avoid_state)
        print("=======================================\n")
    

    if (g_state == 0):
        if (X_CLOSED_DIST > ff and ff > SAFE_DIST_FRONT):
            turtlestop()
            g_state = 1

        elif (b > X_FAR_DIST and l > Y_FAR_DIST):
            turtlestop()
            g_state = 99
        else:
            turtlemove()

    elif (g_state == 1):
        if (abs(yaw) > Y_ORIENT
            # and (b < SAFE_DIST_REAR
            #     and (abs(b_l - b_r) < SAFE_DIST_FOR_ORIENT
            #         and abs(l_r - l_l) < SAFE_DIST_FOR_ORIENT))
            ):
            turtlestop()
            g_state = 2

        else:
            turtleturn_right()

    elif (g_state == 2):
        if (abs(y) > Y_POS_MID):
            g_state = 3

        else:
            turtlemove()

    elif (g_state == 3):
        if (abs(yaw) > X_ORIENT_REAR
            # or (ff > SAFE_DIST_REAR
            #     and (abs(b_l - b_r) < SAFE_DIST_FOR_ORIENT
            #         and abs(r - l) < SAFE_DIST_FOR_ORIENT)
            #     )
            ):
            turtlestop()
            g_state = 4
     
        else:
            turtleturn_right()
    elif (g_state == 4):
        if (X_CLOSED_DIST > ff and ff > SAFE_DIST_FRONT):
            turtlestop()
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


    motor_pub.publish(move)


if __name__ == '__main__':
    rospy.init_node('lidar_driving')

    sub_laser()
    sub_odom()

    move = Twist()
    motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()