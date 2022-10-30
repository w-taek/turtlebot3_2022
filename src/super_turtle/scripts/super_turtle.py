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

g_state = 0
DEBUG = False
TEST = False
LOG = True
LOG_VER = 1
odom = Odometry()
    

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

def refine_ranges(laser):

    laser_arr = np.asarray(laser)

    for i in range(len(laser)):
        if laser_arr[i] == 0.0:
            non_zero_index = i
            while (laser_arr[non_zero_index] == 0 and non_zero_index < len(laser)):
                non_zero_index += 1
            step = non_zero_index - i + 1
            step_size = (laser_arr[non_zero_index] - laser_arr[i - 1]) / step
            j = i
            while (j <= non_zero_index):
                laser_arr[j] = laser_arr[j - 1] + step_size
                j += 1

    return laser_arr

def callback(data_laser):
    global g_state
    
    refined_laser = refine_ranges(data_laser.ranges)
    ff = np.mean([np.mean(refined_laser[0:1]), np.mean(refined_laser[359])])
    fR = np.mean(refined_laser[-18:])    #obstacle range
    fL = np.mean(refined_laser[:17])     #obstacle range

    r = np.mean(refined_laser[268:272])
    r_r = np.mean(refined_laser[270:275])
    r_l = np.mean(refined_laser[265:270])
    l = np.mean(refined_laser[88:92])
    l_r = np.mean(refined_laser[85:90])
    l_l = np.mean(refined_laser[90:95])

    b = np.mean(refined_laser[178:182])
    bb = np.mean(refined_laser[179:181])
    b_r = np.mean(refined_laser[175:180])
    b_l = np.mean(refined_laser[180:185])

    yaw = quaternion_to_euler_angle_yawing(odom)
        
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y

    if (LOG):
        LOG_PATH = '/home/w_taek/log/log_3_0%d.txt' % LOG_VER
        f = open(LOG_PATH, 'a')
        f.write("=======================================\n")
        f.write("state       :%d\n" % g_state)
        f.write("---------------------------------------\n")
        f.write("yaw      :%f\n" % yaw)
        f.write("ff       :%f\n" % ff)
        f.write("fR       :%f\n" % fR)
        f.write("fL       :%f\n" % fL)
        f.write("---------------------------------------\n")
        f.write("rignt    :%f\n" % r)
        f.write("left     :%f\n" % l)
        f.write("---------------------------------------\n")
        f.write("back     :%f\n" % b)
        f.write("---------------------------------------\n")
        f.write("pos_y     :%f\n" % y)
        f.write("pos_x     :%f\n" % x)
        f.write("---------------------------------------\n")
        f.write("abs(b_l - b_r) :%f\n" % abs(b_l - b_r))
        f.write("abs(fR - fL)   :%f\n" % abs(fR - fL))
        f.write("=======================================\n")

    if (TEST):
        print("=======================================")
        print("yaw      :", yaw)
        print("ff       :", ff)
        print("fR       :", fR)
        print("fL       :", fL)
        print("---------------------------------------")
        print("rignt    :", r)
        print("left     :", l)
        print("---------------------------------------")
        print("back     :", b)
        print("=======================================")

    if (TEST):
        print("=======================================")
        print("state    :", g_state)
        print("---------------------------------------")
        print("yaw      :", yaw)
        print("ff       :", ff)
        print("---------------------------------------")
        print("rignt    :", r)
        print("left     :", l)
        print("=======================================")


    SAFE_DIST_FRONT = 0.15
    SAFE_DIST_FOR_ORIENT = 0.1
    SAFE_DIST_REAR = 0.36

    X_CLOSED_DIST = 0.36
    X_FAR_DIST = 1.63
    Y_CLOSED_DIST = 0.51
    Y_MID_DIST = 1.15
    Y_FAR_DIST = 1.5

    Y_ORIENT = 88.0         ##
    X_ORIENT_REAR = 175.0   ##
    X_ORIENT_FRONT = 4.3    ##
    
    Y_POS_MID = 0.6

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
        if (abs(yaw) < X_ORIENT_FRONT
            and (abs(fR - fL) < SAFE_DIST_FOR_ORIENT)
            ):
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