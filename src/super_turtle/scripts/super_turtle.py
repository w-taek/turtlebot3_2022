#! /usr/bin/env python

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
HAVE_TURNED = False

'''
=== g_state ===
0 : go straight
1 : face the wall -> turn right or left (90 deg)
2 : go to next path -> go straight
                        while either r or l is longer than FAR_DIST
3 : yawing for next path
4 : 

99 : stop0
'''


def sub_odom():
    odom_sub = rospy.Subscriber('/odom', Odometry, callback_odom)

def sub_laser():
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback)

def callback_odom(data_odom):
    global odom
    odom = data_odom
    print(odom)

def quaternion_to_euler_angle_yawing(odom):
    x = odom.pose.orientation.x
    y = odom.pose.orientation.y
    z = odom.pose.orientation.z
    w = odom.pose.orientation.w
    ysqr = y * y

    # t0 = +2.0 * (w * x + y * z)
    # t1 = +1.0 - 2.0 * (x * x + ysqr)
    # X = m.degrees(m.atan2(t0, t1))
	
    # t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # Y = m.degrees(m.asin(t2))
	
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = m.degrees(m.atan2(t3, t4))

    return Z

def callback(data_laser):
    global g_state
    global HAVE_TURNED
    
    laser_ranges = data_laser.ranges
    ff = (np.mean(laser_ranges[0:2]) + np.mean(laser_ranges[358:360])) / 2
    fR = np.mean(laser_ranges[350:360])
    fL = np.mean(laser_ranges[0:10])
    fAll = np.mean([fR, fL])

    r = np.mean(laser_ranges[265:275])
    l = np.mean(laser_ranges[85:95])

    b = np.mean(laser_ranges[175:180])

    # yaw = quaternion_to_euler_angle_yawing(odom)
    # print(yaw)
    # print(ff)

    VELO_DEFAULT = 0.13
    ZERO_f = 0.0
    CLOSED_DIST = 0.35
    FAR_DIST = 1.5

    if (g_state == 0):
        if (ff < CLOSED_DIST):
            move.linear.x = ZERO_f
            move.angular.z = ZERO_f
            g_state = 1
            print(g_state)
        elif (l > FAR_DIST and b > FAR_DIST):
            g_state = 99
        else:
            move.linear.x = VELO_DEFAULT
            move.angular.z = ZERO_f
    # elif (g_state == 1 and (not HAVE_TURNED)):
    #     if (89.5 < yaw and yaw < 90.5):
    #         move.linear.x = ZERO_f
    #         move.angular.z = ZERO_f
    #         g_state = 2
    #         print(g_state)
    #     else:
    #         move.linear.x = ZERO_f
    #         move.angular.z = -VELO_DEFAULT
    # elif (g_state == 1 and (HAVE_TURNED)):
    #     if (89.5 < yaw and yaw < 90.5):
    #         print("90")
    #         print(g_state)
    #         move.linear.x = ZERO_f
    #         move.angular.z = ZERO_f
    #         g_state = 2
    #     else:
    #         move.linear.x = ZERO_f
    #         move.angular.z = VELO_DEFAULT
    # elif (g_state == 2):
    #     if (r > FAR_DIST or l > FAR_DIST):
    #         print(g_state)
    #         move.linear.x = ZERO_f
    #         move.angular.z = ZERO_f
    #         g_state = 3
    #     elif (r > l):
    #         move.linear.x = ZERO_f
    #         move.angular.z = -VELO_DEFAULT
    #     else:
    #         move.linear.x = ZERO_f
    #         move.angular.z = VELO_DEFAULT
    # elif (g_state == 3):
    #     if (abs(yaw) > 179.5):
    #         move.linear.x = ZERO_f
    #         move.angular.z = ZERO_f
    #         g_state = 0
    #     elif (HAVE_TURNED):
    #         move.linear.x = ZERO_f
    #         move.angular.z = VELO_DEFAULT
    #     else:
    #         move.linear.x = ZERO_f
    #         move.angular.z = -VELO_DEFAULT
    # elif (g_state == 99):
    #     move.linear.x = ZERO_f
    #     move.angular.z = ZERO_f
    else:
        print("nothing")

    #motor_pub.publish(move)

'''
    if (DEBUG):
        print("=========================================")
        print("g_state: ", g_state)
        print("=========================================")
        print(quaternion_to_euler_angle_yawing(imu))
        print("=======================================================")
        print("\t\t", ff)
        print(fL, "\t\t", fR)
        print("\t\t",b)
        print("=======================================================")


    if DEBUG:
        print("=======================================================")
        print(quaternion_to_euler_angle_yawing(imu))
        print("=======================================================")
        print("\t\t" + np.mean(laser_ranges[0:5]))
        print(np.mean(laser_ranges[85:95]) + "\t\t" + np.mean(laser_ranges[175:185]))
        print(np.mean("\t\t" + laser_ranges[265:275]))
        print("=======================================================")

        print("=========================================")
        print(quaternion_to_euler_angle_yawing(imu))
        print("=========================================")
        print(len(laser_ranges))
        print("=========================================")
        print(np.mean(laser_ranges[0:5]))
        print(np.mean(laser_ranges[85:95]))
        print(np.mean(laser_ranges[175:185]))
        print(np.mean(laser_ranges[265:275]))
        print("=========================================")
'''




if __name__ == '__main__':
    rospy.init_node('lidar_driving')

    sub_laser()
    sub_odom()

    move = Twist()
    motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()