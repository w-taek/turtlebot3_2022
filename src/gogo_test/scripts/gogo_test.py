#! /usr/bin/env python

import sys
import rospy
import math as m
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
#from laser_geometry import LaserProjection
import numpy as np

import pdb

DEBUG = True

g_state = 0
HAVE_TURNED = False
imu = Imu()


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

def mean(a, b):
    return (a + b) / 2

def sub_laser():
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback)

def sub_imu():
    imu_sub = rospy.Subscriber('/imu', Imu, callback_imu)


def callback_imu(data_imu):
    global imu
    imu = data_imu
    
def quaternion_to_euler_angle_yawing(imu):
    x = imu.orientation.x
    y = imu.orientation.y
    z = imu.orientation.z
    w = imu.orientation.w
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = m.degrees(m.atan2(t0, t1))
	
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = m.degrees(m.asin(t2))
	
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = m.degrees(m.atan2(t3, t4))

    return Z

def callback(data_laser):
    g_state = rospy.get_param("/driving/g_state")

    global HAVE_TURNED

    velo_default = 0.3
    zero_f = 0.0
    closed_dist = 0.5
    far_dist = 5.0
    

    laser_ranges = data_laser.ranges
    ff = (np.mean(laser_ranges[0:2]) + np.mean(laser_ranges[358:360])) / 2
    fR = np.mean(laser_ranges[350:360])
    fL = np.mean(laser_ranges[0:10])
    fAll = mean(fR, fL)

    r = np.mean(laser_ranges[265:275])
    l = np.mean(laser_ranges[85:95])

    b = np.mean(laser_ranges[175:180])

    yaw = quaternion_to_euler_angle_yawing(imu)

    if (g_state == 0):
        print(g_state)
        if (ff < closed_dist):
            move.linear.x = zero_f
            move.angular.z = zero_f
            g_state = 1
            motor_pub.publish(move)
        else:
            move.linear.x = 0.3
            move.angular.z = 0.0
            motor_pub.publish(move)
    # elif (g_state == 1 and (not HAVE_TURNED)):
    #     print(g_state)
    #     if (89.5 < yaw and yaw < 90.5):
    #         move.linear.x = zero_f
    #         move.angular.z = zero_f
    #         g_state = 2
    #     else:
    #         move.linear.x = zero_f
    #         move.angular.z = -velo_default
    # elif (g_state == 1 and (HAVE_TURNED)):
    #     print(g_state)
    #     if (89.5 < yaw and yaw < 90.5):
    #         move.linear.x = zero_f
    #         move.angular.z = zero_f
    #         stage = 2
    #     else:
    #         move.linear.x = zero_f
    #         move.angular.z = velo_default
    # elif (g_state == 2):
    #     print(g_state)
    #     if (r > far_dist or l > far_dist):
    #         move.linear.x = zero_f
    #         move.angular.z = zero_f
    #         stage = 3
    #     elif (r > l):
    #         move.linear.x = zero_f
    #         move.angular.z = -velo_default
    #     else:
    #         move.linear.x = zero_f
    #         move.angular.z = velo_default
    # elif (g_state == 3):
    #     print(g_state)
    #     if (abs(yaw) > 179.5):
    #         move.linear.x = zero_f
    #         move.angular.z = zero_f
    #         stage = 0
    #     elif (HAVE_TURNED):
    #         move.linear.x = zero_f
    #         move.angular.z = velo_default
    #     else:
    #         move.linear.x = zero_f
    #         move.angular.z = -velo_default
    # else:
    #     print("nothing")


    motor_pub.publish(move)
            # elif (l > FAR_DIST and b > FAR_DIST):
        #     g_state = 99    
    # elif (g_state == 99):
    #     move.linear.x = ZERO_f
    #     move.angular.z = ZERO_f

    # if (DEBUG):
    #     print("=========================================")
    #     print("g_state: ", g_state)
    #     print("=========================================")
    #     print(quaternion_to_euler_angle_yawing(imu))
    #     print("=======================================================")
    #     print("            ", ff)
    #     print(fL, "                    ", fR)
    #     print("            ", b)
    #     print("=======================================================")


'''
    if DEBUG:
        print("=======================================================")
        print(quaternion_to_euler_angle_yawing(imu))
        print("=======================================================")
        print("\t\t" + np.mean(laser_ranges[0:5]))
        print(np.mean(laser_ranges[85:95]) + "\t\t" + np.mean(laser_ranges[175:185]))
        print(np.mean("\t\t" + laser_ranges[265:275]))
        print("=======================================================")
'''
'''
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
    sub_imu()
    sub_laser()

    move = Twist()
    motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    rospy.spin()