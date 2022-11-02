#! /usr/bin/env python

import rospy
#import math as m
from std_msgs.msg import Int32MultiArray
#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import LaserScan
#from laser_geometry import LaserProjection
#from nav_msgs.msg import Odometry
import numpy as np

import pdb

'''
    arr[0] : left_red
    arr[1] : left_green
    arr[2] : left_rotten
    arr[3] : right_red
    arr[4] : right_green
    arr[5] : right_rotten
'''

TestArray = []
# CONST VAR
a = [[1, 0, 0, 1, 1,1],
 [0, 1, 0, 1, 1, 0],
 [0, 0, 1, 1, 0, 1],
 [1, 1, 0, 0, 1, 1],
 [0, 1, 1, 1, 0, 0],
 [1, 0, 1, 0, 1, 0],
 [1, 1, 1, 0, 0, 1]]


i = 0

def simple_pub():               
    rospy.init_node('sample_pub', anonymous=True)
    pub = rospy.Publisher('/detect_count', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(0.2) # 10hz
    
    while not rospy.is_shutdown():
        global i
        result = Int32MultiArray()
        result.data = a[i]
        i += 1
        i = i % len(a)
        #print(pub)
        rospy.loginfo(result)
        pub.publish(result)
        rate.sleep()

if __name__ == '__main__':  
    try:                    
        simple_pub()        
    except rospy.ROSInterruptException: 
        print("Program terminated")     