#!/usr/bin/env python

import rospy
import geometry_msgs.msg

class MoveTurtle():

    def __init__(self):
        rospy.init_node("move_turtle")
        self.pb= rospy.Publisher("turtle1/cmd_vel",geometry_msgs.msg.Twist,queue_size=10)
        self.tw = geometry_msgs.msg.Twist()
   
    def move_turtle(self):
        self.tw.linear.x = self.tw.angular.z = 0.25
        self.pb.publish(self.tw)

if __name__ == '__main__':
    try:
        x = MoveTurtle()
        while not rospy.is_shutdown():
            x.move_turtle
    except rospy.ROSInterruptException:
        pass