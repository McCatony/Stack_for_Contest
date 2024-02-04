#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from random import *

class Sub_Pub_Class() : 
    def __init__(self) : 
        rp.init_node("Sub_Pub") #1. node name
        rp.Subscriber("/turtle1/pose", Pose, callback=self.sub_callback) #2(role). topic name, topic type, callback_func
        self.pub = rp.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) #2(role). topic name, topic type, qos
        self.pose_data = Pose()
        self.pose_data.x
        self.pose_data.y
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x
        self.cmd_vel_msg.angular.z
    
    def sub_callback(self, data) : 
        if (1 <= data.x <= 10 and 1 <= data.y <= 10): 
            self.cmd_vel_msg.linear.x = random()*2
            self.cmd_vel_msg.angular.z = random()*4 - 2
        else : 
            self.cmd_vel_msg.linear.x = 0.5
            self.cmd_vel_msg.angular.z = 2
        self.pub.publish(self.cmd_vel_msg)
        print("x : ", data.x)
        print("y : ", data.y)
        print("-----")

def main() : 
    sub_pub_class = Sub_Pub_Class()
    rp.spin()

if __name__ == "__main__" : 
    main()