#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.msg import Color
from random import *

class Pub_Sub_Class : 
    def __init__(self) : 
        rp.init_node("Pub_Sub") #1. node name
        self.pub = rp.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) #2(role). topic name, topic type, qos
        rp.Subscriber("/turtle1/pose", Pose, callback=self.pose_callback) #2(role). topic name, topic type, callback_func
        rp.Subscriber("/turtle1/color_sensor", Color, callback=self.color_callback)
        self.cmd_vel_msg = Twist() # load
        self.cmd_vel_msg.angular.z
        self.pose_msg = Pose()
        self.pose_msg.theta
        self.color_msg = Color()
        self.color_msg
    
    def pose_callback(self, msg) : 
        self.pose_msg = msg
    
    def color_callback(self, msg) : 
        self.color_msg = msg
    
    def run(self) : 
        if (1 < self.pose_msg.x < 9 and 1 < self.pose_msg.y < 9) : 
            self.cmd_vel_msg.linear.x = random()*2
            self.cmd_vel_msg.angular.z = random()*5 - 2
        else : 
            self.cmd_vel_msg.linear.x = 0.5
            self.cmd_vel_msg.angular.z = -2
        self.pub.publish(self.cmd_vel_msg) #3. Code
        print("x : ", self.pose_msg.x)
        print("y : ", self.pose_msg.y)

def main() : 
    pub_sub_class = Pub_Sub_Class()
    while not (rp.is_shutdown()) : # for roscore is shutdown(False) or not(True)
        pub_sub_class.run() # most desired method

if __name__ == "__main__" : 
    main()