#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from turtlesim.msg import Pose

class Sub_Class() : 
    def __init__(self) : 
        rp.init_node("Color_Subscriber") #1. node name
        rp.Subscriber("/turtle1/pose", Pose, callback=self.sub_callback) #2(role). topic name, topic type, callback_func
        self.pose_data = Pose()
        self.pose_data.x
        self.pose_data.y
        self.pose_data.theta
    
    def sub_callback(self, data) : 
        print(data)
        print("---")
        print("Exportd x: ", data.x)
        print("Exportd y: ", data.y)
        print("Exportd theta: ", data.theta)
        print("-----")

def main() : 
    sub_class = Sub_Class()
    rp.spin()

if __name__ == "__main__" : 
    main()