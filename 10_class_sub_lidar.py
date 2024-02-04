#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from sensor_msgs.msg import LaserScan
from math import *
from geometry_msgs.msg import Twist


class Sub_Lidar_Class() : 
    def __init__(self) : 
        rp.init_node("LiDAR") #1. node name
        rp.Subscriber("/scan", LaserScan, callback=self.lidar_callback) #2(role). topic name, topic type, callback_func
        self.pub = rp.Publisher("/cmd_vel", Twist, queue_size=10) #2(role). topic name, topic type, qos
        self.lidar_msg = LaserScan()
        self.lidar_msg.angle_min
        self.lidar_msg.angle_increment
        self.lidar_msg.ranges
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x
    
    def lidar_callback(self, data) : 
        angle_min_degree = data.angle_min * 180 /pi
        angle_increment_degree = data.angle_increment * 180 /pi
        
        degrees_from_lidar = [angle_min_degree + angle_increment_degree * i for i, v in enumerate(data.ranges)] # make list from list from lidar and # of ranges is not changable
        
        obstacle_index = [] # FLAG IS IMPORTANT
        for i, v in enumerate(data.ranges) : 
            if 0.0 < v < 0. and abs(degrees_from_lidar[i]) < 30 : 
                obstacle_index.append(i)
                if len(obstacle_index) >= 2 : 
                    if obstacle_index[-1] - obstacle_index[-2] > 50 : 
                        center_space = obstacle_index[-1] - obstacle_index[-2]
                    else : pass
            else : 
                pass
        
        self.cmd_vel_msg.linear.x = 0.3
        if obstacle_index : 
            self.cmd_vel_msg.linear.x = 0
            right_space = obstacle_index[0]
            left_space = len(data.ranges) - obstacle_index[-1]
            largist_spaces = sorted([left_space, center_space, right_space])[-1]
            
            if center_space == None : 
                center_space = 0
            if largist_spaces == left_space : 
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = 0.5
                print("go to left space")
            elif largist_spaces == right_space : 
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = -0.5
                print("go to right space")
            else : 
                self.cmd_vel_msg.linear.x = 0.3
                self.cmd_vel_msg.angular.z = 0
                print("go to center space")
        else : 
            self.cmd_vel_msg.angular.z = 0
        self.pub.publish(self.cmd_vel_msg)

def main() : 
    sub_lidar_class = Sub_Lidar_Class()
    rp.spin()

if __name__ == "__main__" : 
    main()