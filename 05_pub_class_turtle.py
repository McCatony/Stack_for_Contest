#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from geometry_msgs.msg import Twist

class Pub_Class : 
    def __init__(self) : 
        rp.init_node("Publisher") #1. node name
        self.pub = rp.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) #2(role). topic name, topic type, qos
        self.rate = rp.Rate(10) # Rate
        self.msg = Twist()
    
    def run(self) : 
        while not (rp.is_shutdown()) : # for roscore is shutdown(False) or not(True)
            self.msg.angular.z = 3
            self.pub.publish(self.msg) #3. Code
            self.rate.sleep() # taking delay

def main() : 
    pub_class = Pub_Class()
    pub_class.run()

if __name__ == "__main__" : 
    main()