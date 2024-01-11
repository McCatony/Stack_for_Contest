#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from std_msgs.msg import Int32

class Pub_Class : 
    def __init__(self) : 
        rp.init_node("Publisher") #1. node name
        self.pub = rp.Publisher("/test_topic", Int32, queue_size=10) #2(role). topic name, topic type, qos
        self.rate = rp.Rate(10) # Rate
        self.msg = Int32()
    
    def run(self) : 
        i = 0
        while not (rp.is_shutdown()) : # for roscore is shutdown(False) or not(True)
            i += 1
            self.msg.data = i
            self.pub.publish(self.msg) #3. Code
            self.rate.sleep() # taking delay

def main() : 
    pub_class = Pub_Class()
    pub_class.run()

if __name__ == "__main__" : 
    main()