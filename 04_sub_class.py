#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from std_msgs.msg import Int32

class Sub_Class() : 
    def __init__(self) : 
        rp.init_node("Subscriber") #1. node name
        rp.Subscriber("/test_topic", Int32, callback=self.sub_callback) #2(role). topic name, topic type, callback_func
    
    def sub_callback(self, data) : 
        print(data.data)

def main() : 
    sub_class = Sub_Class()
    rp.spin()

if __name__ == "__main__" : 
    main()