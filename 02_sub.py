#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from std_msgs.msg import Int32

def sub_callback(data) : 
    print(data.data * 2)

rp.init_node("Subscriber") #1. node name
rp.Subscriber("/test_topic", Int32, callback=sub_callback) #2(role). topic name, topic type, callback_func
rp.spin()