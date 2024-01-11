#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from std_msgs.msg import Int32

rp.init_node("Publisher") #1. node name
pub = rp.Publisher("/test_topic", Int32, queue_size=10) #2(role). topic name, topic type, qos
rate = rp.Rate(10) # Rate
msg = Int32()

i = 0
while not (rp.is_shutdown()) : # for roscore is shutdown(False) or not(True)
    i += 1
    msg.data = i
    pub.publish(msg) #3. Code
    rate.sleep() # take delay