#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import rospy as rp
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import *
import numpy as np

class Sub_Camera_Class() : 
    def __init__(self) : 
        rp.init_node("Camera") #1. node name
        self.pub = rp.Publisher("/cmd_vel", Twist, queue_size=10) #2(role). topic name, topic type, qos
        rp.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback=self.camera_callback) #2(role). topic name, topic type, callback_func
        self.bridge = CvBridge()
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.angular.z
    
    def camera_callback(self, data) : 
        img_bgr = self.bridge.compressed_imgmsg_to_cv2(data)
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        height, width, channel = img_bgr.shape
        cv2.imshow("img_vgr", img_bgr)
        
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 60, 255])
        yellow_lower_cr = np.array([15, 40, 40])
        yellow_upper_cr = np.array([45, 255, 255])
        
        white_mask = cv2.inRange(img_hsv, white_lower, white_upper)
        yellow_mask = cv2.inRange(img_hsv, yellow_lower_cr, yellow_upper_cr)
        
        white_img = cv2.bitwise_and(img_bgr, img_bgr, mask = white_mask)
        yellow_img = cv2.bitwise_and(img_bgr, img_bgr, mask = yellow_mask)
        combined_img = cv2.bitwise_or(white_img, yellow_img)
        
        # set IoF
        x1 = 40
        y1 = height
        x2 = 200
        y2 = 340
        src1 = [x1, y1]
        src2 = [x2, y2]
        src3 = [width - x2, y2]
        src4 = [width - x1, y1]
        src_list = np.float32([src1, src2, src3, src4])
        
        dst_x = 80 # x1 point where to move in transformed image
        dst1 = [dst_x, height]
        dst2 = [dst_x, 0]
        dst3 = [width - dst_x, 0] 
        dst4 = [width - dst_x, height]
        dst_list = np.float32([dst1, dst2, dst3, dst4])
        
        matrix = cv2.getPerspectiveTransform(src_list, dst_list)
        warp_img = cv2.warpPerspective(combined_img, matrix, [640, 480])
        gray_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
        binary_img = np.zeros_like(gray_img) # zero matrix. same size with gray_img
        binary_img[gray_img != 0] = 1
        # left_highst_index = np.argmax(histogram[0 : width//2])
        # right_highst_index = np.argmax(histogram[width//2 : :]) + width//2 # add width/2 'cause [320] becomes [0]
        margin = 40 # width of box
        num_box = 8
        height_box = height // num_box
        
        left_histograms = []
        right_histograms = []
        # left_center_point = []
        # right_center_point = []
        left_indices = []
        right_indices = []
        left_center_point = []
        right_center_point = []
        center_point = []
        left_binary_img = binary_img[:, 0 : width//2]
        right_binary_img = binary_img[:, width//2 : :]
        threshold = 50 # reference
        center_reference = 320
        side_reference = 100
        for i in range(0, num_box) : 
            top = height - height_box*(i+1)
            bottom = height - height_box*i
            center_y = top - bottom
            left_histograms.append(np.sum(left_binary_img[top:bottom, :], axis = 0))
            right_histograms.append(np.sum(right_binary_img[top:bottom, :], axis = 0))
            left_indices.append(np.where(left_histograms[i] > threshold)[0]) # return index. and bool for noise filtering
            right_indices.append(np.where(right_histograms[i] > threshold)[0])
            right_indices[i] += width//2
            try : 
                left_center_point.append(min(left_indices[i]) + max(left_indices[i]))
                left_window_left = left_center_point[i] - margin
                left_window_right = left_center_point[i] + margin
                # left_window_left = min(left_indices[i])
                left_window_left = min(0, left_window_left)
                # left_window_right = max(left_indices[i])
                left_window_right = min(320, left_window_right)
                cv2.line(warp_img, (left_center_point[i], center_y), (left_center_point[i], center_y), [0, 0, 255], 3)
                cv2.rectangle(warp_img, (left_window_left, top), (left_window_right, bottom), [0, 0, 255], 3)
            except : 
                left_center_point.append(side_reference)
            try : 
                right_center_point.append(min(right_indices[i]) + max(right_indices[i]))
                right_window_left = min(right_indices[i])
                right_window_right = max(right_indices[i])
                cv2.line(warp_img, (right_center_point[i], center_y), (right_center_point[i], center_y), [0, 255, 0], 3)
                cv2.rectangle(warp_img, (right_window_left, top), (right_window_right, bottom), [0, 255, 0], 3)
            except : 
                right_center_point.append(width - side_reference)
            try : # work without using try
                center_point.append((left_center_point[i] + right_center_point[i])//2)
                center_average = np.average(center_point)
            except : 
                pass # not using index
                # if not left_center_point[i] : 
                #     center_reference = width - side_reference #not using index
                # elif not right_center_point[i] : 
                #     center_reference = side_reference
        
        angle_resolution = np.pi/width
        
        if center_average > center_reference : # go to left
            print("You're on right side")
        else : # go to right
            print("You're on left side")
        
        steer = angle_resolution * (center_average + center_reference)
        self.cmd_vel_msg.angular.z = steer
        cv2.imshow("combined_mask", combined_img)
        cv2.imshow("warp_img", warp_img)
        cv2.imshow("warp_img_gray", gray_img)
        cv2.imshow("binary", binary_img)
        cv2.waitKey(1)
        
        self.pub.publish(self.cmd_vel_msg)

def main() : 
    sub_camera_class = Sub_Camera_Class()
    rp.spin()

if __name__ == "__main__" : 
    main()