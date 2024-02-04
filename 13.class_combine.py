#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan
from erp_driver.msg import erpCmdMsg
from cv_bridge import CvBridge
import cv2
import numpy as np
from math import *


class Sub_class:
    def __init__(self):
        rospy.init_node("test_node")  # 01. node 이름 설정
        rospy.Subscriber("/scan", LaserScan, callback=self.lidar_CB)  # 02. node의 역할 설정(subscriber)
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback=self.camera_CB)  # 02. node의 역할 설정(subscriber)
        self.pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size=1)
        self.bridge = CvBridge()
        self.img_msg = CompressedImage()
        self.lidar_msg = LaserScan()
        self.cmd_msg = erpCmdMsg()
        self.camera_flag = False
        self.lidar_flag = False
        self.height, self.width = 0, 0
        self.degrees = []
        self.degree_flag = False

    def camera_CB(self, msg):  # 03. 콜백 함수(카메라 인지)
        if msg != []:
            self.camera_flag = True
            self.img_msg = msg
        else:
            self.camera_flag = False
            pass

    def lidar_CB(self, msg):  # 03. 콜백 함수(라이다 인지)
        if msg != []:
            self.lidar_flag = True
            self.lidar_msg = msg
        else:
            self.lidar_flag = False
            # pass

    def cvt_color(self):  # 카메라 - 판단
        img_bgr = self.bridge.compressed_imgmsg_to_cv2(self.img_msg)
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        self.height, self.width, channel = img_bgr.shape
        return img_bgr, img_hsv

    def colort_detect(self, img_bgr, img_hsv):
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 48, 255])
        white_mask = cv2.inRange(img_hsv, white_lower, white_upper)
        white_img = cv2.bitwise_and(img_bgr, img_bgr, mask=white_mask)
        
        yellow_lower = np.array([0, 128, 64])
        yellow_upper = np.array([45, 255, 255])
        yellow_mask = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
        yellow_img = cv2.bitwise_and(img_bgr, img_bgr, mask=yellow_mask)
        combined_img = cv2.bitwise_or(white_img, yellow_img)
        
        return combined_img

    def img_warp(self, combined_img):
        x1 = 40
        y1 = self.height
        x2 = 200
        y2 = 340
        src1 = [x1, y1]
        src2 = [x2, y2]
        src3 = [self.width - x2, y2]
        src4 = [self.width - x1, self.height]
        srcs = np.float32([src1, src2, src3, src4])
        
        dst_x = 80
        dst1 = [dst_x, self.height]
        dst2 = [dst_x, 0]
        dst3 = [self.width - dst_x, 0]
        dst4 = [self.width - dst_x, self.height]
        dsts = np.float32([dst1, dst2, dst3, dst4])
        
        matrix = cv2.getPerspectiveTransform(srcs, dsts)
        warp_img = cv2.warpPerspective(combined_img, matrix, [640, 480])
        return warp_img

    def img_binary(self, warp_img):
        gray_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
        binary_img = np.zeros_like(gray_img)
        binary_img[gray_img != 0] = 1
        return binary_img

    def sliding_window(self, warp_img, binary_img):
        margin = 40
        box_num = 8
        box_height = self.height // box_num
        
        left_histograms = []
        right_histograms = []
        left_indices = []
        right_indices = []
        left_center_points = []
        right_center_points = []
        center_points = []
        left_binary_img = binary_img[:, 0 : self.width // 2]
        right_binary_img = binary_img[:, self.width // 2 : :]
        # print(left_binary_img.shape)
        threshold = 20
        center_referene = 320
        side_reference = 100
        for i in range(0, box_num):
            top = self.height - box_height * (i + 1)
            bottom = self.height - box_height * i
            center_y = (top + bottom) // 2
            left_histograms.append(np.sum(left_binary_img[top:bottom, :], axis=0))
            # print(len(left_histograms[i]))
            right_histograms.append(np.sum(right_binary_img[top:bottom, :], axis=0))
            left_indices.append(np.where(left_histograms[i] > threshold)[0])
            right_indices.append(np.where(right_histograms[i] > threshold)[0])
            right_indices[i] = right_indices[i] + self.width // 2
            try:
                left_center_points.append((min(left_indices[i]) + max(left_indices[i])) // 2)
                # left_window_left = min(left_indices[i])
                left_window_left = left_center_points[i] - margin
                left_window_left = max(0, left_window_left)
                # left_window_right = max(left_indices[i])
                left_window_right = left_center_points[i] + margin
                left_window_right = min(320, left_window_right)
                cv2.line(warp_img, (left_center_points[i], center_y), (left_center_points[i], center_y), [0, 255, 0], 3)
                cv2.rectangle(warp_img, (left_window_left, top), (left_window_right, bottom), [0, 0, 255], 5)
            except:
                left_center_points.append(side_reference)
            try:
                right_center_points.append((min(right_indices[i]) + max(right_indices[i])) // 2)
                right_window_left = min(right_indices[i], 640)
                right_window_right = max(right_indices[i],320)
                cv2.line(warp_img, (right_center_points[i], center_y), (right_center_points[i], center_y), [0, 0, 255], 3)
                cv2.rectangle(warp_img, (right_window_left, top), (right_window_right, bottom), [0, 255, 0], 5)
            except:
                right_center_points.append(self.width - side_reference)
            center_points.append((left_center_points[i] + right_center_points[i]) // 2)
        return left_center_points, center_points, right_center_points

    def e_stop(self):
        degree_min = self.lidar_msg.angle_min * 180 / pi
        degree_max = self.lidar_msg.angle_max * 180 / pi
        degree_increment = self.lidar_msg.angle_increment * 180 / pi
        
        if self.degree_flag == False : 
            self.degrees = [degree_min + degree_increment * i for i, v in enumerate(self.lidar_msg.ranges)]
            self.degree_flag = True
        else : pass
        obstacle_index = []
        for i, v in enumerate(self.lidar_msg.ranges):
            if 0.0 < v < 2.0 and abs(self.degrees[i]) < 10:
                # print(f"obstacle:{degrees[i]}")
                obstacle_index.append(i)
        
        # print(obstacle_num)
        return obstacle_index

    def ctrl(self, img_bgr, obstacle_num, left_center_points, center_points, right_center_points):
        center_reference = 320
        side_reference = 100
        if obstacle_num != []:
            speed = 0
            brake = 200
            steer = 0
        else:
            speed = 0
            brake = 0
            steer = round((center_average - center_reference)*degree_resolution*p_gain)
        
        center_average = np.average(center_points)
        angle_resolution = np.pi/self.width
        degree_resolution = angle_resolution * 180 / pi
        p_gain = 71
        # if center_average > center_reference : # go to right
        #     print("You're on left side")
        #     print(f'center_average : {center_average}')
        #     steer = round((center_reference - center_average)*degree_resolution*p_gain)
        # else : # go to left
        #     print("You're on right side")
        #     print(f'center_average : {center_average}')
        
        if abs(steer) > 2000 : steer = 2000
        self.cmd_msg.speed = speed
        self.cmd_msg.brake = brake
        self.cmd_msg.steer = steer
        print(f'steer : {self.cmd_msg.steer}')
        self.pub.publish(self.cmd_msg)

    def run(self):
        if self.camera_flag and self.lidar_flag:
            img_bgr, img_hsv = self.cvt_color()
            combined_img = self.colort_detect(img_bgr, img_hsv)
            warp_img = self.img_warp(combined_img)
            binary_img = self.img_binary(warp_img)
            left_center_points, center_points, right_center_points = self.sliding_window(warp_img, binary_img)
            
            cv2.imshow("img_bgr", img_bgr)
            # cv2.imshow("img_hsv", img_hsv)
            cv2.imshow("warp_img", warp_img)
            cv2.waitKey(1)
            obstacle_num = self.e_stop()
            
            self.ctrl(img_bgr, obstacle_num, left_center_points, center_points, right_center_points)


def main():
    sub_class = Sub_class()
    while not rospy.is_shutdown():
        sub_class.run()


if __name__ == "__main__":
    main()
