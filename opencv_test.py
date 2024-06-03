#!/usr/bin/env python3
#-*- codin:utf-8 -*-

import cv2
import numpy as np

img = cv2.imread("traffic copy.jpg", cv2.IMREAD_COLOR)
cv2.namedWindow("img_window", cv2.WINDOW_NORMAL)
cv2.imshow("img_window", img)
cv2.waitKey(0)
cv2.destroyAllWindows()