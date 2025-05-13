#!/usr/bin/env python
#coding=utf-8
from re import T
import rospy
import numpy as np
from hello1 import Sendmessage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time
import math

class deep_calculate:
    def __init__(self):
        self.bridge = CvBridge()
        self.Image_compress_sub = rospy.Subscriber("colormodel_image",Image, self.convert)			# 訂閱攝像頭資訊 #"/kidsize/camera/image_raw" #"compress_image" #"/usb_cam/image_raw"
        # rospy.spin()
        self.first_red = True
        self.ya = np.zeros(32)
        self.aa = np.zeros(32)
        self.ba = np.zeros(32)
        self.x1 = 0
        self.y1 = 0
        self.x2 = 1
        self.y2 = 0
        self.cnt = 0
        self.a = True
        self.b = True
        self.slope = 0
        self.degree = 0
        self.red_width = 0
        self.Y_Dy = 24
        

    def convert(self, imgmsg):
        try:                             #影像通訊
            cv_image = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = cv2.resize(cv_image, (320, 240))
        cv_image_2 = cv2.resize(cv_image, (32, 24))

        self.red_width = 0
        self.R_Deep_Matrix = []
        for compress_width in range(0, 32, 1):                      #紅色深度
            self.r = True
            self.R_Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)
                if (blue == 255 and green == 255 and red == 0) and (self.r == True) :
                    self.red_width += 1
                    self.r = False
                if (blue == 255 and green == 255 and red == 0):
                    self.R_Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.R_Deep_Matrix[compress_width] = 24

        self.x1 = 0
        self.y1 = 0
        self.x2 = 1
        self.y2 = 0
        self.cnt = 0
        self.Y_L_Deep = 0
        self.Y_R_Deep = 0
        self.Y_C_Deep = 0
        self.Y_Deep_sum = 0
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.line_flag = False
        self.Xa = 0
        self.Ya = 0
        self.Xmin = 0
        self.Ymin = 0
        flag = True
        self.redsize = False
        

        for compress_width in range(0, 32, 1):                      #紅門斜率計算
            self.a = True
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)

                if (blue == 255 and green == 255 and red == 0) :
                    self.redsize = True
                    if self.a == True :
                        self.Xa = compress_width                        #紅門最低點的x值
                        self.Ya = 23 - compress_height                  #紅門最低點的y值
                        self.cnt += 1
                        self.a = False
                if (blue == 255 and green == 255 and red == 0) and self.first_red == True:
                    self.first_red = False
                    self.x1 = compress_width                            #紅門第一點的x值
                    self.y1 = 23 - compress_height                      #紅門第一點的y值
                if abs(self.red_width - self.cnt) <= 2 and self.b == True:
                    self.b = False
                    self.x2 = compress_width                            #紅門最後一點的x值
                    self.y2 = 23 - compress_height                      #紅門最後一點的y值
                if self.Ya == min(self.R_Deep_Matrix) :
                    if self.y1 > self.y2 :                              #紅色最低點同值數量很多時保留第一點的值
                        if flag == True:
                            self.Xmin = self.Xa
                            self.Ymin = self.Ya
                            flag = False
                    elif self.y2 >= self.y1 :                           #紅色最低點同值數量很多時更新至最後一點的值
                        self.Xmin = self.Xa
                        self.Ymin = self.Ya

        
        if abs(self.x1 - self.x2) < 1 :                             #若紅色面積過小則不判斷斜率直接給0
            # print('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')
            self.slope = 0
            self.degree = 0
        else : 
            # print('rrrrrrrrrrrrrrrrrrrrrrrrrrr')
            if abs(self.Xmin - self.x1) <= abs(self.Xmin - self.x2):
                self.slope =  (self.y2 - self.Ymin) / (self.x2 - self.Xmin)
            elif abs(self.Xmin - self.x1) > abs(self.Xmin - self.x2):
                self.slope =  (self.Ymin - self.y1) / (self.Xmin - self.x1)
            # elif (self.x2 - self.Xmin) == 0 or (self.Xmin - self.x1) == 0:    #斜率計算公式（利用最低點與某一邊做判斷）
            #     if abs(self.Xmin - self.x1) <= abs(self.Xmin - self.x2):
            #         self.slope =  (self.y2 - self.Ymin) / 0.00001
            #     elif abs(self.Xmin - self.x1) > abs(self.Xmin - self.x2):
            #         self.slope =  (self.Ymin - self.y1) / 0.00001
            self.degree = int(math.degrees(self.slope))
        
        self.first_red = True
        self.b = True
#----------------------------------------------------------------------#藍黃深度（最常用）
        self.Deep_Matrix = []
        for compress_width in range(0, 32, 1):
            self.Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)

                if (blue == 128 and green == 0 and red == 128) or (blue == 128 and green == 128 and red == 0) :
                    self.Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.Deep_Matrix[compress_width] = 24

        self.aa = self.Deep_Matrix
        self.filter_sum_aa = sum([element for element in np.array(self.aa) if element != 24])
        # cv2.imshow("Image_show",cv_image)
        cv2.waitKey(1)
#----------------------------------------------------------------------#黃色深度
        self.Y_Deep_Matrix = []
        for compress_width in range(0, 32, 1):                          
            self.Y_Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)
                if (blue == 128 and green == 128 and red == 0) :
                    self.Y_Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.Y_Deep_Matrix[compress_width] = 24
        self.ya = self.Y_Deep_Matrix
        # cv2.imshow("Image_show",cv_image)
        cv2.waitKey(1)
#----------------------------------------------------------------------#藍色深度
        self.B_Deep_Matrix = []*32
        for compress_width in range(0, 32, 1):
            self.B_Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)
                if (blue == 128 and green == 0 and red == 128):
                    self.B_Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.B_Deep_Matrix[compress_width] = 24
        self.ba = self.B_Deep_Matrix
        # cv2.imshow("Image_show",cv_image)
        cv2.waitKey(1)
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            send = Sendmessage()
            if send.Web == True:
                pass
            if send.Web == False:
                deep_calculate()
                rospy.spin()
    except rospy.ROSInterruptException:
        pass