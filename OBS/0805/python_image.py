#!/usr/bin/env python3
#coding=utf-8
# from turtle import st
import rospy
import numpy as np #import NumPy陣列
from hello1 import Sendmessage
# import sys
# sys.path.append('/home/iclab/Desktop/kid_hurocup/src/strategy')
# from Python_API import Sendmessage
from ddd import deep_calculate
# from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time
import math
import threading


deep            = deep_calculate()  #在ddd
send            = Sendmessage()     #在hello1


HEAD_HORIZONTAL = 2048 #頭水平，位置為馬達目標刻度，2048為正朝前方
HEAD_HEIGHT     = 1550 #頭高，位置為馬達目標刻度，2048為正朝前方
FOCUS_MATRIX    = [7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7]
#===========================================
STAY_X                          = 100
STAY_Y                          = -100
STAY_THETA                      = 0
#=========================================== 
MAX_FORWARD_X                   = 2500                                                     
MAX_FORWARD_Y                   = 100                                                            
MAX_FORWARD_THETA               = 0          
#=========================================== 
SMALL_FORWARD_X                 = 1600                                                     
SMALL_FORWARD_Y                 = -300                                                            
SMALL_FORWARD_THETA             = 0             
#=========================================== 
SMALL_BACK_X                    = -1500                                                     
SMALL_BACK_Y                    = -200                                                            
SMALL_BACK_THETA                = 0         
#=========================================== 
IMU_RIGHT_X                     = 0
IMU_RIGHT_Y                     = 700               
#===========================================                 
TURN_RIGHT_X                    = 0                                                    
TURN_RIGHT_Y                    = 700                                                     
TURN_RIGHT_THETA                = -4           
#=========================================== 
IMU_LEFT_X                      = -200
IMU_LEFT_Y                      = -700 
#===========================================                                         
TURN_LEFT_X                     = -200                                                    
TURN_LEFT_Y                     = -700                                                     
TURN_LEFT_THETA                 = 4  
#===========================================
SLOPE_RIGHT_TRANSLATE_X         = 0   
SLOPE_RIGHT_TRANSLATE_Y         = -1200
SLOPE_RIGHT_TRANSLATE_THETA     = -2
#===========================================
SLOPE_LEFT_TRANSLATE_X          = -200
SLOPE_LEFT_TRANSLATE_Y          = 1200
SLOPE_LEFT_TRANSLATE_THETA      = 2
#===========================================
YY_WALKWAY            =     110 #黃黃通道大小
YY_ERRO               =     15 #黃黃通道中心與畫面中心誤差值
#===========================================     
CRMAX           = 75 # red door 前後修正3 值越大離門越近 #68
CRMIN           = 75 # red door 前後修正3 值越大離門越近 #68  

REDDOOR_FIX     = "imu" #"slpoe" 平移修正方法選擇
REDDOOR_IMU     = False #紅門爬起後修正imu
#===========================================
# PRETURN_LEFT          = False 
PRETURN_LEFT          = True #預轉身左
PRETURN_LEFT_ANGLE    = 30

PRETURN_RIGHT         = False
# PRETURN_RIGHT         = True #預轉身右
PRETURN_RIGHT_ANGLE   = 30
#===========================================
YELLOW_SMALL_TURNHEAD       = False #通道不夠大轉頭
YELLOW_BLUE                 = False#黃線接藍牆
YELLOW_IMU_LEFT        = False #黃色與藍牆夠近時 或 兩個藍色中有洞使用深度判斷 走完轉頭先imu_fix
YELLOW_IMU_RIGHT       = False #黃色與藍牆夠近時 或 兩個藍色中有洞使用深度判斷 走完轉頭先imu_fix  
SIMP_TURN_HEAD              = False #簡單轉頭 原本turn_for_wall的地方會先simp_turn_head再turn_for_wall

FORCE_TURN_LEFT             = False #進轉頭策略 不轉頭強置左轉
FORCE_TURN_RIGHT            = False #進轉頭策略 不轉頭強置右轉

class Walk(): #步態、轉彎、直走速度、IMU
    # def __init__(self):
    def __init__(self, image):        

        self.image = image

        self.total_movement = 1500

        self.face_imu = 0

    def move(self, action_id, z=0, sensor= 0):
        #self.image.calculate()
        imu_flag = send.imu_value_Yaw < 0     #判斷是否<0 
        # self.forward_speed = self.straight_speed()
        slope_x_fix             = 500 if self.image.red_y_max < 90 else -300 if self.image.red_y_max > 100 else 0            #red door 平移 前後修正 值越大越遠
        right_straight_y        = -500 if self.image.center_deep<= 3  else 500 #if self.image.right_deep >= 7 else 0     #turn head 右轉 直走 值越大越遠             
        left_straight_y         = 300 if self.image.center_deep<= 3  else -200 #if self.image.left_deep >= 5 else 0     #turn head 左轉 直走 值越大越遠
        straight_90degree_fix   = -2 if ((imu_flag and abs(send.imu_value_Yaw) < 90) or (not imu_flag and abs(send.imu_value_Yaw) > 90)) else 2   #turn head 保持90度直走         
        # turn_x                  =   self.straight_speed()*2 if self.image.yellow_center_deep < 12 else self.straight_speed()  
        # turn_direction_x        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        actions             = { 'stay'                  : {'x'      : STAY_X,     \
                                                           'y'      : STAY_Y,   \
                                                           'theta'  : STAY_THETA},
                                ################################################
                                'max_speed'             : {'x':  self.total_movement,'y':   MAX_FORWARD_Y,  'theta': MAX_FORWARD_THETA },
                                ################################################
                                'small_back'            : {'x'      : SMALL_BACK_X,            \
                                                           'y'      : SMALL_BACK_Y,             \
                                                           'theta'  : SMALL_BACK_THETA},
                                ##############################################
                                'small_forward'         : {'x'      :  SMALL_FORWARD_X,            \
                                                           'y'      :  SMALL_FORWARD_Y,             \
                                                           'theta'  :  SMALL_FORWARD_THETA},
                                ################################################
                                'imu_fix'               : {'x': IMU_RIGHT_X if (send.imu_value_Yaw-self.face_imu) > 0 else IMU_LEFT_X,  \
                                                           'y': IMU_RIGHT_Y if (send.imu_value_Yaw-self.face_imu) > 0 else IMU_LEFT_Y,\
                                                           'theta': self.imu_angle()  },
                                # 'slope_fix'             : {'x': IMU_RIGHT_X-200 if self.get_imu() > 0 else IMU_LEFT_X-200,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y, 'theta': self.slope()      },
                                'slope_fix'             : {'x'      : IMU_LEFT_X   if deep.slope > 0 else IMU_RIGHT_X,                   \
                                                           'y'      : IMU_LEFT_Y  if deep.slope > 0 else IMU_RIGHT_Y,           \
                                                           'theta'  : self.slope()},
                                ################################################
                                'imu_right_translate'   : {'x'      : 0 + slope_x_fix, \
                                                           'y'      : -1000,            \
                                                           'theta'  : -2 + self.imu_angle()},
                                ################################################
                                'imu_left_translate'    : {'x'      :  0+ slope_x_fix, \
                                                           'y'      :  1200,               \
                                                           'theta'  : 1 + self.imu_angle()      },
                                ################################################
                                'slope_right_translate' : {'x'      : SLOPE_RIGHT_TRANSLATE_X + slope_x_fix, \
                                                           'y'      : SLOPE_RIGHT_TRANSLATE_Y,           \
                                                           'theta'  : SLOPE_RIGHT_TRANSLATE_THETA + self.slope()      },
                                ################################################
                                'slope_left_translate'  : {'x'      :  SLOPE_LEFT_TRANSLATE_X + slope_x_fix, \
                                                           'y'      :  SLOPE_LEFT_TRANSLATE_Y,      
                                                           'theta'  :  SLOPE_LEFT_TRANSLATE_THETA + self.slope()      },
                                ################################################
                                'dx_turn'               : {'x': TURN_RIGHT_X if self.image.deep_x > 0 else TURN_LEFT_X,       'y':  TURN_RIGHT_Y if self.image.deep_x > 0 else TURN_LEFT_Y,     'theta': self.turn_angle()  },
                                'turn_right_for_wall'   : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  },
                                'turn_right_back'       : {'x': IMU_RIGHT_X if send.imu_value_Yaw > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if send.imu_value_Yaw > 0 else IMU_LEFT_Y,            'theta': TURN_LEFT_THETA                 },#.
                                'turn_left_for_wall'    : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },
                                'turn_left_back'        : {'x': IMU_RIGHT_X if send.imu_value_Yaw > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if send.imu_value_Yaw > 0 else IMU_LEFT_Y,            'theta': TURN_RIGHT_THETA                },#.
                                'face_right_forward'    : {'x': MAX_FORWARD_X,    'y':  MAX_FORWARD_Y + right_straight_y ,    'theta': MAX_FORWARD_THETA + straight_90degree_fix    },
                                # 'right_right'         : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'face_left_forward'     : {'x': MAX_FORWARD_X,    'y':  MAX_FORWARD_Y + left_straight_y,     'theta': MAX_FORWARD_THETA + straight_90degree_fix   },
                                # 'left_left'           : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'preturn_left'          : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },
                                'preturn_right'         : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  },
                                'stop'                  : {'x': 0,  'y': 0, 'theta': 0}
                                }
        action              = actions.get(action_id,None)   
        if action is not None:
            x              = action['x']
            y              = action['y']
            theta          = action['theta']
            if action_id == 'max_speed':
                if self.total_movement < MAX_FORWARD_X:
                    self.total_movement += 200
                x = min(self.total_movement, MAX_FORWARD_X)
            else:
                self.total_movement = 1500
            send.sendContinuousValue(x, y, z, theta, sensor)        
        status.action_id = action_id        
        status.ContinuousValue = [x,y,self,theta]

    # def imu_yaw_ini(self):
    #     self.imu_yaw = 0

    # def get_imu(self):
    #     self.imu_yaw = send.imu_value_Yaw
    #     return self.imu_yaw 

    def turn_angle(self):   #一般 旋轉角度        
        turn_ranges = [ (17, -4), 
                        (12, -4), 
                        (8,  -3), 
                        (6,  -3), 
                        (4,  -2), 
                        (2,  -1),  
                        (0,   0),
                        (-2,  1),
                        (-4,  2),
                        (-6,  3),
                        (-8,  3),
                        (-12, 4),
                        (-17, 4)]
        for turn_range in turn_ranges:           
            if  self.image.deep_x >= turn_range[0]:
                return turn_range[1]
        return 0                                 
    
    def imu_angle(self):      #一般 imu修正角度
        imu_ranges = [  (180,  -4),
                        (90,  -4), 
                        (60,  -4), 
                        (45,  -4), 
                        (20,  -3), 
                        (10,  -3), 
                        (5,   -2), 
                        (2,   -1), 
                        (0,    0),
                        (-2,    1),
                        (-5,    2),
                        (-10,   3),
                        (-20,   3),
                        (-45,   4),
                        (-60,   4),
                        (-90,   4),
                        (-180,   4)]
        for imu_range in imu_ranges:           
            if (send.imu_value_Yaw-self.face_imu) >= imu_range[0]:
                return imu_range[1]
        return 0

    def slope(self):    #red 斜率修正角度
#-------------------fix to l---------------------
        if deep.slope > 0:          
            slopel_ranges = [(1,     3), 
                             (0.5,   3), 
                             (0.3,   2), 
                             (0.2,   2), 
                             (0.15,  2), 
                             (0.1,   1), 
                             (0.06,  1), 
                             (0.03,  1), 
                             (0,     0)]
            for slopel_range in slopel_ranges:
                if deep.slope >= slopel_range[0]:
                    return slopel_range[1]
            return 0
#--------------------fix to r--------------------
        elif deep.slope <= 0:     
            slopel_ranges = [(-1,     -3), 
                             (-0.5,   -3), 
                             (-0.3,   -2), 
                             (-0.2,   -2), 
                             (-0.15,  -2), 
                             (-0.1,   -1), 
                             (-0.06,  -1), 
                             (-0.03,  -1), 
                             (0,       0)]
            for slopel_range in slopel_ranges:
                if deep.slope >= slopel_range[0]:
                    return slopel_range[1]
            return 0 
        if send.color_mask_subject_size[5][0] < 5000 :
            slope_angle = 0        
        status.slope_angle = slope_angle
        return slope_angle

    def straight_speed(self):   #一般避障 前進速度        
        speed_ranges = [(24,    2500), 
                        (20,    2500), 
                        (16,    2200), 
                        (14,    1800), 
                        (12,    1400), 
                        (8,     1000), 
                        (6,      600), 
                        (3,      200), 
                        (0,        0)]
        for speed_range in speed_ranges:
            if self.image.deep_y >= speed_range[0]: #最小深度>=24
                return speed_range[1]
        return 0
    

class Normal_Obs_Parameter: #計算各種深度
    def __init__(self):
        self.line_at_left               = False
        self.line_at_right              = False
        self.line_at_right_test         = False 
        self.line_at_left_test          = False
        self.at_reddoor_flag            = False        
        self.deep_y                     = 24
        self.deep_x                     = 0
        # self.yellow_center_deep         = 0       #黃色中心深度值
        self.y_move                     = 0
        # self.deep_sum                   = 0
        # self.y_deep_sum                 = 0
        self.y_deep_left_sum            = 0
        self.y_deep_right_sum           = 0
        self.y_deep_y                   = 0
        self.y_deep_y_left              = 0
        self.y_deep_y_right             = 0
        self.yellow_leftside            = 0
        self.yellow_rightside           = 0
        self.left_deep                  = 0
        self.right_deep                 = 0
        self.center_deep                = 0
        self.red_x_min                  = 0
        self.red_x_max                  = 0
        self.blue_leftside              = 0
        self.blue_rightside             = 0
        self.red_y_max                  = 0
        self.b_y_max                    = 0
        self.b_x_min                    = 0
        self.b_x_max                    = 0
        self.yy = False
        self.YY_2     = False #出現兩個黃障礙
        self.YY_small = False #通道太窄
        self.yellow_center = 0
        self.deep_sum_l             = 0
        self.deep_sum_r             = 0
        self.yellow_through  = False
        self.yellow_move = False
        self.yellow_size = False
        self.center_deep_y = 0

    def calculate(self):
        self.red_y_max = send.color_mask_subject_YMax[5][0]
        if send.color_mask_subject_size[5][0] > 5000: #如果紅門夠大 #有紅時計算紅門資訊
            self.at_reddoor_flag = True #紅門旗標打開
            self.red_x_min = send.color_mask_subject_XMin[5][0] #紅門最左邊
            self.red_x_max = send.color_mask_subject_XMax[5][0] #紅門最右邊

            self.blue_rightside     = 0 #藍門資訊做歸0
            self.blue_leftside      = 0
            self.b_x_min            = 0
            self.b_x_max            = 0

            if send.color_mask_subject_cnts[2] == 1: #畫面裡只有一個藍色障礙物(只看到紅門下的一面牆)
                self.b_x_min = send.color_mask_subject_XMin[2][0] #藍牆最左邊
                self.b_x_max = send.color_mask_subject_XMax[2][0] #藍牆最右邊
            elif send.color_mask_subject_cnts[2] == 2: #畫面裡有兩個藍色障礙物(看到紅門下的兩面牆)
                xmax_one                = send.color_mask_subject_XMax[2][0]
                xmin_one                = send.color_mask_subject_XMin[2][0]
                xmin_two                = send.color_mask_subject_XMin[2][1]
                xmax_two                = send.color_mask_subject_XMax[2][1]            
                self.blue_rightside     = max(xmin_one, xmin_two) #可以算出紅門中間的洞洞
                self.blue_leftside      = min(xmax_one, xmax_two) #可以算出紅門中間的洞洞

        else : 
            self.at_reddoor_flag = False #if 紅門不夠大，紅門旗標關起來(一般避障)
    #----------------Blue_DeepMatrix-----------------
            self.b_y_max        = send.color_mask_subject_YMax[2][0] #藍色YMax
            # self.b_deep_y       = min(deep.ba) #藍色深度最小值(離最近)
            # self.b_deep_sum     = sum(deep.ba) #藍色所有深度和
            # self.b_left_deep    = deep.ba[2]   #第2行深度(藍)
            # self.b_right_deep   = deep.ba[30]  #第30行深度(倒數第二行)(藍)
            self.b_center_deep  = deep.ba[16]  #第16行深度(中間)(藍)
    #----------------Y_line_DeepMatrix---------------
            self.y_deep_y           = min(deep.ya) #黃色深度最小值(離最近)
            # self.y_deep_sum         = sum(deep.ya)
            # self.y_left_deep        = deep.ya[2]
            # self.y_right_deep       = deep.ya[30]
            # self.yellow_center_deep = deep.ya[16]
            self.y_deep_left_sum    = sum(deep.ya[0:15]) #黃色左邊深度總和(0-15行)(黃)
            self.y_deep_right_sum   = sum(deep.ya[16:31]) #黃色右邊深度總和(16-31行)(黃)
            self.y_deep_y_left      = min(deep.ya[0:15])
            self.y_deep_y_right     = min(deep.ya[16:31])
    #----------------Filter_matrix-------------------
            filter_matrix          = [max(0, a - b) for a, b in zip(FOCUS_MATRIX, deep.aa)] #在Foucus area內的障礙物的最大值(V矩陣最大值->最近)
            x_center_num           = sum(i for i, num in enumerate(FOCUS_MATRIX - np.array(deep.aa)) if num >= 0)
            x_center_cnt           = np.sum(np.array(FOCUS_MATRIX) - np.array(deep.aa) >= 0) 
            x_center               = (x_center_num / x_center_cnt) if x_center_cnt > 0 else 0
            left_weight_matrix     = list(range(32))            #0~31 #建立一個包含0到31的整數的列表
            right_weight_matrix    = list(range(31,-1,-1))      #31~0 #建立一個包含31到0的整數的列表
            right_weight           = np.dot(filter_matrix,  right_weight_matrix)#內積
            left_weight            = np.dot(filter_matrix,  left_weight_matrix)
            self.deep_y                 = min(deep.aa)
            self.center_deep_y          = min(deep.aa[8:25])
            # self.deep_sum               = sum(deep.aa)
            self.deep_sum_l             = sum(deep.aa[0:16]) #左邊深度總和(全)
            self.deep_sum_r             = sum(deep.aa[17:32])#右邊深度總和(全)
            self.left_deep              = min(deep.aa[3:6]) 
            self.right_deep             = min(deep.aa[27:30]) 
            self.center_deep            = min(deep.aa[14:19]) 
            x_boundary             = 31 if left_weight > right_weight else 0 #boundary point
            
            # '''
            #通道參數
            Y_XMax1                = send.color_mask_subject_XMax[1][0]
            Y_XMin1               = send.color_mask_subject_XMin[1][0]
            Y_XMin2                = send.color_mask_subject_XMin[1][1]
            Y_XMax2               = send.color_mask_subject_XMax[1][1]
            self.yellow_rightside      = max(Y_XMin1, Y_XMin2)
            self.yellow_leftside     = min(Y_XMax1, Y_XMax2)
            
            if min(send.color_mask_subject_size[1][0],send.color_mask_subject_size[1][1])>8000 and send.color_mask_subject_cnts[1]==2:
                self.yellow_size = True

            
            if send.color_mask_subject_cnts[1] == 2 and \
             (not strategy.turn_head_flag) and self.yellow_size:            

                self.YY_2 = True   #打開兩個黃障礙旗標                
                Y_XMax1                = send.color_mask_subject_XMax[1][0]
                Y_XMin1               = send.color_mask_subject_XMin[1][0]
                Y_XMin2                = send.color_mask_subject_XMin[1][1]
                Y_XMax2               = send.color_mask_subject_XMax[1][1]

                self.yellow_rightside      = max(Y_XMin1, Y_XMin2)

                self.yellow_leftside     = min(Y_XMax1, Y_XMax2)
                self.yellow_center=(self.yellow_rightside + self.yellow_leftside)/2
                self.yellow_erro=self.yellow_center-160
                    
                yellow_right_center  = (Y_XMax1+Y_XMin1)/2
                yellow_left_center  = (Y_XMax2+Y_XMin2)/2                

                if abs(self.yellow_rightside - self.yellow_leftside) < YY_WALKWAY :                                
                    status.yellow_state = "通道不夠大"
                    self.YY_small= True  #打開通道不夠大旗標
                    
                    if( yellow_right_center > 160 )and( yellow_left_center > 160):                        
                        status.yellow_line = "線在右"
                        self.line_at_right = True 
                        self.line_at_left  = False

                    elif (yellow_right_center < 160 )and (yellow_left_center < 160): #黃線在左邊                        
                        status.yellow_line = "線在左"
                        self.line_at_right = False
                        self.line_at_left  = True
                    else:
                        status.yellow_line = ""

                    self.deep_x = x_center - x_boundary
                else:                       
                    self.YY_small= False              
                    status.yellow_line = "無線"
                    if (self.yellow_erro < YY_ERRO*-1) and (not self.yellow_through):#平移到通道中心
                        self.deep_x = -20
                        self.yellow_move = True
                        
                        status.yellow_state = "通道夠大 左平移"
                    elif (self.yellow_erro > YY_ERRO) and (not self.yellow_through):
                        self.deep_x = 20
                        self.yellow_move = True
                    
                        status.yellow_state = "通道夠大 右平移"
                    elif abs(send.imu_value_Yaw) <= 5:
                        status.yellow_state = "直走"                        
                        self.deep_x = 0
                        self.yellow_through = True
                        self.yellow_move = False
                        self.yy = True  
                        
                        if (self.y_deep_left_sum > self.y_deep_right_sum) and (self.y_deep_y_left < 1):
                            status.yellow_line = "線在左"
                            self.line_at_right = False
                            self.line_at_left  = True

                        elif (self.y_deep_left_sum < self.y_deep_right_sum) and (self.y_deep_y_right < 1):
                            status.yellow_line = "線在右"

                            self.line_at_right = True 
                            self.line_at_left  = False                      

            else:
                self.YY_2 = False
                self.YY_small= False
                # self.yellow_move = False
                status.yellow_line = ""
                status.yellow_state = "未進黃黃"
                if self.yy:
                    time.sleep(1)
                    self.yy = False
                else:
                    self.deep_x = x_center - x_boundary #dx=Xc-Xb
            

class RobotStatus:    
    def __init__(self, image, walk):        
        
        self.image = image
        self.walk = walk     
        
        #==========機器人狀態==========#
        self.action_id = "stop"
        self.ContinuousValue = [0,0,0]
        #==========避障狀態==========#
        self.obs_action = "" #無障 一般避障 紅門
        #==========基礎數據==========#
        self.imu = 0
        # self.deep_x = 0
        # self.deep_y = 0
        # self.deep_sum_l = 0
        # self.deep_sum_r = 0
        # self.left_deep = 0
        # self.right_deep = 0
        # self.center_deep = 0
        #==========黃黃==========#
        self.yellow_state = ""
        # self.yellow_leftside = 0
        # self.yellow_rightside = 0        
        self.yellow_walkway = 0
        self.yellow_erro = 0
        self.yellow_line = "" #線在左 線在右
        #==========紅門==========#
        # self.bcnt = 0
        self.reddoor_state = "" #
        # self.blue_leftside = 0
        # self.blue_rightside = 0
        self.slope_angle = 0
        self.CRM = 0
        self.crawl_cnt = 0
        #==========轉頭==========#
        self.turnHead_state = ""
        self.turnHead_line_at = "" #線在左 線在右

        self.last_update_time = time.time()
        self.running    = True

    def update(self):
        #==========機器人狀態==========#
        # self.action_id = "stay"
        # self.ContinuousValue = [0,0,0,0]
        #==========避障狀態==========#
        # self.obs_action = "無障" #無障 一般避障 紅門
        #==========基礎數據==========#
        # self.imu = send.imu_value_Yaw
        # self.deep_x = self.image.deep_x
        # self.deep_y = self.image.deep_y
        # self.deep_sum_l = self.image.deep_sum_l
        # self.deep_sum_r = self.image.deep_sum_r
        # self.left_deep = self.image.left_deep
        # self.right_deep = self.image.right_deep
        # self.center_deep = self.image.center_deep
        #==========黃黃==========#
        # self.yellow_state = ""
        
        # self.yellow_leftside = self.image.yellow_leftside
        # self.yellow_rightside = self.image.yellow_rightside
        self.yellow_walkway = abs(self.image.yellow_rightside - self.image.yellow_leftside)
        # self.yellow_erro = self.image.yellow_center-YY_ERRO if self.image.YY_2 else 0

        # self.yellow_line = "無線" #線在左 線在右
        #==========紅門==========#
        # self.reddoor_state = "" #
        # self.bcnt = send.color_mask_subject_cnts[2]        
        # self.blue_leftside = self.image.blue_leftside
        # self.blue_rightside = self.image.blue_rightside
        # self.slope_angle = self.walk.slope_angle
        # self.CRM = self.image.red_y_max
        # self.crawl_cnt = self.obs.crawl_cnt
        #==========轉頭==========#
        # self.turnHead_state = ""
        # self.turnHead_line_at = "" #線在左 線在右        

    def val_print(self):
        self.update()        
        sys.stdout.write("\033[H\033[J")
        # ''' 
        sys.stdout.write(f"\
#==============機器人狀態==============#\n\
is_start         : {send.is_start}\n\
action_id        : {self.action_id}\n\
ContinuousValue  : {self.ContinuousValue}\n\
#===============避障狀態===============#\n\
obs_action       : {self.obs_action}\n\
#===============基礎數據===============#\n\
imu              : {send.imu_value_Yaw}\n\
deep_x           : {self.image.deep_x}\n\
deep_y           : {self.image.deep_y}\n\
center_deep_y    : {self.image.center_deep_y}\n\
deep_sum_lr      : {self.image.deep_sum_l} {self.image.deep_sum_r}\n\
LCRdeep          : {self.image.left_deep} {self.image.center_deep} {self.image.right_deep}\n\
#=================黃黃=================#\n\
yellow_state     : {self.yellow_state}\n\
yellow_side      : {self.image.yellow_leftside} {self.image.yellow_rightside}\n\
yellow_walkway   : {self.yellow_walkway}\n\
yellow_erro      : {self.yellow_erro}\n\
yellow_size      : {send.color_mask_subject_size[1][0]} {send.color_mask_subject_size[1][1]}\n\
yellow_deep      : {self.image.y_deep_y}\n\
yellow_cnt       : {send.color_mask_subject_cnts[1]}\n\
yellow_line      : {self.yellow_line}\n\
#=================紅門=================#\n\
reddoor_state    : {self.reddoor_state}\n\
bcnt             : {send.color_mask_subject_cnts[2]}\n\
bx min max       : {self.image.b_x_min} {self.image.b_x_max}\n\
blue_side        : {self.image.blue_leftside} {self.image.blue_rightside}\n\
red_y_max        : {self.image.red_y_max}\n\
red_size         : {send.color_mask_subject_size[5][0]}\n\
crawl_cnt        : {self.crawl_cnt}\n\
slope            : {deep.slope}\n\
face_imu         : {self.walk.face_imu}\n\
#=================轉頭=================#\n\
turnHead_state   : {self.turnHead_state}\n\
turnHead_line_at : {self.turnHead_line_at}\n\
deep L R         : {deep.aa[14]} {deep.aa[18]}\n\
        ")
        # '''

        # sys.stdout.write(f"test:{}\n")                

    def draw_function(self):        
            
        if self.image.YY_2:   
            send.drawImageFunction(1,0,0,0,0,0,0,0,0)#左邊深度
            send.drawImageFunction(2,0,0,0,0,0,0,0,0)#右邊深度
            send.drawImageFunction(3,0,160,160,0,240,255,255,255)#畫面中心 (白色)
            send.drawImageFunction(4,0,0,0,0,0,0,0,0)#dx                
            send.drawImageFunction(5,0,self.image.yellow_leftside,self.image.yellow_leftside,0,240,160,32,240)#左邊黃通 (紅棕色)
            send.drawImageFunction(6,0,self.image.yellow_rightside,self.image.yellow_rightside,0,240,160,32,240)#右邊黃通 (紅棕色)
            send.drawImageFunction(7,0,int(self.image.yellow_center),int(self.image.yellow_center),0,240,255,0,0)#中間黃通 (紫色)
        else:
            # send.drawImageFunction(1,0,40,40,int(240-max(deep.aa[3:6])*10),240,192,192,192)#左邊深度 (淺灰色)
            # send.drawImageFunction(2,0,280,280,int(240-max(deep.aa[27:30])*10),240,192,192,192)#右邊深度 (淺灰色)
            # send.drawImageFunction(3,0,160,160,int(240-deep.aa[16]*10),240,128,42,42)#中間深度 (紫色) 
            send.drawImageFunction(1,0,0,0,0,0,0,0,0)#左邊深度
            send.drawImageFunction(2,0,0,0,0,0,0,0,0)#右邊深度
            send.drawImageFunction(3,0,0,0,0,0,255,255,255)#畫面中心 (白色)
            send.drawImageFunction(4,0,int(self.image.deep_x*10 if self.image.deep_x > 0 else (self.image.deep_x+32)*10),\
                               int(self.image.deep_x*10 if self.image.deep_x > 0 else (self.image.deep_x+32)*10),0,240,255,0,0)#dx (紅色)            
            send.drawImageFunction(5,0,0,0,0,0,0,0,0)#左邊黃通 
            send.drawImageFunction(6,0,0,0,0,0,0,0,0)#右邊黃通 
            send.drawImageFunction(7,0,0,0,0,0,0,0,0)#中間黃通                

    def runThread(self):        
        self.running = True
        while send.is_start:
            self.image.calculate()   
            if time.time() - self.last_update_time > 0.1:
                self.val_print()
                self.last_update_time = time.time()
            
            self.draw_function()
            # time.sleep(0.1)  # 不要太頻繁

            if strategy.first_reddoor:
                time.sleep(0.1)
            if not send.is_start :
                self.running = False                
                break        
    
    def run(self):
        self.image.calculate()       
        self.val_print()        
        self.draw_function()                
        
class Obs: #各種避障動作    
    def __init__(self, status):        
        self.status = status
        self.image = status.image
        self.walk = status.walk

        self.blue_at_left           = False
        self.blue_at_right          = False
        self.redoor_distence        = False
        self.need_fix_slope         = True
        self.first_reddoor          = False
        self.start_walking          = False
        self.imu_ok                 = False
        self.need_imu_back          = True
        self.line_at_right_single   = False
        self.line_at_left_single    = False
        self.door_at_left           = False
        self.door_at_right          = False
        self.left_deep_sum          = 0
        self.right_deep_sum         = 0
        self.crawl_cnt              = 0
        self.translate              = False
        self.i                      = 0
        self.temp_dx                = 0
        self.turn_head_flag         = False
        self.yb                     = False

    def red_door(self): #前後修正1 -> 修斜率 -> 前後修正2 -> 平移 -> 前後修正3 -> 趴下
        
        if not self.first_reddoor:
            self.first_reddoor = True   
            send.sendHeadMotor(1,HEAD_HORIZONTAL,100) #頭在中間
            send.sendHeadMotor(2,HEAD_HEIGHT + 150,100)
            time.sleep(0.2)
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)

        # while send.color_mask_subject_YMax[5][0] > 120 and status.running: #red door 前後修正1 值越大離門越近 #離紅門太近了            
        #     self.walk.move('small_back')
        #     status.reddoor_state = "離紅門太近"            

        if abs(deep.slope) > 0.03 :     # red door 修斜率
            status.reddoor_state = "修斜率1"
            while abs(deep.slope) > 0.03 and status.running:                
                self.walk.move('slope_fix') #self.walk.move('imu_fix') #根據斜率修正IMU
                self.image.calculate()   

            self.walk.move('stay')    
            time.sleep(1)    
            self.walk.face_imu = send.imu_value_Yaw

        if (send.color_mask_subject_YMax[5][0] < 85) :     #red door 前後修正2 值越大離門越近 
            status.reddoor_state = "離紅門太遠"
            while send.color_mask_subject_YMax[5][0] < 85 and status.running: #離紅門太遠了                
                self.walk.move('small_forward') 
                
        elif (self.image.red_y_max > 85) or self.image.b_center_deep == 0:   #red door  前後修正2 值越大離門越近 
            status.reddoor_state = "離紅門太近"
            while self.image.red_y_max > 85 or self.image.b_center_deep == 0 and status.running:                
                self.walk.move('small_back')
                
        while 1 and status.running:     
            
            if (self.image.red_x_min < 2 and self.image.red_x_max > 315) and send.color_mask_subject_size[5][0] > 5000: #紅門在眼前
                
                if  self.translate:
                    while abs(deep.slope) > 0.03 and status.running:
                        self.walk.move('slope_fix')
                        status.reddoor_state = "修斜率2"
                    self.walk.move('stay')    
                    time.sleep(0.5)                            
                    self.walk.face_imu = send.imu_value_Yaw
                    self.crawl()
                    break
                elif (send.color_mask_subject_cnts[2] == 1):
                    if (self.image.b_x_min < 2 and self.image.b_x_max > 40):                        
                        self.translate = False
                        # self.walk.move('slope_right_translate')    
                        # self.walk.move('imu_right_translate')   
                        self.walk.move(f"{REDDOOR_FIX}_right_translate")                                      
                        status.reddoor_state = "紅門右平移 1B"
                        
                    elif (self.image.b_x_max > 315 and self.image.b_x_min < 275):                        
                        self.translate = False
                        # self.walk.move('slope_left_translate')
                        # self.walk.move('imu_left_translate')
                        self.walk.move(f"{REDDOOR_FIX}_left_translate")                        
                        status.reddoor_state = "紅門左平移 1B"
                        
                    else:
                        self.translate = True
                        status.reddoor_state = "位置修正完畢"
                
                elif (send.color_mask_subject_cnts[2] == 2):                    
                    if self.image.blue_rightside < 255 and self.image.blue_leftside < 35: #停在太右邊 blue_rightside調大                        
                        # self.walk.move('slope_left_translate')
                        # self.walk.move('imu_left_translate') 
                        self.walk.move(f"{REDDOOR_FIX}_left_translate")
                        self.translate = False                        
                        status.reddoor_state = "紅門左平移 2B"
                        
                    elif self.image.blue_rightside > 255 and self.image.blue_leftside >35: #停在太左邊 blue_leftside調小 #290 65                        
                        # self.walk.move('slope_right_translate')
                        # self.walk.move('imu_right_translate') 
                        self.walk.move(f"{REDDOOR_FIX}_right_translate")
                        self.translate = False                        
                        status.reddoor_state = "紅門右平移 2B"
                        
                    else:
                        self.translate = True
                        status.reddoor_state = "位置修正完畢"
                else :                    
                    status.reddoor_state = "紅門夠大 0B"
                    if self.blue_at_right :
                        # self.walk.move('slope_right_translate')
                        # self.walk.move('imu_right_translate') 
                        self.walk.move(f"{REDDOOR_FIX}_right_translate")
                    elif self.blue_at_left :
                        # self.walk.move('slope_left_translate')
                        # self.walk.move('imu_left_translate') 
                        self.walk.move(f"{REDDOOR_FIX}_left_translate")
                
            elif (self.image.red_x_min < 2 and self.image.red_x_max < 315):                
                # self.walk.move('slope_left_translate')
                # self.walk.move('imu_left_translate') 
                self.walk.move(f"{REDDOOR_FIX}_left_translate")
                status.reddoor_state = "紅門不夠大＆偏左"
            
            elif (self.image.red_x_min > 2 and self.image.red_x_max > 315):                
                # self.walk.move('slope_right_translate')
                # self.walk.move('imu_right_translate') 
                self.walk.move(f"{REDDOOR_FIX}_right_translate")
                status.reddoor_state = "紅門不夠大＆偏右" 
            else :                
                self.walk.move('stay')
                status.reddoor_state = "紅門轉頭尋找"
                # time.sleep(1)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180) #頭往右轉
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180)
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180)
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # time.sleep(1.5)                 
                # send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180) #頭往左轉
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180)
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180)
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # time.sleep(1.5)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL,180) #頭轉正
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL,180)
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # send.sendHeadMotor(1,HEAD_HORIZONTAL,180)
                # send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                # time.sleep(0.3)                
                while abs(deep.slope) > 0.03 and status.running:                
                    self.walk.move('slope_fix') #self.walk.move('imu_fix') #根據斜率修正IMU
                self.walk.move('stay')    
                time.sleep(0.5)                                  
                self.walk.face_imu = send.imu_value_Yaw
                # if self.door_at_right :
                #     while self.image.red_x_min < 160 and status.running:                        
                #         self.walk.move('imu_right_translate')
                # elif self.door_at_left:
                #     while self.image.red_x_max > 160 and status.running:                        
                #         self.walk.move('imu_left_translate')

    def crawl(self):
        self.translate = False        
        while 1 and status.running:            
            if self.translate :
                while abs(deep.slope) > 0.03 and status.running:
                    self.walk.move('slope_fix')                    
                    status.reddoor_state = "修斜率3"
                self.walk.move('stay')    
                time.sleep(0.5)                
                self.walk.face_imu = send.imu_value_Yaw                                        
                break

            elif (send.color_mask_subject_cnts[2] == 1):
                if (self.image.b_x_min < 2 and self.image.b_x_max > 50):                    
                    self.translate = False
                    # self.walk.move('slope_right_translate')
                    # self.walk.move('imu_right_translate')
                    self.walk.move(f"{REDDOOR_FIX}_right_translate")                    
                    status.reddoor_state = "紅門右平移 1B"

                elif (self.image.b_x_max > 315 and self.image.b_x_min < 280):                    
                    self.translate = False
                    # self.walk.move('slope_left_translate')      
                    # self.walk.move('imu_left_translate')    
                    self.walk.move(f"{REDDOOR_FIX}_left_translate")          
                    status.reddoor_state = "紅門左平移 1B"
                else:
                    # self.translate = True
                    status.reddoor_state = "位置修正完畢"
            
            elif (send.color_mask_subject_cnts[2] == 2):                
                if self.image.blue_rightside < 255 and self.image.blue_leftside < 35: #停在太右邊 blue_rightside調大                    
                    # self.walk.move('slope_left_translate')
                    # self.walk.move('imu_left_translate')
                    self.walk.move(f"{REDDOOR_FIX}_left_translate")
                    self.translate = False                    
                    status.reddoor_state = "紅門左平移 2B"
                    
                elif self.image.blue_rightside > 255 and self.image.blue_leftside > 35:  #停在太左邊 blue_leftside調小 #290 65                    
                    # self.walk.move('slope_right_translate') 
                    # self.walk.move('imu_right_translate')
                    self.walk.move(f"{REDDOOR_FIX}_right_translate")
                    self.translate = False                    
                    status.reddoor_state = "紅門右平移 2B"                    
                else:
                    self.translate = True
                    status.reddoor_state = "位置修正完畢"

        if((self.image.red_y_max) < CRMIN):            #離紅門太遠
            status.reddoor_state = "離紅門太遠"
            while(self.image.red_y_max < CRMIN) and status.running:                          
                self.walk.slope()
                self.walk.move('small_forward')                
                                
        elif(self.image.red_y_max > CRMAX):          #離紅門太近
            status.reddoor_state = "離紅門太遠"
            while(self.image.red_y_max > CRMAX) and status.running:                         
                self.walk.slope()
                self.walk.move('small_back')
                
        while abs(deep.slope) > 0.03 and status.running: #紅門太斜
            self.walk.slope()
            self.walk.move('slope_fix')
            status.reddoor_state = "修斜率4"
        # while ( abs(send.imu_value_Yaw) > 2) and status.running:
        #     self.walk.move('imu_fix')
        #     status.reddoor_state = "修斜率4"

        send.sendContinuousValue(0, 0 , 0 , 0 , 0) 
        time.sleep(1)
        
        send.sendBodyAuto(0,0,0,0,1,0) #mode = 1為continue步態 #停下來

        # '''
        time.sleep(2)  
        send.sendBodySector(333) #手水平放下（屁股有障礙物）
        time.sleep(4.4)
        send.sendBodySector(29)   
        time.sleep(0.5)        
        send.sendHeadMotor(2,1080,180)
        send.sendHeadMotor(2,1080,180)
        send.sendHeadMotor(2,1080,180)
        time.sleep(0.3)
        send.sendBodySector(1111)
        time.sleep(8)
        while self.crawl_cnt < 5:    #count 6次            
            send.sendBodySector(2222)
            time.sleep(2)
            self.crawl_cnt += 1
            self.status.crawl_cnt += 1            
        send.color_mask_subject_YMax[1][0] = 0 #黃色YMax =0
        send.sendHeadMotor(1,HEAD_HORIZONTAL,100)
        send.sendHeadMotor(2,2400,100) #頭往上抬
        send.sendHeadMotor(1,HEAD_HORIZONTAL,100)
        send.sendHeadMotor(2,2400,100) #頭往上抬
        send.sendHeadMotor(1,HEAD_HORIZONTAL,100)
        send.sendHeadMotor(2,2400,100) #頭往上抬
        time.sleep(3)
        while self.crawl_cnt < 9 and self.image.deep_y == 24:   #cnt3數到7(4次)            
            send.sendBodySector(2222)
            time.sleep(2)
            self.crawl_cnt += 1 
        if self.crawl_cnt > 8 :            
            time.sleep(1)
            send.sendBodySector(3333)
            time.sleep(11)
            # send.sendBodySector(29)    
            # time.sleep(0.5)
            # send.sendHeadMotor(1,HEAD_HORIZONTAL,100)
            # send.sendHeadMotor(2,HEAD_HEIGHT,100)
            # time.sleep(1)
            # send.sendBodySector(111)
            # time.sleep(3.5)            
            # send.sendBodyAuto(0,0,0,0,1,0) 
            
        else :            
            status.reddoor_state = "提早爬起"
            time.sleep(1)
            send.sendBodySector(3333)
            time.sleep(11)
            send.sendBodySector(29)    
            time.sleep(1)            
            send.sendBodyAuto(0,0,0,0,1,0) 
            while self.i < 300:
                status.reddoor_state = "向前走遠離紅門"
                self.walk.move('max_speed')  
                self.i += 5
                time.sleep(0.05)
                
            send.sendBodyAuto(0,0,0,0,1,0)             
            time.sleep(1)
        send.sendBodySector(29)    
        time.sleep(0.5)
        send.sendHeadMotor(1,HEAD_HORIZONTAL,100)
        send.sendHeadMotor(2,HEAD_HEIGHT,100)
        send.sendBodySector(111)
        time.sleep(3.5)
        send.sendBodyAuto(0,0,0,0,1,0) 
        self.walk.face_imu = 0
        if REDDOOR_IMU:
            while ( abs(send.imu_value_Yaw) > 2) and status.running:
                self.walk.move('imu_fix')
        # '''
        # while True and status.running:
        #     rospy.sleep(0.1)
        status.reddoor_state = "爬行完畢"

    def turn_head(self):
        self.walk.move('stay')
        status.obs_action = "轉頭"
        self.yellow_flag = False
        self.turn_head_flag = True
        if not self.image.line_at_right and not self.image.line_at_left and (not FORCE_TURN_LEFT) and (not FORCE_TURN_RIGHT): 
            time.sleep(1)
            send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180) #頭往右轉
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            time.sleep(1.5) 
            if send.color_mask_subject_YMax[1][0] > 220 and send.color_mask_subject_size[1][0] > 5000: #黃色夠近夠大
                status.turnHead_state = "黃色夠近夠大 跳出"
                self.line_at_right_single = True
            # if send.color_mask_subject_YMax[5][0] > 220 and send.color_mask_subject_size[5][0] > 5000: #紅色夠近夠大
            if send.color_mask_subject_size[5][0] > 9000:                
                status.turnHead_state = "紅色夠近夠大 跳出"
                self.door_at_right = True
                self.door_at_left = False

                # self.door_at_right = False                                         #如果不想爬紅門的話就開啟這裡的程式碼
                # self.door_at_left = True

            self.right_deep_sum = sum(deep.aa) 
            send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180) #頭往左轉
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            time.sleep(1.5)
            if send.color_mask_subject_YMax[1][0] > 220 and send.color_mask_subject_size[1][0] > 3000:
                status.turnHead_state = "黃色夠近夠大 跳出"
                self.line_at_left_single = True
            # if send.color_mask_subject_YMax[5][0] > 220 and send.color_mask_subject_size[5][0] > 5000:
            if send.color_mask_subject_size[5][0] > 9000:
                status.turnHead_state = "紅色夠近夠大 跳出"
                self.door_at_left = True
                self.door_at_right = False

                # self.door_at_left = False                                          #如果不想爬紅門的話就開啟這裡的程式碼
                # self.door_at_right = True

            self.left_deep_sum = sum(deep.aa) #左邊深度總和            
            status.turnHead_line_at = "線在左" if self.line_at_left_single else "線在右" if self.line_at_right_single else ""
            send.sendHeadMotor(1,HEAD_HORIZONTAL,180) #頭回中間
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            send.sendHeadMotor(1,HEAD_HORIZONTAL,180)
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            send.sendHeadMotor(1,HEAD_HORIZONTAL,180)
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            time.sleep(0.5)
        else : #已知黃線在左/右側
            self.right_deep_sum = 0
            self.left_deep_sum = 0
        if (self.right_deep_sum > self.left_deep_sum) or self.image.line_at_left or self.line_at_left_single or self.door_at_right or FORCE_TURN_RIGHT: #turn head 右轉            
            status.turnHead_state = "右轉判斷"
            if (self.image.center_deep) > 8 and (not self.yb):
                while ( self.image.center_deep > 8) and status.running and (not self.yb):                  
                    status.turnHead_state = "距離牆太遠"
                    self.walk.move('small_forward')                    
            elif ( self.image.center_deep < 6) and (not self.yb):
                while ( self.image.center_deep < 6 ) and status.running and (not self.yb):                    
                    status.turnHead_state = "距離牆太近"
                    self.walk.move('small_back')                   
            if abs(send.imu_value_Yaw) < 80:
                while abs(send.imu_value_Yaw) < 80 and status.running:
                    status.turnHead_state = "右轉"
                    self.walk.move('turn_right_for_wall')                    
            send.sendHeadMotor(1,HEAD_HORIZONTAL+601,100) #身體面相右，頭往左轉看牆
            send.sendHeadMotor(2,1600,100) 
            time.sleep(0.5)
            
            # send.sendContinuousValue(0, 0 , 0 , 0 , 0)
            self.walk.move("stay")
            time.sleep(2)            
            
            while abs(self.image.deep_x) >= 5 and status.running and (not YELLOW_IMU_RIGHT):
                status.turnHead_state = "頭面向牆"
                self.walk.move('face_right_forward')                
                if send.color_mask_subject_XMin[5][0] <80 and send.color_mask_subject_cnts[5] >= 1 and send.color_mask_subject_size[5][0] > 3000:
                    break     
            while deep.aa[18] <= 11 and status.running and YELLOW_IMU_RIGHT:                              
                status.turnHead_state = "頭面向牆"
                self.walk.move('face_right_forward')                
                if send.color_mask_subject_XMin[5][0] <80 and send.color_mask_subject_cnts[5] >= 1 and send.color_mask_subject_size[5][0] > 3000:
                    break
                # if YELLOW_DX_BREAK_RIGHT:
                #     if self.image.y_deep_y < 5:
                #         self.yellow_flag = True
                #     if (self.temp_dx/self.image.deep_x) < 0 and (self.yellow_flag):
                #         break                
                #     self.temp_dx = self.image.deep_x
            self.walk.move("stay")
            send.sendHeadMotor(1,HEAD_HORIZONTAL,100) #頭轉正
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(0.5)
            if abs(send.imu_value_Yaw) > 50:
                while abs(send.imu_value_Yaw) > 50 and status.running: 
                    status.turnHead_state = "回正"
                    self.walk.move('turn_right_back')
            if self.door_at_right :
                status.obs_action = "紅門"
                self.red_door()
            self.image.line_at_left = False
            self.line_at_left_single = False
            self.image.line_at_right = False
            self.line_at_right_single = False
            if YELLOW_IMU_RIGHT:
                while ( abs(send.imu_value_Yaw) > 2) and status.running:
                    self.walk.move('imu_fix')
                        
        elif (self.left_deep_sum > self.right_deep_sum) or self.image.line_at_right or self.line_at_right_single or self.door_at_left or FORCE_TURN_LEFT:            
            status.turnHead_state = "左轉判斷"
            if (self.image.center_deep > 9) and (not self.yb):
                while ( self.image.center_deep > 9 ) and status.running and (not self.yb):
                    status.turnHead_state = "距離牆太遠"
                    self.walk.move('small_forward') 
            elif ( self.image.center_deep < 7 ) and (not self.yb):
                while ( self.image.center_deep < 7 ) and status.running and (not self.yb):
                    status.turnHead_state = "距離牆太近"
                    self.walk.move('small_back') 
            if abs(send.imu_value_Yaw) < 70:                           #turn head 旋轉角度  左轉 越大轉越多
                while abs(send.imu_value_Yaw) < 70 and status.running:
                    status.turnHead_state = "左轉"
                    self.walk.move('turn_left_for_wall')                    
            send.sendHeadMotor(1,HEAD_HORIZONTAL-601,100) #身體面相左，頭往右轉看牆
            send.sendHeadMotor(2,1580,100) 
            time.sleep(0.5)
            # send.sendContinuousValue(0, 0 , 0 , 0 , 0)
            self.walk.move("stay")
            time.sleep(2)   

            while abs(self.image.deep_x) >= 5 and status.running and (not YELLOW_IMU_LEFT):
                status.turnHead_state = "頭面向牆"
                self.walk.move('face_left_forward')
                if send.color_mask_subject_XMax[5][0] > 240 and send.color_mask_subject_cnts[5] >= 1 and send.color_mask_subject_size[5][0] > 3000:
                    break            
            while deep.aa[14] <= 11 and status.running and YELLOW_IMU_LEFT:                 
                status.turnHead_state = "頭面向牆"
                self.walk.move('face_left_forward')
                if send.color_mask_subject_XMax[5][0] > 240 and send.color_mask_subject_cnts[5] >= 1 and send.color_mask_subject_size[5][0] > 3000:
                    break
                # if YELLOW_DX_BREAK_LEFT:
                #     if self.image.y_deep_y < 5:
                #         self.yellow_flag = True
                #     if (self.temp_dx/self.image.deep_x) < 0 and (self.yellow_flag):
                #         break                
                #     self.temp_dx = self.image.deep_x
            self.walk.move("stay")            
            send.sendHeadMotor(1,HEAD_HORIZONTAL,100)#頭轉正
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(0.5)            
            if abs(send.imu_value_Yaw) > 50:               #turn head 左轉 回正 數字越小越正對
                while abs(send.imu_value_Yaw) > 50 and status.running:    
                    status.turnHead_state = "回正"
                    self.walk.move('turn_left_back')                    
            if self.door_at_left :
                status.obs_action = "紅門"
                self.red_door()
            self.image.line_at_right = False
            self.line_at_right_single = False
            self.image.line_at_left = False
            self.line_at_left_single = False
            if YELLOW_IMU_LEFT:
                while ( abs(send.imu_value_Yaw) > 2) and status.running:
                    self.walk.move('imu_fix')
        status.turnHead_line_at = ""
        self.turn_head_flag = False
        self.yb                     = False
            
    def simp_turn_head(self):
        self.walk.move('stay')
        status.obs_action = "簡單轉頭"
        time.sleep(1)
        send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180) #頭往右轉
        send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
        send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180)
        send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
        send.sendHeadMotor(1,HEAD_HORIZONTAL-531,180)
        send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
        time.sleep(1.5) 

        self.right_deep_sum = sum(deep.aa) 

        send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180) #頭往左轉
        send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
        send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180)
        send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
        send.sendHeadMotor(1,HEAD_HORIZONTAL+551,180)
        send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
        time.sleep(1.5)

        self.left_deep_sum = sum(deep.aa) #左邊深度總和  

        send.sendHeadMotor(1,HEAD_HORIZONTAL,180) #頭回中間
        send.sendHeadMotor(2,HEAD_HEIGHT,180)
        send.sendHeadMotor(1,HEAD_HORIZONTAL,180)
        send.sendHeadMotor(2,HEAD_HEIGHT,180)
        send.sendHeadMotor(1,HEAD_HORIZONTAL,180)
        send.sendHeadMotor(2,HEAD_HEIGHT,180)
        
        self.simp_turn =  "left" if self.right_deep_sum < self.left_deep_sum else "right" if self.right_deep_sum > self.left_deep_sum else ""
        if self.simp_turn == "left":
            while abs(send.imu_value_Yaw) < 20 and status.running:    
                self.walk.move('turn_left_for_wall')
            
            self.imu_ok = True
        elif self.simp_turn == "right":
            while abs(send.imu_value_Yaw) < 20 and status.running:    
                self.walk.move('turn_right_for_wall')
            
            self.imu_ok = True

    def initt(self):
        self.image.line_at_right = False
        self.line_at_right_single = False
        self.image.line_at_left = False
        self.line_at_left_single = False
        self.image.yy = False
        self.image.YY_2     = False
        self.image.YY_small = False
        status.obs_action = "OFF"
        status.yellow_state = ""
        status.yellow_line = ""
        status.reddoor_state = ""
        status.turnHead_state = ""
        status.turnHead_line_at = ""
        self.image.yellow_move = False
        self.imu_ok                 = False
        self.need_imu_back          = True
        self.door_at_left           = False
        self.door_at_right          = False
        self.turn_head_flag         = False
        self.yb                     = False
        self.walk.face_imu = 0
        self.crawl_cnt=0
        self.image.yellow_size      = False
        self.first_reddoor = False

    def main(self):        
        
        if send.is_start : #策略主指撥開關
            threading.Thread(target=self.status.runThread, daemon=True).start()                 
        #=============================strategy=============================
            if not self.start_walking :                        #指撥後初始動作                
                send.sendSensorReset(1,1,1)
                self.initt()
                #================================================
                # self.preturn_left = False
                self.preturn_left = PRETURN_LEFT
                # self.preturn_left = True
                #================================================
                # self.preturn_right = False
                self.preturn_right = PRETURN_RIGHT
                # self.preturn_right = True
                #================================================
                send.sendHeadMotor(1,HEAD_HORIZONTAL,100) #頭部初始動作
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                send.sendHeadMotor(1,HEAD_HORIZONTAL,100)
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                send.sendHeadMotor(1,HEAD_HORIZONTAL,100)
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                time.sleep(0.5)
                send.sendBodyAuto(0,0,0,0,1,0) #步態呼叫
                self.start_walking = True
            if self.preturn_left:
                while abs(send.imu_value_Yaw) < PRETURN_LEFT_ANGLE and status.running:
                    self.walk.move('preturn_left')
                    
                self.preturn_left = False
            elif self.preturn_right:
                while abs(send.imu_value_Yaw) < PRETURN_RIGHT_ANGLE and status.running:
                    self.walk.move('preturn_right')
                    
                self.preturn_right = False
            if self.image.at_reddoor_flag: #進紅門
                status.obs_action = "紅門"
                self.red_door()
                pass

            else : #進一般避障
                status.obs_action = "一般避障"

                # '''
                if self.image.yellow_move :
                    status.obs_action = "黃黃通道"
                    if self.image.deep_x > 0:
                        self.walk.move('imu_right_translate')                
                    else:
                        self.walk.move('imu_left_translate')

                # '''
                elif ( self.image.left_deep < 15 ) and ( self.image.right_deep < 15 ) and ( self.image.center_deep < 15 ) and\
                (1000 < (max(send.color_mask_subject_size[1])) < 5000) and YELLOW_BLUE:
                    status.obs_action = "轉頭"
                    self.yb = False
                    if max(send.color_mask_subject_XMax[1])<160:
                        self.image.line_at_left = True
                    elif max(send.color_mask_subject_XMax[1])>160:
                        self.image.line_at_right = True                        
                    while ( abs(send.imu_value_Yaw) > 2) and (not self.imu_ok) and status.running:
                        self.walk.move('imu_fix')
                    self.walk.move('stay')
                    # time.sleep(1)
                    self.imu_ok = True
                    self.turn_head()
                # '''

                elif self.image.deep_y < 24:
                # if self.image.deep_y < 24:                    

                    if (self.image.line_at_right or self.image.line_at_left) and self.image.YY_small and (self.image.center_deep_y < 10) and\
                        YELLOW_SMALL_TURNHEAD:
                        self.yb = True
                        status.turnHead_state ="yysmall"
                        while ( abs(send.imu_value_Yaw) > 2) :
                            self.walk.move('imu_fix')                        
                        time.sleep(1)
                        if self.image.center_deep_y <10:
                            self.turn_head()
                    
                    if 13 > self.image.deep_x > 4 : #deep_x = dx  #normal turn 右轉 範圍越大越容易旋轉 三個地方要調整 大的數字不動
                        # self.walk.straight_speed()                        
                        if ((abs(send.imu_value_Yaw) > 5) and (not self.imu_ok)) and (self.image.deep_x >= 6) :       #normal turn 右轉 數值越大 越不容易 修imu
                            while(abs(send.imu_value_Yaw) > 2):
                                self.walk.move('imu_fix')
                            self.walk.move('stay')                            
                            time.sleep(0.5)

                        else:
                            self.walk.move('dx_turn')

                        if abs(send.imu_value_Yaw) <= 5 :
                            self.imu_ok = True
                        
                    elif -3 > self.image.deep_x > -13 :     #normal turn 左轉 範圍越大越容易旋轉 三個地方要調整 大的數字不動
                        # self.walk.straight_speed()
                        
                        if ((abs(send.imu_value_Yaw) > 5) and (not self.imu_ok)) and (self.image.deep_x <= -3.5) :   #normal turn 左轉 數值越大 越不容易 修imu   
                            while(abs(send.imu_value_Yaw) > 2):
                                self.walk.move('imu_fix')
                            self.walk.move('stay')                            
                            time.sleep(0.5)                            
                        else:
                            self.walk.move('dx_turn')

                        if abs(send.imu_value_Yaw) <= 2 :
                            self.imu_ok = True
                        
                    elif (self.image.deep_x < 17 and self.image.deep_x >= 13) or (self.image.deep_x <= -13 and self.image.deep_x > -17) :                        

                        if (self.image.b_y_max >= 170) and ( abs(send.imu_value_Yaw) <= 5 ) and (self.need_imu_back) and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24) :        #離障礙物太近-->後退
                            while (self.image.b_y_max >= 170) and status.running:                                
                                self.walk.move('small_back')                                 
                                if self.image.b_y_max > 0 : #怕機器人的晃動會卡在small back的迴圈
                                    break
                            self.need_imu_back = False
                        if ( abs(send.imu_value_Yaw) > 2) and (not self.imu_ok) :                   #IMU修正
                            while ( abs(send.imu_value_Yaw) > 2) and (not self.imu_ok) and status.running:                                
                                self.walk.move('imu_fix')                                
                                if abs(send.imu_value_Yaw) < 2:        #轉頭策略
                                    if ( self.image.left_deep < 15 ) and ( self.image.right_deep < 15 ) and ( self.image.center_deep < 15 ): #左中右深度皆小於15
                                        self.turn_head()
                                        self.imu_ok = True
                                        break
                                    
                                    elif  self.image.deep_sum_l >= self.image.deep_sum_r and ( self.image.center_deep < 15 ) and (not SIMP_TURN_HEAD):
                                        while abs(send.imu_value_Yaw) < 39 and status.running and (not self.image.YY_2):    
                                            self.walk.move('turn_left_for_wall')
                                            
                                        self.imu_ok = True
                                    elif  self.image.deep_sum_l < self.image.deep_sum_r and ( self.image.center_deep < 15 ) and (not SIMP_TURN_HEAD):
                                        while abs(send.imu_value_Yaw) < 30 and status.running and (not self.image.YY_2):    
                                            self.walk.move('turn_right_for_wall')
                                            
                                        self.imu_ok = True

                                    elif SIMP_TURN_HEAD:
                                        self.simp_turn_head()


                                else:
                                    pass
                        elif (abs(send.imu_value_Yaw) < 2) and (self.imu_ok):
                            if ( self.image.left_deep < 15 ) and ( self.image.right_deep < 15 ) and ( self.image.center_deep < 15 ):
                                self.turn_head()
                                self.imu_ok = True
                            
                            elif  self.image.deep_sum_l >= self.image.deep_sum_r and ( self.image.center_deep < 15 ) and (not SIMP_TURN_HEAD):
                                while abs(send.imu_value_Yaw) < 30 and status.running and (not self.image.YY_2):    
                                    self.walk.move('turn_left_for_wall')
                                    
                                self.imu_ok = True
                            elif  self.image.deep_sum_l < self.image.deep_sum_r and ( self.image.center_deep < 15 ) and (not SIMP_TURN_HEAD):
                                while abs(send.imu_value_Yaw) < 30 and status.running and (not self.image.YY_2):    
                                    self.walk.move('turn_right_for_wall')
                                    
                                self.imu_ok = True

                            elif SIMP_TURN_HEAD:
                                self.simp_turn_head()
                    
                    elif (4 >= self.image.deep_x >= -3) or (20 > abs(self.image.deep_x) >= 17):                  #normal turn 直走 跟一般旋轉值要相等
                        if self.image.center_deep_y <13:
                            self.walk.move('small_forward')
                        else:
                            self.walk.move('max_speed') #dx一定小於等於16，但有可能測出17
                        # self.walk.move('max_speed') #dx一定小於等於16，但有可能測出17
                        self.image.calculate() 
                        if self.image.deep_x == 0 and self.image.center_deep_y >18:
                            self.imu_ok = False
                            self.need_imu_back = True 

                            
                elif self.image.deep_y == 24: #畫面完全沒有障礙物
                    self.image.calculate() 
                    status.obs_action = "無障"
                    self.walk.move('max_speed')
                    self.imu_ok = False
                    
        if not send.is_start :        
            if self.start_walking :
                self.walk.move('stop')
                send.sendContinuousValue(0,0,0,0,0) #x,y,z,theta填入walking介面移動數值
                send.sendBodyAuto(0,0,0,0,1,0) #mode=1為continue步態
                time.sleep(0.5)
                self.initt()

                self.start_walking = False
            
            status.run()



if __name__ == '__main__':

    try:
        aaaa = rospy.init_node('talker', anonymous=True)

        image = Normal_Obs_Parameter()
        walk = Walk(image)
        status = RobotStatus(image, walk)
        strategy = Obs(status)

        r = rospy.Rate(20)
        while not rospy.is_shutdown():            
            strategy.main()
            r.sleep()     
    except rospy.ROSInterruptException:
        pass