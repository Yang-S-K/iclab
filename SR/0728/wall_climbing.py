#!/usr/bin/env python3
#coding=utf-8
import sys
import rospy
import numpy as np
import math
from Python_API import Sendmessage

#--校正量--#
#前進量校正
FORWARD_CORRECTION         = -300
#平移校正
TRANSLATION_CORRECTION     = 0
#旋轉校正
THETA_CORRECTION           = 0
#基礎變化量(前進&平移)
BASE_CHANGE                = 200      

# ---微調站姿開關---#
STAND_CORRECT_CW           = False                 #sector(33) CW_stand微調站姿
DRAW_FUNCTION_FLAG         = False                  #影像繪圖開關
TARGET_COLOAR              = 'Blue'                 #'Orange' 'Yellow' 'Blue' 'Green' 'Black' 'Red' 'White'    

#------------------#
HEAD_HORIZONTAL            = 2048                  #頭水平
HEAD_VERTICAL              = 1400                  #頭垂直 #down 2750

#判斷值
FOOTLADDER_LINE            = 215                   #上梯基準線

FIRST_FORWORD_CHANGE_LINE  = 120                    #小前進判斷線
SECOND_FORWORD_CHANGE_LINE = 130                    #前進判斷線
THIRD_FORWORD_CHANGE_LINE  = 150                   #大前進判斷線
UP_LADDER_DISTANCE         = 63                    #最低上板需求距離

#前後值
BACK_MIN                   = -300                  #小退後
FORWARD_MIN                = 300                  #小前進
FORWARD_NORMAL             = 800                  #前進
FORWARD_BIG                = 1000                  #大前進

#平移值
TRANSLATION_BIG            = 800                  #大平移
TRANSLATION_NORMAL         = 400
#旋轉值
THETA_MIN                  = 3                     #小旋轉
THETA_NORMAL               = 1                    #旋轉
THETA_BIG                  = 5                     #大旋轉

#左基礎參數
LEFT_THETA                 = 1
#右基礎參數
RIGHT_THETA                = -1

BLUE_MID = 260
BLUE_ERROR = 2


send       = Sendmessage()
class WallClimbing:
#CW主策略
    def __init__(self):
        # self.ladder = ObjectInfo(LADDER_COLOAR,'Ladder')

        color_dict = {  'Orange':  0,
                        'Yellow':  1,
                        'Blue'  :  2,
                        'Green' :  3,
                        'Black' :  4,
                        'Red'   :  5,
                        'White' :  6 }
        self.color = color_dict[TARGET_COLOAR]
        self.init()
        self.STAND_CORRECT_CW = STAND_CORRECT_CW
        self.blue_x_middle = 0
        
    def main(self,strategy):
        send.sendHeadMotor(1, self.head_Horizontal, 100)#水平
        send.sendHeadMotor(2, self.head_Vertical, 100)#垂直
        if DRAW_FUNCTION_FLAG:
            self.draw_function()
        sys.stdout.write("\033[J")
        sys.stdout.write("\033[H")
        rospy.loginfo('________________________________________\033[K')
        rospy.loginfo(f'x: {self.now_forward} ,y: {self.now_translation} ,theta: {self.now_theta}\033[K')
        rospy.loginfo(f'Goal_x: {self.forward} ,Goal_y: {self.translation} ,Goal_theta: {self.theta}\033[K')
        rospy.loginfo(f"機器人狀態: {self.state}\033[K")
        rospy.loginfo(f"距離梯: {(FOOTLADDER_LINE - self.lower_blue_ymax)}\033[K")
        rospy.loginfo(f"藍色中心: { (send.color_mask_subject_XMin[self.color][0]+send.color_mask_subject_XMax[self.color][0])/2}\033[K")
        rospy.loginfo(f"blue_x_middle: { self.blue_x_middle}\033[K")
        rospy.loginfo('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')
        
        if strategy == "Wall_Climb_off":
        #關閉策略,初始化設定
            if not self.walkinggait_stop:
                rospy.loginfo("🔊CW parameter reset\033[K")
                send.sendHeadMotor(1,self.head_Horizontal,100)  #水平
                send.sendHeadMotor(2,self.head_Vertical,100)    #垂直
                send.sendBodyAuto(0,0,0,0,1,0)
                send.sendSensorReset(1,1,1)              #IMUreset
                rospy.sleep(2)
                send.sendBodySector(29)             #基礎站姿磁區
                rospy.sleep(1.5)
                # if STAND_CORRECT_CW:
                #     send.sendBodySector(30)             #CW基礎站姿調整磁區
                #     STAND_CORRECT_CW = False 
                rospy.loginfo("reset🆗🆗🆗\033[K")
                self.imu_reset = True
            self.init()
            self.find_ladder()
            rospy.loginfo("turn off\033[K")
            self.walkinggait_stop = True

        elif strategy == "Wall_Climb_on":
            send.sendWalkParameter('send',\
                        walk_mode = 1,\
                        com_y_shift = -1.5,\
                        y_swing = 4.5,\
                        period_t = 270,\
                        t_dsp = 0.1,\
                        base_default_z = 1.2,\
                        com_height = 29.5,\
                        stand_height = 23.5)
        #開啟CW策略 
            if self.state != 'cw_finish':
                if self.STAND_CORRECT_CW:
                    send.sendBodySector(102)             #CW基礎站姿調整磁區
                    while not send.execute:
                        rospy.logdebug("站立姿勢\033[K")
                    send.execute = False
                    self.STAND_CORRECT_CW = False
                    rospy.sleep(2)
                if self.imu_reset:

                    send.sendSensorReset(1,1,1)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    self.imu_reset = False

                rospy.loginfo(f"blue ymax: {self.lower_blue_ymax}\033[K")
                self.find_ladder()
                self.walkinggait(motion=self.edge_judge(strategy))
            self.walkinggait_stop = False
                    
    def init(self):
        #狀態
        self.state                 = '停止'
        
        #步態啟動旗標
        self.walkinggait_stop      = True  
        
        #設定頭部馬達
        self.head_Horizontal       = HEAD_HORIZONTAL
        self.head_Vertical         = HEAD_VERTICAL
        
        #imu_reast
        self.imu_reset             = True

        #步態參數
        self.forward               = FORWARD_NORMAL + FORWARD_CORRECTION
        self.translation           = 0              + TRANSLATION_CORRECTION
        self.theta                 = 0              + THETA_CORRECTION
        self.now_forward           = 0 
        self.now_translation       = 0
        self.now_theta             = 0

        self.new_target_xmin       = 0
        self.new_target_xmax       = 0  
        self.lower_blue_ymax       = 0

    def find_ladder(self):
    #獲取梯子資訊、距離資訊
        self.lower_blue_ymax      = 0
        self.new_target_xmax = 0
        self.new_target_xmin = 0
        self.new_target_ymax = 0
        self.blue_x_middle = 160
        rospy.loginfo(f"blue mask subject cnts: {send.color_mask_subject_cnts[self.color]}\033[K")
        sys.stdout.write("\033[K")
        #-------距離判斷-------#
        for blue_cnt in range (send.color_mask_subject_cnts[self.color]):
            
            if send.color_mask_subject_size[self.color][blue_cnt] > 10:
                self.new_target_xmax = send.color_mask_subject_XMax[self.color][blue_cnt]
                self.new_target_xmin = send.color_mask_subject_XMin[self.color][blue_cnt]
                self.new_target_ymax = send.color_mask_subject_YMax[self.color][blue_cnt]
                
                if self.lower_blue_ymax < self.new_target_ymax:
                    self.lower_blue_ymax = self.new_target_ymax
                    self.blue_x_middle = (self.new_target_xmax + self.new_target_xmin) / 2
                    rospy.logwarn(f"lower blue ymax: {self.lower_blue_ymax}\033[K")
    
    def walkinggait(self,motion):
    #步態函數
        if motion == 'ready_to_cw':
            rospy.loginfo("對正梯子\033[K")
            send.sendBodyAuto(0,0,0,0,1,0)           #停止步態
            send.sendSensorReset(1,1,1)                   #IMU reset 避免機器人步態修正錯誤
            rospy.sleep(3)                           #穩定停止後的搖晃
            send.sendBodySector(29)                  #這是基本站姿的磁區
            while not send.execute:
                rospy.logdebug("站立姿勢\033[K")
            send.execute = False
            rospy.sleep(3) 
            #-爬梯磁區-#
            send.sendBodySector(871)         
            rospy.sleep(10)
            send.sendBodySector(872)
            rospy.sleep(12)
            #---------#
            self.status = 'cw_finish'

        else:
            #前進變化量
            if self.now_forward > self.forward:
                self.now_forward -= BASE_CHANGE
            elif self.now_forward < self.forward:
                self.now_forward += BASE_CHANGE
            else:
                self.now_forward = self.forward

            #平移變化量
            if self.now_translation > self.translation:
                self.now_translation -= BASE_CHANGE
            elif self.now_translation < self.translation:
                self.now_translation += BASE_CHANGE
            else:
                self.now_translation = self.translation

            #旋轉變化量
            # if send.imu_value_Yaw > 1:
            #     self.now_theta = -THETA_NORMAL
            # elif send.imu_value_Yaw < -1:
            #     self.now_theta = THETA_NORMAL
            # else:
            #     self.now_theta = 0
            self.now_theta = self.imu_angle()
            #速度調整
            send.sendContinuousValue(self.now_forward, self.now_translation, 0, self.now_theta, 0)

    def edge_judge(self,strategy):
    #邊緣判斷,回傳機器人走路速度與走路模式
        if (self.lower_blue_ymax >= FOOTLADDER_LINE - UP_LADDER_DISTANCE) and (self.blue_x_middle >= BLUE_MID-BLUE_ERROR) and (self.blue_x_middle <= BLUE_MID+BLUE_ERROR) and abs(send.imu_value_Yaw) < 1.2:
            self.state = "爬梯"
            return "ready_to_cw"
        
        else:
            if (self.lower_blue_ymax > FOOTLADDER_LINE):
                self.theta       = send.imu_value_Yaw/4
                self.forward     = BACK_MIN + FORWARD_CORRECTION
                self.state       = "!!!小心採到梯子,後退!!!"

            elif (self.lower_blue_ymax >= FOOTLADDER_LINE - UP_LADDER_DISTANCE) and (self.blue_x_middle < BLUE_MID):
                self.forward     = BACK_MIN+ FORWARD_CORRECTION
                self.theta       =  0
                self.translation = LEFT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION +100
                self.state       = "左平移"

            elif (self.lower_blue_ymax >= FOOTLADDER_LINE - UP_LADDER_DISTANCE) and (self.blue_x_middle > BLUE_MID):
                self.forward     = BACK_MIN+ FORWARD_CORRECTION
                self.theta       =  0
                self.translation = RIGHT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                self.state       = "右平移"
            
            else:
                if (FOOTLADDER_LINE - self.lower_blue_ymax) < FIRST_FORWORD_CHANGE_LINE:
                    if self.blue_x_middle < BLUE_MID-BLUE_ERROR: #左移
                        self.translation = LEFT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                        self.state       = "左平移  "
                    
                    elif self.blue_x_middle > BLUE_MID+BLUE_ERROR: #右移
                        self.translation = RIGHT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                        self.state       = "右平移  "
                    else:
                        self.translation = TRANSLATION_CORRECTION
                elif (FOOTLADDER_LINE - self.lower_blue_ymax) < SECOND_FORWORD_CHANGE_LINE:
                    if self.blue_x_middle < BLUE_MID-BLUE_ERROR: #左移
                        self.translation = LEFT_THETA * TRANSLATION_BIG+ TRANSLATION_CORRECTION
                        self.state       = "大左平移  "
                    
                    elif self.blue_x_middle > BLUE_MID+BLUE_ERROR: #右移
                        self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                        self.state       = "大右平移  "
                    else:
                        self.translation = TRANSLATION_CORRECTION
                

                
                if (FOOTLADDER_LINE - self.lower_blue_ymax) < FIRST_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_MIN + FORWARD_CORRECTION
                    self.state      = '小前進'
                elif (FOOTLADDER_LINE - self.lower_blue_ymax) < SECOND_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.state      += '前進'
                elif (FOOTLADDER_LINE - self.lower_blue_ymax) < THIRD_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                    self.state      = '大前進'
                
                else:
                    self.theta      = THETA_CORRECTION
                    self.forward    = FORWARD_BIG + FORWARD_CORRECTION
                    self.state     = 'no'
            return 'walking'

    def draw_function(self):
    #畫面顯示繪畫資訊    
        send.drawImageFunction(1, 0, 0, 320, FOOTLADDER_LINE, FOOTLADDER_LINE, 255, 0, 0)   #基準線
        send.drawImageFunction(2, 0, self.blue_x_middle, self.blue_x_middle, 0,240, 255, 255, 0) #物件中心
    
    def imu_angle(self):      #一般 imu修正角度
        imu_ranges = [  (180,  -3),
                        (90,  -3), 
                        (60,  -3), 
                        (45,  -3), 
                        (20,  -3), 
                        (10,  -2), 
                        (5,   -2), 
                        (2,   -1), 
                        (0,    0),
                        (-2,    1),
                        (-5,    2),
                        (-10,   2),
                        (-20,   3),
                        (-45,   3),
                        (-60,   3),
                        (-90,   3),
                        (-180,   3)]
        for imu_range in imu_ranges:           
            if self.imu_yaw >= imu_range[0]:
                return imu_range[1]
        return 0

# ░░░░░░░░░▄░░░░░░░░░░░░░░▄░░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀████▀▄▄⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▄█ 
# ░░░░░░░░▌▒█░░░░░░░░░░░▄▀▒▌░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂█▀░░░░▀▀▄▄▄▄▄⠂⠂⠂⠂▄▄▀▀█⠂⠂
# ░░░░░░░░▌▒▒█░░░░░░░░▄▀▒▒▒▐░░░░░░   ⠂⠂⠂▄⠂⠂⠂⠂⠂⠂⠂█░░░░░░░░░░░▀▀▀▀▄░░▄▀ ⠂⠂
# ░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐░░░░░░  ⠂▄▀░▀▄⠂⠂⠂⠂⠂⠂▀▄░░░░░░░░░░░░░░▀▄▀⠂⠂⠂⠂⠂
# ░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░░░░░░   ▄▀░░░░█⠂⠂⠂⠂⠂⠂█▀░░░▄█▀▄░░░░░░▄█⠂⠂⠂⠂⠂
# ░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░░░░░░   ▀▄░░░░░▀▄⠂⠂⠂█░░░░░▀██▀░░░░░██▄█ ⠂⠂⠂⠂
# ░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌░░░░░  ⠂⠂▀▄░░░░▄▀⠂█░░░▄██▄░░░▄░░▄░░▀▀░█ ⠂⠂⠂⠂
# ░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐░░░░░  ⠂⠂⠂█░░▄▀⠂⠂█░░░░▀██▀░░░░▀▀░▀▀░░▄▀⠂⠂⠂⠂
# ░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌░░░░  ⠂⠂█░░░█⠂⠂█░░░░░░▄▄░░░░░░░░░░░▄▀⠂⠂⠂⠂⠂
# ░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌░░░░  ⠂█░░░█⠂⠂█▄▄░░░░░░░▀▀▄░░░░░░▄░█ ⠂⠂⠂⠂⠂
# ▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐░░░░  ⠂⠂▀▄░▄█▄█▀██▄░░▄▄░░░▄▀░░▄▀▀░░░█ ⠂⠂⠂⠂⠂
# ▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌░░░  ⠂⠂⠂⠂▀███░░░░░░░░░▀▀▀░░░░▀▄░░░▄▀⠂⠂⠂⠂⠂
# ▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐░░░░  ⠂⠂⠂⠂⠂⠂▀▀█░░░░░░░░░▄░░░░░░▄▀█▀ ⠂⠂⠂⠂⠂
# ░▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀█░░░░░▄▄▄▀░░▄▄▀▀░▄▀ ⠂⠂⠂⠂⠂
# ░▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▀▀▄▄▄▄▀⠂▀▀▀⠂▀▀▄▄▄▀⠂⠂⠂⠂⠂
# ░░▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌░░░░░
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++==--::::--==+++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++=-...........:-=+++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++=:..............:=++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++-......:--:......-++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++=......:++++:.....:++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++===----=++=-......-++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++++-:......:=++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++=:......:-++++++++++++++++++++++++++++++++++++++++++
# +++++++++*%@@%*++++++++++++++++++++++++++++++++=.......-=+++++++++++++++++++++++++++++++++++++++++++
# ++++++++*@#+**%@#++++++++++++++++++++++++++++++:.....:=+++++++++++++++++++++++++++++++++++++++++++++
# ++++++++@*+==++*#@%*+++++++++++++++++++++++++++::::::-++++++++++++++++++++++++++++++++++++++++++++++
# +++++++#%------+++#@%++++++++++++++++++++++++++=======++++++++++++++++++++++++++++++++++++++++++++++
# +++++++@*-------=+++#@#++++++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# +++++++@=---------+++*%@#++++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# ++++++*@-----------=+++*%%*++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# ++++++*@--------=*%%#*+++*%%*++++++++++++++++++------=+++++++++++++++++++++++++++++++**#%#++++++++++
# ++++++*@--+%%##%%*-:=#@#*++#@%++++++++++++++++++++++++++++++++++++++++++++++++++++*%@%#**@*+++++++++
# +++++++@+**+=-.  .:-.  -+*++*#@%+++++++++++++++++++++++++++++++++++++++++++++++*%@%*+++++%%+++++++++
# +++++++@%+----:.          .:+++#%#++*##########*++++++++++++++++++++++++++++*#@%#+++++===%%+++++++++
# +++++++-..                 .+++++*#####*****####%%@%##*++++++++++++++++++*#%%#*++++=-----%#+++++++++
# +++++=*%%*=.      .-=+***++++++++++++++++++++++++++**#%%%%*+++++++++++*#@%#*+++==-------=@++++++++++
# ++++#@+.      -+#%%#*+++++++++++++++++++++++++++++++++++**#%%#*++++*%@%#*+++=-----------%%++++++++++
# +++%%==-  .=#%%#*+++++++++++++++++++++++++++++++++++++++++++**#%%%%#*++++++****+=------+@+++++++++++
# +++##%=.=#@#*++++++++++++++++++++++++++++++++++++++++++++++++++*****+--=++++======----=@#+++++++++++
# +++@#-*%#*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=:.      :--:.:--%%++++=+++++++
# +++@@%#*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-.    .-*%%**#+=+++=+++++++
# +#@%*+++++++++++++++-:-++++++++++++++++++++++++++++++++++++++++++++++++++++=:      :+%%*+-+++-++++++
# %%*++++++++++++++=: .=++++++++++++++*%*+++++++++++++++++++++++++++++++++++++++=.   .  -%#+-++*-***++
# *++++++++++++++-.  -**++++++++++++*#@@*+++++++++++++++++++++++++++++++++++++++**=  .+#-:#++=*++=***+
# +++++++++++++:   .*#+++++++++++++#@*+@++++++++++++++++++++++++++++++++++++++++++##:  -%#+++-+++=+**+
# +++++++++++-    -%*++++++++++++#@*: +%++++++++++++++++++++++++*%*++++++++++++++++*%+  =%@++=:*=***+*
# ++++++++++:    *@*++++++++++*#@*:   +@++++++++++++++++++++++++#@%+++++++++++++++++*%#.=@%*+-+++=*+++
# +++++++++:   .#%*++++++++*#%%+.     -@*+++++++++++++++++++++++%*%%++++++++++++++++++%%.*%+*-*+++*+++
# ++++++++:   .%%+++++++*#%%*-         #%++++++++++++++++++++++*@::@#+++++++**+++++++++%%:%#*--+++++++
# +++++++:   .#%++++*##%%+-.           :@#++*++++++++++++++++++%+  -@#++++++*%#+++++++++%%*%+++++*++++
# ++++++-    #%*##%@@%%*===--..         =@#+*+++++++++++++++++#%.   -@#++++++*@#+++++++++%@*++++++++++
# +++++=    +@##@@@@@@@@@@@@@@@%*=-.     =@#*#+++++++++++++++*@-     -@#++++++#@*++++++++*%%++++++++++
# +++++.   -@-.++#%#*+======+*#%@@@@%+-.  -%%*#++**+++++++++*@=       :%%*+++++#@*++++++++*@#+++++++++
# ++++-   .%*       :=*#%%%%%%%#+==+#@@@*- .+%*=-#%@@%####*#@+         .+@#*++++%%+++++++++#@+++++++++
# ++++.   +@.    :*%%#*+=======+*#@*- :=+*=.  :.   ..=+++*##+==++****+==::*@%#*+*@#+++++++++%%++++++++
# +++-   .%*   .*@*+===============*%#:                   =@@@@@@@@@@@@@@@%#%@@%#%@+++++++++*@#+++++++
# +++:   =@:  .%%+===================*%=                  .:::::::--=+=+*#@@@@@%#*@#+++++++++#@+++++++
# +++    #%   +@+=====================+%=                   .:+#@@%%%%%%*=:.-=*%@@@%+++++++++*@#++++++
# ++=    @+   +@+======================#%:                :*@%#*+=======+*%%*:  :=#@#+++++++++#%++++++
# ++-   .@-   .%#======================*@:               =@#+==============+*%#:  .@%*++++++++*@*+++++
# ++:   =@:    :@*=====================%%.              =@#===================*@-  #%++++++++++%#+++++
# ++:   =@.     -%#+=================+%%:              .@%=====================#@. +%++++++++++%%+++++
# ++-   =%       .+%#*=============+#@+.               :@#======================@= +%++++++++++#@+++++
# ++=   +%         .-*%%##*+++**#%@#=.                  %@+=====================@= +%++++++++++#@+++++
# **+   +%             .:=++***+=-.                     -%%====================*@: +%++++++++++*@+++++
# **+:  =@.                                              .*%*+===============+#@=  +%++++++++++#@+++++
# +%*=  :@:                                                :+%%#++========+*%@*:   *%++++++++++#@+++++
# +*@*-  %+                                 .-*#+             :=*%%%%##%%@%*-.     %#++++++++++%%+++++
# ++#%*. *@#=.                            =#@#=-%#:                .:-::..        .@#++++++**+*@*+++++
# +++%@+ .@%%@#=:                         .:.   .+%#.                             =@*++++++#%+#@++++++
# %*++#@+.=@**%@@%+-.                             .:.                             #%++++++*@#*@#++++++
# #%@%##%%+*%%+*%##%@%+-:                                                        :@#++++++%@+%%+++++++
# ++*##%##%@%+:%+#%**##@@%#*=-.                                                 :*%++++++#@*#@++++++++
# ++++++*%#.:--@:.#@#**#@#:-=*#%#*+=-:.                                  .:-=+#%@@*+++++#@#*@@*+++++++
# +++++*@*.   =@.  +@%***%@=    .:-=+*#%%#**+===--::::::::::---===++**#%%%%%##**%%+++++#@#*%*%%+++++++
# ++++*@+     +#    :%%#**#@%-           .:%#=+@#****#####**#@@@%@%+%#*++++++++#%*+++*%%*+*++#@+++++++
# +++*@#      #*     .%@#***%@+            -@*#@.         -#@@%#*#@:=@#++++++++%*+++*@%++++++#@+++++++                                                                  
#                                                           ...                                                                      
#      =+=:        .-==.                              .-=++++++++++++++=+=-:                                                          
#     -=.:=++    ++=-:-+                           :=++==++***++*+++++*++++*#*-:                                                      
#     #.::-.#    #.-::.#.                       .=+==+*+++***++++++++##++++*+*%##+:                                                   
#    :+::::+-    -=::::=-                   =+:++-+*++++++*=#+++++++#=#+++*##*****##+..                                               
#    *::::.#     .#.-:-:*                   -#*+++++******-+#++****#-=#++##**#*******#**:                                             
#    #.:::==      +:-:-.#.                 .*+=+****=----+=+*#+=---=-+####*##**#**#**#=+-                                             
#   .*.:::+*+-    -=:--:+:        .%*=-.  -%=*#-=+=-=+*####*+--+*****##***#*##*****#**+*-                                             
#   :*:--.=.:=*=  .*:--:==         :*=*#*#+-=*++==+#*+====--=*#==++**+#****##**#****#**-#.                                            
#   :*:--:.---.#. .#.---:+          :#-=#++*+-=-**++**+*-=#*++**++-**##*#***#*#*#****#*-%%.                                           
#   :*:--:---:==   #.---:*           =*--==-=+**+++++++*=+*+++++*#-#++#***#**#*********=++#-:                                         
#   .#.------:*    #.---.#           +#+---+-#*+++++++++#*++++++*-**++*###*****#***#***==-==*+                                        
#    *:-----:#.    =+===++          +#+*%#**+#+-+++++++++*++++++*-==*+#***##*#*****#**++..:-:-*:                                      
#    :*:---:*:      ....         :+##++%*+++%+*-*=  =++++++++++++=  -**:##***#*******=.::..:.--*-                                     
#     -*--=*:                 .+%#+=+*##=+++*+*-*.  :+:=+++=++++++  ..#-+*#****++++-*::.::.::::++                                     
#      .:-:++++=-:.            .-=**%-#=+++*++#=*   == -++*.-+++++ ....==+*####%**+== ::... :-:=#.                                    
#         *--:::-==++=              +=*-+++*+**=#::-*+++++++++*+++     --*++++++#==--:.:...:.-:+=#                                    
#        ==:-----==-.#.             #*+=+++#+*==#:*+*+++++++++*+++    .--**+++++*=+--:::.....==+#=                                    
#        .+++=----=:+-             .#+=++++#+*-=*:++*+++++++++*++=    ..-+#++++#==+=-=:::---=-*-                                      
#     .=+++=+==--:::-*-        .    #=-+++**+=:-*-=++*++++++++*++-     .=++*++++#*====*+==---##+                                      
#     +-::--=========.#     .=#+#*. =#-+++*++:.=+=-++#++++++++*+*:.   . ====++++**#*#=+++=++#*+*%:                                    
#     +:=============-+    .#-=.--+  %-+==++*--+*+=+=**====+++#+*==----:==+ *+++++*+*###%%##*#%=-.                                    
#     *:-:--=.:====-=+.   .%= - ::%+.%-+===+*. .*-=-==#====++=*=+       ==+.*+++++++++**%#****%.                                      
#     :*+*%=-=====:-+=    .#:.: -.==+#-+===++:::-*--*=*+====+=+*=::::-: ===:*+*++*++++**%*##**%                                       
#      :+=-=========.#.-=-.%..  .:.-+*-===###****%*#++=*====+%#%#*******#=-.*+++++++++*##*#*%%-                                       
#    .++-=====--:===:==-:+:%. :::-.:*+=-+-*-**=:::*=:+=++=-=-=*+#-::--#-++: ++++*++++**%**#+%-                                        
#    #:=====-++#:+===+++-*:+=  .:. .%+*-*=* +*.   =-. .=.:=++=+**     #.*+. =+++***++*##***+%                                         
#    -*-==-++. *-=+++++=:#.:%.  .-++=#%:=#=::*:   +:        .::-*     % ++  =+*+-::+**%***++%                                         
#     .++=*:  .:+*====+++=  ++-==.**=*@==##+ :*   #       .   . -+   =+:+-  =*-.--. #%#**+++%                                         
#       ..   -*==++...      +#- .=#%#=*%-##-..-+++:  .        .  -++*+.=*.. =-.=- . **#*++++%                                         
#            +:++:#       .*= .+%#=-%-.#***.....    .         .   .....*-   . -:--  #*#+++++%                                         
#            *.++.*      .#: -##-   :%:.#*=.....  .               ....-= .  . =--: +#**+++++%                                         
#            #:++.*      +-:+%=      -%:.%+         .              . .-   . . .= .*%*+*+++++%                                 .       
#           .*-++:*      .#+%:        -%--@-       ..+=====++             .=*=#**#*%*+++++++%                                 .       
#           :+=+*:*       :@-:         =%%##*-      .--:::--=           :==::-=#**+%*+++++++%                                         
#           -=++*:*        ==::         *%#**##+-.           ..     :=+++::-::=*++*#*++++*++%                                         
#           =-++*:*         +-:.       :####*##*##%*++==----====++#+==::-----:*=++##+++++*++%                                         
#           +-*+*:*        =.*-:.      -*%%**#**#*%*#*---=+#=-:--=+**--------:*+++##+++++*++%.                                        
#           +:*+*.*        *  *::.      -#%##%*+*+%*#=+*++*****+*++*----------++++*#+*+++*++%:                                        
#           *:***.*       :=  .#::.      =#%##*+*+#*#:**:.+=+=-*:=*--=++-=-+-+-++#+*+*++*#*+#-                                        
#           *:***:*       :+   .#::.      +%##+++*#*#=*=:.-*+%-=-....  .=%#%+#-++%+*+*++#**+*+                                        
#           *---==+       -+    .*:::    .+#%%=++#*#*##. ..++: ......    :#**#-++%##**++#**++#                                        
#       =++-.-=--:  :-==. :*     .*-::. :=**-*=++%*#=*+ . =.+ . ... . . . =%%%-++#*#%+++#*#++%:                                       
#      :*-==+*.   =+===:* .#  -.   ++::. +.= ===*-*+*#-.  :=.  .-:-::- . .:%%%++=#*##++*%*#+=**                                       
#      *:***-*.   +:***=+: #  =.    :*-::.*.+=+=* :*++***+===+++++++#=.   .%%%%+++%#%==##%#+==%.                                      
#     .*-***-+    :*=***-+ +: =:      #+.::+--=++.#++++++++++++++++++**=.. %%%%%%%%%#==#***##*+*                                      
#     =-***-*.     *:#*#:# :+ --      *+=:::+==#-#*++++++++++++++++++%*+*=.#%%%%%%%%+==%****%*=#:                                     
#     *:***:+      ==***=+: +..*      #-%::--===#%+++++++++++++++++++%:=***#*%%%%%%%===%%%#**##=#                                     
#     #:#**=*+=.   .*=***-= .= *.    .#:*+.=--+*:%*+++++++*+++++++++*%- .-+*%%%%%%%%===%%%%#**#**-                                    
#    .#-#*+--=-++.  #-#*#:*    .+    .*:*::=--#*+-%*++++++#+++++++++#@=  -*=%%%#%%%@=-=%%#*****#=#                                    
#    .#-***.**#=+:  #:#*#:#     -:   :+-+ ----%:=:*%##++++*#++++***#%%+:*#####****#%=-=%%#****#%+#:                                   
#     #-#******-+   *:###-#.     .   -=*: =:-=* :*-%#%%***+*#*%##%**#%##***********#=-=#****##%%==+                                   
#     #:#*#*##:#.   +:###=*.         =-#  =--+: ==:%++####**#***++*##**************#+--%****#%%%=-#                                   
#     =-*#*##-*:    -*==++*          **-  =-++ :-.##*+**###*+++++*+*%***************#--%***#%%%@-:#.                                  
#     .#-*##-*-       ...           -#*#.:=-*+ =  *##**++++++++++*#%#***************%--#***##%%#+-*-                                  
#      :*===*-                      *+####=-#=:- .%**+++++++++*#%#*#%***************#=:+#**%%++*#-+=                                  
#        :-:                       -*=%=..+-%=*  +#++++++*####*+++*%%#%**#***********#:=*#%**++*%-==                                  
#                                  #-##:  *-#-+ .%########+*####***%%%%####**********#=-=+:-*++#%=+=                                  
#                                 +##*    *:+++ +#+++++++%#+++++***%%%##**###++%#*%%+#*:-+::#+=%#=+-                                  
#                                 .#=.    =-+++.#+++++=+*+*#+++++++###**#*=.. ..+:**++-+::*.#=+++-*.