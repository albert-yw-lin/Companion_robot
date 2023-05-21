#!/usr/bin/env python3
from dynamixel.dynamixel import Dynamixel
import mediapipe as mp
import numpy as np
import cv2
from config import *

import rospy
from std_msgs.msg import Float64MultiArray, UInt8MultiArray

class head_arm:
    def __init__(self) -> None:
        self.motor_head  = Dynamixel(IDS_HEAD)
        self.motor_arm  = Dynamixel(IDS_ARM)
        self.pose = [0,0,0,0]
        self.face_center_x, self.face_center_y = (FACE_CENTER_X,FACE_CENTER_Y)
        self.motor_pos = None
        rospy.init_node('dynamixel', anonymous=True)
        rospy.Subscriber('face_center', Float64MultiArray, self.face_center_callback)
        rospy.Subscriber('pose', UInt8MultiArray, self.pose_callback)
        rate = rospy.Rate(24)

    def face_center_callback(self, face_center):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.face_center_x, self.face_center_y = face_center.data
        error_x = FACE_CENTER_X-self.face_center_x
        error_y = self.face_center_y-FACE_CENTER_Y
        add_motor_x = int(error_x*(62.2/360)*4095*P_GAIN_X)
        if(error_y>=0):add_motor_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_DOWN)
        else:add_motor_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_UP)
        self.motor_pos = self.motor_head.sync_read_pos()
        self.motor_pos[0] += add_motor_x
        self.motor_pos[1] += add_motor_y
        self.motor_head.sync_write_pos(self.motor_pos)

    def pose_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        pass

