#!/usr/bin/env python3
import cv2

### socket
HEADER = 64
PORT = 9999
PORT_POSE = 5000
# SERVER = "192.168.0.205" # in lab
# SERVER = "192.168.0.11" # at home, ethernet
SERVER = "192.168.0.13" # at home, wifi
# SERVER = "127.0.0.1"
ADDR = (SERVER, PORT)
ADDR_POSE = (SERVER, PORT_POSE)

### face detection
SMOOTH = 0.2 # 1 for no smooth
FOREHEAD = 0.5 # porprotion with respect to bounding box
EXPAND = 0.15 # porprotion with respect to bounding box
VIDEO_HEIGHT = 480
VIDEO_WIDTH = 640
FRAME_RATE = 30

### webcam
CAMERA_ID = -1

### socket speed
SEND_BYTE_PER_TIME = 32768 #2e14
# SEND_BYTE_PER_TIME = 4096
RECV_BYTE_PER_TIME = 1024

### image
JPEG_QUALITY = 20

### pose 
POSE_CONF = 0.8
SHOULDER_MAX = 90
SHOULDER_MIN = 0
ARM_MAX = 180
ARM_MIN = 0

### dynamiel motor
ID_HEAD_X = 0
ID_HEAD_Y = 1
ID_SHOULDER_L = 4
ID_ARM_L = 5
ID_SHOULDER_R = 2
ID_ARM_R = 3

### motor face tracking
FACE_CENTER_X = 0.5
FACE_CENTER_Y = 0.6
P_GAIN_X = 0.25
P_GAIN_Y_UP = 0.1
P_GAIN_Y_DOWN = 0.6


### turn base
TURN_LEFT = 0
TURN_RIGHT = 1
TURN_STOP = 2
TURN_THRESHOLD = 10