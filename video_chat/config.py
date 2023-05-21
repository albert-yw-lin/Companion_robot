#!/usr/bin/env python3
import cv2

# socket
HEADER = 64
PORT = 9999
PORT_POSE = 5050
# SERVER = "192.168.0.205" # in lab
# SERVER = "192.168.0.11" # at home, ethernet
SERVER = "192.168.0.13" # at home, wifi
# SERVER = "127.0.0.1"
ADDR = (SERVER, PORT)
ADDR_POSE = (SERVER, PORT_POSE)

# face detection
SMOOTH = 0.2 # 1 for no smooth
FOREHEAD = 0.5 # porprotion with respect to bounding box
EXPAND = 0.1 # porprotion with respect to bounding box
VIDEO_HEIGHT = 480
VIDEO_WIDTH = 640
FRAME_RATE = 30

#fps calc
FREQUENCY = 30
FPS_COLOR = (255,255,255)
FPS_TEXT = cv2.FONT_HERSHEY_SIMPLEX
FPS_POSITION = (0,30)

# webcam
CAMERA_ID = 0

# socket
# BYTE_PER_TIME = 32768 #2e14
BYTE_PER_TIME = 4096

# image
JPEG_QUALITY = 20