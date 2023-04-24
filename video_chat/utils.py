import time
import cv2
import socket
import numpy as np
from config import *

class Fps:
    def __init__(self):
        self.time_prev = 0
        self.counter = 0
        self.fps = 0
    def calc_draw_fps(self, image):
        self.counter+=1
        if self.counter%FREQUENCY == 9:
            self.time_prev = time.time()
        if self.counter%FREQUENCY == 0: 
            self.counter = 0 # reset counter
            self.fps = int(1/(time.time() - self.time_prev))
        cv2.putText(image,"FPS:%d"%self.fps,FPS_POSITION,FPS_TEXT,1,FPS_COLOR,2)
        return image

class Smooth_data():
    def  __init__(self, data) -> None:
        # simple exponential smoothing
        pass

def send_image(socket, image):
    encode_image = cv2.imencode('.jpg', image)[1].tobytes()
    ### tell the server(robot) how much data should it receive
    socket.sendall(len(encode_image).to_bytes(4, byteorder='big'))
    while len(encode_image) > 0: # encode_image will varies in the while loop, so cannot use encode_image_length
        ### send BYTE_PER_TIME bytes of data per time to avoid bottleneck and better manage the flow of data
        chunk = encode_image[:BYTE_PER_TIME]
        socket.sendall(chunk)
        encode_image = encode_image[BYTE_PER_TIME:]

def recv_image(socket,):
    buffer = b''
    while True:
        data = socket.recv(BYTE_PER_TIME)

        ### close server and client simultaneously
        if not data: break
        
        buffer += data
        encode_image_length = int.from_bytes(buffer[:4], byteorder='big')
        if len(buffer) > encode_image_length + 4:
            encode_image = buffer[4:encode_image_length+4]
            buffer = buffer[encode_image_length+4:]
            image = np.frombuffer(encode_image, dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            cv2.imshow('recv_image', image)
            cv2.waitKey(1)

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=3264,
    capture_height=2464,
    display_width=640,
    display_height=480,
    framerate=21,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )