#!/usr/bin/env python3
import cv2
import numpy as np
from config import *

### general functions that are used in both latop and robot modules 
### (except gstreamer_pipeline only for pi camera on the robot)

def send_image(socket, image):
    try:
        encode_image = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])[1].tobytes()
        ### tell the server(robot) how much data should it receive
        socket.sendall(len(encode_image).to_bytes(4, byteorder='big'))
        while len(encode_image) > 0: # encode_image will varies in the while loop, so cannot use encode_image_length
            ### send BYTE_PER_TIME bytes of data per time to avoid bottleneck and better manage the flow of data
            chunk = encode_image[:SEND_BYTE_PER_TIME]
            socket.sendall(chunk)
            encode_image = encode_image[SEND_BYTE_PER_TIME:]
    except BrokenPipeError:
        pass

def recv_image(socket, is_shutdown):
    try:
        buffer = b''
        while not is_shutdown:
            data = socket.recv(RECV_BYTE_PER_TIME)

            ### close server and client simultaneously
            if (not data): break
            
            buffer += data
            encode_image_length = int.from_bytes(buffer[:4], byteorder='big')
            if len(buffer) > encode_image_length + 4:
                encode_image = buffer[4:encode_image_length+4]
                buffer = buffer[encode_image_length+4:]
                image = np.frombuffer(encode_image, dtype=np.uint8)
                image = cv2.imdecode(image, cv2.IMREAD_COLOR)
                cv2.imshow('recv_image', image)
                cv2.waitKey(1)
    except ConnectionResetError:
        pass
    except OSError:
        pass

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1640,
    capture_height=1232,
    display_width=640,
    display_height=480,
    framerate=10,
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