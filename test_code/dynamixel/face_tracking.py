from dynamixel import Dynamixel
import mediapipe as mp
import numpy as np
import cv2
from config import *

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1640,
    capture_height=1232,
    display_width=640,
    display_height=480,
    framerate=30,
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

def face_position(results):
    # only select the first face
    detection = results.detections[0]
    box = detection.location_data.relative_bounding_box
    return (box.xmin+0.5*box.width, box.ymin+0.5*box.height)


motor = Dynamixel()
mp_face = mp.solutions.face_detection
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)



with mp_face.FaceDetection(
    model_selection=0, min_detection_confidence=0.5) as face:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            ### If loading a video, use 'break' instead of 'continue'.
            continue
        
        ########################
        ### image processing ###
        ########################

        ### To improve performance, optionally mark the image as not writeable to
        ### pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        ### get results from face detection and pose
        results = face.process(image)

        ### turn the image into writable and BGR mode
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        ### face position calculation
        if results.detections: face_center_x, face_center_y = face_position(results)
        error_x, error_y = FACE_CENTER_X-face_center_x, face_center_y-FACE_CENTER_Y
        add_motor_x = int(error_x*(62.2/360)*4095*P_GAIN_X), 
        if(error_y>=0):add_motor_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_DOWN)
        else:add_motor_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_UP)
        motor_pos = motor.sync_read_pos()
        motor_pos[0] += add_motor_x
        motor_pos[1] += add_motor_y
        motor.sync_write_pos(motor_pos)

        ### Flip the image horizontally for a selfie-view display.
        image = cv2.flip(image, 1)

        cv2.imshow("wqijfqejg", image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
motor.close()