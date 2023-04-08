import cv2, socket, time
import mediapipe as mp
from config import *
from video_chat_utils import vedio_chat_utils

#############
### Setup ###
#############

# mediapipe
mp_face = mp.solutions.face_detection
mp_pose = mp.solutions.pose

# webcam
cap = cv2.VideoCapture(CAMERA_ID)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)

# # socket
# laptop = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# laptop.connect(ADDR)

# utils
utils = vedio_chat_utils()


############
### main ###
############

with mp_face.FaceDetection(
    model_selection=0, min_detection_confidence=0.5) as face, \
    mp_pose.Pose(
    min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # get results from face detection and pose
        results_face = face.process(image)
        results_pose = pose.process(image)

        # turn the image into writable and BGR mode
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # post processing and calculation
        if results_face.detections: image = utils.face_crop(image, results_face)
        if results_pose.pose_landmarks: left_arm_state, right_arm_state = utils.arm_calc(results_pose)
        
        image = utils.post_process(image)
        image = utils.fps_calc_draw(image)

        cv2.imshow('video chat', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
    cap.release()