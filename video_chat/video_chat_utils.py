import cv2, time
import mediapipe as mp
import numpy as np
from config import *

class vedio_chat_utils:
    def __init__(self) -> None:

        # face
        self.xmin_prev = 0
        self.ymin_prev = 0
        self.width_prev = 0
        self.height_prev = 0
        
        # fps calculation
        self.fps = 0
        self.time_prev = time.time()
        self.counter = 0

    def face_crop(self, image, results):
        # only select the first face
        detection = results.detections[0]

        # # example solution
        # mp_drawing.draw_detection(image, detection)

        box = detection.location_data.relative_bounding_box

        # add smooth filter to both box location and box area
        if 0 in (self.xmin_prev, self.ymin_prev, self.width_prev, self.height_prev):
            self.xmin_prev = box.xmin
            self.ymin_prev = box.ymin
            self.width_prev = box.width
            self.height_prev = box.height

        box.xmin = SMOOTH*box.xmin + (1-SMOOTH)*self.xmin_prev
        box.ymin = SMOOTH*box.ymin + (1-SMOOTH)*self.ymin_prev
        box.width = SMOOTH*box.width + (1-SMOOTH)*self.width_prev
        box.height = SMOOTH*box.height + (1-SMOOTH)*self.height_prev

        self.xmin_prev = box.xmin
        self.ymin_prev = box.ymin
        self.width_prev = box.width
        self.height_prev = box.height

        # box area adjustment
        forehead = FOREHEAD*box.height
        expand_width = EXPAND*box.width
        expand_height = EXPAND*box.height

        # bounging box
        xmin = (box.xmin - expand_width)*VIDEO_WIDTH
        ymin = (box.ymin - expand_height - forehead)*VIDEO_HEIGHT
        xmax = (box.xmin + box.width + expand_width)*VIDEO_WIDTH
        ymax = (box.ymin + box.height + expand_height)*VIDEO_HEIGHT

        if xmin < 0: xmin = 0
        if ymin < 0: ymin = 0
        image = image[int(ymin):int(ymax), int(xmin):int(xmax)]

        return image

    def arm_calc(self, results):
        landmark = results.pose_landmarks.landmark

        # left hand
        shoulder_L = np.array([landmark[11].x-landmark[12].x, landmark[11].y-landmark[12].y]).astype(float)
        arm_L = np.array([landmark[13].x-landmark[11].x, landmark[13].y-landmark[11].y]).astype(float)
        forearm_L = np.array([landmark[15].x-landmark[13].x, landmark[15].y-landmark[13].y]).astype(float)

        #right hand
        shoulder_R = -shoulder_L
        arm_R = np.array([landmark[14].x-landmark[12].x, landmark[14].y-landmark[12].y]).astype(float)
        forearm_R = np.array([landmark[16].x-landmark[14].x, landmark[16].y-landmark[14].y]).astype(float)

        # angle calculation
        shoulder_angle_L = (np.arctan2(shoulder_L[1], shoulder_L[0]) - np.arctan2(arm_L[1], arm_L[0]))*180/np.pi
        arm_angle_L = (np.arctan2(arm_L[1], arm_L[0]) - np.arctan2(forearm_L[1], forearm_L[0]))*180/np.pi
        shoulder_angle_R = (np.arctan2(shoulder_R[1], shoulder_R[0]) - np.arctan2(arm_R[1], arm_R[0]))*180/np.pi
        arm_angle_R = (np.arctan2(arm_R[1], arm_R[0]) - np.arctan2(forearm_R[1], forearm_R[0]))*180/np.pi
        # print("shoulder_L: ", shoulder_angle_L, "\tarm_L: ", arm_angle_L, "\n", "shoulder_R: ", shoulder_angle_R, "\tarm_R: ", arm_angle_R)

        left_arm_state = (shoulder_angle_L, arm_angle_L)
        right_arm_state = (shoulder_angle_R, arm_angle_R)
        return left_arm_state, right_arm_state

    def post_process(self, image):

        # resize
        x_size = int(VIDEO_HEIGHT*(image.shape[1]/image.shape[0]))
        image = cv2.resize(image, (x_size,VIDEO_HEIGHT))

        #padding
        pad = abs(VIDEO_WIDTH-x_size)
        if pad%2==0: pad_L = pad_R = pad//2
        else: pad_L = pad//2; pad_R = pad//2+1
        image = cv2.copyMakeBorder(image, 0,0,pad_L,pad_R, cv2.BORDER_CONSTANT, value=0)

        # Flip the image horizontally for a selfie-view display.
        image = cv2.flip(image, 1)

        return image
    
    def fps_calc_draw(self, image):
    
        if self.counter%FREQUENCY == 9:
            self.time_prev = time.time()
        if self.counter%FREQUENCY == 0: 
            self.counter = 0 # reset counter
            self.fps = int(1/(time.time() - self.time_prev))
        cv2.putText(image,"fps:%d"%self.fps,FPS_LOCATION,FPS_TEXT,1,FPS_COLOR,2)
        self.counter+=1

        return image
