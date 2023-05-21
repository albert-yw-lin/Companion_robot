#!/usr/bin/env python3
import cv2, socket, threading, time
import mediapipe as mp
import numpy as np
from utils import *
from config import *

class Laptop:

    def __init__(self) -> None:

        # mediapipe setup
        self.mp_face = mp.solutions.face_detection
        self.mp_pose = mp.solutions.pose

        # webcam setup
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FRAME_RATE) 

        # socket setup
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(ADDR)
        time.sleep(1)
        self.client_pose = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_pose.connect(ADDR_POSE)

        # face detection variable
        self.xmin_prev = 0
        self.ymin_prev = 0
        self.width_prev = 0
        self.height_prev = 0

        self.is_first_send_image = True
        self.is_first_detection = True

    def face_crop(self, image):
        with self.mp_face.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face:
            ### To improve performance, optionally mark the image as not writeable to
            ### pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = face.process(image)
            ### turn the image into writable and BGR mode
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.detections:
                ### only select the first face
                detection = results.detections[0]

                # ### example solution
                # mp_drawing.draw_detection(image, detection)

                box = detection.location_data.relative_bounding_box

                ### add smooth filter to both box location and box area
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

                ### box area adjustment
                forehead = FOREHEAD*box.height
                expand_width = EXPAND*box.width
                expand_height = EXPAND*box.height

                ### bounging box
                xmin = (box.xmin - expand_width)*VIDEO_WIDTH
                ymin = (box.ymin - expand_height - forehead)*VIDEO_HEIGHT
                xmax = (box.xmin + box.width + expand_width)*VIDEO_WIDTH
                ymax = (box.ymin + box.height + expand_height)*VIDEO_HEIGHT

                if xmin < 0: xmin = 0
                if ymin < 0: ymin = 0
                image = image[int(ymin):int(ymax), int(xmin):int(xmax)]
                
            ###post processing
            ### resize
            x_size = int(VIDEO_HEIGHT*(image.shape[1]/image.shape[0]))
            image = cv2.resize(image, (x_size,VIDEO_HEIGHT))
            ### padding
            pad = abs(VIDEO_WIDTH-x_size)
            if pad%2==0: pad_L = pad_R = pad//2
            else: pad_L = pad//2; pad_R = pad//2+1
            image = cv2.copyMakeBorder(image, 0,0,pad_L,pad_R, cv2.BORDER_CONSTANT, value=0)
            ### Flip the image horizontally for a selfie-view display.
            image = cv2.flip(image, 1)

            ### send socket
            if(self.is_first_send_image):
                self.is_first_send_image = False
            else: self.thread_send_image.join()
            self.thread_send_image = threading.Thread(target=send_image, args=(self.client, image))
            self.thread_send_image.start()
    
    def pose(self, image):
        with self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            ### To improve performance, optionally mark the image as not writeable to
            ### pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image)
            ### turn the image into writable and BGR mode
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.pose_landmarks:
                landmark = results.pose_landmarks.landmark

                ### left hand
                shoulder_L = np.array([landmark[11].x-landmark[12].x, landmark[11].y-landmark[12].y]).astype(float)
                arm_L = np.array([landmark[13].x-landmark[11].x, landmark[13].y-landmark[11].y]).astype(float)
                forearm_L = np.array([landmark[15].x-landmark[13].x, landmark[15].y-landmark[13].y]).astype(float)
                ### right hand
                shoulder_R = -shoulder_L
                arm_R = np.array([landmark[14].x-landmark[12].x, landmark[14].y-landmark[12].y]).astype(float)
                forearm_R = np.array([landmark[16].x-landmark[14].x, landmark[16].y-landmark[14].y]).astype(float)
                ### angle calculation
                shoulder_angle_L = (np.arctan2(shoulder_L[1], shoulder_L[0]) - np.arctan2(arm_L[1], arm_L[0]))*180/np.pi
                arm_angle_L = (np.arctan2(arm_L[1], arm_L[0]) - np.arctan2(forearm_L[1], forearm_L[0]))*180/np.pi
                shoulder_angle_R = (np.arctan2(shoulder_R[1], shoulder_R[0]) - np.arctan2(arm_R[1], arm_R[0]))*180/np.pi
                arm_angle_R = (np.arctan2(arm_R[1], arm_R[0]) - np.arctan2(forearm_R[1], forearm_R[0]))*180/np.pi
                # print("shoulder_L: ", shoulder_angle_L, "\tarm_L: ", arm_angle_L, "\n", "shoulder_R: ", shoulder_angle_R, "\tarm_R: ", arm_angle_R)

                self.arm_state = (shoulder_angle_L, arm_angle_L, shoulder_angle_R, arm_angle_R)

            ### send socket
            ### pose: can be a list or tuple contains FOUR floating points
            data = struct.pack('!4f', *self.arm_state)
            self.client_pose.sendall(data)

    def detection(self):
        while self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                ### If loading a video, use 'break' instead of 'continue'.
                continue

            ### start multi-thread
            if(self.is_first_detection):
                self.is_first_detection = False
            else: 
                self.thread_face_crop.join()
                self.thread_pose.join()

            self.thread_face_crop = threading.Thread(target=self.face_crop, args = (image,))
            self.thread_pose = threading.Thread(target=self.pose, args = (image,))
            self.thread_face_crop.start()
            self.thread_pose.start()



                    

############
### main ###
############
if __name__ == '__main__':
    try:
        ### setup
        laptop = Laptop()

        ### set another thread to recceive streaming
        thread_recv_image = threading.Thread(target=recv_image, args=(laptop.client,))
        thread_recv_image.start()

        ### send streaming
        thread_detection = threading.Thread(target=laptop.detection)
        thread_detection.start()

        ### wait till the receive thread to end
        thread_detection.join()
        thread_recv_image.join()
    
    except KeyboardInterrupt:
        print("KeyboardInterrupt.")
    
    finally:
        ### close cap
        laptop.cap.release()
        laptop.client.close()
        laptop.client_pose.close()
        cv2.destroyAllWindows()
        print("Closing the program ...")
