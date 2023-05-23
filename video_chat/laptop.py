#!/usr/bin/env python3
import cv2, socket, threading, time, math, struct
import mediapipe as mp
import numpy as np
from utils import *
from config import *

class Laptop:

    def __init__(self) -> None:

        # mediapipe setup
        self.mp_face = mp.solutions.face_detection
        self.mp_pose = mp.solutions.pose
        self.shoulder_angle_L = 0
        self.arm_angle_L = 0
        self.shoulder_angle_R = 0
        self.arm_angle_R = 0
        self.arm_state = (self.shoulder_angle_L, self.arm_angle_L, self.shoulder_angle_R, self.arm_angle_R)

        # webcam setup
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FRAME_RATE) 

        # face detection variable
        self.xmin_prev = 0
        self.ymin_prev = 0
        self.width_prev = 0
        self.height_prev = 0

        self.is_first_send_image = True
        self.is_first_detection = True

        self.is_system_shutdown = False


        ### record the video for presentation
        self.video_recorder = cv2.VideoWriter('../final_demo/video_face_pose_fps10_rosrate10.mp4', cv2.VideoWriter_fourcc(*'MP4V'), FRAME_RATE, (VIDEO_WIDTH,  VIDEO_HEIGHT))

        # socket setup
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_pose = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(ADDR)
        time.sleep(3)
        self.client_pose.connect(ADDR_POSE)

    def send_face_crop(self, image, face):
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

        ### record video
        self.video_recorder.write(image)

        ### send socket
        if(self.is_first_send_image):
            self.is_first_send_image = False
        else: self.thread_send_image.join()
        self.thread_send_image = threading.Thread(target=send_image, args=(self.client, image))
        self.thread_send_image.start()
    
    def send_pose(self, image, pose):
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
            
            ### calculate confidence
            shoulder_angle_L_conf = landmark[13].visibility>=POSE_CONF and landmark[12].visibility>=POSE_CONF and landmark[11].visibility>=POSE_CONF
            arm_angle_L_conf = landmark[15].visibility>=POSE_CONF and landmark[13].visibility>=POSE_CONF and landmark[11].visibility>=POSE_CONF
            shoulder_angle_R_conf = landmark[14].visibility>=POSE_CONF and landmark[12].visibility>=POSE_CONF and landmark[11].visibility>=POSE_CONF
            arm_angle_R_conf = landmark[16].visibility>=POSE_CONF and landmark[14].visibility>=POSE_CONF and landmark[12].visibility>=POSE_CONF

            ### angle calculation
            if(shoulder_angle_L_conf):
                self.shoulder_angle_L = np.clip(int(-math.acos((shoulder_L@arm_L)/(np.linalg.norm(shoulder_L)*np.linalg.norm(arm_L)))*180/np.pi+90), SHOULDER_MIN, SHOULDER_MAX)
            else:
                self.shoulder_angle_L = 0
            if(arm_angle_L_conf):
                self.arm_angle_L = np.clip(int(math.acos((forearm_L@arm_L)/(np.linalg.norm(forearm_L)*np.linalg.norm(arm_L)))*180/np.pi), ARM_MIN, ARM_MAX)
            else: 
                self.arm_angle_L = 0
            if(shoulder_angle_R_conf):
                self.shoulder_angle_R = np.clip(int(-math.acos((shoulder_R@arm_R)/(np.linalg.norm(shoulder_R)*np.linalg.norm(arm_R)))*180/np.pi+90), SHOULDER_MIN, SHOULDER_MAX)
            else: 
                self.shoulder_angle_R = 0
            if(arm_angle_R_conf):
                self.arm_angle_R = np.clip(int(math.acos((forearm_R@arm_R)/(np.linalg.norm(forearm_R)*np.linalg.norm(arm_R)))*180/np.pi), ARM_MIN, ARM_MAX)
            else: 
                self.arm_angle_R = 0
            
            # print("shoulder_L: ", int(-self.shoulder_angle_L+90), "\tarm_L: ", int(self.arm_angle_L), "\n", "shoulder_R: ", int(-self.shoulder_angle_R+90), "\tarm_R: ", int(self.arm_angle_R))

            self.arm_state = (self.shoulder_angle_L, self.arm_angle_L, self.shoulder_angle_R, self.arm_angle_R)

        ### send socket
        ### pose: can be a list or tuple contains FOUR floating points
        data = struct.pack('!4B', *self.arm_state) #!4B: four Uint8
        self.client_pose.sendall(data)

    def detection(self):
        with self.mp_face.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face, \
             self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            while self.cap.isOpened() and (not self.is_system_shutdown):
                success, image = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    ### If loading a video, use 'break' instead of 'continue'.
                    continue

                ### start multi-thread
                if(self.is_first_detection):
                    self.is_first_detection = False
                else: 
                    self.thread_send_face_crop.join()
                    self.thread_send_pose.join()

                self.thread_send_face_crop = threading.Thread(target=self.send_face_crop, args = (image, face))
                self.thread_send_pose = threading.Thread(target=self.send_pose, args = (image, pose))
                self.thread_send_face_crop.start()
                self.thread_send_pose.start()



                    

############
### main ###
############
if __name__ == '__main__':
    try:
        ### setup
        laptop = Laptop()

        ### set another thread to recceive streaming
        thread_recv_image = threading.Thread(target=recv_image, args=(laptop.client, laptop.is_system_shutdown))
        thread_recv_image.start()

        ### send streaming
        laptop.detection()

        ### wait till the receive thread to end
        thread_recv_image.join()
    
    except KeyboardInterrupt:
        pass

    except Exception as error_code:
        print(error_code)
    
    finally:
        laptop.is_system_shutdown = True
        print("\n #######################################\n", \
                 "### Wait for the system shutown ... ###\n", \
                 "#######################################")
        time.sleep(3)


        ### close cap
        laptop.cap.release()
        laptop.client.close()
        laptop.client_pose.close()
        cv2.destroyAllWindows()
        print("\n ###################################\n", \
                 "### Closing Laptop normally ... ###\n", \
                 "###################################")
