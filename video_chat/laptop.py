import cv2, socket, threading
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

        # socket setup
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(ADDR)

        # face detection variable
        self.xmin_prev = 0
        self.ymin_prev = 0
        self.width_prev = 0
        self.height_prev = 0
        
        # fps setup
        self.fps = Fps()

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

    def socket_send(self):
        with self.mp_face.FaceDetection(
            model_selection=0, min_detection_confidence=0.5) as face, \
            self.mp_pose.Pose(
            min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:

            while self.cap.isOpened():
                success, image = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue
                
                ########################
                ### image processing ###
                ########################

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
                if results_face.detections: image = self.face_crop(image, results_face)
                if results_pose.pose_landmarks: left_arm_state, right_arm_state = self.arm_calc(results_pose)
                
                image = self.post_process(image)
                image = self.fps.calc_draw_fps(image)

                #########################################
                ### sending arm states through socket ###
                #########################################

                ### TODO ###

                #####################################
                ### sending images through socket ###
                #####################################

                encode_image = cv2.imencode('.jpg', image)[1].tobytes()
                # tell the server(robot) how much data should it receive
                encode_image_length = len(encode_image)
                self.client.sendall(encode_image_length.to_bytes(4, byteorder='big'))
                while encode_image_length > 0:
                    # send BYTE_PER_TIME bytes of data per time to avoid bottleneck and better manage the flow of data
                    chunk = encode_image[:BYTE_PER_TIME]
                    self.client.sendall(chunk)
                    encode_image = encode_image[BYTE_PER_TIME:]

                # if press esc then break
                if cv2.waitKey(5) & 0xFF == 27:
                    break

                # show the image on local machine(only for testing)
                # cv2.imshow('video chat', image)
                # if cv2.waitKey(5) & 0xFF == 27:
                #     break

            self.cap.release()
            cv2.destroyAllWindows()

    def socket_recv(self):
        buffer = b''
        while True:
            data = self.client.recv(BYTE_PER_TIME)
            if not data:
                break
            buffer += data
            if len(buffer) <4: continue
            else:
                while True:
                    encode_image_length = int.from_bytes(buffer[:4], byteorder='big')
                    if len(buffer) < encode_image_length + 4:
                        break
                    encode_image = buffer[4:encode_image_length+4]
                    buffer = buffer[encode_image_length+4:]
                    image = np.frombuffer(encode_image, dtype=np.uint8)
                    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
                    cv2.imshow('From Server(Robot)', image)
                    
                    # # if press esc then break
                    # if cv2.waitKey(5) & 0xFF == 27:
                    #     break

############
### main ###
############

def main():
    # setup
    laptop = Laptop()

    # set another thread to recceive streaming
    thread_recv = threading.Thread(target=laptop.socket_recv)
    thread_recv.start()

    # send streaming
    laptop.socket_send()

if __name__ == '__main__':
    main()
