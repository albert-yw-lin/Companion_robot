#!/usr/bin/env python3
import cv2, socket, threading, time
import mediapipe as mp
import numpy as np
from utils import *
from config import *

import rospy
from std_msgs.msg import Float64MultiArray

class Robot:

    def __init__(self) -> None:

        ### mediapipe setup
        self.mp_face = mp.solutions.face_detection

        ### webcam setup
        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

        ### socket setup
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(ADDR)
        self.server.listen(1)
        print("Waiting for client to connect ...")
        self.conn, self.addr = self.server.accept()
        print("Connected to client "+str(self.addr))
        time.sleep(1)

        ### socket for pose calculation setup
        self.server_pose = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_pose.bind(ADDR_POSE)
        self.server_pose.listen(1)
        print("Waiting for client_pose to connect ...")
        self.conn_pose, self.addr_pose = self.server_pose.accept()
        print("Connected to client_pose "+str(self.addr_pose))

        ### setup ROS message and node
        self.face_center = Float64MultiArray()
        self.pose = Float64MultiArray()
        self.face_center_pub = rospy.Publisher('face_center', Float64MultiArray, queue_size=10)
        self.pose_pub = rospy.Publisher('pose', Float64MultiArray, queue_size=10)
        rospy.init_node('camera', anonymous=True)
        self.rate = rospy.Rate(10) # 30Hz

        self.is_first_send = True
        self.is_first_detection = True

        self.face_center_x = 0
        self.face_center_y = 0

    def send_face_center(self, image, face):
        ### To improve performance, optionally mark the image as not writeable to
        ### pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        ### get results from face detection and pose
        results = face.process(image)
        ### turn the image into writable and BGR mode
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.detections:
            ### only select the first face
            detection = results.detections[0]
            ### example solution
            # mp_drawing.draw_detection(image, detection)
            box = detection.location_data.relative_bounding_box
            ### face position calculation, and assign to ROS message
            self.face_center_x, self.face_center_y = (box.xmin+0.5*box.width, box.ymin+0.5*box.height) # the Float64MultiArray data field is a list not tuple
        self.face_center.data = [self.face_center_x, self.face_center_y]
        # rospy.loginfo(self.face_center)
        self.face_center_pub.publish(self.face_center)
        self.rate.sleep()

        ### Flip the image horizontally for a selfie-view display.
        image = cv2.flip(image, 1)
        ### sending images through socket
        if(self.is_first_send):
            self.is_first_send = False
        else: self.thread_send_image.join()
        self.thread_send_image =threading.Thread(target=send_image, args=(self.conn, image))
        self.thread_send_image.start()   

    def recv_pose(self, socket):
        while True:
            ### 32 means 32 byte. ASSUME one floating points is 64bit (8 bytes) in python. 4*8=32
            data = socket.recv(16)
            ### close server and client simultaneously
            if not data: break

            self.pose.data = struct.unpack('!4f', data)
            rospy.loginfo(self.pose)
            self.pose_pub.publish(self.pose)
            self.rate.sleep()

    def detection(self):
        with self.mp_face.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face:
            while self.cap.isOpened() and (not rospy.is_shutdown()):
                success, image = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    ### If loading a video, use 'break' instead of 'continue'.
                    continue

                if(self.is_first_detection):
                    self.is_first_detection = False
                else: 
                    self.thread_send_face_center.join()
                self.thread_send_face_center = threading.Thread(target=self.send_face_center, args = (image, face))
                self.thread_send_face_center.start()
                # self.send_face_center(image)
            
############
### main ###
############
if __name__ == '__main__':
    try: 
        ### setup
        robot = Robot()

        thread_pose = threading.Thread(target=robot.recv_pose, args=(robot.conn_pose,))
        thread_pose.start()

        ### set another thread to recceive streaming
        thread_recv_image = threading.Thread(target=recv_image, args=(robot.conn,))
        thread_recv_image.start()

        ### send streaming
        robot.detection()

        ### wait till the receive thread to end
        thread_recv_image.join()
        thread_pose.join()

    except KeyboardInterrupt:
        pass
    
    except rospy.ROSInterruptException:
        pass

    finally:
        robot.cap.release()
        cv2.destroyAllWindows()
        robot.conn.close()
        robot.conn_pose.close()
        robot.server.close()
        print("Closing the program ...")