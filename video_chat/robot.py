#!/usr/bin/env python3
import cv2, socket, threading
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
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)

        ### socket setup
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(ADDR)
        self.server.listen(1)
        print("Waiting for client to connect ...")
        self.conn, self.addr = self.server.accept()
        print("Connected to client "+str(self.addr))

        ### socket for pose calculation setup
        self.server_pose = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_pose.bind(ADDR_POSE)
        self.server_pose.listen(1)
        print("Waiting for client_pose to connect ...")
        self.conn_pose, self.addr_pose = self.server_pose.accept()
        print("Connected to client_pose "+str(self.addr_pose))
        
        ### fps setup
        # self.fps = Fps()

        ### setup ROS message and node
        # self.face_center = Float64MultiArray()
        # self.pose = Float64MultiArray()
        # self.face_center_pub = rospy.Publisher('face_center', Float64MultiArray, queue_size=10)
        # self.pose_pub = rospy.Publisher('pose', Float64MultiArray, queue_size=10)
        # rospy.init_node('camera', anonymous=True)
        # self.rate = rospy.Rate(30) # 30Hz

    def face_position(self, results):
        ### only select the first face
        detection = results.detections[0]

        ### example solution
        # mp_drawing.draw_detection(image, detection)

        box = detection.location_data.relative_bounding_box
        return [box.xmin+0.5*box.width, box.ymin+0.5*box.height] # the Float64MultiArray data field is a list not tuple

    def detection(self):
        with self.mp_face.FaceDetection(
            model_selection=0, min_detection_confidence=0.5) as face:

            while self.cap.isOpened() and (not rospy.is_shutdown()):
                success, image = self.cap.read()
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

                ### face position calculation, and assign to ROS message
                # if results.detections: self.face_center.data = self.face_position(results)

                ### Flip the image horizontally for a selfie-view display.
                image = cv2.flip(image, 1)
                                
                ### draw fps onto the image
                # image = self.fps.calc_draw_fps(image)

                ###################################
                ### sending and publishing data ###
                ###################################
                
                ### sending images through socket
                send_image(self.conn, image)   

                ### receiving pose array (tuple)
                # self.pose.data = recv_pose(self.conn_pose)
                # rospy.loginfo(self.pose)
                # rospy.loginfo(self.face_center)
                # self.pose_pub.publish(self.pose)
                # self.face_center_pub.publish(self.face_center)
                # self.rate.sleep()


                            

############
### main ###
############
if __name__ == '__main__':
    try: 
        ### setup
        robot = Robot()

        ### set another thread to recceive streaming
        thread_recv = threading.Thread(target=recv_image, args=(robot.conn,))
        thread_recv.start()

        ### send streaming
        robot.detection()

        ### wait till the receive thread to end
        thread_recv.join()

    except KeyboardInterrupt:
        pass
    
    except rospy.ROSInterruptException:
        pass

    finally:
        # robot.cap.release()
        cv2.destroyAllWindows()
        robot.conn.close()
        robot.conn_pose.close()
        robot.server.close()
        print("Closing the program ...")