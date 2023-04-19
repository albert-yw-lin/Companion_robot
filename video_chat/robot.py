import cv2, socket, threading
import mediapipe as mp
import numpy as np
from utils import *
from config import *

class Robot:

    def __init__(self) -> None:

        ### mediapipe setup
        self.mp_face = mp.solutions.face_detection

        ### webcam setup
        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)

        ### socket setup
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(ADDR)
        self.server.listen(1)
        print("Waiting for client to connect ...")
        self.conn, self.addr = self.server.accept()
        print("Connected to "+str(self.addr))
        
        ### fps setup
        self.fps = Fps()

    def face_position(self, results):
        # only select the first face
        detection = results.detections[0]

        # # example solution
        # mp_drawing.draw_detection(image, detection)

        box = detection.location_data.relative_bounding_box


    def socket_send(self):
        with self.mp_face.FaceDetection(
            model_selection=0, min_detection_confidence=0.5) as face:

            while self.cap.isOpened():
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

                ### face position calculation
                if results.detections: face_center = self.face_position(results)

                ### Flip the image horizontally for a selfie-view display.
                image = cv2.flip(image, 1)
                                
                ### draw fps onto the image
                image = self.fps.calc_draw_fps(image)

                #############################
                ### sending face position ###
                #############################

                ### TODO ###

                ### sending images through socket
                send_image(self.conn, image)
        while True:
            image = np.ones((480, 640, 3))
            send_image(self.conn, image)

            # self.cap.release()
            # cv2.destroyAllWindows()
            # self.conn.close()
            # self.server.close()                 

############
### main ###
############
def main():

    try: 
        ### setup
        robot = Robot()

        ### set another thread to recceive streaming
        thread_recv = threading.Thread(target=recv_image, args=(robot.conn,))
        thread_recv.start()

        ### send streaming
        robot.socket_send()

        ### wait till the receive thread to end
        thread_recv.join()

    except KeyboardInterrupt:
        print("KeyboardInterrupt.")

    finally:
        # robot.cap.release()
        cv2.destroyAllWindows()
        robot.conn.close()
        robot.server.close()
        print("Closing the program ...")

if __name__ == '__main__':
    main()