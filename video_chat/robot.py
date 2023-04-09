import cv2, socket, threading, keyboard
import mediapipe as mp
import numpy as np
from utils import *
from config import *

class Robot:

    def __init__(self) -> None:

        # mediapipe setup
        self.mp_face = mp.solutions.face_detection

        # webcam setup
        # self.cap = cv2.VideoCapture(CAMERA_ID)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)

        # socket setup
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(ADDR)
        self.server.listen(1)
        print("Waiting for client to connect ...")
        self.conn, self.addr = self.server.accept()
        print("Connected to "+str(self.addr))
        
        # fps setup
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
                results = face.process(image)

                # turn the image into writable and BGR mode
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # face position calculation
                if results.detections: face_center = self.face_position(results)
                
                # draw fps onto the image
                image = self.fps.calc_draw_fps(image)

                # Flip the image horizontally for a selfie-view display.
                image = cv2.flip(image, 1)

                #############################
                ### sending face position ###
                #############################

                ### TODO ###

                #####################################
                ### sending images through socket ###
                #####################################

                encode_image = cv2.imencode('.jpg', image)[1].tobytes()
                # tell the server(robot) how much data should it receive
                encode_image_length = len(encode_image)
                self.server.sendall(encode_image_length.to_bytes(4, byteorder='big'))
                while encode_image_length > 0:
                    # send BYTE_PER_TIME bytes of data per time to avoid bottleneck and better manage the flow of data
                    chunk = encode_image[:BYTE_PER_TIME]
                    self.conn.sendall(chunk)
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
            self.conn.close()
            self.server.close()

    def socket_recv(self):
        buffer = b''
        while True:
            data = self.conn.recv(BYTE_PER_TIME)
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
                    
            if keyboard.is_pressed('Esc'):
                break

############
### main ###
############
def main():
    # setup
    robot = Robot()

    # set another thread to recceive streaming
    thread_recv = threading.Thread(target=robot.socket_recv)
    thread_recv.start()

    # send streaming
    # robot.socket_send()

if __name__ == '__main__':
    main()