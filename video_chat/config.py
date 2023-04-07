import cv2

# socket
HEADER = 64
PORT = 5050
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "127.0.1.1"
ADDR = (SERVER, PORT)

# face detection
SMOOTH = 0.2 # 1 for no smooth
FOREHEAD = 0.5 # porprotion with respect to bounding box
EXPAND = 0.1 # porprotion with respect to bounding box
VIDEO_HEIGHT = 480
VIDEO_WIDTH = 640

#fps calc
FREQUENCY = 10
FPS_COLOR = (255,255,255)
FPS_TEXT = cv2.FONT_HERSHEY_SIMPLEX
FPS_LOCATION = (0,30)

