import socket, cv2, fps, pickle

HEADER = 64
PORT = 5050
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "127.0.1.1"
ADDR = (SERVER, PORT)

# fps initailization
FPS = fps.FPS((255, 255, 255))

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

# webcam streaming from https://medium.com/analytics-vidhya/video-chat-using-opencv-and-socket-5ca864a54696
cap = cv2.VideoCapture(0)
while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue
    en_image = cv2.imencode('.jpg', image)[1].tobytes()
    client.sendall(en_image)

    # FPS.calc_draw_fps(image)
    # cv2.imshow("client side", image) # demo client side streaming

    if cv2.waitKey(5) & 0xFF == 27:
        break
cap.release()
