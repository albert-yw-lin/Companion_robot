import cv2
import numpy as np
import socket
import threading
import fps

# Server configuration
HOST = '127.0.0.1'  # Replace with your server IP address
PORT = 9999

# OpenCV video capture configuration
cap = cv2.VideoCapture(0)

# Socket server configuration
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)
conn, addr = sock.accept()

# # Thread function to receive frames from the client
# def receive_frames():
#     buffer = b''
#     while True:
#         data = conn.recv(4096)
#         if not data:
#             break
#         buffer += data
#         while True:
#             if len(buffer) < 4:
#                 break
#             frame_length = int.from_bytes(buffer[:4], byteorder='big')
#             if len(buffer) < frame_length + 4:
#                 break
#             frame_data = buffer[4:frame_length+4]
#             buffer = buffer[frame_length+4:]
#             frame = np.frombuffer(frame_data, dtype=np.uint8)
#             frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
#             cv2.imshow('From Client', frame)
#             cv2.waitKey(1)

# # Start the receiving thread
# receive_thread = threading.Thread(target=receive_frames)
# receive_thread.start()

# Main loop to send frames to the client
FPS = fps.FPS((255, 255, 255))
while True:
    ret, frame = cap.read()
    if not ret:
        break
    FPS.calc_draw_fps(frame)
    frame_data = cv2.imencode('.jpg', frame)[1].tobytes()
    frame_length = len(frame_data)
    conn.sendall(frame_length.to_bytes(4, byteorder='big'))
    while len(frame_data) > 0:
        chunk = frame_data[:4096]
        frame_data = frame_data[4096:]
        conn.sendall(chunk)

# # Clean up resources
cap.release()
cv2.destroyAllWindows()
conn.close()
sock.close()