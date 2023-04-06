import socket, cv2, fps, pickle
import numpy as np

HEADER = 64
PORT = 5050
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

# fps initailization
FPS = fps.FPS((255, 255, 255))

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)

def receive(conn, addr):
    print(f"[NEW CONNECTION] {addr} connected.")

    while True:
        en_image = conn.recv(100000) # encoded image
        image_arr = np.frombuffer(en_image,np.uint8) # image array
        image = cv2.imdecode(image_arr, cv2.IMREAD_COLOR)
        if type(image) is type(None):
            pass
        else:
            FPS.calc_draw_fps(image)
            cv2.imshow("server side", image)
            if cv2.waitKey(5) & 0xFF == 27:
                break

    cv2.destroyAllWindows()
    conn.close()


def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    conn, addr = server.accept()
    receive(conn, addr)


print("[STARTING] server is starting...")
start()