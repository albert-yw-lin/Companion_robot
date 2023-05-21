import cv2

cap = cv2.VideoCapture(-1)

while cap.isOpened():
    success, image = cap.read()
    cv2.imshow('recv_image', image)
    cv2.waitKey(1)
