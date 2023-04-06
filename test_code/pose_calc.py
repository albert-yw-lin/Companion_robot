import cv2
import mediapipe as mp
import numpy as np
import fps
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

# fps initailization
FPS = fps.FPS((0,0,0))

# For webcam input:
cap = cv2.VideoCapture(0)
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = pose.process(image)

    # Draw the pose annotation on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # # draw the pose
    mp_drawing.draw_landmarks(
        image,
        results.pose_landmarks,
        mp_pose.POSE_CONNECTIONS,
        landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
    
    # pose angle calculation
    if results.pose_landmarks:
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
      print("shoulder_L: ", shoulder_angle_L, "\tarm_L: ", arm_angle_L, "\n", "shoulder_R: ", shoulder_angle_R, "\tarm_R: ", arm_angle_R)

    # Flip the image horizontally for a selfie-view display.
    image = cv2.flip(image, 1)

    # Calculate and print FPS
    FPS.calc_draw_fps(image)


    cv2.imshow('MediaPipe Pose', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
