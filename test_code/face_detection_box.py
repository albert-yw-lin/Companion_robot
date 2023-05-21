import cv2
import mediapipe as mp
import fps
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(-1)

# fps initailization
FPS = fps.FPS((255, 255, 255))

# initialization
xmin_prev = 0
ymin_prev = 0
width_prev = 0
height_prev = 0

#parameters
smooth_location = 0.2 # smooth factor of box location, 1 for no smooth
smooth_area = 0.2 # smooth factor of box area, 1 for no smooth
forehead_factor = 0.5 # porprotion with respect to bounding box
expand_factor = 0.1 # porprotion with respect to bounding box
video_height = 480
video_width = 640

cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)

# # record the video for presentation
# fourcc = cv2.VideoWriter_fourcc(*'MP4V')
# out = cv2.VideoWriter('./face_detection_demo/none.mp4', fourcc, 7, (video_width,  video_height))

with mp_face_detection.FaceDetection(
    model_selection=0, min_detection_confidence=0.5) as face_detection:
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
    results = face_detection.process(image)

    # Crop the face area
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.detections:

      # only select the first face
      detection = results.detections[0]

      # # example solution
      # mp_drawing.draw_detection(image, detection)

      ### face crop application start

      #get webcam image shape
      image_height = image.shape[0]
      image_width = image.shape[1]

      box = detection.location_data.relative_bounding_box
      print(box)
      # add smooth filter to both box location and box area
      if 0 in (xmin_prev, ymin_prev, width_prev, height_prev):
        xmin_prev = box.xmin
        ymin_prev = box.ymin
        width_prev = box.width
        height_prev = box.height

      box.xmin = smooth_location*box.xmin + (1-smooth_location)*xmin_prev
      box.ymin = smooth_location*box.ymin + (1-smooth_location)*ymin_prev
      box.width = smooth_area*box.width + (1-smooth_area)*width_prev
      box.height = smooth_area*box.height + (1-smooth_area)*height_prev

      xmin_prev = box.xmin
      ymin_prev = box.ymin
      width_prev = box.width
      height_prev = box.height

      # box area adjustment
      forehead = forehead_factor*box.height
      expand_width = expand_factor*box.width
      expand_height = expand_factor*box.height

      # bounging box
      xmin = (box.xmin - expand_width)*image_width
      ymin = (box.ymin - expand_height - forehead)*image_height
      xmax = (box.xmin + box.width + expand_width)*image_width
      ymax = (box.ymin + box.height + expand_height)*image_height

      if xmin < 0: xmin = 0
      if ymin < 0: ymin = 0
      image = image[int(ymin):int(ymax), int(xmin):int(xmax)]
    
    # resize
    x_size = int(video_height*(image.shape[1]/image.shape[0]))
    image = cv2.resize(image, (x_size,video_height))

    #padding
    pad = abs(video_width-x_size)
    if pad%2==0: pad_L = pad_R = pad//2
    else: pad_L = pad//2; pad_R = pad//2+1
    image = cv2.copyMakeBorder(image, 0,0,pad_L,pad_R, cv2.BORDER_CONSTANT, value=0)

    ### face crop application end

    # Flip the image horizontally for a selfie-view display.
    image = cv2.flip(image, 1)

    # Calculate and print FPS
    FPS.calc_draw_fps(image)

    # # record video
    # out.write(image)

    cv2.imshow('MediaPipe Face Detection', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break

cap.release()
    
# # release video writer
# out.release()
