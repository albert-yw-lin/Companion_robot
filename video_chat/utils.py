import time
import cv2
from config import *

class Fps:
    def __init__(self):
        self.time_prev = 0
        self.counter = 0
        self.fps = 0
    def calc_draw_fps(self, image):
        self.counter+=1
        if self.counter%FREQUENCY == 9:
            self.time_prev = time.time()
        if self.counter%FREQUENCY == 0: 
            self.counter = 0 # reset counter
            self.fps = int(1/(time.time() - self.time_prev))
        cv2.putText(image,"FPS:%d"%self.fps,FPS_POSITION,FPS_TEXT,1,FPS_COLOR,2)
        return image

class Smooth_data():
    def  __init__(self, data) -> None:
        # simple exponential smoothing
        self.data_prev_
