import time
import cv2

class FPS:
    def __init__(self, rgb):
        self.fps = 0
        self.time_prev = 0
        self.counter = 0
        self.time_now = 0
        self.freq = 10 # howm many loops calculate once
        self.rgb = rgb #(r, g, b)
    def calc_draw_fps(self, image):
        self.counter+=1
        if self.counter%self.freq == 9:
            self.time_prev = time.time()
        if self.counter%self.freq == 0: 
            self.counter = 0 # reset counter
            self.time_now = time.time()
            self.fps = int(1/(self.time_now - self.time_prev))
        cv2.putText(image,"FPS:%d"%self.fps,(0,30),cv2.FONT_HERSHEY_SIMPLEX,1,self.rgb,2)
