#!/usr/bin/python

import cv2 as cv
import rospy

class Camera:
    '''
    CLASS INFORMATION
    <>Attribute
        - self.device = device kamera di laptop
        - self.camera = ngefungsiin kamera ke cv2
        - self.flipped = 
    <>Method
        - is_opened = ngecek kamera udah nyala atau belum
        - capture   = ngereturn frame dari kamera
        - view      = ngeliatin frame dari kamera
    '''
    def __init__(self, device, fps=30, width=640, height=480):
        self.device = device
        self.flipped = False
        self.camera = cv.VideoCapture(self.device)
        self.set_resolution(width, height)
        self.set_fps(fps)

    def set_resolution(self, width, height):
        self.camera.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, height)

    def set_fps(self, fps):
        self.camera.set(cv.CAP_PROP_FPS, fps)
    
    def is_opened(self):
        if self.camera and self.camera.isOpened():
            return True
        return False

    def capture(self):
        if not self.is_opened():
            return
        ret, frame = self.camera.read()
        if not ret:
            return None
        if self.is_flipped():
            frame = cv.flip(frame, 1)
        return frame

    def flip_cam(self):
        self.flipped = not self.flipped

    def is_flipped(self):
        return self.flipped

    def view(self):
        cv.imshow('hehe', self.capture())

if __name__ == '__main__':
    x = input("Device camera")
    galih = Camera(x)
    galih.flip_cam()
    while(galih.is_opened()):
        galih.view()
        
        k = cv.waitKey(1)
        if k == 27:
            break
    cv.destroyAllWindows()
   

