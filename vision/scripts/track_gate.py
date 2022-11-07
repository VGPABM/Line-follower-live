#!/usr/bin/env python3
import numpy as np
import sys
import rospy
import cv2
import math
from sensor_msgs.msg import Image
from vision.msg import XY_cor_msg
from std_msgs.msg import Bool, UInt64, Int8, Float64, Bool, Int32
from cv_bridge import CvBridge, CvBridgeError

class GatingVision:

  def __init__(self):
    rospy.init_node('landing_vision', anonymous=True)
    self.bridge = CvBridge()

    self.image_sub      = rospy.Subscriber  ("/drone/front_camera/image_raw", Image, self.callback)
    self.tc_pub         = rospy.Publisher   ('/tc', XY_cor_msg, queue_size=10)               #tc = target center position
    self.cntr_area_pub  = rospy.Publisher   ('/ctr_area', UInt64, queue_size=10)             #cntr_area  = contour_area
    self.cntr_pub       = rospy.Publisher   ('/cntr', Bool, queue_size=10)                   #cntr = gate contour boolean
    self.del_y_gate_pub = rospy.Publisher   ('/del_y', Int32, queue_size=10)                 #del_y = difference of height of the square's gate due to perspective
    self.gateRasio_pub  = rospy.Publisher   ("/rasio", Float64, queue_size=10)               #rasio = ratio w/h
    self.yawDirect_pub  = rospy.Publisher   ("/direction", Int8, queue_size=10)              #yawDirect = del_y +- to determine cw or ccw
    self.kotak_pub      = rospy.Publisher   ('/kotak', Bool, queue_size=10)                  #cntr = kotak part of the gate boolean

    self.rate = rospy.Rate(20)
    self.rate.sleep()
    
    rospy.loginfo("Streaming gate")

    # cv2.namedWindow('test')
    # cv2.createTrackbar('thrs1', 'test', 16, 255, self.f)
    # cv2.createTrackbar('thrs2', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs3', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs4', 'test', 146, 180, self.f)
    # cv2.createTrackbar('thrs5', 'test', 255, 255, self.f)
    # cv2.createTrackbar('thrs6', 'test', 255, 255, self.f)
    # cv2.createTrackbar('area', 'test', 100, 20000, self.f)
    # cv2.createTrackbar('epsilon', 'test', 1, 1000, self.f)

  def f(self, x):
    pass

  def bubblesort(self, list):
    temp = -1
    for iter_num in range(len(list)-1,0,-1):
      for idx in range(iter_num):
         if list[idx][0][0] > list[idx+1][0][0]:
            list[idx][0][0], list[idx+1][0][0] = list[idx+1][0][0], list[idx][0][0]
            list[idx][0][1], list[idx+1][0][1] = list[idx+1][0][1], list[idx][0][1]
    if(list[0][0][1] > list[1][0][1] and abs(list[0][0][0] - list[1][0][0]) < 10):
        list[0][0][0], list[0+1][0][0] = list[0+1][0][0], list[0][0][0]
        list[0][0][1], list[0+1][0][1] = list[0+1][0][1], list[0][0][1]
    if(list[2][0][1] > list[3][0][1] and abs(list[2][0][0] - list[3][0][0]) < 10):
        list[2][0][0], list[2+1][0][0] = list[2+1][0][0], list[2][0][0]
        list[2][0][1], list[2+1][0][1] = list[2+1][0][1], list[2][0][1]

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      h, w, c = cv_image.shape
      cv_image = cv_image[0:h, 0:w]
      h, w, c = cv_image.shape
      # print(f'{h} {w} {c}')
    except CvBridgeError as e:
      print(e)

    # self.ilowh = cv2.getTrackbarPos('thrs1', 'test')
    # self.ilows = cv2.getTrackbarPos('thrs2', 'test')
    # self.ilowv = cv2.getTrackbarPos('thrs3', 'test')
    # self.ihighh = cv2.getTrackbarPos('thrs4', 'test')
    # self.ihighs = cv2.getTrackbarPos('thrs5', 'test')
    # self.ihighv = cv2.getTrackbarPos('thrs6', 'test')
    # self.area_thrs = cv2.getTrackbarPos('area', 'test')
    # self.epsilon = cv2.getTrackbarPos('epsilon', 'test')

    #threshold
    # lower = np.array([self.ilowh, self.ilows, self.ilowv])
    # upper = np.array([self.ihighh, self.ihighs, self.ihighv])

    #magenta
    lower = np.array([146, 0, 0])
    upper = np.array([180, 255, 255])

    #dark magenta 
    # lower = np.array([146, 0, 66])
    # upper = np.array([180, 255, 165])

    # red
    # lower = np.array([0, 70, 50])
    # upper = np.array([10, 255, 255])

    self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    self.mask = cv2.inRange(self.hsv, lower, upper)
    contours, hierarchy = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cv2.circle(cv_image, (int(w/2), int(h/2)), 1, (0, 0, 255), 5)

    self.tc = XY_cor_msg() 
    self.epsilon = 100
    self.is_centered = False
    self.there_is_gate = False
    self.ada_kotak = False
    self.rasio = 1.428
    self.del_y = 1000
    self.direction = 0
    self.contour_area = 0
    
    if(contours is not None):
        if len(contours) != 0:
            # cv2.drawContours(cv_image, contours, -1, 255, 3)
            c = max(contours, key = cv2.contourArea)
            self.contour_area = cv2.contourArea(c)
            cv2.drawContours(cv_image, c, -1, 255, 3)
            # print(f'area : {cv2.contourArea(c)}')
            
            if(self.contour_area > 20000): #2500
                self.there_is_gate = True
                x,y,w,h = cv2.boundingRect(c)
                approx = cv2.approxPolyDP(c, (self.epsilon/10000) * cv2.arcLength(c, True), True)
                self.rasio = w/h

                # rospy.loginfo(f'max contour area detected {cv2.contourArea(c)}')
                # print(f'rasio : {self.rasio}, area : {cv2.contourArea(c)}')

                if(len(approx) == 4):
                  self.ada_kotak = True
                  
                  # print(f'before \n {approx}')
                  # print()
                  self.bubblesort(approx)
                  # print(f'after \n {approx}')
                  # print()

                  self.del_y = approx[2][0][1] - approx[0][0][1]
                  # print(f'({approx[0][0][0]}, {approx[0][0][1]}), ({approx[3][0][0]}, {approx[3][0][1]})')
                  # print(f'{approx[3][0][1]}-{approx[0][0][1]} = {del_y}')
                  # print(f'rasio: {w/h}, del_y: {del_y}')

                  cv2.drawContours(cv_image, [approx], -1, 255, 3)

                  # i = 0
                  # for point in approx:
                  #   point = point[0]; # drop extra layer of brackets
                  #   center = (int(point[0]), int(point[1]))
                  #   cv2.circle(cv_image, center, 4, (150, 200, 0), -1)
                  #   # print(str(np.where(approx == point))[11])
                  #   cv_image = cv2.putText(cv_image, str(i), (center[0]-25, center[1]-25), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 132, 255), 2, cv2.LINE_AA)
                  #   i += 1

                  if(self.del_y < 0):
                    self.direction = -1
                  elif(self.del_y > 0):
                    self.direction = 1
                  elif(self.del_y == 0):  
                    self.direction = 0

                  # print(f'{del_y}\n{type(del_y)}')
                  
                  

                x1 = x + int(w/2)
                y1 = y + int(h/2)

                self.tc.x = float(x1)
                self.tc.y = float(y1)
                
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.circle(cv_image, (x1, y1), 1, (0, 255, 255), 5)
                self.tc_pub.publish(self.tc)

    self.gateRasio_pub.publish(self.rasio)
    self.del_y_gate_pub.publish(self.del_y)
    self.cntr_area_pub.publish(int(self.contour_area))    
    self.kotak_pub.publish(self.ada_kotak)
    self.cntr_pub.publish(self.there_is_gate)
    self.yawDirect_pub.publish(self.direction)
       
    cv2.imshow("detect gate", cv_image)
    cv2.waitKey(3)

def main(args):
  gv = GatingVision()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)