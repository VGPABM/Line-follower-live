#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from vision.msg import RotatedRect
from cv_bridge import CvBridge, CvBridgeError

def extract_line(bw, draw_image):
  # print(reversed.shape)
  contours, hierarchy = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  is_exist, x, y, angle = False, 0, 0, 90 #--> 90 for easier control algo, cause 90 is the target, so just set the default as yes crrect
  if(len(contours) > 0):
      is_exist = True
      max_area_cnt = max(contours, key=cv2.contourArea)
      (x, y), (w, h), angle = cv2.minAreaRect(max_area_cnt)
      if w < h:
          angle = 90-angle
      else:
          angle = 180 - angle
      if(angle == 180 or angle==0):
          is_exist = False
      cv2.drawContours(draw_image, [max_area_cnt], 0, (128, 128, 0), 2)  # 0 is the id, of box in [box]
      cv2.putText(draw_image, f'{angle:.4f}', (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
  return is_exist, x, y, angle


class LandingVision:
  # will get called when the class is created
  def __init__(self):
    rospy.init_node('landing_vision', anonymous=True)
    # helper to convert sensor_msgs.Image to opencv/numpy array
    self.bridge = CvBridge()

    # subscribe to front camera data, this will call self.callback with rosspin and will be the main 'loop' to detect stuff
    self.image_sub = rospy.Subscriber("/drone/down_camera/image_raw",Image,self.callback)
    
    # msg holder for landing pad
    self.landing_pad_msg = RotatedRect()
    
    # self.trackbar_sub = rospy.Subscriber("/tb",UInt8MultiArray,self.trackback)
    self.tbval = [0,0,0,0,0,0]
    
    # publisher for landingPad data as 'RotatedRect' msg containing center x,y, width, height, angle (angle ga kepake)
    self.landing_pad_pub = rospy.Publisher("/landingpad_data", RotatedRect, queue_size=10)
    
    # publisher for horizontalGate data as 'RotatedRect' msg containing center x,y, width, height, angle (angle ga kepake)
    self.hgate_pub = rospy.Publisher("/gate/horizontal/data", RotatedRect, queue_size=10)
    # publisher for horizontalGate HELP, yakni garis item di bawah gatenya
    self.hgate_help_pub = rospy.Publisher("/gate/horizontal/help", RotatedRect, queue_size=10)
    
    # same, msg holder
    self.hgate_help_msg = RotatedRect()
    self.hgate_msg = RotatedRect()
    
    # threshold hsv for horizontal gate
    self.lhsvgate = np.array([127, 127, 127])
    self.uhsvgate = np.array([255, 255, 255])
    
    # threshold hsv for landing pad
    self.lhsvpad = np.array([63, 0, 0])
    self.uhsvpad = np.array([127, 255, 255])

    self.kernel = np.ones((5,5),np.uint8)# ga kepake

    # threshold gray for detect garis dibawah horizontal gate
    self.lb = 100
    self.hb = 255
    
    self.hsv = None
    self.rate = rospy.Rate(32)
    self.rate.sleep()
    rospy.loginfo("Streaming landing vision")

    # cv2.namedWindow('test')
    # cv2.createTrackbar('thrs1', 'test', 16, 255, self.f)
    # cv2.createTrackbar('thrs2', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs3', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs4', 'test', 180, 180, self.f)
    # cv2.createTrackbar('thrs5', 'test', 255, 255, self.f)
    # cv2.createTrackbar('thrs6', 'test', 255, 255, self.f)
  
  # detect the line inside the horizontal gate
  def handleHorizontalGate(self, max_area_cnt, draw_image, mask_shape):
    # so it is a gate
    (x, y), (w, h), angle = cv2.minAreaRect(max_area_cnt)
    self.hgate_msg.is_exist = True
    self.hgate_msg.x = x
    self.hgate_msg.y = y
    self.hgate_msg.w = w
    self.hgate_msg.h = h
    self.hgate_msg.angle=angle

    # create new blank mask, will used for later to isolate garis inside horizontal gate
    new_mask = np.zeros(mask_shape, np.uint8)
    # fill the contour on the new mask (now we can isolate the horizontal gate only)
    cv2.drawContours(new_mask, [max_area_cnt], 0, (255, 255, 255), cv2.FILLED)
    # now mask the ori
    # andwised = cv2.bitwise_and(draw_image, new_mask)
    # cvt to gray
    tmp = cv2.cvtColor(draw_image, cv2.COLOR_BGR2GRAY)
    # threshold-it to detect garis item in the frame
    ret, tmp = cv2.threshold(tmp, self.lb, self.hb, cv2.THRESH_BINARY)
    # flip it so that garisnya warna putih and background item
    tmp = cv2.bitwise_not(tmp)
    # isolate it with the new mask
    tmp = cv2.bitwise_and(tmp, new_mask)
    # morpex
    # tmp = cv2.morphologyEx(tmp, cv2.MORPH_OPEN, self.kernel)
    
    # extraact the line from the processed frame
    l_is_exist, l_x, l_y, l_angle = extract_line(tmp, draw_image)
    self.hgate_help_msg.is_exist = l_is_exist
    self.hgate_help_msg.x = l_x
    self.hgate_help_msg.y = l_y
    self.hgate_help_msg.angle = l_angle
    # draw it later
    cv2.drawContours(draw_image, [max_area_cnt], 0, (128, 128, 0), 2)  # 0 is the id, of box in [box]

      # cv2.imshow('noice', tmp)
      # cv2.putText(draw_image, f'{angle:.4f}', (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    #draw center of gate
    cv2.circle(draw_image, (int(self.hgate_msg.x), int(self.hgate_msg.y)), 2, (0, 0, 0), -1)
    cv2.circle(draw_image, (320, 180), 3, (255, 255, 0), -1)

  # as the name suggest
  def detect_contour(self, mask, area_threshold, draw_image, isPad):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
           
    if(len(contours) > 0):
        max_area_cnt = max(contours, key=cv2.contourArea)
        if(cv2.contourArea(max_area_cnt) > area_threshold):
          if isPad:
            # we are detecting pad, so use this algo
            x,y,w,h = cv2.boundingRect(max_area_cnt)
            # comment this if udah lomba, cause no need ngambar (at least i think)
            cv2.rectangle(draw_image,(x,y),(x+w,y+h),(0,255,0),2)
            self.landing_pad_msg.is_exist = True
            self.landing_pad_msg.x = x + w//2
            self.landing_pad_msg.y = y + h//2
            self.landing_pad_msg.w = w
            self.landing_pad_msg.h = h

            #draw center of gate
            cv2.circle(draw_image, (int(self.landing_pad_msg.x), int(self.landing_pad_msg.y)), 2, (0, 0, 0), -1)
            cv2.circle(draw_image, (320, 180), 3, (0, 0, 255), -1)

          else:
            # hm not pad, but gate, use this algo
            self.handleHorizontalGate(max_area_cnt, draw_image, mask.shape)

          
  def f(self, x):
    pass

  def trackback(self, data):
    # self.tbval = data.data
    self.tbval = data.data

  # callback utk data camera bawah, serve as main 'loop'
  def callback(self,data):
    self.landing_pad_msg.is_exist = False
    self.hgate_msg.is_exist = False
    self.hgate_help_msg.is_exist = False
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # self.ilowh = cv2.getTrackbarPos('thrs1', 'test')
    # self.ilows = cv2.getTrackbarPos('thrs2', 'test')
    # self.ilowv = cv2.getTrackbarPos('thrs3', 'test')
    # self.ihighh = cv2.getTrackbarPos('thrs4', 'test')
    # self.ihighs = cv2.getTrackbarPos('thrs5', 'test')
    # self.ihighv = cv2.getTrackbarPos('thrs6', 'test')

    # lower = np.array([self.ilowh, self.ilows, self.ilowv])
    # upper = np.array([self.ihighh, self.ihighs, self.ihighv])

    # self.lower = np.array([self.tbval[0], self.tbval[1], self.tbval[2]])
    # self.upper = np.array([self.tbval[3], self.tbval[4], self.tbval[5]])

    # convert to hsv for easier masking (people said this best to mask object/shape)
    self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # threshold it to detect the pad
    mask_pad = cv2.inRange(self.hsv, self.lhsvpad, self.uhsvpad)
    # print(type(mask_pad[0][0]) ) #->np.uint8

    # same, but for gate
    mask_gate = cv2.inRange(self.hsv, self.lhsvgate, self.uhsvgate)
    # cv2.imshow('maskg', mask_gate)
    # cv2.imshow('maskp', mask_pad)
    # (height, width) = cv_image.shape[:2]

    # detect contour on each mask gate and pad
    self.detect_contour(mask_gate, 9600, cv_image, False)
    self.detect_contour(mask_pad, 6000, cv_image, True)
    
    self.hgate_pub.publish(self.hgate_msg)
    self.hgate_help_pub.publish(self.hgate_help_msg)
    self.landing_pad_pub.publish(self.landing_pad_msg)
    cv2.imshow("detect pad and hgate", cv_image)
    cv2.waitKey(1)


def main(args):
  lv = LandingVision()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)