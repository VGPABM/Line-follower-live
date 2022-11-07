#!/usr/bin/env python3

#import roslib
#roslib.load_manifest('my_package')
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from vision.msg import XY_cor_msg
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

class LandingVision:

  def __init__(self):
    rospy.init_node('landing_vision', anonymous=True)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/drone/down_camera/image_raw",Image,self.callback)
    self.landing_cor_pub = rospy.Publisher("/landing_pos", XY_cor_msg, queue_size=10)
    self.detect_pad_pub = rospy.Publisher("/pad_detected", Bool, queue_size=10)
    self.rate = rospy.Rate(20)
    self.rate.sleep()
    rospy.loginfo("Streaming landing vision")

    # cv2.namedWindow('test')
    # cv2.createTrackbar('thrs1', 'test', 16, 255, self.f)
    # cv2.createTrackbar('thrs2', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs3', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs4', 'test', 180, 180, self.f)
    # cv2.createTrackbar('thrs5', 'test', 255, 255, self.f)
    # cv2.createTrackbar('thrs6', 'test', 255, 255, self.f)

  def f(self, x):
    pass

  def callback(self,data):
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

    lower = np.array([32, 32, 16])
    upper = np.array([160, 255, 255])

    self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(self.hsv, lower, upper)

    (height, width) = cv_image.shape[:2]
    self.is_centered = False
    # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pad_exist = False
    if(contours is not None):
        if len(contours) != 0:
            cv2.drawContours(cv_image, contours, -1, 255, 3)

            c = max(contours, key = cv2.contourArea)
            if(cv2.contourArea(c) > 600):
                pad_exist=True
                x,y,w,h = cv2.boundingRect(c)
                # print(x, y, w, h)
                self.landing_cor_pub.publish(x+w//2, y+h//2)
                # self.camera.publishing(x+w//2-width//2,y+h//2-height//2,0)
                # draw the biggest contour (c) in green
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)

    self.detect_pad_pub.publish(pad_exist)
    cv2.imshow("detect pad", cv_image)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  lv = LandingVision()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)