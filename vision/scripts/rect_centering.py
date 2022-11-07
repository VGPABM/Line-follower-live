#!/usr/bin/env python3

#import roslib
#roslib.load_manifest('my_package')
import numpy as np
import math
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.rate = rospy.Rate(10) # 10hz
    self.center_pub = rospy.Publisher("/centered", Bool, queue_size=10)
    self.rate.sleep()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/drone/front_camera/image_raw",Image,self.callback)

    # cv2.namedWindow('test')
    # cv2.createTrackbar('thrs1', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs2', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs3', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs4', 'test', 180, 180, self.f)
    # cv2.createTrackbar('thrs5', 'test', 255, 255, self.f)
    # cv2.createTrackbar('thrs6', 'test', 100, 255, self.f)
    rospy.loginfo("Streaming")

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

    lower = np.array([0, 155, 192])
    upper = np.array([28, 255, 255])
    
    self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(self.hsv, lower, upper)
    res = cv2.bitwise_and(self.hsv, self.hsv, mask=mask)
    self.blur = cv2.GaussianBlur(res, (5,5), 0)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    self.ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
    cv2.imshow("lww", thresh)

    (a, b) = cv_image.shape[:2]
    self.is_centered = False
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > 4000:
            cv2.drawContours(cv_image, cnt, -1, (255, 0, 0), 2)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            x1 = x + int(w/2)
            y1 = y + int(h/2)
            cv2.circle(cv_image, (x1, y1), 1, (255, 0, 0), 5)
            # if x1 > int(b/2)-20 and x1 < int(b/2)+20 and y1 > int(a/2)-20 and y1 < int(a/2)+20:
            #     cv.putText(frame, "Centered (" + str(x1) + ", " + str(y1) + ")", (200,50), cv.FONT_HERSHEY_DUPLEX, 1, (0,0,0), 2)
            jarakx = int(x1 - b/2)
            jaraky = int(y1 - a/2)
            jarak = math.sqrt(jarakx**2 + jaraky**2)
            if jarak < 20:
                cv2.putText(cv_image, "Centered (" + str(x1) + ", " + str(y1) + ")", (200,50), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,0), 2)
                self.is_centered = True

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.center_pub.publish(self.is_centered)
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
    
