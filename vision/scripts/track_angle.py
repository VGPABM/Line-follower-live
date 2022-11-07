#!/usr/bin/env python3

#import roslib
#roslib.load_manifest('my_package')
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int8, Bool
from vision.msg import XY_cor_msg
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.bridge = CvBridge()
    self.rate = rospy.Rate(20) # 10hz
    self.rate.sleep()

    self.image_sub          = rospy.Subscriber("/drone/down_camera/image_raw",Image,self.callback)
    self.angle_pub          = rospy.Publisher("/angle", Float32, queue_size=10)
    self.line_cor_pub       = rospy.Publisher("line_pos", XY_cor_msg, queue_size=10)
    self.move_assist_pub    = rospy.Publisher("/guide", Int8, queue_size=10)
    self.line_pub       = rospy.Publisher("/line", Bool, queue_size=10)

    # cv2.namedWindow('test')
    # cv2.createTrackbar('thrs1', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs2', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs3', 'test', 0, 255, self.f)
    # cv2.createTrackbar('thrs4', 'test', 180, 180, self.f)
    # cv2.createTrackbar('thrs5', 'test', 255, 255, self.f)
    # cv2.createTrackbar('thrs6', 'test', 100, 255, self.f)
    # cv2.createTrackbar('area', 'test', 0, 30000, self.f)

    rospy.loginfo("Streaming Angle")

  def f(self, x):
     pass

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      h, w, c = cv_image.shape
      cv_image = cv_image[0:h//2, 0:w]
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

    # lower = np.array([self.ilowh, self.ilows, self.ilowv])
    # upper = np.array([self.ihighh, self.ihighs, self.ihighv])

    # self.area_thrs = cv2.getTrackbarPos('area', 'test')
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 100])
    
    self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(self.hsv, lower, upper)
    res = cv2.bitwise_and(self.hsv, self.hsv, mask=mask)
    self.blur = cv2.GaussianBlur(res, (5,5), 0)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    self.ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
    # cv2.imshow("lww", thresh)

    cv2.circle(cv_image, (int(w/2), int(h/2)), 1, (0, 0, 255), 5)

    kiri_white = np.sum(thresh[0: h, 0:w//2])
    kanan_white = np.sum(thresh[0:h, w//2:w])
    if(kiri_white > kanan_white):
      #2 is kiri
      self.move_assist_pub.publish(2)
    elif(kiri_white < kanan_white):
      #1 is kanan
      self.move_assist_pub.publish(1)
    else:
      #0 is confused
      self.move_assist_pub.publish(0)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    angle = -1
    self.line = False
    if len(contours) > 0:
        cnt = max(contours, key=cv2.contourArea)
        # print(cv2.contourArea(cnt))
        if(cv2.contourArea(cnt) < 10000):
          # cnt = contours[-1]
          self.line = True
          (x, y), (w, h), angle = cv2.minAreaRect(cnt)
          # print(f'w, h = {w}, {h}')
          if w < h:
              angle += 180
          else:
              angle += 90
          cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 3)
 
        cv2.circle(cv_image, (int(w//2), int(h//2)), radius=0, color=(0,0,255), thickness=-1)
        # cv2.rectangle(cv_image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (0, 0, 255), 2)
        cv2.putText(cv_image, str(angle), (200, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 0), 2)

    self.line_pub.publish(self.line)
    cv2.imshow("Track_Angle", cv_image)
    cv2.waitKey(3)

    try:
      self.angle_pub.publish(angle)
    except CvBridgeError as e:
      print(e)
    try:
      self.line_cor_pub.publish(x,y)
    except UnboundLocalError as e:
      pass

def main(args):
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
    
