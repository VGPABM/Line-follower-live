import numpy as np
import sys
from numpy.core.numeric import True_
import rospy
import cv2
import math
from sensor_msgs.msg import Image
from vision.msg import XY_cor_msg
from std_msgs.msg import Bool 
from cv_bridge import CvBridge, CvBridgeError

class GateVision:

  def __init__(self):
    rospy.init_node('landing_vision', anonymous=True)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/drone/front_camera/image_raw",Image,self.callback)
    self.tc_pub = rospy.Publisher('/tc', XY_cor_msg, queue_size=10)   #tc = target center position
    self.ctr_pub = rospy.Publisher('/ctr', Bool, queue_size=10)       #ctr  = centered boolean
    self.cntr_pub = rospy.Publisher('/cntr', Bool, queue_size=10)     #cntr = contour boolean
    self.area_ok_pub = rospy.Publisher('/areaok', Bool, queue_size=10)    #areaok = area is ok boolean
    self.gas_pub = rospy.Publisher('/gas', Bool, queue_size=10)           #gas = maju ke depan
    self.ori_pub = rospy.Publisher('/ori', XY_cor_msg, queue_size=10)     #ori = orientation
    self.pseudo_gate_pub = rospy.Publisher('/pseudo', XY_cor_msg, queue_size=10)     #pseudo = fake
    self.rate = rospy.Rate(20)
    self.rate.sleep()
    self.width = 360.0
    self.height = 640.0
    self.rodIg = False
    
    rospy.loginfo("Streaming gate")

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

    # white
    lower = np.array([0, 0, 180])
    upper = np.array([76, 255, 255])
    
    self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(self.hsv, lower, upper)

    (self.height, self.width) = cv_image.shape[:2]
    cv2.circle(cv_image, (int(640/2), int(360/2)), 1, (0, 0, 255), 5)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cntr_2 = sorted(contours, key = cv2.contourArea)

    self.tc = XY_cor_msg() # center of target (gate)
    self.gateBorder = XY_cor_msg()
    self.pseudoTC = XY_cor_msg()
    self.pseudoTC.x = 0
    self.pseudoTC.y = 0
    self.is_centered = False
    self.there_is_gate = False
    self.area_is_ok = False
    self.gasGan = False
    self.gateBorder.x = 0 #->  left height
    self.gateBorder.y = 0 #-> right height
    cv_image2 = cv_image[0:100, 0:100]      #init img
    
    
    if(contours is not None):
        if len(contours) >= 2:
            c = cntr_2[-2]
            the_area = cv2.contourArea(c)
            x,y,w, h = cv2.boundingRect(c)
            self.pseudoTC.x = x+w//2
            self.pseudoTC.y = y+h//2
            if(the_area > 11000):
                self.there_is_gate = True
                self.area_is_ok = the_area > 42000
                
                newbox = np.zeros(mask.shape)
                newbox = cv2.drawContours(newbox, [c], 0, 255, cv2.FILLED)
                newbox = newbox[y:y+h, x:x+w]
                hh, ww = newbox.shape
                startLeft = False
                endLeft=False
                endRight = False
                startRight = False
                tleft = 0
                tright = 0
                #Count panjang kiri dan kanan
                for i in range(0, hh-1):
                  #count left
                    if(not startLeft):
                        startLeft = newbox[i][5]==255
                    else:
                        if(not endLeft and newbox[i][5]==255):
                          tleft += 1
                        else:
                          endLeft = True
                    #count right
                    if(not startRight):
                        startRight = newbox[i][ww-5]==255
                    else:
                        if(not endRight and newbox[i][ww-5]==255):
                          tright += 1
                        else:
                          endRight = True
                    
                    if(endLeft and endRight):
                        break
                
                self.gateBorder.x = tleft
                self.gateBorder.y = tright
                
                cv_image2 = cv2.bitwise_not(mask[y: y+h, x:x+w])
                cv_image2 = cv_image2[h//2:h-1, 0:w]               
                newbox = newbox[h//2:h-1, w//2-w//4:w//2+w//4]
                h, w = newbox.shape
                newbox = newbox[15:h//2, 0:w]
                
                xc,yc,wc,hc = cv2.boundingRect(cv_image2)
                x1 = xc + int(wc/2)
                y1 = yc + int(hc/2)
                self.tc.x = float(x1+x)
                self.tc.y = float(y1+y+int(h))

                self.jarakx = abs(self.tc.x - 320)
                self.jaraky = abs(self.tc.y - 180)
                if self.jarakx < 20 and self.jaraky < 15:
                  self.is_centered = True

                #detect ada gate gerak
                if(not self.rodIg):
                    #belum ada rod
                    self.rodIg = np.count_nonzero(newbox==0) == 0 #cek rod
                elif(self.rodIg):
                    #gas!
                    self.gasGan = np.count_nonzero(newbox==0) > 0


    #reset rod boolean
    self.rodIg = self.rodIg and (self.there_is_gate and self.is_centered and self.area_is_ok)
    self.cntr_pub.publish(self.there_is_gate)
    self.ctr_pub.publish(self.is_centered)
    self.tc_pub.publish(self.tc)
    self.area_ok_pub.publish(self.area_is_ok)
    self.gas_pub.publish(self.gasGan)
    self.ori_pub.publish(self.gateBorder)
    self.pseudo_gate_pub.publish(self.pseudoTC)

    

def main(args):
  gv = GateVision()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)