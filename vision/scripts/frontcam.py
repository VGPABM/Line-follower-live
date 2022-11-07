from math import sqrt
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray, UInt8
# custom ros msg, for detail see: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
from vision.msg import RotatedRect

class MaskTool:
    # will get called when the class is created
    def __init__(self) -> None:
        rospy.init_node('landing_vision', anonymous=True)
        # helper to convert sensor_msgs.Image to opencv/numpy array
        self.bridge = CvBridge()
        
        # subscribe to front camera data, this will call self.callback with rosspin and will be the main 'loop' to detect stuff
        self.image_sub = rospy.Subscriber("/drone/front_camera/image_raw",Image,self.callback)
        
        # publisher for vertical gate data as 'RotatedRect' msg containing center x,y, width, height, angle (angle ga kepake)
        self.vgate_pub = rospy.Publisher("/gate/vertical/data", RotatedRect, queue_size=10)
        
        # publisher for panjang sisi tegak vertical gate (panjang sisi kiri + kanan), to align drone prespective to gate
        self.vgate_help1_pub = rospy.Publisher("/gate/vertical/help/1", Float32MultiArray, queue_size=10)
        
        # self.vgate_help2_pub = rospy.Publisher("/gate/vertical/help/2", UInt8, queue_size=10)
        # self.trackbar_sub = rospy.Subscriber("/tb",UInt8MultiArray,self.trackback)
        self.tbval = [0,0,0,0,0,0]  # ga kepake fornow, usefull for nyoba nyoba pakek trackbar
        
        # msg holder for vertical gate data
        self.vgate_msg = RotatedRect()
        
        # self.vgate_help2 = UInt8()
        # self.vgate_help2.data = 0
        
        # msg holder for vertical gate help
        self.vgate_help1 = Float32MultiArray() # panjang gate kiri kanannya
        self.vgate_help1.data = [0.0, 0.0]
        
        # threshold hsv to mask magenta gate (dapet hasil nyoba nyoba threshold pakek trackbar)
        self.lower = np.array([127, 127, 127])
        self.upper = np.array([255, 255, 255])

        self.prev_area = 0
        self.default_area = 15000
        self.thres_area = 0

        self.rate = rospy.Rate(32)
        self.rate.sleep()

    # callback utk trackbar, tapi ga kepake krg
    def trackback(self, data):
        # self.tbval = data.data
        self.tbval = data.data

    # callback utk data camera depan, serve as main 'loop'
    def callback(self, data):
        # reset msg
        self.vgate_msg.is_exist = False
        # self.vgate_help2.data = 0
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(f'error: {e}')
            return

        # convert to hsv for easier masking
        self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # mask the hsv according to the threshold
        mask = cv2.inRange(self.hsv, self.lower, self.upper)
        # cv2.imshow('mask', mask)
        # (height, width) = cv_image.shape[:2]
        
        # get all the contour in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # print(f'threas_area: {self.thres_area}, prev_area: {self.prev_area}')
        # check if we got contour, alternative writing: if contours:
        if(len(contours) > 0):
            # get contour with max area
            max_area_cnt = max(contours, key=cv2.contourArea)
            # the that contour area
            area = cv2.contourArea(max_area_cnt)
            # check if the area is big enough to be considered as detection

            if self.prev_area == 0:
                self.thres_area = self.default_area
            else:
                self.thres_area = self.prev_area

            if(area > min(self.thres_area, 15000)):
                # get the rect data that bound the contour
                self.prev_area = area - 1500
                x,y,w,h = cv2.boundingRect(max_area_cnt)# minareaRect ga beda jauh, sama ae, gate is already kotak
                # print(f'cnt_are: {area} vs bbox area: {w*h}')
                # estimate the shape of the contour
                epsilon = 0.05*cv2.arcLength(max_area_cnt,True)         # see documentation for full explanation
                approx = cv2.approxPolyDP(max_area_cnt, epsilon, True)  # this is approximation, return list of points in the polygon with shape (n, 1, 2)
                cv2.drawContours(cv_image, [approx], 0, (255, 128, 64), 2)  # 0 is the id, of box in [box]

                # sort the list of points based on their x value to ensure we get ordered point 
                # from top-left, bottom-left, top-right, bottom-right
                approx = sorted(approx, key=lambda c: c[0][0]) 
                # print(np.array(approx).shape)
                total_points= len(approx)
                if(total_points >= 4):
                    self.vgate_msg.is_exist = True
                    self.vgate_msg.x = x + w//2
                    self.vgate_msg.y = y + h//2
                    self.vgate_msg.w = w
                    self.vgate_msg.h = h
                    # cv2.line(cv_image, approx[0][0], approx[1][0], (128, 255, 64), 2, cv2.LINE_AA)
                    # cv2.line(cv_image, approx[2][0], approx[3][0], (128, 255, 64), 2, cv2.LINE_AA)
                    # calculate panjang sisi kiri lewat numpy, cause (belum ditest) but i believe numpy is faaaast ( this is simple distance calc btw)
                    llen = np.linalg.norm(approx[0][0] - approx[1][0]) * (total_points == 4) + 1
                    # same as above
                    rlen = np.linalg.norm(approx[2][0] - approx[3][0]) * (total_points == 4) + 1
                    
                    # store the results
                    self.vgate_help1.data = [llen, rlen]
                    
                    cv2.putText(cv_image, f'{y:.0f}|{abs(llen-rlen):.0f}', (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (32, 32, 255), 2, cv2.LINE_AA)
                    cv2.circle(cv_image, (self.vgate_msg.x, self.vgate_msg.y), 2, (0, 0, 0), -1)
                    cv2.circle(cv_image, (320, 180), 3, (0, 0, 255), -1)
                    # cv2.putText(cv_image, f'{self.vgate_msg.x:.2f}|{self.vgate_msg.y:.2f}', (int(self.vgate_msg.x), int(self.vgate_msg.y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (32, 32, 255), 2, cv2.LINE_AA)
                    # print(f'{llen} vs {rlen}')
                    # dx, dy = approx[0][0][0] - approx[1][0][0], approx[0][0][1] - approx[1][0][1]
                    # print(sqrt(dx*dx + +dy*dy)) # proven sama
                # elif(total_points==3):
                #     self.vgate_msg.is_exist = True
                #     self.vgate_help2.data = 0 # a.k.a mundur cuy
                #     print("mundur?")
                # else:
                #     # banyak sisi polygon != 4, ... sus, is it a gate?, hmm
                #     print(f"approx is: {len(approx)}, not 4 cant do help")'
            else:
                self.prev_area = 0
        else:
            self.prev_area = 0

        self.vgate_pub.publish(self.vgate_msg)
        self.vgate_help1_pub.publish(self.vgate_help1)
        # self.vgate_help2_pub.publish(self.vgate_help2)
        cv2.imshow(f'orii', cv_image)
        cv2.waitKey(1)



if __name__=="__main__":
    maskingTool = MaskTool()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
