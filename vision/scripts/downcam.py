from copy import copy
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from vision.msg import RotatedRect
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# ga kepake
def predict(z):
    m = -12.37784
    b = 66.13229
    min_side = m*z + b
    return min_side

# extract line using minArearect, ada cara lain like manual using atan2, or something called PCA Analysis
def extract_line(bw, draw_image):
        # print(reversed.shape)
        contours, hierarchy = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        is_exist, x, y, angle = False, 0, 0, 0
        if(len(contours) > 0):
            max_area = -1
            max_id = -1
            # do filtering
            for i,cnt in enumerate(contours):
                # estimate the shape of the contour
                epsilon = 0.01*cv2.arcLength(cnt,True)         # see documentation for full explanation
                approx = cv2.approxPolyDP(cnt, epsilon, True)  # this is approximation, return list of points in the polygon with shape (n, 1, 2)
                
                if(len(approx)<4):
                    continue

                # len is 4, possibly a line
                is_exist=True
                # print("ada")
                area = cv2.contourArea(cnt)
                if(area > max_area):
                    max_area=area
                    max_id=i

            # ===== end of for loop ======= #
            if not is_exist:
                return is_exist, x, y, angle

            (x, y), (w, h), angle = cv2.minAreaRect(contours[max_id])
            # this convert the range of minAreaRect angle from [-90,0) to [180,0) (kaya grafik normal)
            if w < h:
                angle = 90-angle
            else:
                angle = 180 - angle
            
            # this piece of codes have to be activated if you use opencv 4.2.0
            # angle += 180 *(angle<90)
            angle %= 180

            # hm, so perfect, so sus
            if(angle == 180 or angle==0):
                is_exist = False
            cv2.drawContours(draw_image, contours, max_id, (128, 128, 0), 2)  # 0 is the id, of box in [box]
            cv2.putText(draw_image, f'{angle:.4f}', (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return is_exist, x, y, angle

class CamDown:
    def __init__(self) -> None:
        rospy.init_node('landing_vision', anonymous=True)
        self.bridge = CvBridge()
        # subscribe data camera bawah
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        # ga kepake, soalnya udah dapet valuenya
        # self.lb_sub = rospy.Subscriber("/lb",UInt8,self.trackbarlb)
        # self.hb_sub = rospy.Subscriber("/hb",UInt8,self.trackbarhb)
        
        # publisher for garis yg didetect di upper half of the frame, contain x,y,w,h,angle (yg dipake cuma y, sama angle)
        self.uline_pub = rospy.Publisher("/upper/line_data", RotatedRect, queue_size=10)
        # publisher for garis yg didetect di lower half of the frame
        self.lline_pub = rospy.Publisher("/lower/line_data", RotatedRect, queue_size=10)
        # threshold grayscale to detect garis
        self.lb = 100
        self.hb = 255
        # msgs holder
        self.uline_msg = RotatedRect()
        self.lline_msg = RotatedRect()
        self.rate = rospy.Rate(32)
        self.rate.sleep()
        
    def trackbarlb(self, data):
        self.lb = data.data
        # print(self.lb)

    def trackbarhb(self, data):
        self.hb = data.data

    # extract the line on a img and publish it
    def extract_line_and_pub(self, img, draw_image, msg, publisher):
        is_exist, x, y, angle = extract_line(img, draw_image)
        # print(f'got: {valid_cnt} contours')
        msg.is_exist = is_exist
        msg.x =x
        msg.y =y
        msg.angle =angle
        publisher.publish(msg)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(f'error: {e}')
            return

        # convert to grayscale
        grayscaled = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow(f'greyscaled',grayscaled)
        ret, binary = cv2.threshold(grayscaled, self.lb, self.hb, cv2.THRESH_BINARY)
        reversed = cv2.bitwise_not(binary)
        # cv2.imshow(f'bw-rev', reversed)

        # bagi 2
        h, w = reversed.shape
        half = h//2
        upper, lower = copy(reversed), copy(reversed)
        # paint black to lower part in upper
        upper[half:h,:] = 0
        # same, but reversed
        lower[0:half,:] = 0
        self.extract_line_and_pub(upper, cv_image, self.uline_msg, self.uline_pub)
        self.extract_line_and_pub(lower, cv_image, self.lline_msg, self.lline_pub)
        cv2.imshow(f'ori', cv_image)
        # cv2.imshow(f'img dim:{cv_image.shape}', cv_image)
        cv2.waitKey(1)



if __name__=="__main__":
    maskingTool = CamDown()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
