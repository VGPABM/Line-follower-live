import queue
import cv2
import rospy
from std_msgs.msg import UInt8MultiArray

def nothing(x):
    pass


if __name__ == "__main__":
    # init all necessary ros
    rospy.init_node("trackbar_node")
    rate = rospy.Rate(20)
    track_pub = rospy.Publisher('/tb', UInt8MultiArray, queue_size=1, latch=True)
    values = UInt8MultiArray()
    # init trackbar open cv
    cv2.namedWindow("trackbar")
    cv2.createTrackbar('0', 'trackbar', 0, 255, nothing)
    cv2.createTrackbar('1', 'trackbar', 0, 255, nothing)
    cv2.createTrackbar('2', 'trackbar', 0, 255, nothing)
    cv2.createTrackbar('3', 'trackbar', 0, 255, nothing)
    cv2.createTrackbar('4', 'trackbar', 0, 255, nothing)
    cv2.createTrackbar('5', 'trackbar', 0, 255, nothing)

    values.data = [0, 0, 0, 0, 0, 0]
    while not rospy.is_shutdown():
        doPub = False
        for i in range(6):
            newdata = cv2.getTrackbarPos(f'{i}', "trackbar")
            olddata = values.data[i]
            if(olddata != newdata):
                values.data[i] = newdata
                doPub = True

        if(doPub):
            track_pub.publish(values)
        cv2.waitKey(1)
        rate.sleep()