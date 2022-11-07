#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from math import atan2, acos, pi

        # double siny_cosp = 2 * (w * z + x * y);
        # double cosy_cosp = 1 - 2 * (y * y + z * z);
        # double yaw = std::atan2(siny_cosp, cosy_cosp); 
        # return yaw;

def getYawFromQuarternion(w, x, y, z):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return yaw

def radian2degree(rad):
    return rad*180/pi

def callback(data):
    w = data.orientation.w
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    yaw = getYawFromQuarternion(w, x, y, z)
    # yaw = radian2degree(yaw)
    # if(yaw <= 0):
    #     yaw+=180
    # else:
    #     yaw -=180
    rospy.loginfo(f'calculated yaw: {yaw}')
    # rospy.loginfo("%f %f",z, w)

def main():
    rospy.init_node('tes_yaw', anonymous=True)
    sub = rospy.Subscriber("/drone/gt_pose",Pose, callback, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(32)
    try:
        rospy.spin()
        rate.sleep()
    except rospy.ROSInterruptException as e:
        print(e)


if __name__ == '__main__':
    main()