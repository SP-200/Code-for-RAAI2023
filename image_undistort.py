#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        # 将ROS图像消息转换为OpenCV图像格式
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 在这里对彩色图像进行处理，例如显示图像
        cv2.imshow("Color Image", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node("color_image_subscriber", anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
