from ctypes import *
import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lanedetector import LaneDetector

bridge = CvBridge()
detector = LaneDetector()

def callback(image_msg):
    try:
        img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img = detector.detect_img(img)
        cv2.imshow("img", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            sys.exit
    except CvBridgeError as e:
        print(e)


if __name__ == "__main__":
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback, queue_size=1)
    rospy.init_node('cameraYOLO')

    rospy.spin()
