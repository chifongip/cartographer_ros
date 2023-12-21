#!/usr/bin/env python
import rospy
import cv2 as cv 
import numpy as np 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()


def imageCallback(data):
    segmentation_pub = rospy.Publisher('image_segmentation', Image, queue_size=10)

    img = bridge.imgmsg_to_cv2(data)

    edges = cv.Canny(img, 100, 200)

    # # gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # ret, thresh = cv.threshold(img, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)

    # # noise removal
    # kernel = np.ones((3, 3), np.uint8)
    # opening = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel, iterations=2)
    # # sure background area
    # sure_bg = cv.dilate(opening, kernel, iterations=3)
    # # Finding sure foreground area
    # dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 5)
    # ret, sure_fg = cv.threshold(dist_transform, 0.7*dist_transform.max(), 255,0)
    # # Finding unknown region
    # sure_fg = np.uint8(sure_fg)
    # unknown = cv.subtract(sure_bg, sure_fg)

    # # Marker labelling
    # ret, markers = cv.connectedComponents(sure_fg)
    # # Add one to all labels so that sure background is not 0, but 1
    # markers = markers + 1
    # # Now, mark the region of unknown with zero
    # markers[unknown==255] = 0

    # img_bgr = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    # markers = cv.watershed(img_bgr, markers)
    
    # img_result = img.copy()
    # img_result[markers==-1] = [255]

    segmentation_pub.publish(bridge.cv2_to_imgmsg(edges))




def imagePublisher():
    rospy.init_node('imagePublisher', anonymous=True)

    rospy.Subscriber("usb_cam/image_rect", Image, imageCallback)

    # rate = rospy.Rate(50)

    rospy.loginfo("Start image segmentation.")
    
    rospy.spin()

    # while not rospy.is_shutdown():
        # landmark_pub.publish(landmark_data)
        # rate.sleep()
    

if __name__ == '__main__':
    try:
        imagePublisher()
    except rospy.ROSInterruptException:
        pass
