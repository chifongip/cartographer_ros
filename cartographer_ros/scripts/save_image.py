#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import os

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print(e)

    # Save the image
    filename = 'img/image{}.jpg'.format(image_callback.count)
    cv2.imwrite(filename, cv_image)
    image_callback.count += 1

if __name__ == '__main__':
    rospy.init_node('image_listener')
    bridge = CvBridge()
    image_topic = "/usb_cam"
    rospy.Subscriber(image_topic, Image, image_callback)
    if not os.path.exists('img'):
        os.makedirs('img')
    image_callback.count = 1
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        rospy.spin_once()
        rate.sleep()