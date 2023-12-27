#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import os
import time 


def image_callback(msg):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print(e)


def imageRecorder():
    rospy.init_node('image_listener')
    bridge = CvBridge()
    image_topic = "/usb_cam"
    rospy.Subscriber(image_topic, Image, image_callback)
    if not os.path.exists('img'):
        os.makedirs('img')
    count = 1

    while(1):
        filename = 'img/image{}.jpg'.format(count)
        cv2.imwrite(filename, cv_image)
        count += 1
        time.sleep(3)

    rospy.spin()


if __name__ == '__main__':
    try:
        imageRecorder()
    except rospy.ROSInterruptException:
        pass