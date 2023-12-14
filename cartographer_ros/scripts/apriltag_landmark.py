#!/usr/bin/env python
import rospy
from tf import transformations

from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList

from apriltag_ros.msg import AprilTagDetectionArray


def landmarkCallback(data):
    landmark_pub = rospy.Publisher('landmark', LandmarkList, queue_size=10)


    landmark_data = LandmarkList()
    landmark_data.header = data.header

    if not data.detections:
        landmark_pub.publish(landmark_data)
    else:
        landmark_entry = LandmarkEntry()
        landmark_entry.id = str(data.detections[0].id)
        landmark_entry.tracking_from_landmark_transform.position = data.detections[0].pose.pose.pose.position
        landmark_entry.tracking_from_landmark_transform.orientation = data.detections[0].pose.pose.pose.orientation
        landmark_entry.translation_weight = 1.0
        landmark_entry.rotation_weight = 1.0    
        landmark_data.landmarks.append(landmark_entry)
        landmark_pub.publish(landmark_data)


def landmarkPublisher():
    rospy.init_node('landmarkPublisher', anonymous=True)

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, landmarkCallback)

    # rate = rospy.Rate(50)

    rospy.loginfo("Getting landmark from apriltag and publish to Cartographer.")
    
    rospy.spin()

    # while not rospy.is_shutdown():
        # landmark_pub.publish(landmark_data)
        # rate.sleep()
    

if __name__ == '__main__':
    try:
        landmarkPublisher()
    except rospy.ROSInterruptException:
        pass
