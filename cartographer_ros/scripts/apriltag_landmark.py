#!/usr/bin/env python
import rospy
import tf
import numpy as np 

from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose


def landmarkCallback(data):
    landmark_pub = rospy.Publisher('landmark', LandmarkList, queue_size=10)
    tf_ros = tf.TransformerROS()

    landmark_data = LandmarkList()
    landmark_data.header.seq = data.header.seq
    landmark_data.header.stamp = data.header.stamp
    landmark_data.header.frame_id = "imu_link"

    if not data.detections:
        landmark_pub.publish(landmark_data)
    else:
        for i in range(len(data.detections)):
            landmark_entry = LandmarkEntry()
            landmark_entry.id = str(data.detections[i].id)

            usb_cam_link_tag_t = [data.detections[i].pose.pose.pose.position.x, data.detections[i].pose.pose.pose.position.y, data.detections[i].pose.pose.pose.position.z]
            usb_cam_link_tag_R = [data.detections[i].pose.pose.pose.orientation.x, data.detections[i].pose.pose.pose.orientation.y, data.detections[i].pose.pose.pose.orientation.z, data.detections[i].pose.pose.pose.orientation.w]
            usb_cam_link_tag_g = tf_ros.fromTranslationRotation(usb_cam_link_tag_t, usb_cam_link_tag_R)
            imu_link_usb_cam_link_g = tf_ros.fromTranslationRotation([0.158, 0.000, 0.028], [-0.500, 0.500, -0.500, 0.500])
            
            imu_link_tag_g = np.matmul(imu_link_usb_cam_link_g, usb_cam_link_tag_g)

            imu_link_tag_t = imu_link_tag_g[:3, 3]
            # imu_link_tag_R = imu_link_tag_g[:3, :3]
            imu_link_tag_q = tf.transformations.quaternion_from_matrix(imu_link_tag_g)

            landmark_entry.tracking_from_landmark_transform.position.x = imu_link_tag_t[0]
            landmark_entry.tracking_from_landmark_transform.position.y = imu_link_tag_t[1]
            landmark_entry.tracking_from_landmark_transform.position.z = imu_link_tag_t[2]
            landmark_entry.tracking_from_landmark_transform.orientation.x = imu_link_tag_q[0]
            landmark_entry.tracking_from_landmark_transform.orientation.y = imu_link_tag_q[1]
            landmark_entry.tracking_from_landmark_transform.orientation.z = imu_link_tag_q[2]
            landmark_entry.tracking_from_landmark_transform.orientation.w = imu_link_tag_q[3]

            landmark_entry.translation_weight = 10.0
            landmark_entry.rotation_weight = 5.0    
            landmark_data.landmarks.append(landmark_entry)

        landmark_pub.publish(landmark_data)


    '''
        convert tag w.r.t. base_link
    '''
    # base_link_tag_pub = rospy.Publisher('base_link_tag', PoseStamped, queue_size=10)
    # tf_ros = tf.TransformerROS()

    # base_link_tag_data = PoseStamped()
    # base_link_tag_data.header.seq = data.header.seq
    # base_link_tag_data.header.stamp = data.header.stamp
    # base_link_tag_data.header.frame_id = "base_link"

    # if not data.detections:
    #     base_link_tag_pub.publish(base_link_tag_data)
    # else:
    #     print(len(data.detections))

    #     usb_cam_link_tag_t = [data.detections[0].pose.pose.pose.position.x, data.detections[0].pose.pose.pose.position.y, data.detections[0].pose.pose.pose.position.z]
    #     usb_cam_link_tag_R = [data.detections[0].pose.pose.pose.orientation.x, data.detections[0].pose.pose.pose.orientation.y, data.detections[0].pose.pose.pose.orientation.z, data.detections[0].pose.pose.pose.orientation.w]
    #     usb_cam_link_tag_g = tf_ros.fromTranslationRotation(usb_cam_link_tag_t, usb_cam_link_tag_R)
    #     base_link_usb_cam_link_g = tf_ros.fromTranslationRotation([0.220, 0.000, 0.118], [-0.500, 0.500, -0.500, 0.500])
        
    #     base_link_tag_g = np.matmul(base_link_usb_cam_link_g, usb_cam_link_tag_g)

    #     base_link_tag_t = base_link_tag_g[:3, 3]
    #     # base_link_tag_R = base_link_tag_g[:3, :3]
    #     base_link_tag_q = tf.transformations.quaternion_from_matrix(base_link_tag_g)

    #     base_link_tag_data.pose.position.x = base_link_tag_t[0]
    #     base_link_tag_data.pose.position.y = base_link_tag_t[1]
    #     base_link_tag_data.pose.position.z = base_link_tag_t[2]
    #     base_link_tag_data.pose.orientation.x = base_link_tag_q[0]
    #     base_link_tag_data.pose.orientation.y = base_link_tag_q[1]
    #     base_link_tag_data.pose.orientation.z = base_link_tag_q[2]
    #     base_link_tag_data.pose.orientation.w = base_link_tag_q[3]

    #     base_link_tag_pub.publish(base_link_tag_data)


def landmarkPublisher():
    rospy.init_node('landmarkPublisher', anonymous=True)

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, landmarkCallback)

    # rate = rospy.Rate(50)

    rospy.loginfo("Getting landmark from apriltag and publish to Cartographer.")
    # rospy.loginfo("Getting transformation from tag to base_link.")
    
    rospy.spin()

    # while not rospy.is_shutdown():
        # landmark_pub.publish(landmark_data)
        # rate.sleep()
    

if __name__ == '__main__':
    try:
        landmarkPublisher()
    except rospy.ROSInterruptException:
        pass
