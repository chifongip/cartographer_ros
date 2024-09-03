#!/usr/bin/env python
import rospy
import tf
import numpy as np 

from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose


class landmarkPublisher:
    def __init__(self):
        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.landmarkCallback, queue_size=1)
        self.landmark_pub = rospy.Publisher('landmark', LandmarkList, queue_size=1)
        self.tf_ros = tf.TransformerROS()


    def landmarkCallback(self, data):
        # # tracking_frame: imu_link
        # landmark_data = LandmarkList()
        # landmark_data.header.stamp = data.header.stamp
        # landmark_data.header.frame_id = "imu_link"

        # if not data.detections:
        #     self.landmark_pub.publish(landmark_data)
        # else:
        #     for i in range(len(data.detections)):
        #         landmark_entry = LandmarkEntry()
        #         landmark_entry.id = str(data.detections[i].id)

        #         usb_cam_link_tag_t = [data.detections[i].pose.pose.pose.position.x, data.detections[i].pose.pose.pose.position.y, data.detections[i].pose.pose.pose.position.z]
        #         usb_cam_link_tag_R = [data.detections[i].pose.pose.pose.orientation.x, data.detections[i].pose.pose.pose.orientation.y, data.detections[i].pose.pose.pose.orientation.z, data.detections[i].pose.pose.pose.orientation.w]
        #         usb_cam_link_tag_g = self.tf_ros.fromTranslationRotation(usb_cam_link_tag_t, usb_cam_link_tag_R)
        #         imu_link_usb_cam_link_g = self.tf_ros.fromTranslationRotation([0.158, 0.000, 0.028], [-0.500, 0.500, -0.500, 0.500])
                
        #         imu_link_tag_g = np.matmul(imu_link_usb_cam_link_g, usb_cam_link_tag_g)

        #         imu_link_tag_t = imu_link_tag_g[:3, 3]
        #         # imu_link_tag_R = imu_link_tag_g[:3, :3]
        #         imu_link_tag_q = tf.transformations.quaternion_from_matrix(imu_link_tag_g)

        #         landmark_entry.tracking_from_landmark_transform.position.x = imu_link_tag_t[0]
        #         landmark_entry.tracking_from_landmark_transform.position.y = imu_link_tag_t[1]
        #         landmark_entry.tracking_from_landmark_transform.position.z = imu_link_tag_t[2]
        #         landmark_entry.tracking_from_landmark_transform.orientation.x = imu_link_tag_q[0]
        #         landmark_entry.tracking_from_landmark_transform.orientation.y = imu_link_tag_q[1]
        #         landmark_entry.tracking_from_landmark_transform.orientation.z = imu_link_tag_q[2]
        #         landmark_entry.tracking_from_landmark_transform.orientation.w = imu_link_tag_q[3]

        #         landmark_entry.translation_weight = 10.0
        #         landmark_entry.rotation_weight = 10.0    
        #         landmark_data.landmarks.append(landmark_entry)

        #     self.landmark_pub.publish(landmark_data)


        # tracking_frame: base_link
        landmark_data = LandmarkList()
        landmark_data.header.stamp = data.header.stamp
        landmark_data.header.frame_id = "base_link"

        if not data.detections:
            self.landmark_pub.publish(landmark_data)
        else:
            for i in range(len(data.detections)):
                landmark_entry = LandmarkEntry()
                landmark_entry.id = str(data.detections[i].id)

                usb_cam_link_tag_t = [data.detections[i].pose.pose.pose.position.x, data.detections[i].pose.pose.pose.position.y, data.detections[i].pose.pose.pose.position.z]
                usb_cam_link_tag_R = [data.detections[i].pose.pose.pose.orientation.x, data.detections[i].pose.pose.pose.orientation.y, data.detections[i].pose.pose.pose.orientation.z, data.detections[i].pose.pose.pose.orientation.w]
                usb_cam_link_tag_g = self.tf_ros.fromTranslationRotation(usb_cam_link_tag_t, usb_cam_link_tag_R)
                base_link_usb_cam_link_g = self.tf_ros.fromTranslationRotation([0.220, 0.000, 0.118], [-0.500, 0.500, -0.500, 0.500])
                
                base_link_tag_g = np.matmul(base_link_usb_cam_link_g, usb_cam_link_tag_g)

                base_link_tag_t = base_link_tag_g[:3, 3]
                # base_link_tag_R = base_link_tag_g[:3, :3]
                base_link_tag_q = tf.transformations.quaternion_from_matrix(base_link_tag_g)

                landmark_entry.tracking_from_landmark_transform.position.x = base_link_tag_t[0]
                landmark_entry.tracking_from_landmark_transform.position.y = base_link_tag_t[1]
                landmark_entry.tracking_from_landmark_transform.position.z = base_link_tag_t[2]
                landmark_entry.tracking_from_landmark_transform.orientation.x = base_link_tag_q[0]
                landmark_entry.tracking_from_landmark_transform.orientation.y = base_link_tag_q[1]
                landmark_entry.tracking_from_landmark_transform.orientation.z = base_link_tag_q[2]
                landmark_entry.tracking_from_landmark_transform.orientation.w = base_link_tag_q[3]

                landmark_entry.translation_weight = 10.0
                landmark_entry.rotation_weight = 10.0    
                landmark_data.landmarks.append(landmark_entry)

            self.landmark_pub.publish(landmark_data)
    

if __name__ == '__main__':
    try:
        rospy.init_node('landmarkPublisher', anonymous=True)
        landmarkPublisher = landmarkPublisher()
        rospy.loginfo("Getting landmark from apriltag and publish to Cartographer.")
        # rospy.loginfo("Getting transformation from tag to base_link.")  
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
