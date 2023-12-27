#!/usr/bin/env python
import rospy
import tf
import numpy as np 

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry


tf_ros = tf.TransformerROS()

def odomCallback(data):
    global curr_linear_vel_x, curr_angular_vel_z
    curr_linear_vel_x = data.twist.twist.linear.x
    curr_angular_vel_z = data.twist.twist.angular.z


def poseCallback(data): 
    if data.detections:
        map_base_link_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

        map_base_link_data = PoseWithCovarianceStamped()
        map_base_link_data.header.seq = data.header.seq
        map_base_link_data.header.stamp = data.header.stamp
        map_base_link_data.header.frame_id = "map"

        # tag w.r.t. map
        tag_detected = []
        for i in range(len(data.detections)):
            # distance between tag and base_link
            if data.detections[i].pose.pose.pose.position.z <= max_detection_dist: 
                tag_detected.append(data.detections[i])
        
        # linear and angular velocity threshold for updating the robot's pose  
        if tag_detected and curr_linear_vel_x <= max_linear_vel_x and curr_angular_vel_z <= max_angular_vel_z:
            tag_sorted = sorted(tag_detected, key=lambda tag_detected: tag_detected.pose.pose.pose.position.z)

            for tag in tag_locations:
                if tag["id"] == int(''.join(map(str, tag_sorted[0].id))): 
                    break
            map_tag_g = tf_ros.fromTranslationRotation([tag["x"], tag["y"], tag["z"]], [tag["qx"], tag["qy"], tag["qz"], tag["qw"]]) 
            
            # tag w.r.t. usb_cam_link
            usb_cam_link_tag_t = [tag_sorted[0].pose.pose.pose.position.x, tag_sorted[0].pose.pose.pose.position.y, tag_sorted[0].pose.pose.pose.position.z]
            usb_cam_link_tag_R = [tag_sorted[0].pose.pose.pose.orientation.x, tag_sorted[0].pose.pose.pose.orientation.y, tag_sorted[0].pose.pose.pose.orientation.z, tag_sorted[0].pose.pose.pose.orientation.w]
            usb_cam_link_tag_g = tf_ros.fromTranslationRotation(usb_cam_link_tag_t, usb_cam_link_tag_R)

            # usb_cam w.r.t. base_link
            base_link_usb_cam_link_g = tf_ros.fromTranslationRotation(base_link_usb_cam_link_t, base_link_usb_cam_link_R)

            # calculation of base_link w.r.t. map
            tag_usb_cam_link_g = np.linalg.inv(usb_cam_link_tag_g)                  # usb_cam_link w.r.t. tag
            usb_cam_link_base_link_g = np.linalg.inv(base_link_usb_cam_link_g)      # base_link w.r.t. usb_cam_link

            map_base_link_g = np.matmul(map_tag_g, np.matmul(tag_usb_cam_link_g, usb_cam_link_base_link_g))

            # get rotation and translation matrix from transformation matrix 
            map_base_link_t = map_base_link_g[:3, 3]
            # map_base_link_R = map_base_link_g[:3, :3]
            map_base_link_q = tf.transformations.quaternion_from_matrix(map_base_link_g)

            map_base_link_data.pose.pose.position.x = map_base_link_t[0]
            map_base_link_data.pose.pose.position.y = map_base_link_t[1]
            map_base_link_data.pose.pose.position.z = map_base_link_t[2]
            map_base_link_data.pose.pose.orientation.x = map_base_link_q[0]
            map_base_link_data.pose.pose.orientation.y = map_base_link_q[1]
            map_base_link_data.pose.pose.orientation.z = map_base_link_q[2]
            map_base_link_data.pose.pose.orientation.w = map_base_link_q[3]

            map_base_link_pub.publish(map_base_link_data)


def posePublisher():
    global tag_locations, max_detection_dist, base_link_usb_cam_link_t, base_link_usb_cam_link_R, max_linear_vel_x, max_angular_vel_z
    
    tag_locations = rospy.get_param("update_robot_pose/tag_locations")    # get tag locations from yaml 
    max_detection_dist = rospy.get_param("update_robot_pose/max_detection_dist")
    max_linear_vel_x = rospy.get_param("update_robot_pose/max_linear_vel_x")
    max_angular_vel_z = rospy.get_param("update_robot_pose/max_angular_vel")

    rospy.init_node('posePublisher', anonymous=True)

    tf_listener = tf.TransformListener()

    # get transformation between base_link and usb_cam_link from tf_tree
    tf_listener.waitForTransform("base_link", "usb_cam_link", rospy.Time(0), rospy.Duration(1.0))
    (base_link_usb_cam_link_t, base_link_usb_cam_link_R) = tf_listener.lookupTransform("base_link", "usb_cam_link", rospy.Time(0))

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, poseCallback)
    rospy.Subscriber("odom", Odometry, odomCallback)

    rospy.loginfo("Getting transformation from base_link to map.")
    
    rospy.spin()
    

if __name__ == '__main__':
    try:
        posePublisher()
    except rospy.ROSInterruptException:
        pass
