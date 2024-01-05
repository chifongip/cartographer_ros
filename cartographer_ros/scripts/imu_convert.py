#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

class imuConvert: 
    def __init__(self):
        self.imu_sub = rospy.Subscriber("imu/data", Imu, self.imuCallback, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)


    def imuCallback(self, data):
        imu_data = data
        imu_data.angular_velocity.x = imu_data.angular_velocity.x * 0.0174532925
        imu_data.angular_velocity.y = imu_data.angular_velocity.y * 0.0174532925
        imu_data.angular_velocity.z = imu_data.angular_velocity.z * 0.0174532925
        self.imu_pub.publish(imu_data)


if __name__ == '__main__':
    try:
        rospy.init_node('imuConvert', anonymous=True)
        imuConvert = imuConvert()
        rospy.loginfo("Converting IMU angular velocity from deg/s to rad/s.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
