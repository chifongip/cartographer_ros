#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu


def imuCallback(data):
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

    imu_data = data
    imu_data.angular_velocity.x = imu_data.angular_velocity.x * 0.0174532925
    imu_data.angular_velocity.y = imu_data.angular_velocity.y * 0.0174532925
    imu_data.angular_velocity.z = imu_data.angular_velocity.z * 0.0174532925
    imu_pub.publish(imu_data)


def imuConvert():
    rospy.init_node('imuConvert', anonymous=True)

    rospy.Subscriber("imu/data", Imu, imuCallback)

    rospy.loginfo("Converting IMU angular velocity from deg/s to rad/s.")
    
    rospy.spin()
    

if __name__ == '__main__':
    try:
        imuConvert()
    except rospy.ROSInterruptException:
        pass
