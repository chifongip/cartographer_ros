#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu


imu_data = Imu()


def imuCallback(data):
    global imu_data
    imu_data = data
    imu_data.angular_velocity.x = imu_data.angular_velocity.x * 0.0174532925
    imu_data.angular_velocity.y = imu_data.angular_velocity.y * 0.0174532925
    imu_data.angular_velocity.z = imu_data.angular_velocity.z * 0.0174532925


def imuConvert():
    rospy.init_node('imuConvert', anonymous=True)

    rospy.Subscriber("imu/data", Imu, imuCallback)

    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    rate = rospy.Rate(50)

    rospy.loginfo("Converting IMU angular velocity from deg/s to rad/s.")
    
    while not rospy.is_shutdown():
        imu_pub.publish(imu_data)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        imuConvert()
    except rospy.ROSInterruptException:
        pass