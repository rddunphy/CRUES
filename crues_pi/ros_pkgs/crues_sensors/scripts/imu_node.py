#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

from crues import IMU


def publish_data(pub, imu):
    vel_msg = Twist()
    linear_speeds, ang_speeds = imu.get_speeds()
    (vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z) = ang_speeds
    (vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z) = linear_speeds
    rospy.loginfo("Gyro :" + ang_speeds + "  linear:" + linear_speeds)
    pub.publish(vel_msg)


def main():
    try:
        IMU_pub = rospy.Publisher('IMU_twist', Twist, queue_size=10)
        rospy.init_node("IMU")
        r = rospy.get_param('~rate', 5)
        rate = rospy.Rate(r)
        imu = IMU.IMU(0x68, r)
        while not rospy.is_shutdown():
            publish_data(IMU_pub, imu)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in ultrasonic node")
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
