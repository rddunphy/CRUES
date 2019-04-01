#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

import RPi.GPIO as GPIO

from crues import IMU


def setup_imu_message((a_x, a_y, a_z), (roll_v, pitch_v, yaw_v)):
    msg = Imu()
    msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0]  # indicate orientation unknown
    msg.linear_acceleration = Vector3(a_y, -a_x, a_z)
    msg.angular_velocity = Vector3(pitch_v, -roll_v, yaw_v)
    return msg


def publish_data(pub, imu):
    linear_acc, ang_speeds = imu.read_values()
    msg = setup_imu_message(linear_acc, ang_speeds)
    rospy.loginfo("Gyro : %s  linear: %s" % (ang_speeds, linear_acc))
    pub.publish(msg)


def main():
    try:
        rospy.init_node("IMU")
        IMU_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
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
