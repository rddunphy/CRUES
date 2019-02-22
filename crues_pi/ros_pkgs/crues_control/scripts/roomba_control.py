#!/usr/bin/env python

import random
import time

import rospy
from std_msgs.msg import Int32
from crues_actuators.msg import MotorControl


pub = rospy.Publisher('motor_control', MotorControl, queue_size=10)
time_to_stop_turning = None


def _mc_msg(ls, rs):
    msg = MotorControl()
    msg.l_speed = ls
    msg.r_speed = rs
    msg.slp = False
    return msg


def handle_new_range(range):
    rospy.loginfo("Roomba received new ultrasonic range: %d" % range.data)
    global time_to_stop_turning
    if range.data > 120:
        pub.publish(_mc_msg(25, 25))
    elif not time_to_stop_turning or time.time() < time_to_stop_turning:
        time_to_stop_turning = time.time() + random.uniform(0.3, 0.6)
        pub.publish(_mc_msg(18, -18))


def main():
    try:
        rospy.init_node('roomba')
        rospy.Subscriber('uc_range', Int32, handle_new_range)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in node 'roomba'")
        rospy.logerr(e)


if __name__ == '__main__':
    main()
