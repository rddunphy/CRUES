#!/usr/bin/env python

import random
import time

import rospy
from std_msgs.msg import Int32
from crues_actuators.msg import MotorControl


pub = rospy.Publisher('motor_control', MotorControl, queue_size=10)


def _mc_msg(ls, rs):
    msg = MotorControl()
    msg.l_speed = ls
    msg.r_speed = rs
    msg.slp = False
    return msg


def handle_new_range(range):
    if range.data > 150:
        pub.publish(_mc_msg(25, 25))
    if range.data > 80:
        pub.publish(_mc_msg(10, 10))
    else:
        pub.publish(_mc_msg(25, -25))
        time.sleep(random.uniform(0.5, 1.3))


def main():
    try:
        print "Running Roomba control node"
        rospy.init_node('roomba')
        rospy.Subscriber('uc_range', Int32, handle_new_range)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        print "ROSInterruptException in node 'roomba'"


if __name__ == '__main__':
    main()
