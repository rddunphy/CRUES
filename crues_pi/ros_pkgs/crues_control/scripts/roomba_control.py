#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from crues_actuators.msg import MotorControl


pub = rospy.Publisher('motor_control', MotorControl, queue_size=10)


def _move_msg():
    msg = MotorControl()
    msg.l_speed = 50
    msg.r_speed = 50
    msg.slp = False
    return msg


def _stop_msg():
    msg = MotorControl()
    msg.l_speed = 0
    msg.r_speed = 0
    msg.slp = False
    return msg


def handle_new_range(range):
    if range.data > 50:
        pub.publish(_move_msg())
    else:
        pub.publish(_stop_msg())


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
