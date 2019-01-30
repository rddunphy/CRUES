#!/usr/bin/env python

import rospy
from motor_control.msg import MotorControl

from crues import motors


def handle_mc_msg(data):
    motors.set_speeds(r_speed=data.r_speed, l_speed=data.l_speed, slp=data.slp)


def main():
    try:
        print "Running motor drive node"
        rospy.init_node('motor_driver')
        rospy.Subscriber('motor_control', MotorControl, handle_mc_msg)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        print "ROSInterruptException in node 'motor_driver'"


if __name__ == '__main__':
    main()
