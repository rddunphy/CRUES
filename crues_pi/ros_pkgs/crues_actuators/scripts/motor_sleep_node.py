#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

from crues.motors import MotorSleepController


def handle_motor_cmd(msg, controller):
    controller.set_sleep(msg.data)


def main():
    rospy.init_node('motor_sleep')
    controller = MotorSleepController()
    rospy.Subscriber('motor_sleep', Bool, handle_sleep_cmd, args=(controller))
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        controller.cleanup()


if __name__ == '__main__':
    main()
