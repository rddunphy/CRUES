#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

from crues.motors import MotorSleepController


def handle_sleep_cmd(msg, controller):
    controller.set_sleep(msg.data)


def main():
    rospy.init_node('motor_sleep')
    pin = rospy.get_param('pins/ms')
    controller = MotorSleepController(pin)
    rospy.Subscriber('motor_sleep', Bool, handle_sleep_cmd, callback_args=controller)
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        controller.cleanup()


if __name__ == '__main__':
    main()
