#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from crues.motors import Motor


def handle_motor_cmd(msg, (motor, offset, scale_factor)):
    motor.set_speed((msg.data - offset) * scale_factor)


def main():
    rospy.init_node('motor')
    pwm_pin = rospy.get_param('~pwm_pin')
    dir_pin = rospy.get_param('~dir_pin')
    range_min = rospy.get_param('~range_min', -1.0)
    range_max = rospy.get_param('~range_max', 1.0)
    motor = Motor(pwm_pin, dir_pin, max_speed)
    offset = (range_min + range_max) / 2
    scale_factor = 200.0 / (range_max - range_min)
    rospy.Subscriber('motor_cmd', Float32, handle_motor_cmd, callback_args=(motor, offset, scale_factor))
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        motor.cleanup()


if __name__ == '__main__':
    main()
