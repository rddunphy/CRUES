#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from crues.motors import Motor


def handle_motor_cmd(msg, motor, scale_factor):
    motor.set_speed(msg.data * scale_factor)


def main():
    rospy.init_node('motor')
    pwm_pin = rospy.get_param('~pwm_pin')
    dir_pin = rospy.get_param('~dir_pin')
    speed_range = rospy.get_param('~speed_range', 255.0)
    max_speed = rospy.get_param('~max_speed', 100.0)
    motor = Motor(pwm_pin, dir_pin, max_speed)
    scale_factor = 100.0 / speed_range
    rospy.Subscriber('motor_cmd', Float32, handle_motor_cmd, args=(motor, scale_factor))
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        motor.cleanup()


if __name__ == '__main__':
    main()
