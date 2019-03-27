#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from crues.motors import Motor


ticks_since_last_cmd = 0
cmd = 0


def tick(motor, timeout_ticks, offset, scale_factor, pwm_min, pwm_max):
    global ticks_since_last_cmd
    ticks_since_last_cmd += 1
    if ticks_since_last_cmd > timeout_ticks:
        motor.set_speed(0)
    else:
        pwm_val = (cmd - offset) * scale_factor
        pwm_val = max(pwm_val, pwm_min)
        pwm_val = min(pwm_val, pwm_max)
        motor.set_speed(pwm_val)


def handle_motor_cmd(msg):
    global cmd, ticks_since_last_cmd
    cmd = msg.data
    ticks_since_last_cmd = 0


def main():
    rospy.init_node('motor')
    pwm_pin = rospy.get_param('~pwm_pin')
    dir_pin = rospy.get_param('~dir_pin')
    range_min = rospy.get_param('~range_min', -1.0)
    range_max = rospy.get_param('~range_max', 1.0)
    timeout_ticks = rospy.get_param('~timeout_ticks', 2)
    pwm_min = rospy.get_param('~pwm_min', -50)
    pwm_max = rospy.get_param('~pwm_max', 50)
    motor = Motor(pwm_pin, dir_pin)
    offset = (range_min + range_max) / 2
    scale_factor = 200.0 / (range_max - range_min)
    rospy.Subscriber('motor_cmd', Float32, handle_motor_cmd)
    rate = rospy.Rate(rospy.get_param('~rate', 50))
    try:
        while not rospy.is_shutdown():
            tick(motor, timeout_ticks, offset, scale_factor, pwm_min, pwm_max)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        motor.cleanup()


if __name__ == '__main__':
    main()
