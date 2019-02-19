#!/usr/bin/env python

import rospy
from crues_actuators.msg import MotorControl

from crues import motors, pin_defs


ml_pwm_pin = None
mr_pwm_pin = None


def handle_mc_msg(data):
    motors.set_speeds(ml_pwm_pin, mr_pwm_pin, r_speed=data.r_speed, l_speed=data.l_speed, slp=data.slp)


def main():
    try:
        global ml_pwm_pin, mr_pwm_pin
        ml_pwm_pin, mr_pwm_pin = motors.gpio_setup()
        rospy.init_node('motor_driver')
        rospy.Subscriber('motor_control', MotorControl, handle_mc_msg)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        pin_defs.cleanup()


if __name__ == '__main__':
    main()
