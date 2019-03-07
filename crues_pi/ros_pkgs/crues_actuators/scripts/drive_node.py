#!/usr/bin/env python

import rospy
from crues_actuators.msg import MotorControl

import RPi.GPIO as GPIO

from crues import motors


ml_pwm_pin = None
mr_pwm_pin = None


def handle_mc_msg(msg):
    motors.set_speeds(ml_pwm_pin, mr_pwm_pin, r_speed=msg.r_speed, l_speed=msg.l_speed, slp=msg.slp)


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
        GPIO.cleanup()


if __name__ == '__main__':
    main()
