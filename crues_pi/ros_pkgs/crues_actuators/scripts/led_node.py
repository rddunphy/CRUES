#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

from crues import leds


def handle_led_msg(data, led):
    if data:
        led.turn_on()
    else:
        led.turn_off()


def main():
    try:
        rospy.init_node('leds')
        rospy.Subscriber('green_led', Bool, handle_led_msg, callback_args=leds.green_led)
        rospy.Subscriber('red_led', Bool, handle_led_msg, callback_args=leds.red_led)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
