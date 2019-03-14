#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

from crues.leds import LED


def handle_led_msg(msg, led):
    if msg.data:
        led.turn_on()
    else:
        led.turn_off()


def main():
    rospy.init_node('led')
    led = LED(rospy.get_param('~pin'))
    rospy.Subscriber('led', Bool, handle_led_msg, callback_args=led)
    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        led.cleanup()


if __name__ == '__main__':
    main()
