#!/usr/bin/env python
try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO

import rospy
from std_msgs.msg import Bool


class MotorSleepController:
    def __init__(self):
        rospy.init_node('motor_sleep')
        self.pin = rospy.get_param('pins/ms')
        rospy.Subscriber('motor_sleep', Bool, self._sleep_callback)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.HIGH)

    def spin(self):
        try:
            rospy.spin()
        finally:
            self._cleanup()

    def _cleanup(self):
        GPIO.cleanup(self.pin)

    def _sleep_callback(self, msg):
        if msg.data:
            GPIO.output(self.pin, GPIO.LOW)
        else:
            GPIO.output(self.pin, GPIO.HIGH)


if __name__ == '__main__':
    try:
        ms = MotorSleepController()
        ms.spin()
    except rospy.ROSInterruptException:
        pass
