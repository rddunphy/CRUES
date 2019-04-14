#!/usr/bin/env python
try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues_tools import GPIO_MOCK as GPIO

import rospy
from std_msgs.msg import Int16


class Encoder:
    def __init__(self):
        rospy.init_node('encoder')
        self.pin_a = rospy.get_param("~pin_a")
        self.pin_b = rospy.get_param("~pin_b")
        self.min = rospy.get_param("~min_val", -32768)
        self.max = rospy.get_param("~max_val", 32767)
        self.rate = rospy.Rate(rospy.get_param("~rate", 10))
        self.pub = rospy.Publisher('wheel', Int16, queue_size=10)
        self.count = 0
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.pin_a, self.pin_b], GPIO.IN)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._callback_a)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._callback_b)

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self.pub.publish(self.count)
                self.rate.sleep()
        finally:
            self._cleanup()

    def _callback_a(self, _):
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        if a == b:
            self._inc()
        else:
            self._dec()

    def _callback_b(self, _):
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        if a == b:
            self._dec()
        else:
            self._inc()

    def _inc(self):
        self.count += 1
        if self.count > self.max:
            self.count = self.min

    def _dec(self):
        self.count -= 1
        if self.count < self.min:
            self.count = self.max

    def _cleanup(self):
        GPIO.cleanup([self.pin_a, self.pin_b])


if __name__ == '__main__':
    try:
        enc = Encoder()
        enc.spin()
    except rospy.ROSInterruptException:
        pass
