#!/usr/bin/env python
try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO

import rospy
from std_msgs.msg import Float32


class Motor:
    def __init__(self):
        rospy.init_node('motor')
        self.pwm_pin = rospy.get_param('~pwm_pin')
        self.dir_pin = rospy.get_param('~dir_pin')
        self.range_min = rospy.get_param('~range_min', -1.0)
        self.range_max = rospy.get_param('~range_max', 1.0)
        self.timeout_ticks = rospy.get_param('~timeout_ticks', 2)
        self.pwm_min = rospy.get_param('~pwm_min', -50)
        self.pwm_max = rospy.get_param('~pwm_max', 50)
        self.offset = (self.range_min + self.range_max) / 2
        self.scale_factor = 200.0 / (self.range_max - self.range_min)
        self.ticks_since_last_cmd = 0
        self.cmd = 0
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 100)
        self.pwm.start(0)
        rospy.Subscriber('motor_cmd', Float32, self._cmd_callback)

    def spin(self):
        rate = rospy.Rate(rospy.get_param('~rate', 50))
        try:
            while not rospy.is_shutdown():
                self._tick()
                rate.sleep()
        finally:
            self._cleanup()

    def _tick(self):
        self.ticks_since_last_cmd += 1
        if self.ticks_since_last_cmd > self.timeout_ticks:
            self.pwm.ChangeDutyCycle(0)
        else:
            pwm_val = (self.cmd - self.offset) * self.scale_factor
            pwm_val = max(pwm_val, self.pwm_min)
            pwm_val = min(pwm_val, self.pwm_max)
            GPIO.output(self.dir_pin, pwm_val < 0)
            self.pwm.ChangeDutyCycle(abs(pwm_val))

    def _cmd_callback(self, msg):
        self.cmd = msg.data
        self.ticks_since_last_cmd = 0

    def _cleanup(self):
        GPIO.cleanup([self.dir_pin, self.pwm_pin])


if __name__ == '__main__':
    try:
        motor = Motor()
        motor.spin()
    except rospy.ROSInterruptException:
        pass

