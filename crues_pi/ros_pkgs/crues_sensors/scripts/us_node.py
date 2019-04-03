#!/usr/bin/env python
import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO

import rospy
from std_msgs.msg import Int32


# Approx. speed of sound at 20C in m/s
SPEED_OF_SOUND = 343


class UltrasonicTimeout(Exception):
    """Error for indicating that an ultrasonic sensor has timed out (e.g. because GPIO missed an edge)."""

    def __init__(self, name, timeout):
        super(UltrasonicTimeout, self).__init__("%s ultrasonic sensor timed out after %f s" % (name, timeout))
        self.name = name
        self.timeout = timeout


class Ultrasonic:
    def __init__(self, name, trig_pin, echo_pin, sensor_timeout, pulse_duration=0.00001):
        self.name = name
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.sensor_timeout = sensor_timeout
        self.pulse_duration = pulse_duration
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(echo_pin, GPIO.BOTH, callback=self.logTime)

    def get_range(self):
        """Get range from ultrasonic sensor in millimetres.

        :return: (int) Approx. range in millimetres
        :except: (UltrasonicTimeout) If module timed out waiting for GPIO input change
        """
        self.start_time = -1
        self.stop_time = -1
        
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(self.pulse_duration)
        
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(self.sensor_timeout)
        
        if self.start_time == -1 or self.stop_time == -1:
            raise UltrasonicTimeout(self.name, self.sensor_timeout)
            
        
        duration = self.stop_time - self.start_time
        distance = duration * SPEED_OF_SOUND * 500
        
        return int(round(distance))
    
    def logTime(self, channel):
        if GPIO.input(self.echo_pin):
            self.start_time = time.time()
        else:
            self.stop_time = time.time()

    def cleanup(self):
        GPIO.cleanup([self.trig_pin, self.echo_pin])


class UltrasonicScanner:
    def __init__(self):
        rospy.init_node("ultrasonic")
        self.scan_increment = rospy.get_param('~scan_increment', 0.05)
        timeout = self.scan_increment * 0.9
        sensor_angle = rospy.get_param('~sensor_angle', 30)
        self.left = Ultrasonic("Left", rospy.get_param('pins/ult'), rospy.get_param('pins/ule'), timeout)
        self.centre = Ultrasonic("Centre", rospy.get_param('pins/uct'), rospy.get_param('pins/uce'), timeout)
        self.right = Ultrasonic("Right", rospy.get_param('pins/urt'), rospy.get_param('pins/ure'), timeout)
        self.rate = rospy.Rate(rospy.get_param('~rate', 5))
        self.pub_l = rospy.Publisher('ul_range', Int32, queue_size=10)
        self.pub_c = rospy.Publisher('uc_range', Int32, queue_size=10)
        self.pub_r = rospy.Publisher('ur_range', Int32, queue_size=10)

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self._scan()
                self.rate.sleep()
        finally:
            self.left.cleanup()
            self.centre.cleanup()
            self.right.cleanup()

    def _scan(self):
        start = time.time()
        self._publish_range(self.left, self.pub_l)
        time_remaining = start + self.scan_increment - time.time()
        if time_remaining > 0:
            time.sleep(time_remaining)
            
            
        self._publish_range(self.centre, self.pub_c)
        time_remaining = start + 2 * self.scan_increment - time.time()
        if time_remaining > 0:
            time.sleep(time_remaining)
            
            
        self._publish_range(self.right, self.pub_r)
        # Additionally publish scan object for SLAM node?

    def _publish_range(self, sensor, pub):
        try:
            r = sensor.get_range()
        except UltrasonicTimeout as e:
            rospy.logwarn(str(e))
            r = -1
        else:
            rospy.logdebug("%s ultrasonic node range: %d", sensor.name, r)
        finally:
            pub.publish(r)


if __name__ == '__main__':
    try:
        scanner = UltrasonicScanner()
        scanner.spin()
    except rospy.ROSInterruptException:
        pass
