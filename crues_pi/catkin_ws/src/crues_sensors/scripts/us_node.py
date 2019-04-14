#!/usr/bin/env python
import time
import math

try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues_tools import GPIO_MOCK as GPIO

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

# Approx. speed of sound at 20C in m/s
SPEED_OF_SOUND = 343


class UltrasonicTimeout(Exception):
    """Error for indicating that an ultrasonic sensor has timed out (e.g. because GPIO missed an edge)."""

    def __init__(self, name, timeout, is_on_rising_edge):
        if is_on_rising_edge:
            msg = "%s ultrasonic sensor timed out after %f s" % (name, timeout)
        else:
            msg = "%s ultrasonic sensor missed rising edge" % name
        super(UltrasonicTimeout, self).__init__(msg)
        self.name = name
        self.timeout = timeout
        self.is_on_rising_edge = is_on_rising_edge


class Ultrasonic:
    def __init__(self, name, trig_pin, echo_pin, sensor_timeout, pulse_duration=0.00001, response=1.0, offset=0.0):
        self.name = name
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.sensor_timeout = sensor_timeout
        self.pulse_duration = pulse_duration
        self.start_time = -1
        self.stop_time = -1
        self.response = response
        self.offset = offset
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(echo_pin, GPIO.BOTH, callback=self._log_time)

    def get_range(self):
        """Get range from ultrasonic sensor in metres.

        :return: (float) Approx. range in metres
        :except: (UltrasonicTimeout) If module timed out waiting for GPIO input change
        """
        self.start_time = -1
        self.stop_time = -1
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(self.pulse_duration)
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(self.sensor_timeout)
        if self.start_time < 0 or self.stop_time < 0:
            raise UltrasonicTimeout(self.name, self.sensor_timeout, self.start_time < 0)
        duration = self.stop_time - self.start_time
        distance = duration * SPEED_OF_SOUND * 0.5
        return self.response * distance - self.offset

    def _log_time(self, _):
        if GPIO.input(self.echo_pin):
            self.start_time = time.time()
        else:
            self.stop_time = time.time()

    def cleanup(self):
        GPIO.cleanup([self.trig_pin, self.echo_pin])


class UltrasonicScanner:
    def __init__(self):
        rospy.init_node("ultrasonic")
        self.time_increment = rospy.get_param('~scan_increment', 0.05)
        timeout = self.time_increment * 0.9
        self.angle_min = rospy.get_param('~angle_min', -math.pi / 6)
        self.angle_max = rospy.get_param('~angle_max', math.pi / 6)
        self.angle_increment = rospy.get_param('~angle_increment', math.pi / 6)
        self.offset_centre = rospy.get_param('~offset_scan_centre', 0.127)
        self.offset_outer = rospy.get_param('~offset_scan_outer', 0.121)
        self.range_min = rospy.get_param('~range_min', 0) + self.offset_outer
        self.range_max = rospy.get_param('~range_max', 1.0) + self.offset_outer
        self.scan_frame_id = rospy.get_param('~scan_frame_id', 'us_scan_frame')
        self.left = Ultrasonic("Left", rospy.get_param('pins/ult'), rospy.get_param('pins/ule'), timeout,
                               response=rospy.get_param('~response_left', 1.0),
                               offset=rospy.get_param('~offset_left', 0.02))
        self.centre = Ultrasonic("Centre", rospy.get_param('pins/uct'), rospy.get_param('pins/uce'), timeout,
                                 response=rospy.get_param('~response_centre', 1.0),
                                 offset=rospy.get_param('~offset_centre', 0.02))
        self.right = Ultrasonic("Right", rospy.get_param('pins/urt'), rospy.get_param('pins/ure'), timeout,
                                response=rospy.get_param('~response_right', 1.0),
                                offset=rospy.get_param('~offset_right', 0.02))
        f = rospy.get_param('~rate', 5)
        self.rate = rospy.Rate(f)
        self.scan_time = 1.0 / f
        self.pulse_offset = rospy.get_param('~pulse_offset', 0)
        self.pub_l = rospy.Publisher('ul_range', Float32, queue_size=10)
        self.pub_c = rospy.Publisher('uc_range', Float32, queue_size=10)
        self.pub_r = rospy.Publisher('ur_range', Float32, queue_size=10)
        self.scan_pub = rospy.Publisher('sonar_scan', LaserScan, queue_size=10)

    def _duration_till_next_tick(self):
        now = rospy.get_time()
        next_tick = math.floor(now) + self.pulse_offset
        while next_tick < now:
            next_tick += self.scan_time
        return next_tick - now

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self._scan()
                rospy.sleep(self._duration_till_next_tick())
        finally:
            self.left.cleanup()
            self.centre.cleanup()
            self.right.cleanup()

    def _scan(self):
        start = time.time()
        stamp = rospy.Time.now()
        range_r = self._publish_range(self.right, self.pub_r)
        time_remaining = start + self.time_increment - time.time()
        if time_remaining > 0:
            time.sleep(time_remaining)
        range_c = self._publish_range(self.centre, self.pub_c)
        time_remaining = start + 2 * self.time_increment - time.time()
        if time_remaining > 0:
            time.sleep(time_remaining)
        range_l = self._publish_range(self.left, self.pub_l)
        self._publish_scan(range_r, range_c, range_l, stamp)

    def _publish_scan(self, range_r, range_c, range_l, timestamp):
        scan = LaserScan()
        scan.header.stamp = timestamp
        scan.header.frame_id = self.scan_frame_id
        scan.scan_time = self.scan_time
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.time_increment = self.time_increment
        scan.ranges = [range_r + self.offset_outer, range_c + self.offset_centre, range_l + self.offset_outer]
        scan.intensities = []
        self.scan_pub.publish(scan)

    @staticmethod
    def _publish_range(sensor, pub):
        r = -1000
        try:
            r = sensor.get_range()
        except UltrasonicTimeout as e:
            if e.is_on_rising_edge:
                rospy.logerr(str(e))
            else:
                rospy.logwarn(str(e))
        else:
            rospy.logdebug("%s ultrasonic node range: %d", sensor.name, r)
        finally:
            pub.publish(r)
            return r


if __name__ == '__main__':
    try:
        scanner = UltrasonicScanner()
        scanner.spin()
    except rospy.ROSInterruptException:
        pass
