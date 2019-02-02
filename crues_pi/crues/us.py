import time

import RPi.GPIO as GPIO

from pin_defs import UC_ECHO, UC_TRIG, UR_ECHO, UR_TRIG, UL_ECHO, UL_TRIG

LEFT = 0
CENTRE = 1
RIGHT = 2

_dir_str = {LEFT: "Left", CENTRE: "Centre", RIGHT: "Right"}

# Approx. speed of sound at 20C in m/s
speed_of_sound = 343

# Max. time to wait for sensor response in s
sensor_timeout = 0.019

# Time period between pulses on adjacent sensors in s
scan_increment = 0.02

# Angle between sensors in degrees
sensor_angle = 30


class UltrasonicTimout(Exception):
    """Error for indicating that an ultrasonic sensor has timed out (e.g. because GPIO missed an edge)."""

    def __init__(self, dir, timeout):
        super(UltrasonicTimout, self).__init__("%s ultrasonic sensor timed out after %f s" % _dir_str.get(dir), timeout)
        self.dir = dir
        self.timeout = timeout


class USScan:
    """This class represents a scan of ranges from ultrasonic sensors.

    USScans should contain all information necessary to form a ROS LaserScan message.
    """

    def __init__(self, ranges):
        self.angle = sensor_angle
        self.increment = scan_increment
        self.ranges = ranges or []


def _get_pins(dir):
    if dir == LEFT:
        return UL_TRIG, UL_ECHO
    elif dir == CENTRE:
        return UC_TRIG, UC_ECHO
    elif dir == RIGHT:
        return UR_TRIG, UR_ECHO
    else:
        raise ValueError("ultrasonic.get_range(dir) must take valid direction as input.")


def _calculate_range(pulse_start, pulse_end):
    duration = pulse_end - pulse_start
    distance = duration * speed_of_sound * 500
    return int(round(distance))


def configure_gpio():
    """Configure GPIO pins used by ultrasonic sensors.

    This function should be called once before using the sensors.
    GPIO.cleanup() should be called on exit.
    """
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([UL_TRIG, UC_TRIG, UR_TRIG], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup([UL_ECHO, UC_ECHO, UR_ECHO], GPIO.IN)


def get_range(dir):
    """Get range from ultrasonic sensor in millimetres.

    :param dir: (int) One of ultrasonic.US_LEFT, ultrasonic.US_CENTRE, or ultrasonic.US_RIGHT
    :return: (int) Approx. range in millimetres
    :except: (UltrasonicTimeout) If module timed out waiting for GPIO input change
    """
    trig_pin, echo_pin = _get_pins(dir)
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)
    pulse_start = time.time()
    timeout = pulse_start + sensor_timeout
    while not GPIO.input(echo_pin) and pulse_start < timeout:
        pulse_start = time.time()
    pulse_end = pulse_start
    while GPIO.input(echo_pin) and pulse_end < timeout:
        pulse_end = time.time()
    if pulse_end >= timeout:
        raise UltrasonicTimout(dir, sensor_timeout)
    return _calculate_range(pulse_start, pulse_end)


def get_scan():
    """Get scan of ranges from all ultrasonic sensors.

    :return: (USScan) Scan of ranges
    :except: (UltrasonicTimeout) If module timed out waiting for GPIO input change
    """
    start = time.time()
    rl = get_range(LEFT)
    time_remaining = start + scan_increment - time.time()
    if time_remaining > 0:
        time.sleep(time_remaining)
    rc = get_range(CENTRE)
    time_remaining = start + 2 * scan_increment - time.time()
    if time_remaining > 0:
        time.sleep(time_remaining)
    rr = get_range(RIGHT)
    return USScan([rl, rc, rr])
