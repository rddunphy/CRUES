import time

import RPi.GPIO as GPIO

from pin_defs import Pins

LEFT = 0
CENTRE = 1
RIGHT = 2

pins = Pins()

sensor_str = {LEFT: "Left", CENTRE: "Centre", RIGHT: "Right"}

# Approx. speed of sound at 20C in m/s
speed_of_sound = 343

# Max. time to wait for sensor response in s
sensor_timeout = 0.049

# Time period between pulses on adjacent sensors in s
scan_increment = 0.05

# Angle between sensors in degrees
sensor_angle = 30


class UltrasonicTimout(Exception):
    """Error for indicating that an ultrasonic sensor has timed out (e.g. because GPIO missed an edge)."""

    def __init__(self, dir, timeout):
        super(UltrasonicTimout, self).__init__("%s ultrasonic sensor timed out after %f s" % (sensor_str.get(dir), timeout))
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
        return pins.ULT, pins.ULE
    elif dir == CENTRE:
        return pins.UCT, pins.UCE
    elif dir == RIGHT:
        return pins.URT, pins.URE
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
    GPIO.setup([pins.ULT, pins.UCT, pins.URT], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup([pins.ULE, pins.UCE, pins.URE], GPIO.IN)


def get_range(dir):
    """Get range from ultrasonic sensor in millimetres.

    :param dir: (int) One of us.LEFT, us.CENTRE, or us.RIGHT
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
