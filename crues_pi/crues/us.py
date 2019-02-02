import time

import RPi.GPIO as GPIO

from pin_defs import UC_ECHO, UC_TRIG, UR_ECHO, UR_TRIG, UL_ECHO, UL_TRIG

LEFT = 0
CENTRE = 1
RIGHT = 2

# Approx. speed of sound at 20C in m/s
speed_of_sound = 343

scan_increment = 0.02
sensor_angle = 15


class USScan:

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


def get_range(dir):
    """Get range from ultrasonic sensor in millimetres.

    :param dir: (int) One of ultrasonic.US_LEFT, ultrasonic.US_CENTRE, or ultrasonic.US_RIGHT
    :return: (int) Approx. range in millimetres
    """
    trig_pin, echo_pin = _get_pins(dir)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(trig_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)
    pulse_start = time.time()
    while not GPIO.input(echo_pin):
        pulse_start = time.time()
    pulse_end = pulse_start
    while GPIO.input(echo_pin):
        pulse_end = time.time()
    GPIO.cleanup([trig_pin, echo_pin])
    return _calculate_range(pulse_start, pulse_end)


def get_scan():
    start = time.time()
    rl = get_range(LEFT)
    while time.time() < start + scan_increment:
        pass
    rc = get_range(CENTRE)
    while time.time() < start + 2 * scan_increment:
        pass
    rr = get_range(RIGHT)
    return USScan([rl, rc, rr])
