import time

import RPi.GPIO as GPIO

from pin_defs import UC_ECHO, UC_TRIG, UR_ECHO, UR_TRIG, UL_ECHO, UL_TRIG

LEFT = 0
CENTRE = 1
RIGHT = 2

# Approx. speed of sound at 20C in m/s
speed_of_sound = 343


def get_range(dir):
    """Get range from ultrasonic sensor in millimetres.

    :param dir: (int) One of ultrasonic.US_LEFT, ultrasonic.US_CENTRE, or ultrasonic.US_RIGHT
    :return: (int) Approx. range in millimetres
    """
    if dir == LEFT:
        echo_pin = UL_ECHO
        trig_pin = UL_TRIG
    elif dir == CENTRE:
        echo_pin = UC_ECHO
        trig_pin = UC_TRIG
    elif dir == RIGHT:
        echo_pin = UR_ECHO
        trig_pin = UR_TRIG
    else:
        raise RuntimeError("ultrasonic.get_range(dir) must take valid direction as input.")
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(trig_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    pulse_start = time.time()
    while not GPIO.input(echo_pin):
        pulse_start = time.time()
    pulse_end = pulse_start
    while GPIO.input(echo_pin):
        pulse_end = time.time()
    duration = pulse_end - pulse_start
    distance = duration * speed_of_sound * 500
    GPIO.cleanup()
    return int(round(distance))
