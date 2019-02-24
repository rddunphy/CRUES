from crues.pin_defs import Pins

import RPi.GPIO as GPIO


pins = Pins()


def gpio_setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([pins.MLD, pins.MRD], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(pins.MS, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup([pins.MLP, pins.MRP], GPIO.OUT)
    ml_pin = GPIO.PWM(pins.MLP, 50)
    mr_pin = GPIO.PWM(pins.MRP, 50)
    ml_pin.start(0)
    mr_pin.start(0)
    return ml_pin, mr_pin


def set_speeds(ml_pin, mr_pin, l_speed=0, r_speed=0, slp=True):
    """Set the speed of both motors. Values for the speed should be in the
    range [-100, 100], where negative values result in moving backwards, positive
    in moving forward, 0 results in no movement.

    :param r_speed: Speed of the right motor (as percentage)
    :param l_speed: Speed of the left motor (as percentage)
    :param slp: True to turn off motor break and coast. r_speed and l_speed will
        be disregarded if slp==True
    """
    # SLP pin is active low, so invert output
    #
    # GPIO.setmode(GPIO.BOARD)
    # GPIO.setup([ML_DIR, MR_DIR], GPIO.OUT, initial=GPIO.LOW)
    # GPIO.setup(M_SLP, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.output(pins.MS, not slp)
    # Low output = forward
    GPIO.output(pins.MLD, l_speed < 0)
    GPIO.output(pins.MRD, r_speed < 0)
    ml_pin.ChangeDutyCycle(abs(l_speed))
    mr_pin.ChangeDutyCycle(abs(r_speed))
