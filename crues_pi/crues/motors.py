from crues.pin_defs import ML_DIR, ML_PWM, MR_DIR, MR_PWM, M_SLP

import RPi.GPIO as GPIO


def gpio_setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([ML_DIR, MR_DIR], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(M_SLP, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup([ML_PWM, MR_PWM], GPIO.OUT)
    ml_pin = GPIO.PWM(ML_PWM, 50)
    mr_pin = GPIO.PWM(MR_PWM, 50)
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
    GPIO.output(M_SLP, not slp)
    # Low output = forward
    GPIO.output(ML_DIR, l_speed < 0)
    GPIO.output(MR_DIR, r_speed < 0)
    ml_pin.ChangeDutyCycle(abs(l_speed))
    mr_pin.ChangeDutyCycle(abs(r_speed))
