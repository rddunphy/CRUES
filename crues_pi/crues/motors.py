from crues.pin_defs import ML_DIR, ML_PWM, MR_DIR, MR_PWM, M_SLP

import RPi.GPIO as GPIO


def set_speeds(r_speed=0, l_speed=0, slp=True):
    """Set the speed of both motors. Values for the speed should be in the
    range [-100, 100], where negative values result in moving backwards, positive
    in moving forward, 0 results in no movement.

    :param r_speed: Speed of the right motor (as percentage)
    :param l_speed: Speed of the left motor (as percentage)
    :param slp: True to turn off motor break and coast. r_speed and l_speed will
        be disregarded if slp==True
    """
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([ML_DIR, ML_PWM, MR_DIR, MR_PWM, M_SLP], GPIO.OUT)
    ml_pwm_pin = GPIO.PWM(ML_PWM, 50)  # set up pwm object at 50Hz
    mr_pwm_pin = GPIO.PWM(MR_PWM, 50)  # set up pwm object at 50Hz
    ml_pwm_pin.start(1)  # Not sure what the parameter does?
    mr_pwm_pin.start(1)

    ml_pwm_pin.ChangeDutyCycle(abs(l_speed) if not slp else 0)
    # Low output = forward
    GPIO.output(ML_DIR, l_speed < 0)
    mr_pwm_pin.ChangeDutyCycle(abs(r_speed) if not slp else 0)
    GPIO.output(MR_DIR, r_speed < 0)
    # SLP pin is active low, so invert output
    GPIO.output(M_SLP, not slp)
