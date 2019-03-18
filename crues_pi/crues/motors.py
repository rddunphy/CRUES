from crues.pin_defs import Pins

import RPi.GPIO as GPIO


class Motor:
    def __init__(self, pwm_pin_str, dir_pin_str, max_speed):
        self.max_speed = max_speed
        pins = Pins()
        self.pwm_pin_n = pins.get(pwm_pin_str)
        self.dir_pin = pins.get(dir_pin_str)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pwm_pin_n, GPIO.OUT)
        self.pwm_pin = GPIO.PWM(self.pwm_pin_n, 50)
        self.pwm_pin.start(0)

    def set_speed(self, speed):
        speed = min(self.max_speed, speed)
        speed = max(-self.max_speed, speed)
        GPIO.output(self.dir_pin, speed < 0)
        self.pwm_pin.ChangeDutyCycle(abs(speed))

    def cleanup(self):
        GPIO.cleanup([self.dir_pin, self.pwm_pin_n])


class MotorSleepController:
    def __init__(self):
        pins = Pins()
        self.pin = pins.MS
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.HIGH)

    def set_sleep(self, sleep):
        if sleep:
            GPIO.output(self.pin, GPIO.LOW)
        else:
            GPIO.output(self.pin, GPIO.HIGH)

    def cleanup(self):
        GPIO.cleanup(self.pin)
