from crues.pin_defs import Pins

try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO


class Motor:
    def __init__(self, pwm_pin, dir_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 100)
        self.pwm.start(0)

    def set_speed(self, speed):
        # speed = min(self.max_speed, speed)
        # speed = max(-self.max_speed, speed)
        GPIO.output(self.dir_pin, speed < 0)
        self.pwm.ChangeDutyCycle(abs(speed))

    def cleanup(self):
        GPIO.cleanup([self.dir_pin, self.pwm_pin])


class MotorSleepController:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.HIGH)

    def set_sleep(self, sleep):
        if sleep:
            GPIO.output(self.pin, GPIO.LOW)
        else:
            GPIO.output(self.pin, GPIO.HIGH)

    def cleanup(self):
        GPIO.cleanup(self.pin)
