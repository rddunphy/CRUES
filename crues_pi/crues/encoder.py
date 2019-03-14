import RPi.GPIO as GPIO

from crues.pin_defs import Pins


MIN_VAL = -32768
MAX_VAL = 32767


class Encoder:
    def __init__(self, pin_a_str, pin_b_str):
        self.count = 0
        pins = Pins()
        GPIO.setmode(GPIO.BOARD)
        self.pin_a = pins.get(pin_a_str)
        self.pin_b = pins.get(pin_b_str)
        GPIO.setup([self.pin_a, self.pin_b], GPIO.IN)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self.callback_a)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self.callback_b)

    def callback_a(self, _):
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        if a == b:
            self.inc()
        else:
            self.dec()

    def callback_b(self, _):
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        if a == b:
            self.dec()
        else:
            self.inc()

    def inc(self):
        self.count += 1
        if self.count > MAX_VAL:
            self.count = MIN_VAL

    def dec(self):
        self.count -= 1
        if self.count < MIN_VAL:
            self.count = MAX_VAL

    def cleanup(self):
        GPIO.cleanup([self.pin_a, self.pin_b])
