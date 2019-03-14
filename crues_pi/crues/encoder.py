import RPi.GPIO as GPIO

from crues.pin_defs import Pins


MIN_VAL = -32768
MAX_VAL = 32767

pins = Pins()


class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0

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


def get_encoders():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([pins.ELA, pins.ELB, pins.ERA, pins.ERB], GPIO.IN)
    l_enc = Encoder(pins.ELA, pins.ELB)
    GPIO.add_event_detect(pins.ELA, GPIO.BOTH, callback=l_enc.callback_a)
    GPIO.add_event_detect(pins.ELB, GPIO.BOTH, callback=l_enc.callback_b)
    r_enc = Encoder(pins.ERA, pins.ERB)
    GPIO.add_event_detect(pins.ERA, GPIO.BOTH, callback=r_enc.callback_a)
    GPIO.add_event_detect(pins.ERB, GPIO.BOTH, callback=r_enc.callback_b)
    return l_enc, r_enc


def cleanup():
    GPIO.cleanup([pins.ELA, pins.ELB, pins.ERA, pins.ERB])
