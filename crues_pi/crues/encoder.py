try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO

MIN_VAL = -32768
MAX_VAL = 32767


class Encoder:
    def __init__(self, pin_a, pin_b):
        self.count = 0
        self.pin_a = pin_a
        self.pin_b = pin_b
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.pin_a, self.pin_b], GPIO.IN)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._callback_a)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._callback_b)

    def _callback_a(self, _):
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        if a == b:
            self._inc()
        else:
            self._dec()

    def _callback_b(self, _):
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        if a == b:
            self._dec()
        else:
            self._inc()

    def _inc(self):
        self.count += 1
        if self.count > MAX_VAL:
            self.count = MIN_VAL

    def _dec(self):
        self.count -= 1
        if self.count < MIN_VAL:
            self.count = MAX_VAL

    def cleanup(self):
        GPIO.cleanup([self.pin_a, self.pin_b])
