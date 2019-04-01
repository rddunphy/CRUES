import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO

# Approx. speed of sound at 20C in m/s
SPEED_OF_SOUND = 343


class UltrasonicTimeout(Exception):
    """Error for indicating that an ultrasonic sensor has timed out (e.g. because GPIO missed an edge)."""

    def __init__(self, name, timeout):
        super(UltrasonicTimeout, self).__init__("%s ultrasonic sensor timed out after %f s" % (name, timeout))
        self.name = name
        self.timeout = timeout


class Ultrasonic:
    def __init__(self, name, trig_pin, echo_pin, sensor_timeout, pulse_duration=0.00001):
        self.name = name
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.sensor_timeout = sensor_timeout
        self.pulse_duration = pulse_duration
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(echo_pin, GPIO.IN)

    def get_range(self):
        """Get range from ultrasonic sensor in millimetres.

        :return: (int) Approx. range in millimetres
        :except: (UltrasonicTimeout) If module timed out waiting for GPIO input change
        """
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(self.pulse_duration)
        GPIO.output(self.trig_pin, GPIO.LOW)
        pulse_start = time.time()
        timeout = pulse_start + self.sensor_timeout
        while not GPIO.input(self.echo_pin) and pulse_start < timeout:
            pulse_start = time.time()
        pulse_end = pulse_start
        while GPIO.input(self.echo_pin) and pulse_end < timeout:
            pulse_end = time.time()
        if pulse_end >= timeout:
            raise UltrasonicTimeout(self.name, self.sensor_timeout)
        duration = pulse_end - pulse_start
        distance = duration * SPEED_OF_SOUND * 500
        return int(round(distance))

    def cleanup(self):
        GPIO.cleanup([self.trig_pin, self.echo_pin])
