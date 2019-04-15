BCM = 0
BOARD = 1
OUT = 0
IN = 1
LOW = OFF = 0
HIGH = ON = 1
FALLING = 0
RISING = 1
BOTH = 2
PUD_DOWN = 0

_output_pins = []
_input_pins = []


class GPIO_ERROR(Exception):
    pass


def setmode(mode):
    print("Setting GPIO mode to %d" % mode)


def setup(pin, mode, initial=LOW, pull_up_down=PUD_DOWN):
    if isinstance(pin, list):
        for p in pin:
            setup(p, mode, initial=initial, pull_up_down=pull_up_down)
    else:
        if pin in _input_pins or pin in _output_pins:
            raise GPIO_ERROR("Multiple pin definitions for %d" % pin)
        if mode == OUT:
            _output_pins.append(pin)
            print("Setting pin %d as output with initial value %d" % (pin, initial))
        else:
            _input_pins.append(pin)
            print("Setting pin %d as input with pull-up/down %d" % (pin, pull_up_down))


def add_event_detect(pin, edge, callback=None):
    edge_str = "both edges"
    if edge == RISING:
        edge_str = "rising edge"
    elif edge == FALLING:
        edge_str = "falling edge"
    if callback is not None:
        print("Registering %s for %s on pin %d" % (callback.__name__, edge_str, pin))


def input(pin):
    print("Getting input from pin %d" % pin)
    return 0


def output(pin, signal):
    print("Writing %d to pin %d" % (signal, pin))


def cleanup(pin):
    if isinstance(pin, list):
        for p in pin:
            cleanup(p)
    else:
        if pin not in _input_pins and pin not in _output_pins:
            raise GPIO_ERROR("Pin %d not yet defined" % pin)
        print("Cleaning up %d" % pin)


class PWM:
    def __init__(self, pin, rate):
        self.pin = pin
        self.rate = rate

    def start(self, val):
        print("Start PWM on pin %d at rate %f with duty cycle %f" % (self.pin, self.rate, val))

    def stop(self):
        print("Stopping PWM on pin %d" % self.pin)

    def ChangeDutyCycle(self, val):
        print("Setting duty cycle on pin %d to %f" % (self.pin, val))

    def ChangeFrequency(self, freq):
        print("Setting frequency for pin %d to %f" % (self.pin, freq))
