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


def setmode(mode):
    print("Setting GPIO mode to %d" % mode)


def setup(pins, mode, initial=LOW, pull_up_down=PUD_DOWN):
    if isinstance(pins, list):
        for pin in pins:
            setup(pin, mode, initial=initial, pull_up_down=pull_up_down)
    else:
        if mode == OUT:
            print("Setting pin %d as output with initial value %d" % (pins, initial))
        else:
            print("Setting pin %d as input with pull-up/down %d" % (pins, pull_up_down))


def add_event_detect(pin, edge, callback=None):
    edge_str = "both edges"
    if edge == RISING:
        edge_str = "rising edge"
    elif edge == FALLING:
        edge_str = "falling edge"
    if not callback is None:
        print("Registering %s for %s on pin %d" % callback.__name__, edge_str, pin)


def input(pin):
    print("Getting input from pin %d" % pin)
    return 0


def output(pin, signal):
    print("Writing %d to pin %d" % (signal, pin))


def cleanup(pins):
    if isinstance(pins, list):
        for pin in pins:
            cleanup(pin)
    else:
        print("Cleaning up %d" % pins)


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
