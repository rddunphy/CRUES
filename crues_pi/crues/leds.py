from threading import Event, Thread

import RPi.GPIO as GPIO

from crues.pin_defs import Pins


class FlashInterrupt(Exception):
    """Used by LED class to interrupt flash worker thread."""
    pass


class LED:
    def __init__(self, pin):
        self.pin = pin
        self.on = False
        self.flashing = False
        self.exit_flash = Event()
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    def turn_on(self):
        """Turn on the LED. (Interrupts flashing.)"""
        self._interrupt_flash()
        if not self.on:
            GPIO.output(self.pin, GPIO.HIGH)
            self.on = True

    def turn_off(self):
        """Turn off the LED. (Interrupts flashing.)"""
        self._interrupt_flash()
        if self.on:
            GPIO.output(self.pin, GPIO.LOW)
            self.on = False

    def toggle(self):
        """Toggle LED (Interrupts flashing.)"""
        self._interrupt_flash()
        GPIO.output(self.pin, GPIO.LOW if self.on else GPIO.HIGH)
        self.on = not self.on

    def _interrupt_flash(self):
        if self.flashing:
            self.exit_flash.set()
            while self.exit_flash.is_set():
                pass

    def _flash_toggle(self, t):
        if not self.exit_flash.is_set():
            GPIO.output(self.pin, GPIO.LOW if self.on else GPIO.HIGH)
            self.on = not self.on
            self.exit_flash.wait(t)
        else:
            raise FlashInterrupt("Flash interrupted")

    def _flash_thread_worker(self, n, f):
        self.flashing = True
        t = 1.0 / (2 * f)
        try:
            if n < 0:
                while True:
                    self._flash_toggle(t)
            else:
                for _ in range(n * 2):
                    self._flash_toggle(t)
        except FlashInterrupt:
            pass
        finally:
            self.flashing = False
            self.exit_flash.clear()

    def flash(self, n=1, f=2.0):
        """Start flashing the LED. Is interrupted by new call to turn_on, turn_off, toggle, or flash.

        :param n: Number of flashes. Negative value for constant flashing.
        :param f: Frequency of flash in Hz.
        """
        self.turn_off()
        proc = Thread(target=self._flash_thread_worker, args=[n, f])
        proc.start()


pins = Pins()
green_led = LED(pins.LG)
red_led = LED(pins.LR)
