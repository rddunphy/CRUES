#!/usr/bin/env python


from threading import Event, Thread

try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO

import rospy
from std_msgs.msg import Bool, Int32


class FlashInterrupt(Exception):
    """Used by LED class to interrupt flash worker thread."""
    pass


class LED:
    def __init__(self):
        rospy.init_node('led')
        self.pin = rospy.get_param('~pin')
        self.on = False
        self.flashing = False
        self.exit_flash = Event()
        rospy.Subscriber('led', Bool, self._led_callback)
        rospy.Subscriber('led_flash', Int32, self._flash_callback)

    def spin(self):
        try:
            self.setup()
            rospy.spin()
        finally:
            self.cleanup()

    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

    def cleanup(self):
        GPIO.cleanup(self.pin)

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

    def flash(self, n=1, f=2.0):
        """Start flashing the LED. Is interrupted by new call to turn_on, turn_off, toggle, or flash.

        :param n: Number of flashes. Negative value for constant flashing.
        :param f: Frequency of flash in Hz.
        """
        self.turn_off()
        proc = Thread(target=self._flash_thread_worker, args=[n, f])
        proc.start()

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

    def _led_callback(self, msg):
        if msg.data:
            self.turn_on()
        else:
            self.turn_off()

    def _flash_callback(self, msg):
        code = msg.data
        if code == 0:
            self.turn_off()
        elif code == 1:
            self.flash()
        elif code == 2:
            self.flash(n=3)
        elif code == 3:
            self.flash(n=5, f=10)
        elif code == 4:
            self.flash(n=-1)
        elif code == 5:
            self.flash(n=-1, f=10)


if __name__ == '__main__':
    try:
        led = LED()
        led.spin()
    except rospy.ROSInterruptException:
        pass
