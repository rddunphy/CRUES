#!/usr/bin/env python

import time
import random

from crues import pin_defs, motors, us
from crues.us import UltrasonicTimout


def main():
    try:
        pl, pr = motors.gpio_setup()
        us.configure_gpio()
        while True:
            try:
                d = us.get_range(us.CENTRE)
            except UltrasonicTimout as e:
                print e
            if d > 150:
                motors.set_speeds(pl, pr, l_speed=25, r_speed=25, slp=False)
            elif d > 80:
                motors.set_speeds(pl, pr, l_speed=10, r_speed=10, slp=False)
            else:
                motors.set_speeds(pl, pr, l_speed=25, r_speed=-25, slp=False)
                time.sleep(random.uniform(0.5, 1.3))
    finally:
        pin_defs.cleanup()


if __name__ == '__main__':
    main()
