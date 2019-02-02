"""Pin definitions for Raspberry Pi GPIO."""

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import RPi.GPIO as GPIO

# Ultrasonic sensors
UL_TRIG = 38
UL_ECHO = 40
UC_TRIG = 37
UC_ECHO = 35
UR_TRIG = 31
UR_ECHO = 29

# IMU serial pins
IMU_SCL = 5
IMU_SDA = 3

# Encoder outputs
EL_A = 22
EL_B = 18
ER_A = 13
ER_B = 11

# Motor control pins
ML_DIR = 12
ML_PWM = 32
MR_DIR = 7
MR_PWM = 33
M_SLP = 16

# LED indicators
LED_RED = 36
LED_GREEN = 28


def cleanup():
    GPIO.cleanup()


def print_config():
    print("Ultrasonic")
    print("UL_TRIG: {}".format(UL_TRIG))
    print("UL_ECHO: {}".format(UL_ECHO))
    print("UC_TRIG: {}".format(UC_TRIG))
    print("UC_ECHO: {}".format(UC_ECHO))
    print("UR_TRIG: {}".format(UR_TRIG))
    print("UR_ECHO: {}".format(UR_ECHO))
    print()
    print("IMU")
    print("IMU_SCL: {}".format(IMU_SCL))
    print("IMU_SDA: {}".format(IMU_SDA))
    print()
    print("Encoders")
    print("EL_A: {}".format(EL_A))
    print("EL_B: {}".format(EL_B))
    print("ER_A: {}".format(ER_A))
    print("ER_B: {}".format(ER_B))
    print()
    print("Motor control")
    print("ML_DIR: {}".format(ML_DIR))
    print("ML_PWM: {}".format(ML_PWM))
    print("MR_DIR: {}".format(MR_DIR))
    print("MR_PWM: {}".format(MR_PWM))
    print("M_SLP: {}".format(M_SLP))
    print()
    print("LEDs")
    print("LED_RED: {}".format(LED_RED))
    print("LED_GREEN: {}".format(LED_GREEN))
