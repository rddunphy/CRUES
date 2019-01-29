#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from crues.pin_defs import ML_DIR, ML_PWM, MR_DIR, MR_PWM, M_SLP

import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)

GPIO.setup([ML_DIR, ML_PWM, MR_DIR, MR_PWM, M_SLP], GPIO.OUT)

ml_pwm = GPIO.PWM(ML_PWM, 50)  # set up pwm object at 50Hz
mr_pwm = GPIO.PWM(MR_PWM, 50)  # set up pwm object at 50Hz


def set_pins(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    L_dc = data.L_dc
    L_dir = data.L_dir
    R_dc = data.R_dc
    R_dir = data.R_dir
    slp = data.slp

    ml_pwm.ChangeDutyCycle(L_dc)
    GPIO.output(ML_DIR, L_dir)


    mr_pwm.ChangeDutyCycle(R_dc)
    GPIO.output(MR_DIR, R_dir)
    GPIO.output(M_SLP, slp)


def motor_driver():
    rospy.init_node('motor_driver')

    rospy.Subscriber("motor_inputs", String, set_pins)

    rospy.spin()


# spin() simply keeps python from exiting until this node is stopped
# rospy.spin()

if __name__ == '__main__':
    motor_driver()
