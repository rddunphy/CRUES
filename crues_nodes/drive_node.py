from pin_defs import ML_DIR, ML_PWM, MR_DIR, MR_PWM, EL_A, EL_B, ER_A, ER_B
from simple_pid import PID

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Use Sudo")


def setup():
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup([ML_DIR, ML_PWM, MR_DIR, MR_PWM], GPIO.OUT)
    GPIO.setup([EL_A, EL_B, ER_A, ER_B], GPIO.OUT)


def read_encoder(enc_a, enc_b):
    # speed = (pulses*pi*wheel_diameter)/ (pulses per rotation *    sampletime)
    print("")




def main():
    ######################## usually desired speeds read from ros
    DESIRED_SPEED_L = 0
    DESIRED_SPEED_R = 0
    #######################
    setup()
    ml_pwm  = GPIO.PWM(ML_PWM,50)  # set up pwm object at 50Hz
    mr_pwm = GPIO.PWM(MR_PWM, 50)  # set up pwm object at 50Hz

    motor_pid = PID(1, 0.1, 0.05, setpoint=0, output_limits=(0, 1))  # speed = 0 to start, duty cycle between 0 and 1

    while(True):
        measured_speed_L = read_encoder(EL_A, EL_B)
        measured_speed_R = read_encoder(ER_A, ER_B)
        motor_pid.setpoint = DESIRED_SPEED_L
        L_dc = motor_pid(measured_speed_L)
        motor_pid.setpoint = DESIRED_SPEED_R
        R_dc = motor_pid(measured_speed_R)
        ml_pwm.ChangeDutyCycle(L_dc)
        mr_pwm.ChangeDutyCycle(R_dc)
        # update desired speeds???




if __name__ == '__main__':
    main()