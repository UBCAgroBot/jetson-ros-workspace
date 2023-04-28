import RPi.GPIO as GPIO
from time import sleep, time
from constants import *

# turns stepper motor in desired direction


def turn_wheels(direction):
    GPIO.output(GL_DIR, direction)
    GPIO.output(GR_DIR, direction)
    generate_pulse()

# generates pulse for stepper motors


def generate_pulse():
    GPIO.output(GL_PULSE, GPIO.HIGH)
    GPIO.output(GR_PULSE, GPIO.HIGH)
    sleep(ANGULAR_SPEED)
    GPIO.output(GL_PULSE, GPIO.LOW)
    GPIO.output(GR_PULSE, GPIO.LOW)
    sleep(ANGULAR_SPEED)


# turns gearbox motor in desired direction
def rotate_wheels(direction, pwm_controls):
    GPIO.output(DFL_DIR, int(direction))
    GPIO.output(DFR_DIR, int(not direction))
    GPIO.output(DBL_DIR, int(direction))
    GPIO.output(DBR_DIR, int(not direction))

    generate_pwm(PWM_SPEED, pwm_controls)

# generate pwm for gearbox motors


def generate_pwm(speed, pwm_controls):
    for pwmc in pwm_controls:
        pwmc.ChangeDutyCycle(int((speed // 10) * 10))

# Resets the stepper motor angle to 0 degrees


def reset_angle(current_angle):
    dir = DIR_RIGHT if current_angle < 0 else DIR_LEFT
    inc_value = REV_ANGLE if current_angle < 0 else -REV_ANGLE
    debug_print("Reseting angle")

    turn_angle = int(abs(current_angle) // REV_ANGLE)
    for _ in range(turn_angle):
        turn_wheels(dir)
        current_angle += inc_value
    debug_print("Reset complete")

    return current_angle


def setup_rpi(pwm_controls):
    GPIO.setwarnings(False)  # disable warnings
    GPIO.setmode(GPIO.BOARD)  # set pin numbering system
    GPIO.setup(PINS, GPIO.OUT)

    for pwm in PWMS:
        pwm_control = GPIO.PWM(pwm, PWM_FREQUENCY)  # create PWM instance with frequency
        pwm_control.start(0)  # set 0 Duty Cycle
        pwm_controls.append(pwm_control)


last_debug_print_time = 0


def debug_print(*output: object):
    if DEBUG_LEVEL == 0:
        if time() - last_debug_print_time > DEBUG_FREQUNCY:
            print(output)
            last_debug_print_time = time()
    elif DEBUG_LEVEL == 1:
        print(output)
