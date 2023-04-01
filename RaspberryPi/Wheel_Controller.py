import RPi.GPIO as GPIO
from time import sleep
from enum import Enum
from pynput import keyboard

import cv2 as cv
from ..Navigation.algorithms import SeesawAlgorithm as Seesaw

# LEFT and RIGHT output values for stepper motor
DIR_RIGHT = 1
DIR_LEFT = 0

# FORWARD and BACKWARD output values for gearbox motor
DIR_FORWARD = True
DIR_BACKWARD = False

# Angular Speed, recommended setting 7, DO NOT DECREASE TO < 3
ANGULAR_SPEED = 7 / 1000  # python sleep uses seconds

# Strength of PWM, between 0 and 255, higher is faster
PWM_SPEED = 100
PWM_MAX_SPEED = 255
PWM_FREQUENCY = 100

# 1 pulse to angle
# 400 revs per 360 degrees
REV_ANGLE = 360.0 / 400.0

# max turning angle
MAX_ANGLE = 90.0

# enum for vertical directions


class V_Directions(Enum):
    halted = 0
    forward = 1
    backward = 2

# enum for horizontal directions


class H_Directions(Enum):
    straight = 0
    left = 1
    right = 2


# variable for stepper direction [left, right]
angular_dir = H_Directions.straight

# variable for gearbox direction [forward, backward]
movement_dir = V_Directions.halted

# current_angle of stepper motor
current_angle = 0.0

movement_arr = (V_Directions.halted, H_Directions.straight)

'''HARDWARD PWM'''
# from rpi_hardware_pwm import HardwarePWM
# GearboxLeftPWM = HardwarePWM(pwm_channel=0, hz=60) # pwm channel 0: GPIO_18
# GearboxRightPWM = HardwarePWM(pwm_channel=1, hz=60) # pwm channel 1: GPIO_19

# GearboxLeftPWM.start(0)
# GearboxRightPWM.start(0)

# GearboxLeftPWM.stop()
# GearboxRightPWM.stop()

# pwm.change_duty_cycle(50)
# pwm.change_frequency(25_000)


'''PINS'''
# RPI 3B+ pinout diagram: https://pinout.xyz/
# G: Gearbox Connection (Stepper motors)
# D: DC Motor Driver
# Each wheel uses a DC Motor controlled by a DC Motor Driver
# the front two wheels uses a Gearbox connection to control the stepper motor for direction
DFL_DIR = 11
DFL_PWM = 13
DFR_DIR = 15
DFR_PWM = 16
DBL_DIR = 18
DBL_PWM = 22
DBR_DIR = 29
DBR_PWM = 31
GL_DIR = 32
GL_PULSE = 33
GR_DIR = 36
GR_PULSE = 37

PINS = (
    DFL_DIR,
    DFL_PWM,
    DFR_DIR,
    DFR_PWM,
    DBL_DIR,
    DBL_PWM,
    DBR_DIR,
    DBR_PWM,
    GL_DIR,
    GL_PULSE,
    GR_DIR,
    GR_PULSE,
)

PWMS = (
    DFL_PWM,
    DFR_PWM,
    DBL_PWM,
    DBR_PWM,
)

pwm_controls = []


def setup_rpi():
    GPIO.setwarnings(False)  # disable warnings
    GPIO.setmode(GPIO.BOARD)  # set pin numbering system
    GPIO.setup(PINS, GPIO.OUT)

    for pwm in PWMS:
        pwm_control = GPIO.PWM(pwm, PWM_FREQUENCY)  # create PWM instance with frequency
        pwm_control.start(0)  # set 0 Duty Cycle
        pwm_controls.append(pwm_control)

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
def rotate_wheels(direction):
    GPIO.output(DFL_DIR, int(direction))
    GPIO.output(DFR_DIR, int(not direction))
    GPIO.output(DBL_DIR, int(direction))
    GPIO.output(DBR_DIR, int(not direction))

    generate_pwm(PWM_SPEED)

# generate pwm for gearbox motors


def generate_pwm(speed):
    for pwmc in pwm_controls:
        pwmc.ChangeDutyCycle(speed / PWM_MAX_SPEED)

# Resets the stepper motor angle to 0 degrees


def reset_angle():
    global current_angle
    dir = DIR_RIGHT if current_angle < 0 else DIR_LEFT
    inc_value = REV_ANGLE if current_angle < 0 else -REV_ANGLE
    print("Reseting angle")

    turn_angle = abs(current_angle) // REV_ANGLE
    for _ in range(turn_angle):
        turn_wheels(dir)
        current_angle += inc_value
    print("Reset complete")


def run():
    global current_angle, PWM_SPEED, movement_arr
    print("speed ", PWM_SPEED)
    movement, turning = movement_arr
    print(movement, turning)
    print("angle ", current_angle)
    try:
        if movement == V_Directions.forward:
            print("Moving forward")
            PWM_SPEED = min(PWM_SPEED + 10, PWM_MAX_SPEED)
            rotate_wheels(DIR_FORWARD)
        elif movement == V_Directions.backward:
            print("Moving backward")
            PWM_SPEED = max(PWM_SPEED - 10, 0)
            rotate_wheels(DIR_BACKWARD)
        else:
            # stop moving
            generate_pwm(0)

        if turning == H_Directions.left:
            if current_angle <= -MAX_ANGLE:
                print("Max angle reached")
            else:
                print("Turning left")
                current_angle -= REV_ANGLE
                turn_wheels(DIR_LEFT)
        elif turning == H_Directions.right:
            if current_angle >= MAX_ANGLE:
                print("Max angle reached")
            else:
                print("Turning right")
                current_angle += REV_ANGLE
                turn_wheels(DIR_RIGHT)
        else:
            # reset angle
            reset_angle()

    except AttributeError:
        print("Attribute Error")


# testing with keyboard
def setup_keyboard():
    def on_press(key):
        try:
            if key.char == "w":
                movement_arr[0] = V_Directions.forward
            elif key.char == "s":
                movement_arr[0] = V_Directions.backward
            elif key.char == "a":
                movement_arr[1] = H_Directions.left
            elif key.char == "d":
                movement_arr[1] = H_Directions.right
        except AttributeError:
            print(f'special key {key} pressed')

    def on_release(key):
        try:
            if key.char == "w" or key.char == "s":
                movement_arr[0] = V_Directions.halted
            elif key.char == "a" or key.char == "d":
                movement_arr[1] = H_Directions.straight
            print(f'{key} released')
        except AttributeError:
            if key == keyboard.Key.esc:
                # Stop listener
                return False

    # Start the keyboard listener in a non-blocking manner
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)

    listener.start()


def run_algorithm(alg, vid_file):
    vid = cv.VideoCapture(vid_file)

    if not vid.isOpened():
        print('Error Opening Video File')

    while vid.isOpened():
        ret, frame = vid.read()

        if not ret:
            print('No More Frames Remaining')
            break

        # frame = pre_process.standardize_frame(frame)
        processed_image, angle = alg.process_frame(frame, show=args.show)

        print(angle)

        key = cv.waitKey(25)

        # Exit if Esc key is pressed
        if key == 27:
            break


def main():
    setup_rpi()
    setup_keyboard()
    run_algorithm(Seesaw, "../Navigation/videos/down1.mp4")


main()
