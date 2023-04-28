import RPi.GPIO as GPIO
from RaspberryPi.rpi_helper import generate_pwm
from constants import *
from rpi_helper import reset_angle, setup_rpi, debug_print
import signal
import sys

import cv2 as cv
from ..Navigation.algorithms import SeesawAlgorithm as Seesaw

# global variables
current_angle = 0.0
movement_arr = [V_Directions.halted, H_Directions.straight]
pwm_controls = []


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

        debug_print(angle)

        key = cv.waitKey(25)

        # Exit if Esc key is pressed
        if key == 27:
            break


def handler(signum, frame):
    # reset angle
    reset_angle(current_angle=current_angle)
    # stop moving
    generate_pwm(0, pwm_controls=pwm_controls)
    sys.exit(0)


signal.signal(signal.SIGINT, handler)


def main():
    setup_rpi(pwm_controls=pwm_controls)
    run_algorithm(Seesaw, "../Navigation/videos/down1.mp4")


main()
