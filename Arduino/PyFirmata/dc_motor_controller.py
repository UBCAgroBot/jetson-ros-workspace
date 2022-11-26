#!/usr/bin/env python3

import pyfirmata
import time

# Change this!
MYPORT = '/dev/tty.usbmodem1101'

enablePin = 5
forwardPin = 3
backwardPin = 4

board = pyfirmata.Arduino(MYPORT)
print("Communication Successfully started")

# set up pins
forward = board.digital[forwardPin]
backward = board.digital[backwardPin]
motor = board.digital[enablePin]

# set speed as pwm pin
motor.mode = pyfirmata.PWM


def spinMotor(speed):
    assert(speed <= 1 and speed >= -1)
    motor.write(0)
    if speed < 0:
        forward.write(0)
        backward.write(1)
    else:
        forward.write(1)
        backward.write(0)

    motor.write(abs(speed))


if __name__ == '__main__':

    while True:
        for speed in [0.6, 0.8, 1.0]:
            spinMotor(speed)
            time.sleep(5)

            spinMotor(0)
            time.sleep(1)

            spinMotor(-speed)
            time.sleep(5)

            spinMotor(0)
            time.sleep(1)
