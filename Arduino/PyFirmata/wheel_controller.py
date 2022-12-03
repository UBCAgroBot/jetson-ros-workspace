import pyfirmata
import time

# Change this!
MYPORT = '/dev/tty.usbmodem1101'

board = pyfirmata.Arduino(MYPORT)
print("Communication Successfully started")

# define pins for all motors of wheels
GFL_DIR = 30
GFL_PWM = 10

GFR_DIR = 42
GFR_PWM = 11

GBL_DIR = 32
GBL_PWM = 8

GBR_DIR = 26
GBR_PWM = 9

# define left and right servo motor pins for front servos
SL_DIR = 48
SL_PULSE = 50
SR_DIR = 49
SR_PULSE = 51

# get_pin takes a string 'a/d:#:i/o/p'
# ‘a’ or ‘d’ (analog or digital pin), the pin number, and mode (‘i’ for input, ‘o’ for output, ‘p’ for pwm)
wheelFL_dir = board.get_pin(f'd:{GFL_DIR}:o')
wheelFL_speed = board.get_pin(f'd:{GFL_PWM}:p')

wheelFR_dir = board.get_pin(f'd:{GFR_DIR}:o')
wheelFR_speed = board.get_pin(f'd:{GFR_PWM}:p')

wheelBL_dir = board.get_pin(f'd:{GBL_DIR}:o')
wheelBL_speed = board.get_pin(f'd:{GBL_PWM}:p')

wheelBR_dir = board.get_pin(f'd:{GBR_DIR}:o')
wheelBR_speed = board.get_pin(f'd:{GBR_PWM}:p')

  

# LEFT and RIGHT output values for stepper motor
DIR_RIGHT = 1;
DIR_LEFT = 0;

# FORWARD and BACKWARD output values for gearbox motor
DIR_FORWARD = 1;
DIR_BACKWARD = 0;

# Angular Speed, recommended setting 7, DO NOT DECREASE TO < 3
ANGULAR_SPEED = 7;

# Strength of PWM, between 0 and 255, higher is faster
PWM_SPEED = 100;

# 1 pulse to angle
# 400 revs per 360 degrees
REV_ANGLE = 360.0 / 400.0;

# max turning angle
MAX_ANGLE = 90.0;