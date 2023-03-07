import serial
from typing import Enum

#  PWM: Analog
#  DIR: Digital
#  PULSE: Digital

# Define pins for motors
# G: Gearbox Motor
# S: Stepper Motor
# F: Front
# B: Back
# L: Left
# R: Right
GFL_DIR = 30
GFL_PWM = 10

GFR_DIR = 42
GFR_PWM = 11

GBL_DIR = 32
GBL_PWM = 8

GBR_DIR = 26
GBR_PWM = 9

SL_DIR = 48
SL_PULSE = 50

SR_DIR = 49
SR_PULSE = 51

# LEFT and RIGHT output values for stepper motor
DIR_RIGHT = 1
DIR_LEFT = 0

# FORWARD and BACKWARD output values for gearbox motor
DIR_FORWARD = 1
DIR_BACKWARD = 0

# Angular Speed, recommended setting 7, DO NOT DECREASE TO < 3
ANGULAR_SPEED = 7

# Strength of PWM, between 0 and 255, higher is faster
PWM_SPEED = 100

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

# input string buffer
buf = bytearray(80)

# serial communication setup
ser = serial.Serial('/dev/ttyACM0', 9600)

# TODO: figure out the OUTPUT
# TODO: figure out pinMode
def setup():
    # start serial communication, logger
    ser.flushInput()
    ser.flushOutput()

    # setup pin modes
    pinMode(GFL_DIR, "OUTPUT")
    pinMode(GFL_PWM, "OUTPUT")

    pinMode(GFR_DIR, "OUTPUT")
    pinMode(GFR_PWM, "OUTPUT")

    pinMode(GBL_DIR, "OUTPUT")
    pinMode(GBL_PWM, "OUTPUT")

    pinMode(GBR_DIR, "OUTPUT")
    pinMode(GBR_PWM, "OUTPUT")

    pinMode(SL_DIR, "OUTPUT")
    pinMode(SL_PULSE, "OUTPUT")

    pinMode(SR_DIR, "OUTPUT")
    pinMode(SR_PULSE, "OUTPUT")


def loop():
    is_valid_input = True

    if ser.in_waiting > 0:
        readline(ser.read(), buf, 80)
        is_valid_input = update_dir_enums(buf.decode('utf-8').strip())
        
    if is_valid_input:
        run()


# Read a line of text from serial input
def readline(readch, buffer, len):
    pos = 0

    if readch > 0:
        if readch == 13: # Ignore CR
            pass
        elif readch == 10: # Return on new-line
            rpos = pos
            pos = 0  # Reset position index ready for next time
            return rpos
        else:
            if pos < len-1 and readch > 0 and readch < 128:
                buffer[pos] = readch
                pos += 1
                buffer[pos] = 0
    return 0


# Update direction enums
# Returns True if input is a valid value, otherwise returns False
def update_dir_enums(input_str):
    if input_str[0] == 'H':
        halt()
    elif input_str[0] == 'F':
        set_forward()
    elif input_str[0] == 'B':
        set_backward()
    else:
        return False
      
    if input_str[1] == 'L':
        set_left()
    elif input_str[1] == 'S':
        reset_angle()
    elif input_str[1] == 'R':
        set_right()
    else:
        return False
    return True

# Sets direction value to desired value if it is not already set to that value,
# Otherwise, interpret action as an instruction to stop, and set to undefined
def set_forward():
    global movement_dir
    movement_dir = V_Directions.forward

def set_left():
    global angular_dir
    angular_dir = H_Directions.left

def set_backward():
    global movement_dir
    movement_dir = V_Directions.backward

def set_right():
    global angular_dir
    angular_dir = H_Directions.right

# Resets the stepper motor angle to 0 degrees
def reset_angle():
    global current_angle, angular_dir
    dir = DIR_RIGHT if current_angle < 0 else DIR_LEFT
    inc_value = REV_ANGLE if current_angle < 0 else -REV_ANGLE
    angular_dir = H_Directions.straight

    print("Reseting angle")
    turn_angle = abs(current_angle) // REV_ANGLE
    for i in range(turn_angle):
        turn_wheels(dir)
        current_angle += inc_value
    print("Reset complete")


# Halts any actions
def halt():
    global movement_dir
    movement_dir = V_Directions.halted
    print("Halt")


# Manipulates motors based on angular_dir and movement_dir values
def run():
    global current_angle, angular_dir, movement_dir
    if angular_dir == H_Directions.left:
        if current_angle <= -MAX_ANGLE:
            print("Max angle reached")
        else:
            print("Turning left")
            current_angle -= REV_ANGLE
            turn_wheels(DIR_LEFT)
    elif angular_dir == H_Directions.right:
        if current_angle >= MAX_ANGLE:
            print("Max angle reached")
        else:
            print("Turning right")
            current_angle += REV_ANGLE
            turn_wheels(DIR_RIGHT)

    if movement_dir == V_Directions.forward:
        print("Moving forward")
        rotate_wheels(DIR_FORWARD)
    elif movement_dir == V_Directions.backward:
        print("Moving backward")
        rotate_wheels(DIR_BACKWARD)

    # Stop PWM
    if movement_dir == V_Directions.halted:
        generate_pwm(0)


# Turns stepper motor in desired direction
def turn_wheels(dir):
    # Set direction of stepper motors
    digitalWrite(SL_DIR, dir)
    digitalWrite(SR_DIR, dir)

    # Generate pulse
    generate_pulse()


# Generates pulse for stepper motors
def generate_pulse():
    digitalWrite(SL_PULSE, HIGH)
    digitalWrite(SR_PULSE, HIGH)
    delay(ANGULAR_SPEED)
    digitalWrite(SL_PULSE, LOW)
    digitalWrite(SR_PULSE, LOW)
    delay(ANGULAR_SPEED)


# Turns gearbox motor in desired direction
def rotate_wheels(dir):
    # Set direction of gearbox motors
    # The motors on the right side are flipped, hence not dir
    digitalWrite(GFL_DIR, dir)
    digitalWrite(GFR_DIR, not dir)
    digitalWrite(GBL_DIR, dir)
    digitalWrite(GBR_DIR, not dir)

    # Generate PWM
    generate_pwm(PWM_SPEED)


# generate pwm for gearbox motors
def generate_pwm(spd):
    analogWrite(GFL_PWM, spd)
    analogWrite(GFR_PWM, spd)
    analogWrite(GBL_PWM, spd)
    analogWrite(GBR_PWM, spd)
