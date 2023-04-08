from enum import Enum

# LEFT and RIGHT output values for stepper motor
DIR_RIGHT = 1;
DIR_LEFT = 0;

# FORWARD and BACKWARD output values for gearbox motor
DIR_FORWARD = 1;
DIR_BACKWARD = 0;

# Angular Speed, recommended setting 7, DO NOT DECREASE TO < 3
ANGULAR_SPEED = 7 / 1000; # python sleep uses seconds

# Strength of PWM, between 0 and 255, higher is faster
PWM_SPEED = 80; # % duty cycle
PWM_MAX_SPEED = 100;
PWM_FREQUENCY = 100;

# 1 pulse to angle
# 400 revs per 360 degrees
REV_ANGLE = 360.0 / 400.0;

# max turning angle
MAX_ANGLE = 90.0;

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

# level 0: once every DEBUG_FREQUENCY seconds
# level 1: no timing restrictions
DEBUG_LEVEL = 1
DEBUG_FREQUNCY = 1 # seconds

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
DFL_DIR = 11;
DFL_PWM = 13;
DFR_DIR = 15;
DFR_PWM = 16;
DBL_DIR = 18;
DBL_PWM = 22;
DBR_DIR = 29;
DBR_PWM = 31;
GL_DIR = 32;
GL_PULSE = 33;
GR_DIR = 36;
GR_PULSE = 37;

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