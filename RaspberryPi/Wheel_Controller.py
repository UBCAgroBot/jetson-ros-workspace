import RPi.GPIO as GPIO
from time import sleep
from rpi_hardware_pwm import HardwarePWM

# LEFT and RIGHT output values for stepper motor
DIR_RIGHT = 1;
DIR_LEFT = 0;

# FORWARD and BACKWARD output values for gearbox motor
DIR_FORWARD = 1;
DIR_BACKWARD = 0;

# Angular Speed, recommended setting 7, DO NOT DECREASE TO < 3
ANGULAR_SPEED = 7 / 1000; # python sleep uses seconds

# Strength of PWM, between 0 and 255, higher is faster
PWM_SPEED = 100;
PWM_MAX_SPEED = 255;
PWM_FREQUENCY = 100;

# 1 pulse to angle
# 400 revs per 360 degrees
REV_ANGLE = 360.0 / 400.0;

# max turning angle
MAX_ANGLE = 90.0;

'''HARDWARD PWM'''
# GearboxLeftPWM = HardwarePWM(pwm_channel=0, hz=60) # pwm channel 0: GPIO_18
# GearboxRightPWM = HardwarePWM(pwm_channel=1, hz=60) # pwm channel 1: GPIO_19

# GearboxLeftPWM.start(0)
# GearboxRightPWM.start(0)

# GearboxLeftPWM.stop()
# GearboxRightPWM.stop()

# pwm.change_duty_cycle(50)
# pwm.change_frequency(25_000)


'''PINS'''
# RPI 3B+ pinout diagram: https://www.etechnophiles.com/wp-content/uploads/2020/12/R-Pi-3-B-Pinout.jpg?ezimgfmt=ng:webp/ngcb40

GFL_DIR = 11;
GFL_PWM = 13;

GFR_DIR = 15;
GFR_PWM = 16;

GBL_DIR = 18;
GBL_PWM = 22;

GBR_DIR = 29;
GBR_PWM = 31;

SL_DIR = 32;
SL_PULSE = 33;

SR_DIR = 36;
SR_PULSE = 37;

PINS = (
  GFL_DIR,
  GFL_PWM,
  GFR_DIR,
  GFR_PWM,
  GBL_DIR,
  GBL_PWM,
  GBR_DIR,
  GBR_PWM,
  SL_DIR,
  SL_PULSE,
  SR_DIR,
  SR_PULSE,
)

PWMS = (
  GFL_PWM,
  GFR_PWM,
  GBL_PWM,
  GBR_PWM,
)

GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(PINS,GPIO.OUT)

pwm_controls = []
for pwm in PWMS:
  pwm_control = GPIO.PWM(PWMS,PWM_FREQUENCY) # create PWM instance with frequency
  pwm_control.start(0) # set 0 Duty Cycle 
  pwm_controls.append(pwm_control)


## turns stepper motor in desired direction
def turn_wheels(direction):
  GPIO.output(SL_DIR, direction)
  GPIO.output(SR_DIR, direction)
  generate_pulse()

## generates pulse for stepper motors
def generate_pulse():
  GPIO.output(SL_PULSE, GPIO.HIGH)
  GPIO.output(SR_PULSE, GPIO.HIGH)
  sleep(ANGULAR_SPEED)
  GPIO.output(SL_PULSE, GPIO.LOW)
  GPIO.output(SR_PULSE, GPIO.LOW)
  sleep(ANGULAR_SPEED)
    

## turns gearbox motor in desired direction
def rotate_wheels(direction):
  GPIO.output(GFL_DIR, direction)
  GPIO.output(GFR_DIR, not direction)
  GPIO.output(GBL_DIR, direction)
  GPIO.output(GBR_DIR, not direction)

  generate_pwm(PWM_SPEED)

## generate pwm for gearbox motors
def generate_pwm(speed):
  for pwmc in pwm_control:
    pwmc.ChangeDutyCycle(speed / PWM_MAX_SPEED)

def main():
  while True:
    pass