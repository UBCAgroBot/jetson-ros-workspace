import RPi.GPIO as GPIO
from time import sleep
import serial

SERIAL_PORT = '/dev/ttyAMA0'

# LEFT and RIGHT output values for stepper motor
DIR_RIGHT = 1;
DIR_LEFT = 0;

# FORWARD and BACKWARD output values for gearbox motor
DIR_FORWARD = True;
DIR_BACKWARD = False;

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

pwm_controls = []

def setup():
  GPIO.setwarnings(False)			#disable warnings
  GPIO.setmode(GPIO.BOARD)		#set pin numbering system
  GPIO.setup(PINS,GPIO.OUT)

  for pwm in PWMS:
    pwm_control = GPIO.PWM(pwm, PWM_FREQUENCY) # create PWM instance with frequency
    pwm_control.start(0) # set 0 Duty Cycle 
    pwm_controls.append(pwm_control)

## turns stepper motor in desired direction
def turn_wheels(direction):
  GPIO.output(GL_DIR, direction)
  GPIO.output(GR_DIR, direction)
  generate_pulse()

## generates pulse for stepper motors
def generate_pulse():
  GPIO.output(GL_PULSE, GPIO.HIGH)
  GPIO.output(GR_PULSE, GPIO.HIGH)
  sleep(ANGULAR_SPEED)
  GPIO.output(GL_PULSE, GPIO.LOW)
  GPIO.output(GR_PULSE, GPIO.LOW)
  sleep(ANGULAR_SPEED)
    

## turns gearbox motor in desired direction
def rotate_wheels(direction):
  GPIO.output(DFL_DIR, int(direction))
  GPIO.output(DFR_DIR, int(not direction))
  GPIO.output(DBL_DIR, int(direction))
  GPIO.output(DBR_DIR, int(not direction))

  generate_pwm(PWM_SPEED)

## generate pwm for gearbox motors
def generate_pwm(speed):
  for pwmc in pwm_control:
    pwmc.ChangeDutyCycle(speed / PWM_MAX_SPEED)


# update direction enums
# returns true if input is a valid value, 
# othwerise returns false
int update_dir_enums(String input) {
  switch (input.charAt(0)) {
    case 'H':
      halt();
      break;
    case 'F':
      set_forward();
      break;
    case 'B':
      set_backward();
      break;
    default:
      return false;
  }
      
  switch (input.charAt(1)) {
    case 'L':
      set_left();
      break;
    case 'S':
      reset_angle();
      break;
    case 'R':
      set_right();
      break;
    default:
      return false;
  }
  return true;
}

def read_serial():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    while True:
      if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        return line
      
def run(cmd):
  movement, turning = cmd[0], cmd[1]
  print(movement, turning)

def main():
  setup()
  while True:
    cmd = read_serial()
    run(cmd)
    pass

main()