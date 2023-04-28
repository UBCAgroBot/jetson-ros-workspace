import RPi.GPIO as GPIO
from time import sleep, time
from pynput import keyboard
from constants import *
from rpi_helper import turn_wheels, rotate_wheels, generate_pwm, reset_angle, debug_print, setup_rpi
import signal
import sys


# global variables
current_angle = 0.0
movement_arr = [V_Directions.halted, H_Directions.straight]
pwm_controls = []

def run():
  global current_angle, movement_arr
  movement, turning = movement_arr
  debug_print(movement, turning, "angle ", current_angle)
  try:
      if movement == V_Directions.forward:
        debug_print("Moving forward")
        rotate_wheels(DIR_FORWARD, pwm_controls=pwm_controls)
      elif movement == V_Directions.backward:
        debug_print("Moving backward")
        rotate_wheels(DIR_BACKWARD, pwm_controls=pwm_controls)
      else:
        # stop moving
        generate_pwm(0, pwm_controls=pwm_controls)
         

      if turning == H_Directions.left:
        if current_angle <= -MAX_ANGLE:
            debug_print("Max angle reached")
        else:
            debug_print("Turning left")
            current_angle -= REV_ANGLE
            turn_wheels(DIR_LEFT)
      elif turning == H_Directions.right:
        if current_angle >= MAX_ANGLE:
            debug_print("Max angle reached")
        else:
            debug_print("Turning right")
            current_angle += REV_ANGLE
            turn_wheels(DIR_RIGHT)
      else:
         # reset angle
         current_angle = reset_angle(current_angle=current_angle)


  except AttributeError:
      print("Attribute Error")



### testing with keyboard
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
          debug_print(f'{key} released')
      except AttributeError:
          if key == keyboard.Key.esc:
              # Stop listener
              return False

  # Start the keyboard listener in a non-blocking manner
  listener = keyboard.Listener(on_press=on_press, on_release=on_release)

  listener.start()

def handler(signum, frame):
    # reset angle
    reset_angle(current_angle=current_angle)
    # stop moving
    generate_pwm(0, pwm_controls=pwm_controls)
    sys.exit(0)
signal.signal(signal.SIGINT, handler)

def main():
  setup_rpi(pwm_controls=pwm_controls)
  setup_keyboard()
  while True:
    run()
    

main()
