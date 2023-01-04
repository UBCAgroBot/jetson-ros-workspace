"""
Listens to keyboard presses and sends a string message to the Arduino
Output Format: "XY" where
    X = (H = Halt, F = Forward, B = Backward)
    Y = (S = Straight, L = Left, R = Right)
To be used in conjunction with the StringReceiver.ino
"""
import time
import pyfirmata
from pynput import keyboard

'''
Debug flag for button press outputs and output messages
0 = no debug(only final string printed), 1 = output message array, 2 = key presses
'''
debug_level = 1


def debug_print(output, level=0):
    """
    Function that handles debug levels
    """
    if debug_level >= level:
        print(output)


# We will keep track of movement state in this array
send_str_arr = ["H", "S"]

"""
Setup keyboard listener
"""


def on_press(key):
    try:
        debug_print(f'alphanumeric key {key.char} pressed', 2)
        if key.char == "w":
            send_str_arr[0] = "F"
        elif key.char == "s":
            send_str_arr[0] = "B"
        elif key.char == "a":
            send_str_arr[1] = "L"
        elif key.char == "d":
            send_str_arr[1] = "R"
    except AttributeError:
        debug_print(f'special key {key} pressed', 2)


def on_release(key):
    try:
        if key.char == "w":
            send_str_arr[0] = "H"
        elif key.char == "s":
            send_str_arr[0] = "H"
        elif key.char == "a":
            send_str_arr[1] = "S"
        elif key.char == "d":
            send_str_arr[1] = "S"
        debug_print(f'{key} released', 2)
    except AttributeError:
        debug_print(f'special key {key} released', 2)
        if key == keyboard.Key.esc:
            # Stop listener
            return False


# Start the keyboard listener in a non-blocking manner
listener = keyboard.Listener(on_press=on_press, on_release=on_release)

listener.start()

'''
PyFirmata Section
'''
# Update the port as required
board = pyfirmata.Arduino('/dev/cu.usbmodem14201')

# Run the program
while True:
    message = send_str_arr.copy()
    message.append("\n")
    debug_print("".join(message), 0)
    message = [ord(c) for c in message]
    debug_print(message, 1)
    board.send_sysex(pyfirmata.START_SYSEX, message)
    time.sleep(1)
    pass
