import time
import pyfirmata
import signal

'''
Debug flag for button press outputs and output messages
0 = no debug(only final string printed), 1 = output message array, 2 = key presses
'''
debug_level = 1
arduino_path = '/dev/ttyACM0'
message_delay = 0.2
flip_delay = 3


def debug_print(output, level=0):
    """
    Function that handles debug levels
    """
    if debug_level >= level:
        print(output)


# We will keep track of movement state in this array
send_str_arr1 = ["F", "S"]
send_str_arr2 = ["B", "S"]
actual_str_arr = send_str_arr1
is_moving_forward = True
last_sent_time = time.monotonic()

def shutdown(signal, frame):
    # your shutdown code goes here
    message = ["H", "S"]
    message.append("\n")
    debug_print("".join(message), 0)
    message = [ord(c) for c in message]
    debug_print(message, 1)
    board.send_sysex(pyfirmata.START_SYSEX, message)
    print("Shutting down...")
    exit(0)

# register the shutdown function to be called on SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, shutdown)



'''
PyFirmata Section
'''
# Update the port as required
board = pyfirmata.Arduino(arduino_path)

# Run the program
while True:
    message = actual_str_arr.copy()
    message.append("\n")
    debug_print("".join(message), 0)
    message = [ord(c) for c in message]
    debug_print(message, 1)
    board.send_sysex(pyfirmata.START_SYSEX, message)
    time.sleep(message_delay)
    elapsed_time = time.monotonic() - last_sent_time
    if elapsed_time >= 5:
        if is_moving_forward:
            actual_str_arr = send_str_arr2
            is_moving_forward = False
        else:
            actual_str_arr = send_str_arr1
            is_moving_forward = True
        last_sent_time = time.monotonic()
    pass
