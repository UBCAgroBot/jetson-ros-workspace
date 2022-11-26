#!/usr/bin/env python3

import pyfirmata
import time

# Change this!
MYPORT = '/dev/tty.usbmodem1101'


if __name__ == '__main__':
    board = pyfirmata.Arduino(MYPORT)
    print("Communication Successfully started")
    
    # will make the board led light flash rapidly
    while True:
        board.digital[13].write(1)
        time.sleep(0.05)
        board.digital[13].write(0)
        time.sleep(0.05)