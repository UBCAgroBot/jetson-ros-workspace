import time
import math
import serial

class arduino_control:

    def __init__(self, port="/dev/ttyACM0", debug_mode=False):
        self.arduino = serial.Serial(port=port, baudrate=115200, timeout=0.1)
        self.debug_mode = debug_mode

    def send(self, angle, speed=1):
        self.arduino.write(bytes(str(angle),"ascii"))
        if (self.debug_mode):
            print("ARDUINO CONFIRMS THIS DATA:", int(str(self.arduino.readline()).split("\\")[0].split("'")[1]))

    def search_ports():
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print("PORT FOUND:", p)
