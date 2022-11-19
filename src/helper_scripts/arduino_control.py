import time
import math
import serial


class arduino_control:
    """ Send data to arduino through serial USB port.
    """

    def __init__(self, port="/dev/ttyACM0", debug_mode=False):
        """
        Parameters
        ----------
        port: The USB port used by arduino, Default port is /dev/ttyACM0 which is the one used by Jetson. PCs usually to use COM4.
        debug_mode: When True, it prints messages sent by arduino via arduino's serial monitor.
        """
        self.arduino = serial.Serial(port=port, baudrate=115200, timeout=0.1)
        self.debug_mode = debug_mode

    def send(self, angle, speed=1):
        """ Current version only sends angle to the arduino
        Parameters
        ----------
        angle: Must be between -90 and 90.
        """
        self.arduino.write(bytes(str(angle), "ascii"))
        if (self.debug_mode):
            print("ARDUINO CONFIRMS THIS DATA:", int(str(self.arduino.readline()).split("\\")[0].split("'")[1]))

    def search_ports():
        """ Show which USB ports are available. When arduino is conneced, one of these ports connects to it.
        """
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print("PORT FOUND:", p)
