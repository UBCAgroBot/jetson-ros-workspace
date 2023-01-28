import pyfirmata

class arduino_control:
    """ Send angle data to arduino through serial USB port.
    """

    def __init__(self, port="/dev/ttyACM0", debug_mode=False):
        """
        Parameters
        ----------
        port: The USB port used by arduino, Default port is /dev/ttyACM0 which is the one used by Jetson. PCs use /dev/cu.usbmodem14201.
        debug_mode: When True, it prints messages sent by arduino via arduino's serial monitor.
        """
        self.arduino = pyfirmata.Arduino(port)
        self.debug_mode = debug_mode

    def send(self, move="H", turn="S"):
        """ Send turning and movement information as strings
        Parameters
        ----------
        move: "F" = forward "H" = hold "B" = backwards
        turn: "L" = left "S" = straigh "R" = right
        """
        message = [move, turn, "\n"]
        message = [ord(c) for c in message]
        print("Message sent to arduino:", message)
        self.arduino.send_sysex(pyfirmata.START_SYSEX, message)
