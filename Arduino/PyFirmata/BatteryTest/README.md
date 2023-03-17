# Usage

This code is designed to test the battery of the robot using the PyFirmata library and an Arduino board. 
To use this code, you will need to have an Arduino board connected to your computer and have 
the PyFirmata library installed.

1. Connect your Arduino board to your computer using a USB cable.
2. Modify the `arduino_path` variable to match the serial port of your Arduino board. This can usually be found in the Arduino IDE under Tools > Port.
3. The `send_str_arr1` and `send_str_arr2` variables define the movement commands you want to send to the robot. The first element in each array is the command to move forward or backward, and the second element is the command to stop movement. 
4. Modify the `flip_delay` variable to set the time delay between flips of the movement command.
5. Run the code and observe the movement of your device.

Note: You can press `Ctrl+C` to stop the program and shut down the device properly.
