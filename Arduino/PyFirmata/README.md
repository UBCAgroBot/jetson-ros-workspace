# This code will let you control the arduino using python on your laptop or raspberry pi

Setting up pyfirmata [Tutorial](https://roboticsbackend.com/control-arduino-with-python-and-pyfirmata-from-raspberry-pi/)

Also for full tutorial documentation see the pdf [here](https://images-na.ssl-images-amazon.com/images/I/D1oC-c3G5TS.pdf)
DC motor is tutorial 29

Notes:
run [blink_led_example.py](blink_led_example.py) to test that pyfirmata is working and write high and low pin 13 on the arduino

# DC motor control
Allow for control of DC motor direction and speed

[TUTORIAL](https://www.youtube.com/watch?v=_v0qlJ2J-xw)

Connect the external power source as per the video tutorial

run [dc_motor_controller.py](dc_motor_controller.py)
with the schematic shown in the video at [7:26](https://youtu.be/_v0qlJ2J-xw?t=446)

should spin the motor forwards and backwards at the designated speeds
