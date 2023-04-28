from evdev import list_devices, InputDevice, categorize, ecodes

# https://stackoverflow.com/questions/44934309/how-to-access-the-joysticks-of-a-gamepad-using-python-evdev

CENTER_TOLERANCE = 150
STICK_MAX = 65536

dev = InputDevice(list_devices()[0])

axis = {
    ecodes.ABS_X: 'ls_x',  # 0 - 65,536   the middle is 32768
    ecodes.ABS_Y: 'ls_y',
    ecodes.ABS_RX: 'rs_x',
    ecodes.ABS_RY: 'rs_y',
    2: 'lt',  # 0 - 1023
    5: 'rt',

    ecodes.ABS_HAT0X: 'dpad_x',  # -1 - 1
    ecodes.ABS_HAT0Y: 'dpad_y'
}

center = {
    'ls_x': STICK_MAX/2,
    'ls_y': STICK_MAX/2,
    'rs_x': STICK_MAX/2,
    'rs_y': STICK_MAX/2
}

last = {
    'ls_x': STICK_MAX/2,
    'ls_y': STICK_MAX/2,
    'rs_x': STICK_MAX/2,
    'rs_y': STICK_MAX/2,
    'rt': 0,
    'lt': 0
}


def listen_gamepad():
    move, turn = 'H', 'S'
    for event in dev.read_loop():
        # deleted calibration, might need in the future, look  at stackoverflow answer above
        if event.type == ecodes.EV_ABS and (event.code in list(axis)):
            last[axis[event.code]] = event.value
            value = event.value
            if abs(value) <= CENTER_TOLERANCE:
                value = 0
            if event.code == 2:
                if value == 0:
                    move = 'H'
                else:
                    move = 'B'
            elif event.code == 5:
                if value == 0:
                    move = 'H'
                else:
                    move = 'F'

            elif axis[event.code] == 'ls_x':
                print(value)
                if value < 0:
                    turn = 'L'
                elif value == 0:
                    turn = 'S'
                else:
                    turn = 'R'
        print(move + ", " + turn)
