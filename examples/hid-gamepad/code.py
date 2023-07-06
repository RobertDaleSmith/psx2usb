# SPDX-FileCopyrightText: 2023 Robert Dale Smith w/ Tod Kurt ps2controller library
#
# SPDX-License-Identifier: MIT
# Simple PlayStation controller to standard USB HID gamepad with DirectInput button mapping.
# Tested on QT Py RP2040

import math
import time
import board
import usb_hid
import neopixel
from ps2controller import PS2Controller

CTRL_TYPE_STANDARD = 65
CTRL_TYPE_DUALSHOCK = 115
CTRL_TYPE_NEGCON = 35
CTRL_TYPE_MOUSE = 18

# turn on neopixel
led = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.1)
led.fill(0x331000)  # amber while we wait for controller to connect

# create controller object with QT Py wiring
psx = PS2Controller(
    dat=board.A0,
    cmd=board.A1,
    att=board.A2,
    clk=board.A3
)
led.fill(0x0010ee)  # a nice PlayStation blue

buttonmap = {
            ("SQUARE"): (0, 0x1), #B1
            ("CROSS"): (0, 0x2), #B2
            ("CIRCLE"): (0, 0x4), #B3
            ("TRIANGLE"): (0, 0x8), #B4
            ("L1"): (0, 0x10), #L1
            ("R1"): (0, 0x20), #R1
            ("L2"): (0, 0x40), #L2
            ("R2"): (0, 0x80), #R2
            ("SELECT"): (0, 0x100), #S1
            ("START"): (0, 0x200), #S2
            ("L3"): (0, 0x400), #L3
            ("R3"): (0, 0x800), #R3
}

print("PlayStation to HID Gamepad")

lastX1 = 0
lastY1 = 0
lastX2 = 0
lastY2 = 0

# Initialize your hat switch states. Let's assume initially no button is pressed.
dpad_state = {
    "UP": 0x0,
    "DOWN": 0x0,
    "LEFT": 0x0,
    "RIGHT": 0x0,
}

# Define a dictionary mapping from the dpad_state tuples to the hat switch value.
hat_map = {
    (0, 0, 0, 0): 0x08,  # Released
    (1, 0, 0, 0): 0x00,  # N
    (1, 1, 0, 0): 0x01,  # NE
    (0, 1, 0, 0): 0x02,  # E
    (0, 1, 0, 1): 0x03,  # SE
    (0, 0, 0, 1): 0x04,  # S
    (0, 0, 1, 1): 0x05,  # SW
    (0, 0, 1, 0): 0x06,  # W
    (1, 0, 1, 0): 0x07,  # NW
}

gamepad_device = None

for device in usb_hid.devices:
    if device.usage_page == 0x01 and device.usage == 0x05:
        gamepad_device = device
        break

if gamepad_device is not None:
    print("Gamepad device found!")
else:
    print("Gamepad device not found.")

report = bytearray(19)
report[2] = 0x08  # default released hat switch value
prev_report = bytearray(report)

# defined USB HID devices
for i, device in enumerate(usb_hid.devices):
    print('Device {}:'.format(i+1))
    print('  Usage Page: {}'.format(device.usage_page))
    print('  Usage ID: {}'.format(device.usage))
    print()

while True:
    events = psx.update()
    data = psx.data

    # print(data) # raw controller data

    currX1 = 0
    currY1 = 0
    currX2 = 0
    currY2 = 0

    if (len(data) > 7): # has left analog
        currX1 = psx.analog_left()[0]
        currY1 = psx.analog_left()[1]
    if (len(data) > 5): # has right analog
        currX2 = psx.analog_right()[0]
        currY2 = psx.analog_right()[1]

    if ((lastX1 != currX1) | (lastY1 != currY1) | (lastX2 != currX2) | (lastY2 != currY2)):
        print("[x:", currX1, "y:", currY1, "z:", currX2, "rz:", currY2, "]")
        lastX1 = currX1
        lastY1 = currY1
        lastX2 = currX2
        lastY2 = currY2
        report[3] = currX1
        report[4] = currY1
        report[5] = currX2
        report[6] = currY2

    if events:
        print(events)
        # print("[Analog: L:", psx.analog_left(), "R:", psx.analog_right(), "]")
        # for x in range(15):
        #     print("Button ", x, ":", psx.analog_button(x))

        for event in events:
            if event.name in dpad_state:  # d-pad button
                dpad_state[event.name] = 1 if event.pressed else 0

            elif buttonmap[event.name][0] == 0:  # regular button
                if event.pressed:
                    # Use bitwise OR to set the corresponding bit in the report.
                    button_value = buttonmap[event.name][1]
                    if button_value <= 0xFF:
                        report[0] |= button_value
                    else:
                        report[1] |= (button_value >> 8)
                else:
                    # If the button is not pressed, use bitwise AND with the inverse
                    # of the button value to clear the corresponding bit in the report.
                    button_value = buttonmap[event.name][1]
                    if button_value <= 0xFF:
                        report[0] &= ~button_value
                    else:
                        report[1] &= ~(button_value >> 8)
    # prevent oposite directions from occuring
    if (dpad_state["UP"] & dpad_state["DOWN"]):
        dpad_state["DOWN"] = 0
    if (dpad_state["LEFT"] & dpad_state["RIGHT"]):
        dpad_state["RIGHT"] = 0

    # Use the hat_map to get the hat switch value based on the current dpad state.
    hat_switch_value = hat_map[tuple(dpad_state.values())]
    report[2] = hat_switch_value  # update the hat switch value in the report

    if prev_report != report:
        gamepad_device.send_report(report) # Send the report.
        prev_report[:] = report
