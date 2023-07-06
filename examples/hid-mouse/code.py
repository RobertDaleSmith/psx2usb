# SPDX-FileCopyrightText: 2023 Robert Dale Smith w/ Tod Kurt ps2controller library
#
# SPDX-License-Identifier: MIT
# Simple PlayStation controller to standard USB HID mouse.
# Tested on QT Py RP2040

import math
import time
import board
import usb_hid
import neopixel
from adafruit_hid.mouse import Mouse
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

mouse = Mouse(usb_hid.devices)

buttonmap = {
            ("L1"): (0, Mouse.RIGHT_BUTTON), #L1
            ("R1"): (0, Mouse.LEFT_BUTTON), #R1
}

print("PlayStation to HID Mouse")

lastX2 = 0
lastY2 = 0

# defined USB HID devices
for i, device in enumerate(usb_hid.devices):
    print('Device {}:'.format(i+1))
    print('  Usage Page: {}'.format(device.usage_page))
    print('  Usage ID: {}'.format(device.usage))
    print()

while True:
    time.sleep(0.00005)  # debounce delay
    events = psx.update()
    data = psx.data

    print(data) # raw controller data

    currX2 = 0
    currY2 = 0
    mouseX = 0
    mouseY = 0

    if (len(data) > 5): # has right analog
        currX2 = psx.analog_right()[0]
        currY2 = psx.analog_right()[1]

    if ((lastX2 != currX2) | (lastY2 != currY2)):
        print("[z:", currX2, "rz:", currY2, "]")
        lastX2 = currX2
        lastY2 = currY2

        if data[1] == CTRL_TYPE_MOUSE:
            if currX2 != 0:
                if currX2 <= 127:
                    mouseX = currX2
                else:
                    mouseX = (currX2 - 256)
            if currY2 != 0:
                if currY2 <= 127:
                    mouseY = currY2
                else:
                    mouseY = (currY2 - 256)

            print("Mouse: x:", mouseX, " y:", mouseY)
            mouse.move(x=mouseX)
            mouse.move(y=mouseY)
    if events:
        print(events)

        for event in events:
            if buttonmap[event.name][0] == 0:  # regular button
                if event.pressed:
                    if event.name == "R1":
                        mouse.press(buttonmap[event.name][1])
                    if event.name == "L1":
                        mouse.press(buttonmap[event.name][1])
                else:
                    mouse.release_all()