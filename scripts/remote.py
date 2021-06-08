#!/usr/bin/env python

import evdev
import rclpy
from rclpy.node import Node
from marine_presenter.srv import ButtonPress
from sys import exit
from time import sleep

# code
# 104 KEY_PAGEUP
# 109 KEY_PAGEDOWN
# 48 KEY_B

# value = 1

rclpy.init()

node = Node('remote')

rospy.wait_for_service('remote')
remote = rospy.ServiceProxy('remote', ButtonPress)

# find presenter
dev = None
while dev == None:
    for fn in evdev.list_devices():
        dev = evdev.InputDevice(fn)
        if 'Kensington' in dev.name:
            break
        
    if 'Kensington' in dev.name:
        break
    sleep(1)

keys = {}
keys[evdev.ecodes.KEY_PAGEDOWN] = ButtonPressRequest.BUTTON_NEXT
keys[evdev.ecodes.KEY_PAGEUP] = ButtonPressRequest.BUTTON_PREV
keys[evdev.ecodes.KEY_B] = ButtonPressRequest.BUTTON_ALT

# loop and call service with pressed button
for event in dev.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
        print(evdev.categorize(event))
        print(event.type)
        print(event.value)
        print(event.code)
        
        if event.value and event.code in keys:
            remote.call(keys[event.code])
        
    if rospy.is_shutdown():
        exit(0)
