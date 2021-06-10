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
keys[evdev.ecodes.KEY_PAGEDOWN] = ButtonPress.Request.BUTTON_NEXT
keys[evdev.ecodes.KEY_PAGEUP] = ButtonPress.Request.BUTTON_PREV
keys[evdev.ecodes.KEY_B] = ButtonPress.Request.BUTTON_ALT

class RemoteNode(Node):

    def __init__(self):
        super().__init__('remote')
        self.cli = self.create_client(ButtonPress, 'remote')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ButtonPress.Request()

    def send_request(self, button):
        self.req.button = button
        self.future = self.cli.call_async(self.req)



rclpy.init()

remote_client = RemoteNode()

for event in dev.read_loop():
    
    if not rclpy.ok():
        break
    
    if event.type == evdev.ecodes.EV_KEY:
        print(evdev.categorize(event))
        print(event.type)
        print(event.value)
        print(event.code)
        
        if event.value and event.code in keys:
            remote_client.send_request(keys[event.code])
            
remote_client.destroy_node()
rclpy.shutdown()
