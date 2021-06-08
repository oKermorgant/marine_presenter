#!/usr/bin/env python3

'''
No-op node to store world models without actual Gazebo / Plankton simulation
'''

import rclpy
from rclpy.node import Node

class WorldPublisher(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)
        pass

rclpy.init()
world_pub = WorldPublisher('publish_world_models')
rclpy.spin(world_pub)
