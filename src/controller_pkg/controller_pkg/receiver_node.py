#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ReceiverNode(Node):

    def __init__(self):
        super().__init__("receiver")
        self.get