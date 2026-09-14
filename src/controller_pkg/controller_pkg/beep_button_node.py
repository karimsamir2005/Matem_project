#!/usr/bin/env python3
#
# beep_button_node — watches one configurable button index on /base_joy
# and publishes std_msgs/Bool True on /beep whenever that button is pressed.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

BUTTON_INDEX = 0


class BeepButtonNode(Node):

    def __init__(self):
        super().__init__('beep_button_node')

        self._pub = self.create_publisher(Bool, '/beep', 10)
        self._sub = self.create_subscription(
            Joy, '/base_joy', self._joy_cb, 10)

    def _joy_cb(self, msg: Joy) -> None:
        if BUTTON_INDEX >= len(msg.buttons):
            return
        out = Bool()
        out.data = bool(msg.buttons[BUTTON_INDEX])
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BeepButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
