#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String

ROS_IN_TOPIC  = '/pick_color'
ESP_IN_TOPIC  = '/arm_ack'

ESP_OUT_TOPIC = '/esp_cmd'
ROS_OUT_TOPIC = '/box_done'

COLOR_TO_CMD = {
    1: '1',
    2: '2',
    3: '3',
}

CMD_TO_DONE = {
    '1': 'done red',
    '2': 'done green',
    '3': 'done blue',
}

class ArmSequencer(Node):

    def __init__(self):
        super().__init__('arm_sequencer')

        self.create_subscription(Int8,   ROS_IN_TOPIC, self._pick_cb, 10)
        self.create_subscription(String, ESP_IN_TOPIC, self._done_cb, 10)

        self._cmd_pub  = self.create_publisher(String, ESP_OUT_TOPIC, 10)
        self._done_pub = self.create_publisher(String, ROS_OUT_TOPIC, 10)

    def _pick_cb(self, msg: Int8) -> None:

        cmd = COLOR_TO_CMD.get(msg.data)
        if cmd is None:
            self.get_logger().warn(f'Invalid pick_color value: {msg.data}')
            return
        self._last_cmd = cmd
        self._cmd_pub.publish(String(data=cmd))

    def _done_cb(self, msg: String) -> None:

        confirm = msg.data.strip()
        done = CMD_TO_DONE.get(confirm)
        if done is None:
            return
        self._done_pub.publish(String(data=done))

def main(args=None):
    rclpy.init(args=args)
    node = ArmSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
