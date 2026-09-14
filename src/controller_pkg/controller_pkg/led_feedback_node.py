#!/usr/bin/env python3
#
# led_feedback_node — reflects robot mode/module on the DualSense RGB LED.
#
# Subscribes:
#   /robot_mode   (std_msgs/String)  "ARM" | "BASE"
#   auto_active   (std_msgs/String)  "MAN" | "AUTO"
#   /camera_data  (std_msgs/String)  "red" | "green" | "blue"  (or "Color:<name>")
#
# LED colors:
#   BASE + MAN  → blue   (0,   255,   255)
#   ARM  + MAN  → green  (255,   255, 0  )
#   AUTO        → red    (255, 0,   255  )
#   QR red      → red    (255, 0,   0  )   for 5 s then restores
#   QR green    → green  (0,   255, 0  )   for 5 s then restores
#   QR blue     → blue   (0,   0,   255)   for 5 s then restores

import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MODULE_ARM  = 'ARM'
MODULE_BASE = 'BASE'
MODE_AUTO   = 'AUTO'
MODE_MANUAL = 'MAN'

COLOR_BASE_MAN = (0,   255,   255)
COLOR_ARM_MAN  = (255,   255, 0  )
COLOR_AUTO     = (255, 0,   255  )
COLOR_OFF      = (255,   255,   255  )

QR_COLOR_MAP = {
    'red':   (255, 0,   0  ),
    'green': (0,   255, 0  ),
    'blue':  (0,   0,   255),
}

QR_FLASH_DURATION = 5.0  # seconds

SCRIPT_PATH = '/home/matem/kalaks_ws/set_led.sh'


def _set_led(r: int, g: int, b: int) -> None:
    subprocess.run(
        ['sudo', SCRIPT_PATH, str(r), str(g), str(b)],
        capture_output=True,
    )


class LedFeedbackNode(Node):

    def __init__(self):
        super().__init__('led_feedback_node')

        self._module = MODULE_BASE
        self._mode   = MODE_MANUAL
        self._qr_timer = None

        self._sub_module = self.create_subscription(
            String, '/robot_mode', self._module_cb, 10)
        self._sub_mode = self.create_subscription(
            String, 'auto_active', self._mode_cb, 10)
        self._sub_camera = self.create_subscription(
            String, '/camera_data', self._camera_cb, 10)

        self._update_led()
        self.get_logger().info('led_feedback_node up')

    def _module_cb(self, msg: String) -> None:
        val = msg.data.upper()
        if val != self._module:
            self._module = val
            if self._qr_timer is None:
                self._update_led()

    def _mode_cb(self, msg: String) -> None:
        val = msg.data.upper()
        if val != self._mode:
            self._mode = val
            if self._qr_timer is None:
                self._update_led()

    def _camera_cb(self, msg: String) -> None:
        data = msg.data
        if data.lower().startswith('color:'):
            color_name = data.split(':', 1)[1].strip().lower()
        else:
            color_name = data.strip().lower()
        color = QR_COLOR_MAP.get(color_name)
        if color is None:
            self.get_logger().warn(f'Unknown QR color: {color_name}')
            return

        # Cancel any existing flash timer
        if self._qr_timer is not None:
            self._qr_timer.cancel()
            self._qr_timer = None

        _set_led(*color)
        self.get_logger().info(f'QR flash → color={color_name} rgb={color}')

        self._qr_timer = self.create_timer(QR_FLASH_DURATION, self._qr_timer_cb)

    def _qr_timer_cb(self) -> None:
        self._qr_timer.cancel()
        self._qr_timer = None
        self._update_led()
        self.get_logger().info('QR flash ended, restoring LED')

    def _update_led(self) -> None:
        if self._mode == MODE_AUTO:
            color = COLOR_AUTO
        elif self._module == MODULE_ARM:
            color = COLOR_ARM_MAN
        else:
            color = COLOR_BASE_MAN

        _set_led(*color)
        self.get_logger().info(
            f'LED → module={self._module} mode={self._mode} rgb={color}')


def main(args=None):
    rclpy.init(args=args)
    node = LedFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _set_led(*COLOR_OFF)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
