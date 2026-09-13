#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# Topics
INPUT_TOPIC         = '/arm_joy'
OUTPUT_TOPIC        = '/arm_axes'


class ArmInputNode(Node):

    def __init__(self):
        super().__init__('arm_input_node')

        # Axis
        self._axis_lx           = 0     # L3_X
        self._axis_ly           = 1     # L3_Y
        self._axis_rx           = 3     # R3_X
        self._axis_ry           = 4     # R3_Y
        # Buttons
        self._btn_grip          = 0     # X
        self._btn_release       = 1     # O
        self._btn_stepper_reset = 3     # SQ
        self._btn_wrist_toggle  = 5     # R1
        # Deadzone
        self._deadzone          = 0.15     # Drift deadzone
        # Axis inversion
        self._invert_lx         = False
        self._invert_ly         = False
        self._invert_rx         = False
        self._invert_ry         = False
        # Previous buttons states
        self._prev_grip          = None
        self._prev_release       = None
        self._prev_stepper_reset = None
        self._prev_wrist_toggle  = None

        # Pub / Sub
        self._sub = self.create_subscription(Joy, INPUT_TOPIC, self._joy_callback, 10)
        self._pub = self.create_publisher(Float32MultiArray,  OUTPUT_TOPIC, 10)

        self.get_logger().info('arm_input_node up')

    # Helpers
    def _get_axis(self, axes, idx):
        if idx >= len(axes): return None
        return float(axes[idx])

    def _get_button(self, buttons, idx):
        if idx >= len(buttons): return None
        return bool(buttons[idx])

    def _apply_deadzone(self, value):
        return value if abs(value) > self._deadzone else 0.0

    @staticmethod
    def _apply_inversion(value, invert):
        return -value if invert else value

    @staticmethod
    def _rising_edge(current, previous):
        if previous is None: return False, current
        return (current and not previous), current

    # Callback
    def _joy_callback(self, msg: Joy) -> None:
        # Read axes; drop frame if any required axis is missing.
        raw_lx = self._get_axis(msg.axes, self._axis_lx)
        raw_ly = self._get_axis(msg.axes, self._axis_ly)
        raw_rx = self._get_axis(msg.axes, self._axis_rx)
        raw_ry = self._get_axis(msg.axes, self._axis_ry)
        if None in (raw_lx, raw_ly, raw_rx, raw_ry): return

        # Read buttons; drop frame if any required button is missing.
        grip_now    = self._get_button(msg.buttons, self._btn_grip)
        release_now = self._get_button(msg.buttons, self._btn_release)
        reset_now   = self._get_button(msg.buttons, self._btn_stepper_reset)
        wrist_now   = self._get_button(msg.buttons, self._btn_wrist_toggle)
        if None in (grip_now, release_now, reset_now, wrist_now): return

        # Deadzone + inversion
        lx = self._apply_inversion(self._apply_deadzone(raw_lx), self._invert_lx)
        ly = self._apply_inversion(self._apply_deadzone(raw_ly), self._invert_ly)
        rx = self._apply_inversion(self._apply_deadzone(raw_rx), self._invert_rx)
        ry = self._apply_inversion(self._apply_deadzone(raw_ry), self._invert_ry)

        # Rising-edge detection (and previous-state update)
        grip_edge,    self._prev_grip          = self._rising_edge(grip_now,    self._prev_grip)
        release_edge, self._prev_release       = self._rising_edge(release_now, self._prev_release)
        reset_edge,   self._prev_stepper_reset = self._rising_edge(reset_now,   self._prev_stepper_reset)
        wrist_edge,   self._prev_wrist_toggle  = self._rising_edge(wrist_now,   self._prev_wrist_toggle)

        if grip_edge:    self.get_logger().info('GRIP pressed')
        if release_edge: self.get_logger().info('RELEASE pressed')
        if reset_edge:   self.get_logger().info('STEPPER RESET pressed')
        if wrist_edge:   self.get_logger().info('WRIST TOGGLE pressed')

        out = Float32MultiArray()
        out.data = [
            float(lx),                          
            float(ly),                          
            float(rx),                          
            float(ry),                          
            1.0 if grip_edge    else 0.0,
            1.0 if release_edge else 0.0,
            1.0 if reset_edge   else 0.0,
            1.0 if wrist_edge   else 0.0,
        ]

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArmInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
