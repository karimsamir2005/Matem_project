#!/usr/bin/env python3
#
# ── DS4 / joy_linux mapping reference ──────────────────────────────────────────
#
#  AXES  idx   range        label
#  ────  ───   ──────────   ─────
#        0     -1.0 .. 1.0  L3_X      (left  stick horizontal, right = -1)
#        1     -1.0 .. 1.0  L3_Y      (left  stick vertical,   up    = +1)
#        2      1.0 .. -1.0 L2_analog (released = +1, fully pressed = -1)
#        3     -1.0 .. 1.0  R3_X      (right stick horizontal, right = -1)
#        4     -1.0 .. 1.0  R3_Y      (right stick vertical,   up    = +1)
#        5      1.0 .. -1.0 R2_analog (released = +1, fully pressed = -1)
#        6     -1.0 .. 1.0  DPAD_X    (left = +1, right = -1)
#        7     -1.0 .. 1.0  DPAD_Y    (up   = +1, down  = -1)
#
#  BUTTONS  idx   label
#  ───────  ───   ──────
#           0     X  (Cross)
#           1     O  (Circle)
#           2     △  (Triangle)
#           3     □  (Square)
#           4     L1
#           5     R1
#           6     L2  (digital)
#           7     R2  (digital)
#           8     Share
#           9     Options
#           10    PS  (home)
#           11    L3  (left  stick click)
#           12    R3  (right stick click)
#
# ───────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# Topics
INPUT_TOPIC  = '/arm_joy'
OUTPUT_TOPIC = '/arm_axes'


class ArmInputNode(Node):

    def __init__(self):
        super().__init__('arm_input_node')

        # ── Axes ──────────────────────────────────────────────────────────────
        self._axis_lx        = 0     # L3_X
        self._axis_ly        = 1     # L3_Y
        self._axis_l2_analog = 2     # L2 analog
        self._axis_rx        = 3     # R3_X
        self._axis_ry        = 4     # R3_Y
        self._axis_r2_analog = 5     # R2 analog
        self._axis_dpad_x    = 6     # D-pad horizontal
        self._axis_dpad_y    = 7     # D-pad vertical

        # ── Buttons ───────────────────────────────────────────────────────────
        self._btn_x       = 0     # X  (Cross)
        self._btn_o       = 1     # O  (Circle)
        self._btn_tri     = 2     # △  (Triangle)
        self._btn_sq      = 3     # □  (Square)
        self._btn_l1      = 4     # L1
        self._btn_r1      = 5     # R1
        self._btn_l2      = 6     # L2 digital
        self._btn_r2      = 7     # R2 digital
        self._btn_share   = 8     # Share
        self._btn_options = 9     # Options
        self._btn_ps      = 10    # PS (home)
        self._btn_l3      = 11    # L3 (left stick click)
        self._btn_r3      = 12    # R3 (right stick click)

        # ── Deadzone ──────────────────────────────────────────────────────────
        self._deadzone = 0.15

        # ── Axis inversion ────────────────────────────────────────────────────
        self._invert_lx = True
        self._invert_ly = False
        self._invert_rx = False
        self._invert_ry = False

        # ── Previous button states ────────────────────────────────────────────
        self._prev_x       = None
        self._prev_o       = None
        self._prev_tri     = None
        self._prev_sq      = None
        self._prev_l1      = None
        self._prev_r1      = None
        self._prev_l2      = None
        self._prev_r2      = None
        self._prev_share   = None
        self._prev_options = None
        self._prev_ps      = None
        self._prev_l3      = None
        self._prev_r3      = None

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self._sub = self.create_subscription(Joy, INPUT_TOPIC, self._joy_callback, 10)
        self._pub = self.create_publisher(Float32MultiArray, OUTPUT_TOPIC, 10)

        self.get_logger().info('arm_input_node up')

    # ── Helpers ───────────────────────────────────────────────────────────────
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

    # ── Callback ──────────────────────────────────────────────────────────────
    def _joy_callback(self, msg: Joy) -> None:
        # Read required axes; drop frame if any are missing.
        raw_lx = self._get_axis(msg.axes, self._axis_lx)
        raw_ly = self._get_axis(msg.axes, self._axis_ly)
        raw_rx = self._get_axis(msg.axes, self._axis_rx)
        raw_ry = self._get_axis(msg.axes, self._axis_ry)
        if None in (raw_lx, raw_ly, raw_rx, raw_ry): return

        # Read optional axes (None → 0.0 fallback).
        raw_l2a = self._get_axis(msg.axes, self._axis_l2_analog) or 0.0
        raw_r2a = self._get_axis(msg.axes, self._axis_r2_analog) or 0.0
        raw_dpx = self._get_axis(msg.axes, self._axis_dpad_x)   or 0.0
        raw_dpy = self._get_axis(msg.axes, self._axis_dpad_y)   or 0.0

        # Read required buttons; drop frame if any are missing.
        x_now  = self._get_button(msg.buttons, self._btn_x)
        o_now  = self._get_button(msg.buttons, self._btn_o)
        sq_now = self._get_button(msg.buttons, self._btn_sq)
        r1_now = self._get_button(msg.buttons, self._btn_r1)
        if None in (x_now, o_now, sq_now, r1_now): return

        # Read remaining buttons (optional — missing = not pressed).
        tri_now     = self._get_button(msg.buttons, self._btn_tri)     or False
        l1_now      = self._get_button(msg.buttons, self._btn_l1)      or False
        l2_now      = self._get_button(msg.buttons, self._btn_l2)      or False
        r2_now      = self._get_button(msg.buttons, self._btn_r2)      or False
        share_now   = self._get_button(msg.buttons, self._btn_share)   or False
        options_now = self._get_button(msg.buttons, self._btn_options) or False
        ps_now      = self._get_button(msg.buttons, self._btn_ps)      or False
        l3_now      = self._get_button(msg.buttons, self._btn_l3)      or False
        r3_now      = self._get_button(msg.buttons, self._btn_r3)      or False

        # Deadzone + inversion for primary sticks.
        lx = self._apply_inversion(self._apply_deadzone(raw_lx), self._invert_lx)
        ly = self._apply_inversion(self._apply_deadzone(raw_ly), self._invert_ly)
        rx = self._apply_inversion(self._apply_deadzone(raw_rx), self._invert_rx)
        ry = self._apply_inversion(self._apply_deadzone(raw_ry), self._invert_ry)

        # Rising-edge detection.
        x_edge,       self._prev_x       = self._rising_edge(x_now,       self._prev_x)
        o_edge,       self._prev_o       = self._rising_edge(o_now,       self._prev_o)
        sq_edge,      self._prev_sq      = self._rising_edge(sq_now,      self._prev_sq)
        r1_edge,      self._prev_r1      = self._rising_edge(r1_now,      self._prev_r1)
        tri_edge,     self._prev_tri     = self._rising_edge(tri_now,     self._prev_tri)
        l1_edge,      self._prev_l1      = self._rising_edge(l1_now,      self._prev_l1)
        l2_edge,      self._prev_l2      = self._rising_edge(l2_now,      self._prev_l2)
        r2_edge,      self._prev_r2      = self._rising_edge(r2_now,      self._prev_r2)
        share_edge,   self._prev_share   = self._rising_edge(share_now,   self._prev_share)
        options_edge, self._prev_options = self._rising_edge(options_now, self._prev_options)
        ps_edge,      self._prev_ps      = self._rising_edge(ps_now,      self._prev_ps)
        l3_edge,      self._prev_l3      = self._rising_edge(l3_now,      self._prev_l3)
        r3_edge,      self._prev_r3      = self._rising_edge(r3_now,      self._prev_r3)

        out = Float32MultiArray()
        out.data = [
            float(lx),                    # 0  L3_X
            float(ly),                    # 1  L3_Y
            float(rx),                    # 2  R3_X
            float(ry),                    # 3  R3_Y
            1.0 if x_edge  else 0.0,      # 4  X
            1.0 if o_edge  else 0.0,      # 5  O
            1.0 if sq_edge else 0.0,      # 6  □
            1.0 if l1_edge else 0.0,      # 7  L1
            1.0 if l2_edge else 0.0,      # 8  L2 digital
            1.0 if r2_edge else 0.0,      # 9  R2 digital
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
