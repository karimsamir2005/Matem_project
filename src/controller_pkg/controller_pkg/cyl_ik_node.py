#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# Topics
IN_AXES_TOPIC  = '/arm_axes'
IN_FB_TOPIC    = '/arm_feedback'
OUT_JT_TOPIC   = '/arm_jt_targets'
OUT_POS_TOPIC  = '/arm_position'
OUT_ST_TOPIC   = '/joint_states'

# Link lengths (m)
L1 = 0.13
L2 = 0.18
L3 = 0.195

# Singularity guards
R_MIN = abs(L1 - L2) + 0.017
R_MAX = L1 + L2 - 0.01

# ── Per-frame rate cap for shoulder and elbow ─────────────────────────────────
MAX_DELTA_DEG = 3.0
MAX_DELTA_RAD = math.radians(MAX_DELTA_DEG)

# ── Cartesian step per tick at full stick ─────────────────────────────────────
TIP_SPEED = 0.12   # m/s

# Frequency
TICK_HZ  = 50.0
TICK_DT  = 1.0 / TICK_HZ

# ── Watchdog ──────────────────────────────────────────────────────────────────
AXES_STALE_TIMEOUT_S = 0.20

# ── Joystick zone thresholds ──────────────────────────────────────────────────
ZONE_DEAD = 0.15
ZONE_FAST = 0.75

# ── /arm_axes input layout ────────────────────────────────────────────────────
IDX_LX, IDX_LY, IDX_RX, IDX_RY              = 0, 1, 2, 3
IDX_GRIP, IDX_RELEASE, IDX_RESET, IDX_WRIST = 4, 5, 6, 7
ARM_AXES_LEN = 8

# ── /arm_feedback input layout (degrees from embedded) ───────────────────────
FB_IDX_ST, FB_IDX_SH, FB_IDX_EL, FB_IDX_WR = 0, 1, 2, 3
FB_LEN = 4

# ── /arm_jt_targets output layout ────────────────────────────────────────────
OUT_IDX_ST, OUT_IDX_SH, OUT_IDX_EL, OUT_IDX_WR = 0, 1, 2, 3
OUT_IDX_GRIP, OUT_IDX_RST                       = 4, 5
OUT_LEN = 6

URDF_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

ORIENT_HORIZONTAL = 'HORIZONTAL'
ORIENT_DOWN       = 'DOWN'


# Helpers
def clamp(value: float, low: float, high: float):
    return low if value < low else high if value > high else value

def joy_zone(value: float):
    if abs(value) < ZONE_DEAD: return 0
    sign = 1 if value > 0 else -1
    return sign * (2 if abs(value) >= ZONE_FAST else 1)

def solve_ik(r: float, z: float):
    D = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2)
    D = clamp(D, -1.0, 1.0)
    elbow = -math.acos(D)      # negative = elbow-up
    shoulder = math.atan2(z, r) - math.atan2(L2 * math.sin(elbow), L1 + L2 * math.cos(elbow))
    return shoulder, elbow

def solve_fk(shoulder: float, elbow: float):
    return (
        L1 * math.cos(shoulder) + L2 * math.cos(shoulder + elbow),
        L1 * math.sin(shoulder) + L2 * math.sin(shoulder + elbow),
    )

class CylIkNode(Node):

    def __init__(self):
        super().__init__('cyl_ik_node')

        self._sign_j1 =  1
        self._sign_j2 =  1
        self._sign_j3 = -1
        self._sign_j4 =  1

        self._theta_st = 0.0
        self._theta_sh = 0.0#(138.0 - 9.0)
        self._theta_el = 0.0#(38.0 - 132.0)
        self._theta_wr = 0.0
        self._feedback_received = False

        self._wrist_orient   = ORIENT_HORIZONTAL
        self._gripper_closed = False
        self._reset_state    = False

        self._latest_axes: list | None = None
        self._latest_axes_t = self.get_clock().now()
        self._pending_grip    = False
        self._pending_release = False
        self._pending_reset   = False
        self._pending_wrist   = False

        self._sub_axes     = self.create_subscription(Float32MultiArray, IN_AXES_TOPIC, self._axes_callback, 10)
        # self._sub_fb       = self.create_subscription(Float32MultiArray, IN_FB_TOPIC, self._feedback_callback, 10)
        self._pub_targets  = self.create_publisher(Float32MultiArray, OUT_JT_TOPIC,  10)
        # self._pub_position = self.create_publisher(Float32MultiArray, OUT_POS_TOPIC, 10)
        self._pub_states   = self.create_publisher(JointState,        OUT_ST_TOPIC,  10)

        self._timer = self.create_timer(TICK_DT, self._tick)

        self.get_logger().info('cyl_ik_node up')

    # Callback
    def _feedback_callback(self, msg: Float32MultiArray):
        if len(msg.data) < FB_LEN: return

        self._theta_st = math.radians(msg.data[FB_IDX_ST])
        self._theta_sh = math.radians(msg.data[FB_IDX_SH])
        self._theta_el = math.radians(msg.data[FB_IDX_EL])
        self._theta_wr = math.radians(msg.data[FB_IDX_WR])

        # For debugging
        if not self._feedback_received:
            self._feedback_received = True
            self.get_logger().debug(
                f'embedded init: st={msg.data[0]:.1f}°  sh={msg.data[1]:.1f}°  '
                f'el={msg.data[2]:.1f}°  wr={msg.data[3]:.1f}°'
            )

        # Compute and publish current EE position from feedback angles
        r, z = solve_fk(self._theta_sh, self._theta_el)
        pos = Float32MultiArray()
        pos.data = [msg.data[FB_IDX_ST], r, z]   # [stepper°, r_m, z_m]
        self._pub_position.publish(pos)

    def _axes_callback(self, msg: Float32MultiArray):
        self._latest_axes   = list(msg.data[:ARM_AXES_LEN])
        self._latest_axes_t = self.get_clock().now()
        if msg.data[IDX_GRIP]    == 1: self._pending_grip    = True
        if msg.data[IDX_RELEASE] == 1: self._pending_release = True
        if msg.data[IDX_RESET]   == 1: self._pending_reset   = True
        if msg.data[IDX_WRIST]   == 1: self._pending_wrist   = True

    # ── Tick ──────────────────────────────────────────────────────────────────
    def _tick(self):
        # Watchdog
        if self._latest_axes is None:
            axes = [0.0] * ARM_AXES_LEN
        else:
            age_s = (self.get_clock().now() - self._latest_axes_t).nanoseconds * 1e-9
            if age_s > AXES_STALE_TIMEOUT_S:
                self.get_logger().warn(
                    f'/arm_axes stale ({age_s*1000:.0f} ms) — zeroing',
                    throttle_duration_sec=2.0,
                )
                axes = [0.0] * ARM_AXES_LEN
            else:
                axes = self._latest_axes

        lx = axes[IDX_LX]
        rx = axes[IDX_RX]
        ry = axes[IDX_RY]

        # Consume pending button edges
        if self._pending_grip:
            self._gripper_closed = True
            self.get_logger().info('gripper -> CLOSED')
        if self._pending_release:
            self._gripper_closed = False
            self.get_logger().info('gripper -> OPEN')
        if self._pending_reset:
            self._reset_state = not self._reset_state
            self.get_logger().info(f'reset -> {"ON" if self._reset_state else "OFF"}')
        if self._pending_wrist:
            self._wrist_orient = (
                ORIENT_DOWN if self._wrist_orient == ORIENT_HORIZONTAL
                else ORIENT_HORIZONTAL
            )
            self.get_logger().info(f'wrist -> {self._wrist_orient}')
        self._pending_grip = self._pending_release = \
            self._pending_reset = self._pending_wrist = False

        # ── Stepper: zone joystick, update shadow for RViz ────────────────────
        st_zone = joy_zone(lx)
        if st_zone != 0:
            step = math.radians(1.0 if abs(st_zone) == 1 else 2.0)
            self._theta_st += step * (1 if st_zone > 0 else -1)

        # ── Shoulder / Elbow: solve_fk → Cartesian target → IK → rate-cap → shadow ──
        # Only run the IK round-trip when the right stick is actually deflected.
        # Otherwise the FK→clamp→IK loop is not identity at the workspace edges
        # and the joints drift to a singularity-boundary attractor.
        if rx != 0.0 or ry != 0.0:
            r_cur, z_cur = solve_fk(self._theta_sh, self._theta_el)
            r_cur = clamp(r_cur, R_MIN, R_MAX)

            r_des = clamp(r_cur + rx * TIP_SPEED * TICK_DT, R_MIN, R_MAX)
            z_des = z_cur + ry * TIP_SPEED * TICK_DT

            sh_new, el_new = solve_ik(r_des, z_des)

            d_sh = clamp(sh_new - self._theta_sh, -MAX_DELTA_RAD, MAX_DELTA_RAD)
            d_el = clamp(el_new - self._theta_el, -MAX_DELTA_RAD, MAX_DELTA_RAD)

            self._theta_sh += d_sh
            self._theta_el += d_el

        # ── Wrist: feedforward for global orientation, sent as 0.0 or -90.0 ───
        # ESP32 adds -(theta_sh + theta_el) on its side to compensate arm pose.
        # RViz gets the full feedforward-corrected angle for accurate rendering.
        wr_base = 0.0 if self._wrist_orient == ORIENT_HORIZONTAL else -90.0  # degrees
        wr_rad  = math.radians(wr_base) - (self._theta_sh + self._theta_el)
        self._theta_wr = wr_rad

        # ── Publish /arm_jt_targets ───────────────────────────────────────────
        out = Float32MultiArray()
        out.data = [0.0] * OUT_LEN
        out.data[OUT_IDX_ST]   = float(st_zone)
        out.data[OUT_IDX_SH]   = -math.degrees(self._theta_sh)+138
        out.data[OUT_IDX_EL]   = math.degrees(self._theta_el) +132
        out.data[OUT_IDX_WR]   = -math.degrees(self._theta_wr)+65
        out.data[OUT_IDX_GRIP] = 1.0 if self._gripper_closed else 0.0
        out.data[OUT_IDX_RST]  = 1.0 if self._reset_state    else 0.0
        self._pub_targets.publish(out)

        # ── Publish /joint_states for RViz ────────────────────────────────────
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(URDF_JOINTS)
        js.position = [
            self._sign_j1 * self._theta_st,
            self._sign_j2 * self._theta_sh,
            self._sign_j3 * self._theta_el,
            self._sign_j4 * wr_rad,
        ]
        self._pub_states.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = CylIkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
