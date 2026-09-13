# #!/usr/bin/env python3

# import threading
# import time

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import serial

# # ── Config ────────────────────────────────────────────────────────────────────
# PORT  = '/dev/ttyUSB0'
# BAUD  = 921600

# IN_TOPIC  = '/arm_jt_targets'
# OUT_TOPIC = '/arm_feedback'

# FEEDBACK_LEN       = 4
# READ_PERIOD_SEC    = 0.02   # 50 Hz
# RETRY_INTERVAL_SEC = 3.0
# WRITE_TIMEOUT_SEC  = 0.5
# READ_TIMEOUT_SEC   = 0.08

# IDX_ST, IDX_SH, IDX_EL, IDX_WR, IDX_GRIP, IDX_RST = range(6)


# class SerialBridgeNode(Node):

#     def __init__(self):
#         super().__init__('serial_bridge_node')

#         self._serial = None
#         self._lock   = threading.Lock()

#         self.create_subscription(Float32MultiArray, IN_TOPIC, self._tx_cb, 10)
#         self._fb_pub = self.create_publisher(Float32MultiArray, OUT_TOPIC, 10)
#         self.create_timer(READ_PERIOD_SEC, self._rx_cb)

#         threading.Thread(target=self._reconnect_loop, daemon=True).start()

#     # ── Connection ────────────────────────────────────────────────────────────

#     def _reconnect_loop(self):
#         while rclpy.ok():
#             if self._serial is None:
#                 try:
#                     ser = serial.Serial()
#                     ser.port, ser.baudrate = PORT, BAUD
#                     ser.timeout, ser.write_timeout = READ_TIMEOUT_SEC, WRITE_TIMEOUT_SEC
#                     ser.dtr = ser.rts = False
#                     ser.open()
#                     ser.reset_input_buffer()
#                     ser.reset_output_buffer()
#                     with self._lock:
#                         self._serial = ser
#                     self.get_logger().info(f'Serial connected → {PORT} @ {BAUD}')
#                 except serial.SerialException as e:
#                     self.get_logger().warn(
#                         f'Serial connect failed: {e} — retry in {RETRY_INTERVAL_SEC}s',
#                         throttle_duration_sec=RETRY_INTERVAL_SEC,
#                     )
#             time.sleep(RETRY_INTERVAL_SEC)

#     def _drop(self, reason: str):
#         with self._lock:
#             if self._serial is not None:
#                 try: self._serial.close()
#                 except Exception: pass
#                 self._serial = None
#         self.get_logger().error(f'Serial dropped ({reason}) — reconnecting...')

#     # ── TX ────────────────────────────────────────────────────────────────────

#     def _tx_cb(self, msg: Float32MultiArray):
#         d = msg.data
#         line = (
#             f'${int(round(d[IDX_ST]))},'
#             f'{int(round(d[IDX_SH]))},'
#             f'{int(round(d[IDX_EL]))},'
#             f'{int(round(d[IDX_WR]))},'
#             f'{int(round(d[IDX_GRIP]))},'
#             f'{int(round(d[IDX_RST]))}\n'
#         )

#         with self._lock:
#             if self._serial is None:
#                 return
#             try:
#                 self._serial.write(line.encode('utf-8'))
#                 failed = None
#             except (serial.SerialTimeoutException, serial.SerialException) as e:
#                 failed = str(e)

#         if failed:
#             self._drop(f'write: {failed}')
#         else:
#             self.get_logger().info(f'TX → {line.strip()}')

#     # ── RX ────────────────────────────────────────────────────────────────────

#     def _rx_cb(self):
#         with self._lock:
#             if self._serial is None:
#                 return
#             try:
#                 if self._serial.in_waiting == 0:
#                     return
#                 raw = self._serial.readline()
#                 failed = None
#             except serial.SerialException as e:
#                 raw = None
#                 failed = str(e)

#         if failed:
#             self._drop(f'read: {failed}')
#             return

#         line = raw.decode('utf-8', errors='replace').strip()
#         if not line or ',' not in line:
#             return

#         parts = line.split(',')
#         if len(parts) < FEEDBACK_LEN:
#             self.get_logger().warn(
#                 f'feedback: expected ≥{FEEDBACK_LEN}, got {len(parts)}: "{line}"',
#                 throttle_duration_sec=5.0,
#             )
#             return

#         try:
#             angles = [float(p) for p in parts[:FEEDBACK_LEN]]
#         except ValueError as e:
#             self.get_logger().warn(f'feedback parse error "{line}": {e}',
#                                    throttle_duration_sec=5.0)
#             return

#         out = Float32MultiArray()
#         out.data = angles
#         self._fb_pub.publish(out)

#     # ── Cleanup ───────────────────────────────────────────────────────────────

#     def destroy_node(self):
#         with self._lock:
#             if self._serial is not None and self._serial.is_open:
#                 self._serial.close()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialBridgeNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
serial_bridge_node.py
---------------------
Subscribes to /arm_jt_targets (Float32MultiArray, 6 values) and writes a
CSV frame "$st,sh,el,wr,grip,reset\n" to the ESP32 over serial.

Also reads feedback from the ESP32 at 50 Hz and publishes to /arm_feedback
(Float32MultiArray, 4 values: joint angles).

Networking is based on the stable old serial_bridge_node:
  - Connects immediately on startup (no initial sleep)
  - _connected flag guards TX callback — no writes before port is open
  - _try_connect() / _reconnect_loop() with clean _handle_disconnect()
  - serial.Serial() constructed directly (no separate .open() call)
  - No dtr/rts manipulation or buffer resets (removed — didn't help)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial


# ── Constants ─────────────────────────────────────────────────────────────────
NUM_IN              = 6
FEEDBACK_LEN        = 4
RETRY_INTERVAL_SEC  = 3.0
SERIAL_TIMEOUT_SEC  = 0.08
WRITE_TIMEOUT_SEC   = 0.5
READ_PERIOD_SEC     = 0.02   # 50 Hz

DEFAULT_PORT        = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
DEFAULT_BAUD        = 921600

IN_TOPIC  = '/arm_jt_targets'
OUT_TOPIC = '/arm_feedback'

IDX_ST, IDX_SH, IDX_EL, IDX_WR, IDX_GRIP, IDX_RST = range(6)


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', DEFAULT_PORT)
        self.declare_parameter('baud_rate',   DEFAULT_BAUD)

        self._port = self.get_parameter('serial_port').get_parameter_value().string_value
        self._baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self._serial      = None
        self._serial_lock = threading.Lock()
        self._connected   = False

        # qos depth 1 — drop stale, keep latest
        self.create_subscription(Float32MultiArray, IN_TOPIC, self._tx_cb, 1)
        self._fb_pub = self.create_publisher(Float32MultiArray, OUT_TOPIC, 10)
        self.create_timer(READ_PERIOD_SEC, self._rx_cb)

        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop, daemon=True, name='serial_reconnect'
        )
        self._reconnect_thread.start()

    # ── Connection management ────────────────────────────────────────────────

    def _try_connect(self) -> bool:
        try:
            ser = serial.Serial(
                port          = self._port,
                baudrate      = self._baud,
                timeout       = SERIAL_TIMEOUT_SEC,
                write_timeout = WRITE_TIMEOUT_SEC,
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            with self._serial_lock:
                self._serial    = ser
                self._connected = True
            self.get_logger().info(f'Serial connected -> {self._port} @ {self._baud} baud')
            return True

        except serial.SerialException as e:
            self.get_logger().warn(
                f'Serial connection failed: {e} - retrying in {RETRY_INTERVAL_SEC}s',
                throttle_duration_sec=RETRY_INTERVAL_SEC,
            )
            return False

    def _reconnect_loop(self) -> None:
        """Connects immediately on first iteration, then retries on failure."""
        while rclpy.ok():
            if not self._connected:
                self._try_connect()
            time.sleep(RETRY_INTERVAL_SEC)

    def _handle_disconnect(self) -> None:
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._connected = False
        self.get_logger().error(
            f'Serial port {self._port} lost. Retrying every {RETRY_INTERVAL_SEC}s ...'
        )

    # ── TX ────────────────────────────────────────────────────────────────────

    def _tx_cb(self, msg: Float32MultiArray) -> None:
        if not self._connected:
            return

        d = msg.data
        line = (
            f'${int(round(d[IDX_ST]))},'
            f'{int(round(d[IDX_SH]))},'
            f'{int(round(d[IDX_EL]))},'
            f'{int(round(d[IDX_WR]))},'
            f'{int(round(d[IDX_GRIP]))},'
            f'{int(round(d[IDX_RST]))}\n'
        )
        self._send_serial(line)

    def _send_serial(self, line: str) -> None:
        write_failed = False
        with self._serial_lock:
            if self._serial is None:
                return
            try:
                self._serial.write(line.encode('ascii'))
                # No flush(): OS handles it, tcdrain just adds latency.
            except serial.SerialTimeoutException:
                self.get_logger().error('Serial write timeout!')
                write_failed = True
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')
                write_failed = True

        if write_failed:
            self._handle_disconnect()
        else:
            self.get_logger().debug(f'TX -> {line.strip()}')

    # ── RX ────────────────────────────────────────────────────────────────────

    def _rx_cb(self) -> None:
        if not self._connected:
            return

        read_failed = False
        raw = None

        with self._serial_lock:
            if self._serial is None:
                return
            try:
                if self._serial.in_waiting == 0:
                    return
                raw = self._serial.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}')
                read_failed = True

        if read_failed:
            self._handle_disconnect()
            return

        line = raw.decode('utf-8', errors='replace').strip()
        if not line or ',' not in line:
            return

        parts = line.split(',')
        if len(parts) < FEEDBACK_LEN:
            self.get_logger().warn(
                f'feedback: expected >={FEEDBACK_LEN}, got {len(parts)}: "{line}"',
                throttle_duration_sec=5.0,
            )
            return

        try:
            angles = [float(p) for p in parts[:FEEDBACK_LEN]]
        except ValueError as e:
            self.get_logger().warn(
                f'feedback parse error "{line}": {e}',
                throttle_duration_sec=5.0,
            )
            return

        out = Float32MultiArray()
        out.data = angles
        self._fb_pub.publish(out)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
                self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()