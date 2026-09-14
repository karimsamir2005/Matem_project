#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial

# ── Serial config ─────────────────────────────────────────────────────────────
PORT           = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
BAUD           = 921600
RETRY_INTERVAL = 5.0
SERIAL_TIMEOUT = 0.08   # read timeout — keeps _rx_cb non-blocking
WRITE_TIMEOUT  = 0.5    # write timeout before declaring a failure
READ_PERIOD    = 0.02   # RX poll rate: 50 Hz

# ── Topics ────────────────────────────────────────────────────────────────────
MAN_TOPIC     = '/arm_jt_targets'
AUTO_TOPIC    = '/esp_cmd'
MODE_TOPIC    = 'auto_active'
OUT_TOPIC     = '/arm_feedback'
OUT_ACK_TOPIC = '/arm_ack'

# ── Mode strings (must match mode_manager constants) ──────────────────────────
MODE_MANUAL      = 'MAN'
MODE_AUTO        = 'AUTO'
MODE_AUTO_BONUS  = 'AUTO_BONUS'

# ── Valid autonomous sub-mode commands ────────────────────────────────────────
ALLOWED_CMDS = {'1', '2', '3'}

BONUS_REMAP = {'1': '4', '2': '5', '3': '3'}

# ── Joint index mapping inside the Float32MultiArray ─────────────────────────
FEEDBACK_LEN = 5   # ESP32 sends back 4 joint angles + color
IDX_ST, IDX_SH, IDX_EL, IDX_WR, IDX_GRP, IDX_RST = range(6)
IDX_CLR = 6        # color field index in the shared frame

# [stepper, shoulder, elbow, wrist, gripper, reset, color]
FRAME_LEN = 7


class SerialBridgeNode(Node):

    # ── Init ──────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('serial_bridge_node')

        self._serial      = None
        self._serial_lock = threading.Lock()
        self._connected   = False

        self._mode  = MODE_MANUAL

        self._frame = [0] * FRAME_LEN
        self._last_ack = None   # tracks last published ack value to detect changes

        self.create_subscription(Float32MultiArray, MAN_TOPIC,  self._man_cb,  1)
        self.create_subscription(String,            AUTO_TOPIC, self._auto_cb, 10)
        self.create_subscription(String,            MODE_TOPIC, self._mode_cb, 10)

        self._fb_pub  = self.create_publisher(Float32MultiArray, OUT_TOPIC,     10)
        self._ack_pub = self.create_publisher(String,            OUT_ACK_TOPIC, 10)

        self.create_timer(READ_PERIOD, self._rx_cb)
        self._auto_timer = self.create_timer(READ_PERIOD, self._auto_tx_cb)
        self._auto_timer.cancel()   # only active in AUTO mode

        self._reconnect_thread = threading.Thread(target=self._reconnect_loop, daemon=True)
        self._reconnect_thread.start()

    # ── Connection management ─────────────────────────────────────────────────

    def _try_connect(self) -> bool:
        try:
            ser = serial.Serial(port = PORT, baudrate = BAUD, timeout = 0.08, write_timeout = 0.5,)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            with self._serial_lock:
                self._serial    = ser
                self._connected = True
            self.get_logger().info(f'ESP connected -> {PORT} @ {BAUD} baud')
            return True

        except serial.SerialException as e:
            # self.get_logger().warn(f'Connection failed: {e} - retrying in {RETRY_INTERVAL}s')
            return False

    def _reconnect_loop(self) -> None:
        while rclpy.ok():
            if not self._connected:
                self._try_connect()
            time.sleep(RETRY_INTERVAL)

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
            # f'Serial port {PORT} lost. Retrying every {RETRY_INTERVAL}s ...'
        )

    # ── Mode ──────────────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String) -> None:
        self._mode = msg.data.upper()
        if self._mode == MODE_MANUAL:
            self._auto_timer.cancel()
            self._frame[IDX_CLR] = 0
        elif self._mode in (MODE_AUTO, MODE_AUTO_BONUS):
            self._auto_timer.reset()

    # ── TX ────────────────────────────────────────────────────────────────────

    def _man_cb(self, msg: Float32MultiArray) -> None:
        if not self._connected:
            return
        
        d = msg.data
        self._frame[IDX_ST]   = int(round(d[IDX_ST]))
        self._frame[IDX_SH]   = int(round(d[IDX_SH]))
        self._frame[IDX_EL]   = int(round(d[IDX_EL]))
        self._frame[IDX_WR]   = int(round(d[IDX_WR]))
        self._frame[IDX_GRP]  = int(round(d[IDX_GRP]))
        self._frame[IDX_RST]  = int(round(d[IDX_RST]))
        self._frame[IDX_CLR]  = 0

        self._send_frame()

    def _auto_cb(self, msg: String) -> None:
        cmd = msg.data.strip()
        if cmd not in ALLOWED_CMDS:
            self.get_logger().warn(f'rejecting unknown command: "{cmd}"', throttle_duration_sec=2.0,)
            return
        if self._mode == MODE_AUTO_BONUS:
            cmd = BONUS_REMAP[cmd]
        self._frame[IDX_CLR] = int(cmd)
        self._auto_timer.reset()

    def _auto_tx_cb(self) -> None:
        if not self._connected: return
        self._send_frame()

    def _send_frame(self) -> None:
        f = self._frame
        line = (
            f'${f[IDX_ST]},'
            f'{f[IDX_SH]},'
            f'{f[IDX_EL]},'
            f'{f[IDX_WR]},'
            f'{f[IDX_GRP]},'
            f'{f[IDX_RST]},'
            f'{f[IDX_CLR]}\n'
        )
        self._send_serial(line)
#        self.get_logger().info(line.strip())

    def _send_serial(self, line: str) -> None:
        write_failed = False
        with self._serial_lock:
            if self._serial is None:
                return
            try:
                self._serial.write(line.encode('ascii'))
            except serial.SerialTimeoutException:
                self.get_logger().error('Serial write timeout!')
                write_failed = True
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')
                write_failed = True

        if write_failed:
            self._handle_disconnect()

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
        if not line:
            return

        parts = line.split(',')
        if len(parts) < FEEDBACK_LEN:
            self.get_logger().warn(
                f'feedback: expected {FEEDBACK_LEN} fields, got {len(parts)}: "{line}"',
                throttle_duration_sec=1.0,
            )
            return

        try:
            values = [float(p) for p in parts[:FEEDBACK_LEN]]
        except ValueError as e:
            self.get_logger().warn(
                f'feedback parse error "{line}": {e}',
                throttle_duration_sec=1.0,
            )
            return

        out = Float32MultiArray()
        out.data = values[:4]
        self._fb_pub.publish(out)

        ack_val = int(values[4])
        if ack_val != self._last_ack:
            self._last_ack = ack_val
            self._ack_pub.publish(String(data=str(ack_val)))


    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
                self.get_logger().info('Serial port closed.')
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

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
