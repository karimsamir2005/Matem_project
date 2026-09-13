#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial

TARGETS_LEN  = 6    # /arm_jt_targets length
FEEDBACK_LEN = 4    # values read from ESP32 feedback [st°,sh°,el°,wr°]
TIMER_SEC    = 0.02 # 50 Hz feedback read

IN_IDX_ST, IN_IDX_SH, IN_IDX_EL, IN_IDX_WR = 0, 1, 2, 3
IN_IDX_GRIP, IN_IDX_RST                     = 4, 5

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        self._port    = '/dev/ttyUSB0'
        self._baud    = 921600
        in_jt_topic   = '/arm_jt_targets'
        out_fb_topic  = '/arm_feedback'

        self._serial = serial.Serial(port=self._port, baudrate=self._baud, timeout=0.08)
        self.get_logger().info(f'serial connected → {self._port} @ {self._baud} baud')

        self._jt_sub = self.create_subscription(Float32MultiArray, in_jt_topic, self._jt_callback, 10)

        self._feedback_pub = self.create_publisher(Float32MultiArray, out_fb_topic, 10)

        self._read_timer = self.create_timer(TIMER_SEC, self._read_callback)

    def _jt_callback(self, msg: Float32MultiArray):
        st   = int(round(msg.data[IN_IDX_ST]))  
        sh   = msg.data[IN_IDX_SH]              
        el   = msg.data[IN_IDX_EL]              
        wr   = msg.data[IN_IDX_WR]              
        grip = int(round(msg.data[IN_IDX_GRIP]))
        rst  = int(round(msg.data[IN_IDX_RST])) 

        line = f'${st},{sh:.2f},{el:.2f},{wr:.1f},{grip},{rst}\n'
        # line = f'${st},{int(round(sh))},{int(round(el))},{int(round(wr))},{grip},{rst}\n'
        self._serial.write(line.encode('utf-8'))
        self.get_logger().info(f'TX → {line.strip()}')

    def _read_callback(self) -> None:
        if self._serial.in_waiting == 0: return

        raw = self._serial.readline()
        if not raw: return
        
        line = raw.decode('utf-8', errors='replace').strip()
        if not line: return
        
        if ',' not in line: return
        
        self._parse_and_publish(line)

    def _parse_and_publish(self, line: str):
        parts = line.split(',')

        if len(parts) < FEEDBACK_LEN:
            self.get_logger().warn(
                f'ESP32 feedback: expected ≥{FEEDBACK_LEN} values, '
                f'got {len(parts)} in "{line}". Dropping.',
                throttle_duration_sec=5.0,
            )
            return

        try:
            angles = [float(p.strip()) for p in parts[:FEEDBACK_LEN]]
        except ValueError as e:
            self.get_logger().warn(
                f'ESP32 feedback parse error "{line}": {e}',
                throttle_duration_sec=5.0,
            )
            return

        out = Float32MultiArray()
        out.data = angles
        self._feedback_pub.publish(out)

        self.get_logger().debug(
            f'feedback → st={angles[0]:.0f}°  sh={angles[1]:.0f}°  '
            f'el={angles[2]:.0f}°  wr={angles[3]:.0f}°'
        )

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.get_logger().info('serial port closed.')
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
