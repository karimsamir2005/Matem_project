import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0',

class VisionManager(Node):
    def __init__(self):
        super().__init__("vision_switch_manager")

        # Publishers for the separate nodes
        self.lane_pub = self.create_publisher(Image, 'lane_camera_raw', 10)
        self.qr_pub = self.create_publisher(Image, 'qr_camera_raw', 10)

        # Mode Subscriber
        self.mode_sub = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.module_sub = self.create_subscription(String, 'auto_active', self.module_callback, 10)

        # Hardware Setup
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture( 2,cv.CAP_V4L2)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        self.module = "MAN"
        self._last_mode = None
        self._last_module = None

        self.current_mode = "BASE"
        self.timer = self.create_timer(0.05, self.stream_logic)
        self.get_logger().info("Vision Switcher Online. Hardware locked to Manager.")

    def mode_callback(self, msg):
        new_mode = msg.data.upper()
        if new_mode != self._last_mode:
            self.get_logger().info(f"Mode changed: {new_mode}")
            self._last_mode = new_mode
        self.current_mode = new_mode

    def module_callback(self, msg):
        new_module = msg.data.upper()
        if new_module != self._last_module:
            self.get_logger().info(f"Module changed: {new_module}")
            self._last_module = new_module
        self.current_module = new_module

    def stream_logic(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Convert once
        img_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')

        # ROUTING: Only publish to the active node's topic
        if self.module == "MAN":
            if self.current_mode == "BASE":
                self.lane_pub.publish(img_msg)
            elif self.current_mode == "ARM":
                self.qr_pub.publish(img_msg)
        else:
            self.lane_pub.publish(img_msg)

def main():
    rclpy.init()
    node = VisionManager()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()