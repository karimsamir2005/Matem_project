import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
# '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0',


class qrNode(Node):
    def __init__(self):
        super().__init__("qr_node")

        #---------------------------- Subscribers
        self.mode_sub = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.module_sub = self.create_subscription(String, 'auto_active', self.module_callback, 10)
        
        #---------------------------- Publishers
        self.qr_pub = self.create_publisher(String, '/camera_data', 10)
        self.qr_pub_draw = self.create_publisher(Image, '/qr_processed', 10)
        self.bridge = CvBridge()
        self.frame_count = 0

        #---------------------------- Hardware Setup
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0 ,cv2.CAP_V4L2)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        self.module = "MAN"
        self.current_mode = "BASE"
        self.current_module = "MAN"
        self.current_mode = "BASE"
        self.frame_count = 0
        self.timer = self.create_timer(0.1, self.process_frame)

    def mode_callback(self, msg):
        self.current_mode = msg.data.upper()

    def module_callback(self, msg):
        self.current_module = msg.data.upper()


    def process_frame(self):

        # Only run camera in MANUAL ARM mode
        if not (self.current_module == "MAN" and self.current_mode == "ARM"):
            return

        self.frame_count += 1
        if self.frame_count % 3 != 0:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded = pyzbar.decode(gray)

        for obj in decoded:

            qr_data = obj.data.decode('utf-8').strip().lower()
            msg = String()
            msg.data = qr_data
            self.qr_pub.publish(msg)
            pts = obj.polygon

            if len(pts) > 4:
                hull = cv2.convexHull(np.array([(p.x, p.y) for p in pts],dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))

            else:
                hull = [(p.x, p.y) for p in pts]

            for j in range(len(hull)):
                cv2.line(frame,hull[j],hull[(j + 1) % len(hull)],(0, 255, 0),2)
            
        self.display(gray, decoded)
        processed_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        self.qr_pub_draw.publish(processed_msg)


    def display(self, im, decoded):
        # (Your existing display logic remains here)
        for decodedObject in decoded:
            points = decodedObject.polygon
            if len(points) > 4 :
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else :
                hull = points
            n = len(hull)
            for j in range(0,n):
                cv2.line(im, hull[j], hull[ (j+1) % n], (0,0,0), 5)


def main(args=None):
    rclpy.init(args=args)
    node = qrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()