import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String

class qrNode(Node):
    def __init__(self):
        super().__init__("qr_node")

        # 1. Subscriber: Listen to the Manager instead of opening the camera
        self.subscription = self.create_subscription(Image, '/qr_camera_raw', self.image_callback, 10)
        
        # 2. Setup Tools
        self.qr_pub = self.create_publisher(String, '/camera_data', 10)
        self.qr_pub_draw = self.create_publisher(Image, '/qr_processed', 10)
        self.bridge = CvBridge()
        self.frame_count = 0

    def image_callback(self, msg):
        """This function runs only when the Manager is in ARM mode"""
        self.frame_count += 1
        if self.frame_count % 3 != 0:
            return

        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        # Process the frame using your existing logic
        #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        decodedObjects = self.decode(frame)
        
        # publish data if QR is found
        for obj in decodedObjects:
            msg_str = String()
            msg_str.data = "Color:"+obj.data.decode('utf-8')
            self.qr_pub.publish(msg_str)
            self.get_logger().info("publishing frame")            
            self.get_logger().info(f"QR Detected: {msg_str.data}")
            # Publish the final processed image
            
        self.display(frame, decodedObjects)
        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
        self.get_logger().info("publishing frame")
        self.qr_pub_draw.publish(processed_msg)

    def decode(self, im):
        return pyzbar.decode(im)

    def display(self, im, decodedObjects):
        # (Your existing display logic remains here)
        for decodedObject in decodedObjects:
            points = decodedObject.polygon
            if len(points) > 4 :
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else :
                hull = points
            n = len(hull)
            for j in range(0,n):
                cv2.line(im, hull[j], hull[ (j+1) % n], (0,0,0), 5)
        '''# Publish the final processed image
            processed_msg = self.bridge.cv2_to_imgmsg(im, encoding='mono8')
            self.lane_publisher_.publish(processed_msg)'''

def main():
    rclpy.init()
    node = qrNode()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()