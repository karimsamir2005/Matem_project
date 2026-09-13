import rclpy
import math
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String


class laneNode(Node):
    def __init__(self):
        super().__init__("lanes")

        # subscriber
        self.subscription = self.create_subscription(Image, 'lane_camera_raw', self.image_callback, 10)

        self.bridge = CvBridge()

        # publishers
        self.lane_publisher_ = self.create_publisher(Image, 'lane_processed', 10)
        self.offset_publisher_ = self.create_publisher(String, 'tilt_offset', 10)

        self.frame_count = 0

        self.prev_left = None
        self.prev_right = None
        self.alpha = 0.2

    # ==================================================

    def smooth(self, lanes):
        if lanes is None or len(lanes) != 2:
            return lanes

        if self.prev_left is None:
            self.prev_left, self.prev_right = lanes
            return lanes

        left = self.alpha * lanes[0] + (1 - self.alpha) * self.prev_left
        right = self.alpha * lanes[1] + (1 - self.alpha) * self.prev_right

        self.prev_left = left
        self.prev_right = right

        return np.array([left, right])

    # ==================================================

    def image_callback(self, msg):

        try:
            self.frame_count += 1

            if self.frame_count % 3 != 0:
                return

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            edges = self.initialFilters(frame)

            #edges = self.region_of_interest(edges)

            result = self.houghLineTransformProbablistic(edges, frame)

            if result is None:
                result = frame

            out_msg = self.bridge.cv2_to_imgmsg(result, encoding='mono8')

            self.lane_publisher_.publish(out_msg)

        except Exception as e:
            self.get_logger().error(str(e))

    # ==================================================

    def initialFilters(self, frame):
        try:
            blur = cv.GaussianBlur(frame, (5, 5), 0)
            return cv.Canny(blur, 50, 100)
        except Exception as e:
            self.get_logger().error(f"Filter error: {e}")
            return None
        
    # ==================================================

    # def region_of_interest(self, edges):

    #     height, width = edges.shape

    #     mask = np.zeros_like(edges)

    #     polygon = np.array([[
    #     (int(width * 0.1), height),         # bottom-left (wider)
    #     (int(width * 0.9), height),         # bottom-right
    #     (int(width * 0.6), int(height * 0.55)),  # top-right
    #     (int(width * 0.4), int(height * 0.55))   # top-left
    #   ]], np.int32)
        
    #     cv.fillPoly(mask, polygon, 255)

    #     return cv.bitwise_and(edges, mask)

    # ==================================================

    def make_lane_lines(self, lines):
        if lines is None:
            return None

        left, right = [], []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 == x1:
                continue

            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1

            if slope < 0:
                left.append((slope, intercept))
            else:
                right.append((slope, intercept))

        if len(left) == 0 or len(right) == 0:
            return None

        left_avg = np.mean(left, axis=0)
        right_avg = np.mean(right, axis=0)      # because axis=0 computes column-wise averages so calculate the average slope alone and average intercept and put them into an array

        return np.array([left_avg, right_avg])

    # ==================================================

    def get_line_points(self, frame, line_params):
        slope, intercept = line_params

        height = frame.shape[0]

        y1 = height
        y2 = int(height * 0.6)

        # avoid division crash
        if slope == 0:
            return None

        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return (x1, y1, x2, y2)

    # ==================================================

    def houghLineTransformProbablistic(self, edges, frame):

        lines_p = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=40, minLineLength=50, maxLineGap=40)

        if lines_p is None:
            return frame

        lanes = self.make_lane_lines(lines_p)
      
        if lanes is None:
            return frame
      
        lanes_smooth = self.smooth(lanes)

        if lanes_smooth is None or len(lanes_smooth) != 2:
            return frame

        for lane in lanes_smooth:
            pts = self.get_line_points(frame, lane)
            if pts is None:
                continue

            x1, y1, x2, y2 = pts
            cv.line(frame, (x1, y1), (x2, y2), 255, 10)

        self.compute_offset_tilt(frame, lanes_smooth)

        return frame

    # ==================================================

    def compute_offset_tilt(self, frame, lanes):

        left, right = lanes

        left_points = self.get_line_points(frame, left)
        right_points = self.get_line_points(frame, right)

        if left_points is None or right_points is None:
            return

        width = frame.shape[1]

        lane_center = (left_points[0] + right_points[0]) / 2
        camera_center = width / 2

        offset_pixel = camera_center - lane_center
        offset_meter = offset_pixel * 0.000258

        tilt = math.atan(right[0])

        message = (offset_meter, tilt)

        msg = String()
        msg.data = str(message)

        self.offset_publisher_.publish(msg)

        cv.line(frame, (int(camera_center), 50), (int(camera_center), frame.shape[0]), 255, 2)
        cv.putText(frame, f"offset: {offset_meter:.2f}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
        cv.putText(frame, f"tilt: {tilt:.2f}", (50, 100), cv.FONT_HERSHEY_SIMPLEX, 1, 255, 2)

# ==================================================

def main():
    rclpy.init()
    node = laneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()