import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedQuantizer(Node):
    def __init__(self):
        super().__init__('speed_quantizer')
        #The new mapping for speeds is here
        # 1. Subscription to raw teleop (Input)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel_raw', self.quantize_callback, 10)
            
        # 2. Publisher to IK/Wheel Calculator (Output)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. Define Step Levels (Input Threshold, Output Speed)
        # Linear X (Forward/Backward) and Y (Strafing Left/Right)
        self.linear_levels = [(0.5, 0.4), (0.25, 0.2), (0.0, 0.0)]
        
        # Angular Z (Rotation) - typically higher values for turning speed
        self.angular_levels = [(0.5, 1.2), (0.25, 0.6), (0.0, 0.0)]

    def apply_thresholds(self, input_val, levels):
        """Maps continuous joystick input to discrete speed steps."""
        abs_val = abs(input_val)
        sign = 1.0 if input_val >= 0 else -1.0
        
        for threshold, speed in levels:
            if abs_val >= threshold:
                return speed * sign
        return 0.0

    def quantize_callback(self, msg):
        quantized_msg = Twist()
        
        # Quantize Forward/Backward (X)
        quantized_msg.linear.x = self.apply_thresholds(msg.linear.x, self.linear_levels)
        
        # Quantize Strafing Left/Right (Y) - Essential for Mecanum!
        quantized_msg.linear.y = self.apply_thresholds(msg.linear.y, self.linear_levels)
        
        # Quantize Rotation (Z)
        quantized_msg.angular.z = self.apply_thresholds(msg.angular.z, self.angular_levels)
        
        self.publisher.publish(quantized_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedQuantizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()