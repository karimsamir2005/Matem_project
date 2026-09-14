
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # must match the publisher


# Define a ROS 2 Node that listens to temperature data
class DisplayNode(Node):
    def __init__(self):
        # Initialize the Node with the name "display"
        super().__init__("display")

        # Create a subscription:
        # - Message type: Float32
        # - Topic: "temperature"
        # - Callback function: self.callback_temperature
        # - Queue size: 10
        self.subscriber_ = self.create_subscription(
            String, "auto_active", self.callback_temperature, 10
        )

        self.get_logger().info("Display Node started and waiting for the autonmous activation...")

    # Callback function that runs whenever a new message is received
    def callback_temperature(self, msg: String):
        # Print the temperature received from the publisher
        self.get_logger().info(f"state :{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()