from base_control.fuzzy_base_control import fuzzy as fz
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class Base_control(Node):
    def __init__(self):
        super().__init__('Base_control')
        self.control=fz()
        self.sub_=self.create_subscription(Float32MultiArray,"auto_base",self.send_speed_callback,10)
        self.pub_=self.create_publisher(Twist,"cmd_vel",10)


    def send_speed_callback(self,msg:Float32MultiArray):

        if len(msg.data) < 3:
            return
        car_speeds=self.control.fuzzy_logic_calc(msg.data[0],msg.data[1],msg.data[2])
        car=Twist()
        car.linear.x=car_speeds[0]
        car.linear.y=car_speeds[1]
        car.angular.z=car_speeds[2]
        self.pub_.publish(car)

        #logger for debugging
        self.get_logger().info(
            f"ex={msg.data[0]:.3f}, "
        f"ey={msg.data[1]:.3f}, "
        f"ez={msg.data[2]:.3f}  -->  "
        f"vx={car_speeds[0]:.3f}, "
        f"vy={car_speeds[1]:.3f}, "
        f"wz={car_speeds[2]:.3f}"
)

def main(args=None):
    rclpy.init(args=args)
    node = Base_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()