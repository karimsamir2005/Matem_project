from base_control.fuzzy_base_control import fuzzy_logic_calc
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class Base_control(Node):
    def __init__(self):
        super().__init__('Base_control')
        self.sub_=self.create_subscription(Float32MultiArray,"auto_base",self.send_speed_callback,10)
        self.pub_=self.create_publisher(Twist,"cmd_vel",10)
    def send_speed_callback(self,msg:Float32MultiArray):
        car_speeds=fuzzy_logic_calc(msg.data[0],msg.data[1],msg.data[2])
        car=Twist()
        car.linear.x,car.linear.y,car.angular.z=car_speeds
        self.pub_.publish(car)
        self.get_logger().info(f"car speed is {car.linear.x} in x,{car.linear.y} in y and {car.angular.z} in rotation about z.")

def main(args=None):
    rclpy.init(args=args)
    node=Base_control() 
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__=='__main__':       
    main()

