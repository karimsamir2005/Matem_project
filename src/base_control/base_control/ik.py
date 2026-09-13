import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from std_msgs.msg import String

"""
w1 front left wheel 
w2 front right wheel 
w3 bottom left wheel 
w4 bottom right wheel 

vx forward and backward velocity 
vy lateral velocity 
wz angulary velocity 


inverse kinematics :

[w1]            [  1   -1  -(lx+ly) ] [vx]
[w2]    1/r *
     =          [  1    1   (lx+ly) ] [vy]
[w3]            [  1    1  -(lx+ly) ] [w]
[w4]            [  1   -1   (lx+ly) ]

"""

r= 0.04 #radius of the wheel in m
lx=0.18 #half of the distance between right and left wheels.
ly=0.165 #half of the distance between front wheel and the rear wheels.
l=lx+ly

class IK (Node):
    def __init__(self):
        super().__init__("wheel_calculator")
        self.sub_= self.create_subscription(
            Twist,"cmd_vel",self.callback_ik,10
        )
        self.pub_=self.create_publisher(Float32MultiArray,"wheels_uart",10)
        self.pub2_=self.create_publisher(String,"wheels_dash",10)
        self.pub3_=self.create_publisher(String,"velocity_dash",10)

    def callback_ik(self,msg:Twist):
        transformation=np.array([[1,-1,-l],[1,1,l],[1,1,-l],[1,-1,l]])
        velocities=np.array([[msg.linear.x],[msg.linear.y],[msg.angular.z]])  #subscribed values  
        wheels=np.round((1/r)*np.matmul(transformation,velocities),4)
        wheels_velocities=Float32MultiArray()
        wheels_velocities.data=wheels.flatten().tolist()
        self.pub_.publish(wheels_velocities)
        dash_msg_1=String()
        dash_msg_1.data=f"wheels velocities:\nw1={wheels_velocities.data[0]}rad/sec\nw2={wheels_velocities.data[1]}rad/sec\nw3={wheels_velocities.data[2]}rad/sec\nw4={wheels_velocities.data[3]}rad/sec"
        self.pub2_.publish(dash_msg_1)
        dash_msg_2=String()
        dash_msg_2.data=f"Car velocity:\nVx={msg.linear.x}meter/sec\nVy={msg.linear.y}meter/sec\nWz={msg.angular.z}rad/sec"
        self.pub3_.publish(dash_msg_2)
       # self.get_logger().info(f"Published velocities: w1={wheels_velocities.data[0]} w2={wheels_velocities.data[1]} w3={wheels_velocities.data[2]} w4={wheels_velocities.data[3]}")
        


def main(args=None):
    rclpy.init(args=args)
    node = IK()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()