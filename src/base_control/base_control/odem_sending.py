import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
import numpy as np
import math

RADIUS_FACTOR = 0.04 / 4
LX = 0.18
LY = 0.165
L = LX + LY


class Odem_prep(Node):
    def __init__(self):
        super().__init__('odemetry_prep')

        #This subscribes to the odem data of wheel velocities coming from the serial node
        self.ser_sub = self.create_subscription(
            Float32MultiArray,
            'odem_ser',
            self.odem_ser_callback,
            10
        )

        #This subscribes to the imu data (yaw and vyaw) coming from the serial node
        self.imu_sub = self.create_subscription(
            Float32MultiArray,
            'imu_ser',
            self.imu_callback,
            10
        )

        #This subscribes to the mode manager node to make the autonomous initialization logic
        self.auto_sub = self.create_subscription(
            String,
            'auto_active',
            self.auto_activation_callback,
            10
        )

        #Publishes the complete odem message having x and y and theta to be used by the error_calc node
        self.odom_pub = self.create_publisher(
            Odometry,
            'odem_final',
            10
        )

        #detecting and mode toggling nodes
        self.mode = None
        self.prev_mode = None

        #clock for distance calculations 
        self.time_old = self.get_clock().now()

        #state varaibles of the odem message
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        #imu varaibles and offset for correct initial heading
        self.imu_yaw = 0.0
        self.imu_vyaw = 0.0
        self.theta_offset = 0.0

        self.auto_ready = False

    #converting wheel velocities into car velocity in Vx and Vy
    def fk(self, *wheels):
        transformation = np.array([
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-1 / L, 1 / L, -1 / L, 1 / L]
        ])

        wheel_rotations = np.array([[wheels[0]], [wheels[1]], [wheels[2]], [wheels[3]]])

        car_velocities = (
            RADIUS_FACTOR * np.matmul(transformation, wheel_rotations)
        ).flatten().tolist()

        return car_velocities

    #Time calculation function
    def dt(self):
        now = self.get_clock().now()
        dt = (now - self.time_old).nanoseconds * 1e-9
        self.time_old = now
        return dt

    #taking IMU yaw and vyaw and initializing the yaw value when swicthing to AUTO
    def imu_callback(self, msg: Float32MultiArray):
        self.imu_yaw = msg.data[0]
        self.imu_vyaw = msg.data[1]

    #the logic that happens when switching the mode to AUTO
    def auto_activation_callback(self, msg: String):
        self.mode = msg.data

        if self.mode == "AUTO" and self.prev_mode != "AUTO":

            #self.get_logger().info("AUTO MODE ENTERED → RESETTING STATE")
            self.x = 0.0
            self.y = 0.0
            # capture IMU reference
            self.theta_offset = self.imu_yaw

            #for resseting the time when auto is switched to
            self.time_old = self.get_clock().now()

            self.auto_ready = True

        if self.mode != "AUTO":
            self.auto_ready = False

        self.prev_mode = self.mode


    def odem_ser_callback(self, msg: Float32MultiArray):

        #ignore this in manyal mode
        # if not self.auto_ready:
        #     return 

        vx, vy, omega = self.fk(*msg.data)
        dt = self.dt()+0.5

        self.theta = self.imu_yaw - self.theta_offset

        #converting velocities and angles into distance
        self.x += vx*dt
        #(vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y +=vy*dt 
        #(vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        #publish the odometry message to the mecanum topics
        odom = Odometry()

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        self.odom_pub.publish(odom)

        self.get_logger().info(
            f"ODOM: x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f}"
        )
        self.get_logger().info(f"dt = {dt:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = Odem_prep()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()