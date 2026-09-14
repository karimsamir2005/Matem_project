import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String,Int8
from geometry_msgs.msg import Twist, Quaternion
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
         # Signal that waypoint has been reached
        self.zero_sub= self.create_subscription(
            Int8,
            'state_finish',
            self.zero_callback,
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

        self.zero=None
        self.zero_old=None

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
        self.imu_received = False
        self.offset_locked = False

        self.auto_ready = False

    def zero_callback(self,msg:Int8):
        self.zero=msg.data
        if self.zero and not self.zero_old :

            #self.get_logger().info("AUTO MODE ENTERED → RESETTING STATE")
            self.x = 0.0
            self.y = 0.0
            self.time_old = self.get_clock().now()

            #self. = True
        self.zero_old=self.zero
        return
    
    

    def fk_displacement(self, t1, t2, t3, t4):

        TICKS_PER_REV =  1976
        r = 0.04
        L = LX + LY

        # ticks → radians
        w1 = (2*math.pi / TICKS_PER_REV) * t1
        w2 = (2*math.pi / TICKS_PER_REV) * t2
        w3 = (2*math.pi / TICKS_PER_REV) * t3
        w4 = (2*math.pi / TICKS_PER_REV) * t4

        transformation = np.array([
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-1/L, 1/L, -1/L, 1/L]
        ])

        wheel = np.array([[w1],[w2],[w3],[w4]])

        result = (r/4) * transformation @ wheel

        dx = float(result[0])
        dy = float(result[1])

        return dx, dy

        # wheel_rotations = np.array([[wheels[0]], [wheels[1]], [wheels[2]], [wheels[3]]])

        # car_velocities = (
        #     RADIUS_FACTOR * np.matmul(transformation, wheel_rotations)
        # ).flatten().tolist()

        # return car_velocities
    #taking IMU yaw and vyaw and initializing the yaw value when swicthing to AUTO
    def imu_callback(self, msg: Float32MultiArray):
        self.imu_yaw = math.radians(msg.data[0])
        self.imu_vyaw = math.radians(msg.data[1])
        self.imu_received = True

        if not self.offset_locked:
            self.theta_offset = self.imu_yaw
            self.offset_locked = True
            self.get_logger().info(f"Startup heading locked at: {msg.data[0]:.2f}°")

    def euler_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    #the logic that happens when switching the mode to AUTO
    def auto_activation_callback(self, msg: String):
        self.mode = msg.data

        if self.mode == "AUTO" and self.prev_mode != "AUTO":

            #self.get_logger().info("AUTO MODE ENTERED → RESETTING STATE")
            self.x = 0.0
            self.y = 0.0
            #for resseting the time when auto is switched to
            self.time_old = self.get_clock().now()

            self.auto_ready = True

        if self.mode != "AUTO":
            self.auto_ready = False

        self.prev_mode = self.mode


    def odem_ser_callback(self, msg: Float32MultiArray):

        # 1. INPUT = encoder ticks
        t1, t2, t3, t4 = msg.data

        dx, dy = self.fk_displacement(t1, t2, t3, t4)

        # 3. Get current heading relative to when the state/mode started
        imu_theta = self.imu_yaw - self.theta_offset
        self.theta = self.wrap_angle(imu_theta)

        # 5. Accumulate position 
        self.x += dx
        self.y += dy

        # 6. publish odometry
        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta)

        self.odom_pub.publish(odom)

#        self.get_logger().info(
 #       f"x={self.x:.3f} y={self.y:.3f} "
  #      f"dx={dx:.5f} dy={dy:.5f} "
   #     f"yaw={math.degrees(self.theta):.5f}"
    #    )
def main(args=None):
    rclpy.init(args=args)
    node = Odem_prep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
