import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
import math

# Maximum limits for linear and angular velocities
MAX_VX = 0.3
MAX_VY = 0.3
MAX_WZ = 1.0

LIMIT = 0.1  # Integral clamp

# ----------------------------
# X PID gains (UNCHANGED)
# ----------------------------
KP_X = 0.27
KI_X = 0.09
KD_X = 0.2

KP_Y = 0.27
KI_Y = 0.097
KD_Y = 0.21

KP_YAW = 0.255
KI_YAW = 0.117
KD_YAW = 0.21


class Base_control(Node):
    def __init__(self):
        super().__init__('Base_control')

        self.old_time = None

        self.sub_ = self.create_subscription(
            Float32MultiArray, "auto_base", self.send_speed_callback, 10)

        self.mode_sub = self.create_subscription(
            String, "auto_active", self.mode_callback, 10)

        self.pub_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer_ = self.create_timer(0.020, self.control_loop)

        self.auto_ready = False
        self.first_run = True

        # errors
        self.ex = 0.0
        self.ey = 0.0
        self.ez = 0.0

        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_ez = 0.0

        # integral
        self.x_integral = 0.0
        self.y_integral = 0.0
        self.ez_integral = 0.0

        # derivative filtered
        self.drev_ex = 0.0
        self.drev_ey = 0.0
        self.drev_ez = 0.0

        # output
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def integral_calc(self, t):   
        self.x_integral += self.ex * t
        self.y_integral += self.ey * t
        self.ez_integral += self.ez * t

        self.x_integral = max(min(self.x_integral, LIMIT), -LIMIT)
        self.y_integral = max(min(self.y_integral, LIMIT), -LIMIT)
        self.ez_integral = max(min(self.ez_integral, LIMIT), -LIMIT)

    def drev_calc(self, t):

        alpha = 0.8

        raw_ex = (self.ex - self.prev_ex) / t
        self.drev_ex = alpha * self.drev_ex + (1 - alpha) * raw_ex

        raw_ey = (self.ey - self.prev_ey) / t
        self.drev_ey = alpha*self.drev_ey+(1-alpha)* raw_ey

        raw_ez = (self.ez - self.prev_ez) / t
        self.drev_ez = alpha*self.drev_ez+(1-alpha)* raw_ez

    def pid_calc(self):
        self.vx = (
            KP_X * self.ex +
            KI_X * self.x_integral +
            KD_X * self.drev_ex
        )

        self.vy = (
            KP_Y * self.ey + 
            KI_Y * self.y_integral + 
            KD_Y * self.drev_ey
        )

        self.wz =(
            KP_YAW * self.ez +
            KI_YAW * self.ez_integral +
            KD_YAW * self.drev_ez
        )

    def mode_callback(self, msg: String):
        if msg.data == "AUTO":
            if not self.auto_ready:
                self.x_integral = 0.0
                self.y_integral = 0.0
                self.ez_integral = 0.0
                self.first_run = True
                self.old_time = self.get_clock().now()
                self.auto_ready = True
        else:
            self.auto_ready = False

    def send_speed_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3 or not self.auto_ready:
            return

        self.ex = msg.data[0]
        self.ey = msg.data[1]
        self.ez = msg.data[2]

    def control_loop(self):
        if not self.auto_ready:
            return

        new_time = self.get_clock().now()
        if self.old_time is None:
            self.old_time = new_time
            return

        dt = (new_time - self.old_time).nanoseconds * 1e-9
        dt = max(0.005, min(dt, 0.05))

        self.old_time = new_time

        if self.first_run:
            self.prev_ex = self.ex
            self.prev_ey = self.ey
            self.prev_ez = self.ez
            self.first_run = False
            return

        self.integral_calc(dt)
        self.drev_calc(dt)
        self.pid_calc()


        # --- 1. Deadband Thresholding ---
        if abs(self.vx) < 0.01:
            self.vx = 0.0
        if abs(self.vy) < 0.01:
            self.vy = 0.0
        if abs(self.wz) < 0.01:
            self.wz = 0.0

        # --- 2. Max Velocity Clamping ---
        self.vx = max(min(self.vx, MAX_VX), -MAX_VX)
        self.vy = max(min(self.vy, MAX_VY), -MAX_VY)
        self.wz = max(min(self.wz, MAX_WZ), -MAX_WZ)

        # --- 3. Map to Twist Message ---
        car = Twist()
        car.linear.x = float(self.vx)
        car.linear.y = float(self.vy) 
        car.angular.z = float(self.wz)

        self.pub_.publish(car)

        # update history
        self.prev_ex = self.ex
        self.prev_ey = self.ey
        self.prev_ez = self.ez


#        self.get_logger().info(
#            f"ex={self.ex:.3f}, ey={self.ey:.3f}, eyaw={math.degrees(self.ez)}, vx_cmd={self.vx:.3f}, d={self.drev_ex:.3f}"
#        )


def main(args=None):
    rclpy.init(args=args)
    node = Base_control()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

