import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int8
from std_msgs.msg import String
import math
from scipy.spatial.transform import Rotation

"""
Coordinate Frame Convention:

+X -> Forward
+Y -> Left
+Z -> Up

Positive yaw (+yaw)  -> Counter-clockwise rotation
Negative yaw (-yaw)  -> Clockwise rotation
"""

# ============================================================
# Navigation Setpoints (target locations for each box color)
# ============================================================

RED_X = 1.0
RED_Y = 0.0

GREEN_X = 0.0
GREEN_Y = 1.0

BLUE_X = -1.0
BLUE_Y = 0.0

FINISH_X = 0.0
FINISH_Y = -1.0




# ============================================================
# State IDs sent to the picking node
# ============================================================

PLACE_RED = 1
PLACE_GREEN = 2
PLACE_BLUE = 3
FINSHED = 0

# ============================================================
# General constants
# ============================================================

ONE = 1
ZERO = 0

# ============================================================
# Position tolerance used to determine if waypoint is reached
# ============================================================

X_REACH = 0.3
Y_REACH = 0.3
YAW_REACH=15

#===============================================================
#auto setpoints 
#===============================================================

START_AUTO_X=0.6
START_AUTO_Y=0.0
START_AUTO_ROTATE=90
FLAG_1=ZERO

CHANGE_1_X=-0.5
CHANGE_1_Y=0.0
FLAG_2=ZERO

CHANGE_2_X=-0.0
CHANGE_2_Y=0.5
CHANGE_2_ROTATE=180
FLAG_3=ZERO

MID_SETPOINT_3_X=1
MID_SETPOINT_3_Y=0
FLAG_4=ZERO


def euler_from_quaternion(quaternion):
    """
    Convert quaternion orientation to Euler angles.

    Input:
        quaternion (x,y,z,w)

    Returns:
        [roll, pitch, yaw]
    """
    r = Rotation.from_quat(
        [quaternion.x,
         quaternion.y,
         quaternion.z,
         quaternion.w]
    )

    return r.as_euler('xyz', degrees=False)


class Error_calc(Node):

    def __init__(self):

        super().__init__('error_calculation')

        # ====================================================
        # Flags
        # ====================================================

        # Published when robot reaches a target position
        self.zero_flag = Int8()
        self.zero_flag.data = ONE

        # Prevents sending the same color command repeatedly
        self.color_sent_flag = ZERO

        # ====================================================
        # Robot mode and current target
        # ====================================================

        # Start in manual mode
        self.mode = 'MAN'

        # First target is the red box
        self.color_to_place = 'RED'
        self.color_to_place_number = PLACE_RED

        # True when current waypoint is reached
        self.stage_reached_flag = False

        # ====================================================
        # Subscribers
        # ====================================================

        # Robot pose from EKF/localization
        self.pose_sub = self.create_subscription(
            Odometry,
            'odem_final',
            self.pose_callback,
            10
        )

        # AUTO / MANUAL mode selection
        self.sub = self.create_subscription(
            String,
            'auto_active',
            self.auto_callback,
            10
        )

        # Message indicating box placement completed
        self.next_stage_sub = self.create_subscription(
            String,
            'box_done',
            self.switch_stage_callback,
            10
        )

        # ====================================================
        # Publishers
        # ====================================================

        # Position error sent to controller
        self.error_pub = self.create_publisher(
            Float32MultiArray,
            'auto_base',
            10
        )

        # Signal that waypoint has been reached
        self.zero_pub = self.create_publisher(
            Int8,
            'state_finish',
            10
        )

        # Tell picker node which color should be handled
        self.color_pub = self.create_publisher(
            Int8,
            'pick_color',
            10
        )

        # ====================================================
        # Robot state variables
        # ====================================================

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_yaw = 0.0

        self.orien = None

        # Current waypoint coordinates
        self.setpoint_in_x = 0.0
        self.setpoint_in_y = 0.0
        self.setpoint_in_yaw = 0.0

    def stage_reached(self, delta_x, delta_y,delta_yaw):
        """
        Check whether the robot is inside the allowed
        tolerance window around the target.
        """

        self.stage_reached_flag = (
            (-X_REACH <= delta_x <= X_REACH)
            and
            (-Y_REACH <= delta_y <= Y_REACH)
            and 
            (-YAW_REACH<= delta_yaw <= YAW_REACH)
        )
    def manuvers(self):
        return
    def update_setpoint(self):
        """
        Update target waypoint according to the
        currently requested box color.
        """

        if self.color_to_place == 'RED':
            self.setpoint_in_x = RED_X
            self.setpoint_in_y = RED_Y

        elif self.color_to_place == 'GREEN':
            self.setpoint_in_x = GREEN_X
            self.setpoint_in_y = GREEN_Y

        elif self.color_to_place == 'BLUE':
            self.setpoint_in_x = BLUE_X
            self.setpoint_in_y = BLUE_Y

        elif self.color_to_place == 'FINISHED':
            self.setpoint_in_x = FINISH_X
            self.setpoint_in_y = FINISH_Y

    def auto_callback(self, msg: String):
        """
        Receives current operation mode.

        AUTO -> start navigation.
        MAN  -> ignore pose processing.
        """

        self.mode = msg.data

        if self.mode == 'AUTO':
            self.update_setpoint()

    def robot_transformation(
            self,
            ex: float,
            ey: float,
            eyaw: float,
            yaw: float):
        """
        Transform global position error into robot frame.

        World Frame:
            ex, ey

        Robot Frame:
            ex_robot, ey_robot

        Returns:
            [ex_robot, ey_robot, eyaw]
        """

        if self.mode != 'AUTO':
            return None

        ex_robot = (
            math.cos(yaw) * ex
            +
            math.sin(yaw) * ey
        )

        ey_robot = (
            -math.sin(yaw) * ex
            +
            math.cos(yaw) * ey
        )

        # Normalize yaw error to [-pi, pi]
        eyaw_ = math.atan2(
            math.sin(eyaw),
            math.cos(eyaw)
        )

        return [ex_robot, ey_robot, eyaw_]

    def switch_stage_callback(self, msg: String):
        """
        Called after a box is successfully placed.

        Advances the mission to the next color target.
        """

        if msg.data == 'done red':
            self.color_to_place = 'GREEN'
            self.color_to_place_number = PLACE_GREEN

        elif msg.data == 'done green':
            self.color_to_place = 'BLUE'
            self.color_to_place_number = PLACE_BLUE

        elif msg.data == 'done blue':
            self.color_to_place = 'FINISHED'
            self.color_to_place_number = FINSHED

        # Reset state for next target
        self.stage_reached_flag = False

        # Load new waypoint
        self.update_setpoint()

    def pose_callback(self, msg: Odometry):
        """
        Main navigation loop.

        Runs every time odometry is received.
        Computes error between robot pose and target pose.
        """

        # Ignore odometry while not in AUTO mode
        if self.mode != 'AUTO':
            return

        # ====================================================
        # Read robot pose
        # ====================================================

        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        self.orien = msg.pose.pose.orientation

        self.pos_yaw = euler_from_quaternion(
            self.orien
        )[2]

        # ====================================================
        # Compute position errors
        # ====================================================

        transformed_error = self.robot_transformation(
            self.setpoint_in_x - self.pos_x,
            self.setpoint_in_y - self.pos_y,
            self.setpoint_in_yaw - self.pos_yaw,
            self.pos_yaw
        )

        # Check if waypoint reached
        self.stage_reached(
            self.setpoint_in_x - self.pos_x,
            self.setpoint_in_y - self.pos_y,
            transformed_error[2]
        )

        # ====================================================
        # Navigation phase
        # ====================================================

        if not self.stage_reached_flag:

            # Allow color message to be sent once
            # after waypoint is reached
            self.color_sent_flag = False

            error_msg = Float32MultiArray()
            error_msg.data = transformed_error

            # Send controller error
            self.error_pub.publish(error_msg)

        # ====================================================
        # Waypoint reached phase
        # ====================================================

        else:

            if not self.color_sent_flag:

                color_msg = Int8()
                color_msg.data = self.color_to_place_number

                # Tell picker which color should be placed
                self.color_pub.publish(color_msg)

                # Notify that navigation finished
                self.zero_pub.publish(self.zero_flag)

                # Prevent repeated publishing
                self.color_sent_flag = True


def main(args=None):

    rclpy.init(args=args)

    node = Error_calc()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()