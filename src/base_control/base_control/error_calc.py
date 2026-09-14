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

TICK_STOP_THRESHOLD = 2

RED_X = 1.0
RED_Y = 0.0
RED_YAW = 0.0 #in degrees

GREEN_X = 0.0
GREEN_Y = 0.2
GREEN_YAW = 0.0

BLUE_X = -1.0
BLUE_Y = 0.0
BLUE_YAW = 0.0 # in degrees ( Those are absolute value that should always be measured from teh starting point)

FINISH_X = 0.0
FINISH_Y = -0.2
FINISH_YAW = 0.0




# ============================================================
# State IDs sent to the picking node
# ============================================================

PLACE_RED = 1
PLACE_GREEN = 2
PLACE_BLUE = 3
FINISHED = 0

# ============================================================
# General constants
# ============================================================

ONE = 1
ZERO = 0

# ============================================================
# Position tolerance used to determine if waypoint is reached
# ============================================================

X_REACH = 0.1
Y_REACH = 0.1

YAW_REACH_DEG = 5.0  

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
        self.color_sent_flag = False

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

        self.current_step = 0

        # Matrix: [X_target, Y_target, Yaw_target_degrees, arm_action_id]
        #i am doing  this because the yaw readings are absolute
        self.mission_steps = {
            0: [0.6,   0.0,   -90.0,   None],          # 60cm forward
            1: [0.0,   0.0,   0.0,  None],          # Turn 90 deg CCW
            2: [1.3,   0.0,   0.0,   PLACE_RED],     # 100cm forward -> TRIGGER RED
            3: [-0.225,  0.0,   0.0,   None],          # 20cm backward
            4: [0.0,   1.7,   0.0,   PLACE_GREEN],   # 80cm sideways -> TRIGGER GREEN
            5: [0.0,  -0.3,   0.0,   None],          # -20cm sideways
            6: [0.0,   0.0,   -180.0, None],          # Rotate 180 deg CCW
            7: [1.362,   0.0,   -180.0,   None],          # Forward 40cm
            8: [0.623, 0.27, -180.0,   PLACE_BLUE],    # Diagonally 50cm -> TRIGGER BLUE
            9: [0.0,   0.0,   -180.0,   FINISHED]       # Finished / Stop
        }
        # ====================================================
        # Subscribers
        # ====================================================

        # Robot pose from odem sending 
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
        self.ser_sub = self.create_subscription(
            Float32MultiArray,
            'odem_ser',
            self.odem_ser_callback,
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
        self.pos_yaw = 0.0  #in degree
        self.orien = None

          #for tracking ticks values to use for state transitioning
        self.delta_left  = 999.0  #inital value
        self.delta_right = 999.0   #inital value

        # Current waypoint coordinates
        self.setpoint_in_x = 0.0
        self.setpoint_in_y = 0.0
        self.setpoint_in_yaw = 0.0 #in degrees
        self.odom_counter=0

    def intial_zero(self,x,y):
        self.pos_x=x
        self.pos_y=y
    
    def stage_reached(self, delta_x, delta_y,delta_yaw_deg):
        self.stage_reached_flag = (
            (-X_REACH <= delta_x <= X_REACH)
              and
              (-Y_REACH <= delta_y <= Y_REACH)
            and 
            (-YAW_REACH_DEG<= delta_yaw_deg <= YAW_REACH_DEG)
        )
         # Step 2: Check physical constraints (Both encoders must be idling)
        physically_stopped = abs(self.delta_left) <= TICK_STOP_THRESHOLD and \
                             abs(self.delta_right) <= TICK_STOP_THRESHOLD
        return self.stage_reached_flag and physically_stopped
    
   
    def update_setpoint(self):
        if self.current_step in self.mission_steps:
            step_data = self.mission_steps[self.current_step]
            self.setpoint_in_x = step_data[0]
            self.setpoint_in_y = step_data[1]
            self.setpoint_in_yaw = step_data[2]
            #self.get_logger().info(f"Loaded Step {self.current_step}: X={self.setpoint_in_x}, Y={self.setpoint_in_y}, Yaw={self.setpoint_in_yaw}")

    def auto_callback(self, msg: String):

        previous_mode = self.mode
        self.mode = msg.data

        #to allow for consectuive autonomous runs and switching between manual and autonomous
        if self.mode == 'AUTO':
            if previous_mode != 'AUTO':
                self.color_sent_flag = False
                self.stage_reached_flag = False
                self.odom_counter = 0 
                self.current_step = 0
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

        ex_robot = ex
        #(
        #     math.cos(yaw) * ex
        #     +
        #     math.sin(yaw) * ey
        # )

        ey_robot = ey
        # (
        #     -math.sin(yaw) * ex
        #     +
        #     math.cos(yaw) * ey
        # )
        eyaw_rad = math.radians(eyaw)
        wrapped_yaw_rad = math.atan2(math.sin(eyaw_rad), math.cos(eyaw_rad))
        eyaw = math.degrees(wrapped_yaw_rad)

        return [ex_robot, ey_robot, eyaw]

    #Just takes the readings coming from the base odometry
    def odem_ser_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.delta_left  = msg.data[0]
            self.delta_right = msg.data[1]
    
    def switch_stage_callback(self, msg: String):
        """Called when the arm signals completion of its task."""
        # Arm finished! Advance to the next step and clear latch locks
        self.current_step += 1
        self.stage_reached_flag = False
        self.color_sent_flag = False
        self.update_setpoint()

    def pose_callback(self, msg: Odometry):
        if self.mode != 'AUTO':
            self.odom_counter = 0
            return

        self.odom_counter += 1
        if self.odom_counter >= 10:
            
            # Read robot pose registers
            self.pos_x = msg.pose.pose.position.x
            self.pos_y = msg.pose.pose.position.y
            self.orien = msg.pose.pose.orientation

            radian_yaw = euler_from_quaternion(self.orien)[2]
            self.pos_yaw = math.degrees(radian_yaw)
            
            # --- LATCH INTERCEPT BLOCK ---
            if self.color_sent_flag:
                # Command absolute zero velocities down to motor controller nodes
                error_msg = Float32MultiArray(data=[0.0, 0.0, 0.0])
                self.error_pub.publish(error_msg)
                
                self.zero_flag.data = ONE
                self.zero_pub.publish(self.zero_flag)
                
                arm_action = self.mission_steps[self.current_step][3]
                if arm_action is None:
                    # FIXED: Only advance the intermediate steps IF your odometry node has 
                    # successfully processed the hardware reset signal back near (0, 0).
                    if abs(self.pos_x) < X_REACH and abs(self.pos_y) < Y_REACH:
                        self.current_step += 1
                        self.color_sent_flag = False  # Safely release execution latch lock
                        self.update_setpoint()
                return

            # Compute tracking coordinate discrepancies
            global_ex = self.setpoint_in_x - self.pos_x
            global_ey = self.setpoint_in_y - self.pos_y
            global_eyaw = self.setpoint_in_yaw - self.pos_yaw

            transformed_error = self.robot_transformation(
                global_ex,
                global_ey,
                global_eyaw,
                self.pos_yaw
            )

            if transformed_error is None:
                return

            # Validate target boundary conditions
            flag = self.stage_reached(
                self.setpoint_in_x - self.pos_x,
                self.setpoint_in_y - self.pos_y,
                transformed_error[2]
            )

            # --- NAVIGATION PHASE: Run toward target coordinates ---
            if not flag:
                error_msg = Float32MultiArray()
                error_msg.data = [
                    float(transformed_error[0]),
                    float(transformed_error[1]),
                    float(math.radians(transformed_error[2]))
                ]

                self.error_pub.publish(error_msg)
                self.zero_flag.data = ZERO
                self.zero_pub.publish(self.zero_flag)

            # --- ARREST PHASE: Destination reached ---
            else:
                arm_action = self.mission_steps[self.current_step][3]

                if arm_action is None:
                    # INTERMEDIATE STEP TRANSITION
                    self.zero_flag.data = ONE
                    self.zero_pub.publish(self.zero_flag)
                    
                    self.color_sent_flag = True  # Locks latch to protect physics synchronization loop
                    error_msg = Float32MultiArray(data=[0.0, 0.0, 0.0])
                    self.error_pub.publish(error_msg)
                else:
                    # ZONE TARGET OBJECTIVE STATE
                    if not self.color_sent_flag:
                        self.zero_flag.data = ONE
                        color_msg = Int8(data=arm_action) # FIXED dynamic allocation assignment rule

                        # Fire picker arm action sequence trigger
                        self.color_pub.publish(color_msg)
                        self.zero_pub.publish(self.zero_flag)
                    
                        self.color_sent_flag = True
                        error_msg = Float32MultiArray(data=[0.0, 0.0, 0.0])
                        self.error_pub.publish(error_msg)
                        
                        if arm_action == FINISHED:
                            self.get_logger().info("Competition Routine Completed successfully. Base Locked.")
        else: 
            error_msg = Float32MultiArray(data=[0.0, 0.0, 0.0])
            self.error_pub.publish(error_msg)
            return


def main(args=None):
    rclpy.init(args=args)
    node = Error_calc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
