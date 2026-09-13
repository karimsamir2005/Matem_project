#!/usr/bin/env python3
"""
ekf_fusion_node.py
------------------
Standalone ROS 2 Extended Kalman Filter sensor fusion node.

Subscribes:
  /mecanum_wheels/odom  (nav_msgs/Odometry)  — position + velocity
  /bno085/imu           (sensor_msgs/Imu)    — orientation + angular velocity

Publishes:
  /odometry/filtered    (nav_msgs/Odometry)

State vector  x = [x, y, yaw, vx, vy, vyaw]  (6-DOF planar)
Covariance    P = 6×6

This replicates the robot_localization ekf_node configuration for a
differential/mecanum base with an IMU — no dependency on that package.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String , Int8


#Indecis for each state variable in the state vector of the estimation.
#Those are the variables that construct the state vector of the car.
X     = 0
Y     = 1
YAW   = 2
VX    = 3
VY    = 4
VYAW  = 5
N_STATES = 6


#This is the function that is responsible for taking care of the sudden jump from -180
def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


#the function used to change the quaternions bieng send to euler angles
def euler_from_quaternion(q) -> tuple:
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    return r.as_euler('xyz', degrees=False)



class EKFFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        #Decalring parameters for estimation uncertainity
        self.declare_parameter('process_noise_x',    0.2)
        self.declare_parameter('process_noise_y',    0.2)
        self.declare_parameter('process_noise_yaw',  0.05)
        self.declare_parameter('process_noise_vx',   0.3)
        self.declare_parameter('process_noise_vy',   0.3)
        self.declare_parameter('process_noise_vyaw', 0.025)


        #Declaring parameters for initial estimation uncertainity
        self.declare_parameter('initial_cov_x',    1.0)
        self.declare_parameter('initial_cov_y',    1.0)
        self.declare_parameter('initial_cov_yaw',  1.0)
        self.declare_parameter('initial_cov_vx',   1.0)
        self.declare_parameter('initial_cov_vy',   1.0)
        self.declare_parameter('initial_cov_vyaw', 1.0)

        #Those are for covariance of sensor uncertainity over ride for configuring sensor uncertainity
        self.declare_parameter('fallback_odom_pose_var_x', 0.05)
        self.declare_parameter('fallback_odom_pose_var_y' , 0.05)
        self.declare_parameter('fallback_odom_twist_var_x', 0.02)
        self.declare_parameter('fallback_odom_twist_var_y', 0.02)
        self.declare_parameter('fallback_imu_yaw_var', 0.01)
        self.declare_parameter('fallback_imu_vyaw_var', 0.005)

        self.declare_parameter('predict_frequency', 50.0)

        self._x = np.zeros(N_STATES)

        #Those are the actual matrices
        self._P = np.diag([
            self.get_parameter('initial_cov_x').value,
            self.get_parameter('initial_cov_y').value,
            self.get_parameter('initial_cov_yaw').value,
            self.get_parameter('initial_cov_vx').value,
            self.get_parameter('initial_cov_vy').value,
            self.get_parameter('initial_cov_vyaw').value,
        ])

        self._Q = np.diag([
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_yaw').value,
            self.get_parameter('process_noise_vx').value,
            self.get_parameter('process_noise_vy').value,
            self.get_parameter('process_noise_vyaw').value,
        ])

        self._last_predict_time = None

        #Imu initialization variables to make the ekf start at zero
        self._imu_initialized = False
        self._yaw_offset = 0.0
        self._imu_samples = []

        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_yaw = 0.0

        # AUTO frame shift reference defenitions
        self._auto_ref_x = 0.0
        self._auto_ref_y = 0.0
        self._auto_ref_yaw = 0.0
        self._auto_active = False

        # STATE frame shift reference defenitions
        self._state_ref_x = 0.0
        self._state_ref_y = 0.0
        self._state_ref_yaw = 0.0
        self._state_active = False
        self._last_state_id = -1


        self._last_mode = None
        self._last_state_finish = 0

        #--------Subscribtions-----------
        self.create_subscription(Odometry,'mecanum_wheels/odom',self._odom_callback,10)
        self.create_subscription(Imu,'bno085/imu',self._imu_callback,10)
        self.create_subscription(String,'auto_active',self._auto_callback,10)
        self.create_subscription(Int8,'state_finish',self._state_callback,10)

        #--------Publishers----------------
        self._pub = self.create_publisher(Odometry,'/odometry/filtered',10)
        self.create_timer(1.0 / self.get_parameter('predict_frequency').value,
                          self._predict_and_publish)

    # =========================
    # MODE RESET (AUTO FRAME)
    # =========================
    def _auto_callback(self , msg: String):

        #take the value of the current mode and check whether the mode changed or not
        mode = msg.data

        if mode == self._last_mode:
            return

        self._last_mode = mode

        #if the mode is switched to autonomous make the new autonomous refernce varaibles from the global state variable
        if mode == "AUTO":
            self._auto_ref_x = self._x[X]
            self._auto_ref_y = self._x[Y]
            self._auto_ref_yaw = self._x[YAW]
            self._auto_active = True

        if mode != "AUTO":
            self._auto_active = False


    # =========================
    # STATE RESET (STATE FRAME SHIFT)
    # =========================
    def _state_callback(self, msg: Int8):

        state = msg.data

        if state == self._last_state_finish:
            return

        self._last_state_finish = state

        #if the state is finished it will be 1
        if state == 1:

            #those are the variables for the  new state frame
            self._state_ref_x = self._x[X]
            self._state_ref_y = self._x[Y]
            self._state_ref_yaw = self._x[YAW]
            self._state_active = True
            self._last_state_id += 1

            self._origin_x = self._x[X]
            self._origin_y = self._x[Y]
            self._origin_yaw = self._x[YAW]

            np.fill_diagonal(self._P, [
                self.get_parameter('initial_cov_x').value,
                self.get_parameter('initial_cov_y').value,
                self.get_parameter('initial_cov_yaw').value,
                self.get_parameter('initial_cov_vx').value,
                self.get_parameter('initial_cov_vy').value,
                self.get_parameter('initial_cov_vyaw').value,
            ])


    #This is the function usef for initializing with very state change or mode change
    def _apply_origin(self, x, y, yaw):

        if self._auto_active:
            x = x - self._auto_ref_x
            y = y - self._auto_ref_y
            yaw = wrap_angle(yaw - self._auto_ref_yaw)

        if self._state_active:
            x = x - self._state_ref_x
            y = y - self._state_ref_y
            yaw = wrap_angle(yaw - self._state_ref_yaw)

        return (
            x - self._origin_x,
            y - self._origin_y,
            wrap_angle(yaw - self._origin_yaw)
        )

    def _odom_callback(self, msg: Odometry):
        yaw = float(self._x[YAW])

        # extract raw encoder velocities (BODY FRAME)
        vx_b = msg.twist.twist.linear.x
        vy_b = msg.twist.twist.linear.y

        # get yaw estimate from IMU-free source OR previous EKF state
        yaw = self._x[YAW]   # IMPORTANT: use EKF current estimate

        cy = np.cos(yaw)
        sy = np.sin(yaw)

        # transform to WORLD frame
        vx_w = vx_b * cy - vy_b * sy
        vy_w = vx_b * sy + vy_b * cy

        z = np.array([
        # msg.pose.pose.position.x,
        # msg.pose.pose.position.y,
        vx_w,
        vy_w,
        ])
        obs_idx = [VX, VY]

        pose_cov  = np.array(msg.pose.covariance).reshape(6, 6)
        twist_cov = np.array(msg.twist.covariance).reshape(6, 6)

        fb_pose_x  = self.get_parameter('fallback_odom_pose_var_x').value
        fb_pose_y  = self.get_parameter('fallback_odom_pose_var_y').value
        fb_twist_x = self.get_parameter('fallback_odom_twist_var_x').value
        fb_twist_y = self.get_parameter('fallback_odom_twist_var_y').value

        R = np.diag([
            pose_cov[0, 0]  if pose_cov[0, 0]  > 1e-5 else fb_pose_x,
            pose_cov[1, 1]  if pose_cov[1, 1]  > 1e-5 else fb_pose_y,
            twist_cov[0, 0] if twist_cov[0, 0] > 1e-5 else fb_twist_x,
            twist_cov[1, 1] if twist_cov[1, 1] > 1e-5 else fb_twist_y,
        ])
        self._ekf_update(z, obs_idx, R)

    def _imu_callback(self, msg: Imu):

        roll, pitch, raw_yaw = euler_from_quaternion(msg.orientation)

        if not self._imu_initialized:
            self._imu_samples.append(raw_yaw)

            if len(self._imu_samples) >= 10:
                self._yaw_offset = np.arctan2(
                    np.mean(np.sin(self._imu_samples)),
                    np.mean(np.cos(self._imu_samples))
                )
                self._imu_initialized = True
            return

        yaw = wrap_angle(raw_yaw - self._yaw_offset)

        z = np.array([yaw, msg.angular_velocity.z])
        obs_idx = [YAW, VYAW]

        orient_cov = np.array(msg.orientation_covariance).reshape(3, 3)
        angvel_cov = np.array(msg.angular_velocity_covariance).reshape(3, 3)

        fb_yaw_var = self.get_parameter('fallback_imu_yaw_var').value
        fb_vyaw_var = self.get_parameter('fallback_imu_vyaw_var').value

        R = np.diag([
            orient_cov[2, 2]  if orient_cov[2, 2]  > 1e-5 else fb_yaw_var,
            angvel_cov[2, 2]  if angvel_cov[2, 2]  > 1e-5 else fb_vyaw_var,
        ])

        self._ekf_update(z, obs_idx, R)

    def _ekf_predict(self, dt: float):

        if dt <= 0.0:
            return

        x, y, yaw, vx, vy, vyaw = self._x
        cyaw = np.cos(yaw)
        syaw = np.sin(yaw)

        vx_w = vx * cyaw - vy * syaw
        vy_w = vx * syaw + vy * cyaw

        self._x[X] += dt * vx_w
        self._x[Y] += dt * vy_w
        self._x[YAW]  = wrap_angle(self._x[YAW] + dt * vyaw)

        F = np.eye(N_STATES)
        F[X, YAW] = dt * (-vx * syaw - vy * cyaw)
        F[Y, YAW] = dt * ( vx * cyaw - vy * syaw)
        F[X, VX]  = dt * cyaw
        F[X, VY]  = dt * -syaw
        F[Y, VX]  = dt * syaw
        F[Y, VY]  = dt * cyaw
        F[YAW, VYAW] = dt

        self._P = F @ self._P @ F.T + self._Q * dt

    def _ekf_update(self, z, obs_idx, R):

        H = np.zeros((len(obs_idx), N_STATES))
        for i, c in enumerate(obs_idx):
            H[i, c] = 1.0

        y = z - H @ self._x

        for i, idx in enumerate(obs_idx):
            if idx == YAW:
                y[i] = wrap_angle(y[i])

        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)

        self._x = self._x + K @ y
        self._x[YAW] = wrap_angle(self._x[YAW])

        I = np.eye(N_STATES)
        self._P = (I - K @ H) @ self._P


    def _predict_and_publish(self):

        if not self._imu_initialized:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        if self._last_predict_time is None:
            self._last_predict_time = now
            return

        dt = now - self._last_predict_time
        self._last_predict_time = now

        self._ekf_predict(dt)
        self._publish()


    def _publish(self):

        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.header.stamp = self.get_clock().now().to_msg()

        x, y, yaw = self._apply_origin(
            self._x[X], self._x[Y], self._x[YAW]
        )

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        q = Rotation.from_euler('z', yaw).as_quat()
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.twist.twist.linear.x = self._x[VX]
        msg.twist.twist.linear.y = self._x[VY]
        msg.twist.twist.angular.z = self._x[VYAW]

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()