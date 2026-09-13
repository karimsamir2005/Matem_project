import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray,String
from scipy.spatial.transform import Rotation

# def euler_from_quaternion(quaternion):
#     """
#     Converts quaternion (x, y, z, w) to euler (roll, pitch, yaw)
#     """
#     r = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
#     return r.as_euler('xyz', degrees=False)


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_readings_publisher')

        self.publisher_ = self.create_publisher(Imu, 'bno085/imu', 10)
        self.sub=self.create_subscription(String,'auto_active',self.auto_callback,10)
        self.sub_imu_ser=self.create_subscription(Float32MultiArray,'imu_ser',self.imu_recive,10)
        self.mode=None
        
    def auto_callback(self,msg:String):
        self.mode=msg.data

    def imu_recive(self,imu:Float32MultiArray):
        # if self.mode != 'AUTO':
        #     return # Skip if not in AUTO mode

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        # FIX: Using scipy instead of the missing function
        r = Rotation.from_euler('xyz', [0.0, 0.0, imu.data[0]])
        q = r.as_quat() # returns [x, y, z, w]

        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        msg.angular_velocity.z = imu.data[1]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = IMUPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()