import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Float32
import serial 
from std_msgs.msg import String

#usb-STMicroelectronics_STM32_Virtual_ComPort_307438693130-if00
#'/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_204B358F4B56-if00'
PORT='/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_307438693130-if00'
BAUD=115200
class DisplayNode(Node):
    def __init__(self):
        super().__init__("serial")
        self.ser=serial.Serial(PORT,BAUD,timeout=0.1)#add time out 
        self.subscriber_ = self.create_subscription(
            Float32MultiArray, "wheels_uart", self.callback_serial, 10
        )
        self.pub_wheels=self.create_publisher(Float32MultiArray,'odem_ser',10)

        self.pub_imu=self.create_publisher(Float32MultiArray,'imu_ser',10)

        self.pub_batt=self.create_publisher(Float32,'battery_ser',10)

        self.pub_ir=self.create_publisher(Float32MultiArray,'ir_ser',10)

        self.get_logger().info("Display Node started and waiting for the wheels speed to be calculated ...")
        
        self.create_timer(0.02,self.recive_parse_callback)
    
    def recive_parse_callback(self):
        try:
            message=self.ser.readline().decode(errors='ignore').strip()
            if not message:
                return
            values=[[float(x) for x in part.split(',')]
                    for part in message.split('|')]
            if len(values)<4:
                return
        except Exception:
            return
        imu=Float32MultiArray()
        imu.data=values[0]
        self.pub_imu.publish(imu)

        wheels=Float32MultiArray()
        wheels.data=values[1]
        self.pub_wheels.publish(wheels)
        
        battery = Float32()
        battery.data = values[2][0]
        self.pub_batt.publish(battery)

        Ir=Float32MultiArray()
        Ir.data=values[3]
        self.pub_ir.publish(Ir)


    def callback_serial(self, msg: Float32MultiArray):
        #self.get_logger().info(f"The velocities are being sent to arduino via uart\nspeed velocities are: w1={msg.data[0]} w2={msg.data[1]} w3={msg.data[2]} w4={msg.data[3]}")
        line = "{:.3f},{:.3f},{:.3f},{:.3f}\n".format(
            msg.data[0], msg.data[1], msg.data[2], msg.data[3]
            )
        self.ser.write(line.encode())




# Main entry point of the program
def main(args=None):
    rclpy.init(args=args)        # Initialize ROS 2 communication
    node = DisplayNode()         # Create the node
    rclpy.spin(node)             # Keep it alive and processing callbacks
    rclpy.shutdown()             # Shutdown ROS 2 when done


# Run main() only if this file is executed directly
if __name__ == "__main__":
    main()