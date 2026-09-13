import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


SERIAL_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'    # Change to /dev/ttyACM0 if connected via ESP32 native USB
BAUD_RATE   = 921600


#--------------------------------------------------------------Launch Decription-------------------------------------------------------------------------
def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('matem_bringup'),
        'config',                                   
        'base_ps4_config.yaml',
    )

    config_dir = get_package_share_directory('matem_bringup')


#------------------------Those are the 3 argumnets coming from the launch file of qaffas i don't know what are those----------------------
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial/by-id/'
                      'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        description='Serial device path for the ESP32 connection',
    )

    joy_dev_id_arg = DeclareLaunchArgument(
        'joy_dev_id',
        default_value='0',
        description='Joystick device index (/dev/input/js<N>)',
    )

    # serial_port = LaunchConfiguration('serial_port')


    #----------------------------------------------------------------General System Nodes----------------------------------------------------------------

    # # 1. Foxglove Bridge (CLEANED LINE 25)
    # foxglove_bridge_dir = get_package_share_directory('foxglove_bridge')
    # foxglove_launch_file = os.path.join(foxglove_bridge_dir, 'launch', 'foxglove_bridge_launch.xml')

    # foxglove_bridge = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(foxglove_launch_file),
    #     launch_arguments={
    #         'port': '8765',
    #         'address': '0.0.0.0',
    #         'topic_whitelist': "['/velocity_dash', '/wheels_dash', '/lane_processed', '/qr_processed', '/robot_mode', '/camera_data', '/lane_camera_raw', '/qr_camera_raw','/arm_motor_zones','/cmd_vel' , 'tilt_offset']"
    #     }.items()
    # )

    # # '/camera/.*', '/qr_result', '/imu/yaw', '/battery_voltage', '/lane_camera_raw', , '/parameter_events', '/qr_camera_raw'

    # 2. Joystick Driver(This is the node repsonsible for reading from the PS4 Joystick)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'deadzone': 0.05},{'autorepeat_rate': 50.0},{'coalesce_interval_ms': 10}],
    )


    #----------------------------------------------------------------Base Nodes-------------------------------------------------------------------------

    # 3. Teleop Bridge(Node used for mapping the buttons of the ps4 to the base velocity values)
    teleop_node = Node (
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node_modified',
        parameters=[config_path], 
        remappings=[('/joy', '/base_joy'),
                    ('/cmd_vel', '/cmd_vel_raw')]
       
    )

    # 3.5 Speed Quantizer(This is an intermidiate node responsible for making analog velocity values into steps)
    quantizer_node = Node(
        package='base_control',
        executable='speed_quantizer',
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_raw'),
            ('/cmd_vel_quantized', '/cmd_vel')
        ]
    )

    # 4. Inverse Kinematic node (This is the Inverse kinematic control model of the mecanum base)
    ik_node = Node(
        package='base_control',
        executable='ik',
        name='wheel_calculator'
    )

    # 5. Serial Node ( The node responsible for serial communication between the pi and the low-level MCU's)
    serial_node = Node(
        package='base_control',
        executable='ser'
    )


    #6. This is the Finite State Machine repsonsible for active mode switching
    mode_manager=Node(
        package='base_control',
        executable="manager",
    )


    #7.This is the forward kinematic control node in the autonomous stage
    auto_control=Node(
        package='base_control',
        executable='auto_control'
    )

    #8.This is a communication debugging node for turning csv messages to standard odometry message
    odem_sending=Node(
        package='base_control',
        executable='odem_send'
    )


    # #10.This is a communication testing node for generaating IMU readings
    # imu_sending=Node(
    #     package='base_control',
    #     executable='imu_sending'
    # )

    #11.This is the node responsible for error calculation and stage changing of the autonomous navigation
    error_calc=Node(
        package='base_control',
        executable='error_calc'
    )

    # #12. This is the Extended-kalman filter logic node
    # ekfnode = Node(
    #     package ='base_control',
    #     executable='ekf_fusion_node',
    #         parameters=[{
    #     'process_noise_x':    0.05,
    #     'process_noise_yaw':  0.06,
    #     'predict_frequency':  50.0,
    # }]
    # )

    #----------------------------------------------------------------Camera Nodes-------------------------------------------------------------------------

    #13. Camera Node Responsible for scanning the qr code and sending the results
    qrNode = Node(
        package='cam',            # The package that contains the executable
        executable='qr_scan',                # The actual Python executable (node) to run

    )

    #14. Camera Node responsible for Lane detection and calculating the tilt and offset
    laneNode = Node(
        package='cam',            # Package containing the subscriber node
        executable='lane_detection',               # Subscriber node executable

    )

    #15. This is the mode siwtching code between the camera's Functions
    VisionManager = Node(
        package='cam',            # The package that contains the executable
        executable='vision_manager',                # The actual Python executable (node) to run
        # name='edges_node_publisher',                  # Name of the node (can be different from executable)
    )

    #----------------------------------------------------------------Manipulator Nodes--------------------------------------------------------------------

    #16. Node resposnible for taking and filtering the inputs coming from the joy node
    arm_input_node = Node(
        package='controller_pkg',
        executable='arm_input_node',
    )

    #17. This is the inverse kinematic engine of the manipulator
    cyl_ik_node = Node(
        package='controller_pkg',
        executable='cyl_ik_node',
    )


    #18. This is the serial communication node for the manipulator module
    serial_bridge_node = Node(
        package='controller_pkg',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            #'serial_port': serial_port,
            'baud_rate':   921600,
        }],
    )

    #18.There should be a node subscribing to odom_filtered


    return LaunchDescription([
        joy_node,
        teleop_node,
        quantizer_node,
        ik_node,
        serial_node,
        mode_manager,
        auto_control,
        odem_sending,
        #imu_sending,
        qrNode,
        laneNode,
        VisionManager,   
        #foxglove_bridge,
        #ekfnode,
        error_calc,
        arm_input_node,
        cyl_ik_node,
        serial_bridge_node,
        #I don't know what is the function of those 2 below(They are communication testing nodes)
        serial_port_arg,
        joy_dev_id_arg
    ])