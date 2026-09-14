#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction  # Required for grouping architecture

#--------------------------------------------------------------Launch Description-------------------------------------------------------------------------
def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('matem_bringup'),
        'config',                                   
        'base_ps4_config.yaml',
    )

    config_dir = get_package_share_directory('matem_bringup')


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

    # 2. Joystick Driver(This is the node responsible for reading from the PS4 Joystick)
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

    # 6. This is the Finite State Machine responsible for active mode switching
    mode_manager=Node(
        package='base_control',
        executable="manager",
    )

    # 7.This is the forward kinematic control node in the autonomous stage
    auto_control=Node(
        package='base_control',
        executable='auto_control'
    )

    # ===================================================================================
    # BASE NAVIGATION STACK GROUP (Fully Aligned & Streamlined)
    # ===================================================================================
    base_navigation_group = GroupAction(
        actions=[
            # 3.5 Speed Quantizer (Moved inside the core driving chain)
            Node(
                package='base_control',
                executable='speed_quantizer',
                remappings=[
                    ('/cmd_vel_raw', '/cmd_vel_raw'),
                    ('/cmd_vel_quantized', '/cmd_vel')
                ]
            ),
            # 4. Inverse Kinematic node
            Node(
                package='base_control',
                executable='ik',
                name='wheel_calculator'
            ),
            # 5. Serial Node
            Node(
                package='base_control',
                executable='ser'
            ),
            # 8. Communication debugging node
            Node(
                package='base_control',
                executable='odem_send'
            ),
            # 11. Error calculation node
            Node(
                package='base_control',
                executable='error_calc'
            ),
            # 20. Plan B control for the base
            Node(
                package='base_control',
                executable='pd_control',
            )
        ]
    )

    # ===================================================================================
    # CAMERA VISION MODULE GROUP
    # ===================================================================================
    camera_vision_group = GroupAction(
        actions=[
            # 13. Camera Node Responsible for scanning the qr code
            Node(
                package='cam',            
                executable='qr_scan',                
            ),
            # # 15. Mode switching code between the camera's Functions
            # Node(
            #     package='cam',            
            #     executable='vision_manager',                
            # )
        ]
    )

    # ===================================================================================
    # MANIPULATOR MODULE GROUP
    # ===================================================================================
    robotic_arm_group = GroupAction(
        actions=[
            # 16. Node responsible for taking and filtering inputs
            Node(
                package='controller_pkg',
                executable='arm_input_node',
            ),
            # 17. Inverse kinematic engine of the manipulator
            Node(
                package='controller_pkg',
                executable='cyl_ik_node',
            ),
            # 18. Serial communication node for the manipulator module
            Node(
                package='controller_pkg',
                executable='serial_bridge_node',
            ),
            # 19. Arm sequencer
            Node(
                package='controller_pkg',
                executable='arm_sequencer',
            )
        ]
    )

    led_feedback_node = Node(
        package='controller_pkg',
        executable='led_feedback_node',
    )

    beep_button_node = Node(
        package='controller_pkg',
        executable='beep_button_node',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        mode_manager,
        led_feedback_node,
        beep_button_node,
        #auto_control,
        # Isolated Scoped Groups
        base_navigation_group,
        camera_vision_group,
        robotic_arm_group,
    ])