# =============================================================================
# viz.launch.py
# =============================================================================
# Visualization-only launch for desk development.
#   - Loads the arm URDF
#   - Runs robot_state_publisher (URDF -> TF)
#   - Optionally runs joint_state_publisher_gui (sliders) when gui:=true
#   - Runs RViz with the arm config
#
# Two ways to use it:
#
#   A) Pure URDF check (no real nodes running):
#        ros2 launch arm_bringup viz.launch.py
#      Default gui:=true. Slide each joint, confirm meshes/limits/axes.
#
#   B) Live system viz (cyl_ik_node etc. running, providing /joint_states):
#        ros2 launch arm_bringup viz.launch.py gui:=false
#      jsp_gui is suppressed; RViz animates from whoever else publishes
#      /joint_states (cyl_ik_node in our case).
#
# This launch never starts joy / arm_input / cyl_ik / serial_bridge — those
# belong in control.launch.py. Keep the split so the Pi install never has
# to pull in RViz.
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare('arm_description')

    xacro_file  = PathJoinSubstitution([desc_share, 'urdf',  'arm.urdf.xacro'])
    rviz_config = PathJoinSubstitution([desc_share, 'rviz',  'manual_arm.rviz'])

    # value_type=str prevents ROS 2 from misparsing the URDF XML as YAML.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run joint_state_publisher_gui (slider testing). '
                    'Set false when cyl_ik_node owns /joint_states.',
    )
    gui = LaunchConfiguration('gui')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher,
        jsp_gui,
        rviz,
    ])
