from ntpath import join
import os
from ament_index_python.packages import get_package_share_directory
from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from adafruit_servokit import ServoKit

servo_driver_config = os.path.join(
    get_package_share_directory('xmo_py'),
        'config',
        'servo_driver.yaml'
    )

servo_node_config = os.path.join(
    get_package_share_directory('xmo_py'),
        'config',
        'servo_node.yaml'
    )

drive_node_config = os.path.join(
    get_package_share_directory('xmo_py'),
        'config',
        'drive_mode_node.yaml'
    )

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joystick'
        ),
        Node(
            package='xmo_py',
            executable='servo_driver',
            name='servo_driver_node',
            parameters=[servo_driver_config]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                {'enable_button': -1},
                {'require_enable_button': False},
                {'axis_linear.x': 0},
                {'axis_linear.y': 1},
                {'axis_linear.z': 1},
                {'axis_angular.pitch': 3},
                {'axis_angular.roll': 2},
                {'axis_angular.yaw': 3},
                {'scale_linear.x': -90.0},
                {'scale_linear.y': 90.0},
                {'scale_linear.z': 1.0},
                {'scale_angular.pitch': 90.0},
                {'scale_angular.roll': -90.0},
                {'scale_angular.yaw': 1.0}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='lf_steer_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='lf_drive_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='rf_steer_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='rf_drive_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='lm_steer_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='lm_drive_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='rm_steer_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='rm_drive_node',
            parameters=[servo_node_config]
        ),        
        Node(
            package='xmo_py',
            executable='servo_node',
            name='lr_steer_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='lr_drive_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='rr_steer_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='rr_drive_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='ackerman_drive_node',
            name='ackerman_drive_node',
            parameters=[drive_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='camera_x_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='servo_node',
            name='camera_y_node',
            parameters=[servo_node_config]
        ),
        Node(
            package='xmo_py',
            executable='camera_position_node',
            name='camera_x_position_node',
            parameters=[servo_node_config]
        )
    ])
