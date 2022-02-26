from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from adafruit_servokit import ServoKit

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joystick'
        ),
        Node(
            package='xmo_py',
            executable='servos',
            name='servos_node'
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
        )
    ])
