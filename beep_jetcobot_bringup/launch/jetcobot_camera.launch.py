from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'beep_jetcobot_control',
            executable = 'aruco_detector',
            name = 'jetcobot_cam',
            output = 'screen',
        )
    ])