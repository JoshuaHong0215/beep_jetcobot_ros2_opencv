from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beep_jetcobot_control',
            executable='joint_control',
            name='joint_control_node',
            output='screen',
        ),
        Node(
            package='beep_jetcobot_control',
            executable='yolo_detector',
            name='yolo_detector_node',
            output='screen',
        ),
        Node(
            package='beep_jetcobot_control',
            executable='pick_place_action_server_ver6',
            name='pick_place_action_server',
            output='screen',
        ),
    ])
