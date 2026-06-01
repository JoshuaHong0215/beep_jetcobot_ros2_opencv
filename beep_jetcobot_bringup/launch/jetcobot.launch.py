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
    ])
