import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('beep_jetcobot_description'),
        'urdf', 'mycobot_280_pi.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='beep_jetcobot_control',
            executable='joint_control',
            name='joint_control_node',
            output='screen',
        ),
        Node(
            package='beep_jetcobot_control',
            executable='pick_place_ver2',
            name='pick_place_ver2',
            output='screen',
        ),
        Node(
            package='beep_jetcobot_control',
            executable='contour_detector',
            name='contour_detector',
            output='screen',
        ),
    ])
