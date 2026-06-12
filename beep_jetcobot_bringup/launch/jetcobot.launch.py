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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='beep_jetcobot_control',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
        ),
        Node(
            package='beep_jetcobot_control',
            executable='pick_place_action_server',
            name='pick_place_action_server',
            output='screen',
        ),
    ])
