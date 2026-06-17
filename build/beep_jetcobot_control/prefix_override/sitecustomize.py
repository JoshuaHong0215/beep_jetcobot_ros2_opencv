import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jetcobot/jetcobot_ws/src/beep_jetcobot_ros2/install/beep_jetcobot_control'
