import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class JointControlNode(Node):
    def __init__(self):
        # 노드 이름 등록
        super().__init__('joint_control_node')

        self.joint_pub = self.create_publisher(
            Float32MultiArray, 
            '/mycobot/joint_states_cmd', 
            10
            )
        
    
    def timer_callback(self):
        pass



def main(args=None):
    rp.init(args=args)


