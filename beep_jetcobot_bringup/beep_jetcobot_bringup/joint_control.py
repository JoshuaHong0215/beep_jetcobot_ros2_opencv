import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointControlNode(Node):
    def __init__(self):
        # 노드 이름 등록
        super().__init__('joint_control_node')

        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states', 
            10
            )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('jetcobot 관절 제어 노드가 켜졌습니다')

    def timer_callback(self):
        msg = JointState()
        msg.name = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6']
        
        msg.position = [
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0
            ]
        
        self.joint_pub.publish(msg)



def main(args=None):
    rp.init(args=args)
    node = JointControlNode()

    try:
        rp.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()


