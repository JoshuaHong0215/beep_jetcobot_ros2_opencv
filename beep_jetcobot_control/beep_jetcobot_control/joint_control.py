import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int32

from pymycobot.mycobot import MyCobot
import math

class JointControlNode(Node):
    def __init__(self):
        # 노드 이름 등록
        super().__init__('joint_control_node')

        # 장치 연결
        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000)
        # 속도
        self.speed = 30


        self.create_subscription(
            Float32MultiArray,
            '/joint_command',
            self.joint_command_cb,
            10
            )

        self.create_subscription(
            Float32MultiArray,
            '/coord_command',
            self.coords_command_cb,
            10
            )

        self.create_subscription(
            Int32, 
            '/gripper_command', 
            self.gripper_command_cb, 
            10
            )

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
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]
            
        
        angles = self.mc.get_angles()
        if angles and len(angles) == 6:
            msg.position = [math.radians(a) for a in angles]
        else:
            return
        self.joint_pub.publish(msg)



    def joint_command_cb(self, msg):
        angles = list(msg.data)
        if len(angles) == 6:
            self.mc.send_angles(angles, self.speed)


    def coords_command_cb(self, msg):
        coords = list(msg.data)
        if len(coords) == 6:
            self.mc.send_coords(coords, self.speed, 0)


    def gripper_command_cb(self, msg):
        self.mc.set_gripper_value(msg.data, self.speed)



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


