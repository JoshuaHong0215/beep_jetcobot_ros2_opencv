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
            Float32MultiArray,
            '/coord_servo',
            self.servo_command_cb,
            10
            )

        self.create_subscription(
            Int32, 
            '/gripper_command', 
            self.gripper_command_cb, 
            10
            )

        self.create_subscription(
            Float32MultiArray,
            '/single_joint_command',
            self.single_joint_cb,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/joint_limit',
            self.joint_limit_cb,
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states', 
            10
            )

        self.ee_pub = self.create_publisher(
            Float32MultiArray,
            '/ee_coords',
            10
        )

        self.ee_angles_pub = self.create_publisher(
            Float32MultiArray,
            '/ee_angles',
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

        angles_msg = Float32MultiArray()
        angles_msg.data = [float(a) for a in angles]
        self.ee_angles_pub.publish(angles_msg)


        coords = self.mc.get_coords()
        if coords and len(coords) == 6:
            coord_msg = Float32MultiArray()
            coord_msg.data = [float(v) for v in coords]
            self.ee_pub.publish(coord_msg)



    def joint_command_cb(self, msg):
        angles = list(msg.data)
        if len(angles) == 6:
            self.mc.send_angles(angles, self.speed)


    def coords_command_cb(self, msg):
        coords = list(msg.data)
        if len(coords) == 6:
            self.mc.send_coords(coords, self.speed, 0)

    def servo_command_cb(self, msg):
        coords = list(msg.data)
        self.get_logger().info(f'>>> servo_cb 진입: coords={[round(c,1) for c in coords]}')
        if len(coords) == 6:
            ret = self.mc.send_coords(coords, self.speed, 1)
            self.get_logger().info(f'>>> mc.send_coords mode=1 반환: {ret}')

    def single_joint_cb(self, msg):
        if len(msg.data) == 2:
            joint_id = int(msg.data[0])
            angle = float(msg.data[1])
            self.mc.send_angle(joint_id, angle, self.speed)

    def joint_limit_cb(self, msg):
        # msg.data = [joint_id, min_deg, max_deg]
        if len(msg.data) != 3:
            return
        jid = int(msg.data[0])
        mn  = float(msg.data[1])
        mx  = float(msg.data[2])
        try:
            self.mc.set_joint_min(jid, mn)
            self.mc.set_joint_max(jid, mx)
            self.get_logger().info(f'>>> J{jid} 제한 설정: [{mn:.1f}, {mx:.1f}]')
        except Exception as e:
            self.get_logger().warn(f'joint_limit 적용 실패: {e}')


    def gripper_command_cb(self, msg):
        self.mc.set_gripper_value(msg.data, self.speed)

    def destroy_node(self):
        self.mc.stop()
        super().destroy_node()



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


