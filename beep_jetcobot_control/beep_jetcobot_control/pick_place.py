import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32


import time

# 클래스 생성
class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        # 동작 정의
        self.speed = 30
        self.home_angles = [0, 0, 0, 0, 0, 0]
        self.pick_coords = [150.0, 0.0, 120.0, -180.0, 0.0, 90.0]
        self.middle_coords = [150.0, 60.0, 200.0, -180.0, 0.0, 90.0]
        self.place_coords = [0.0, 150.0, 120.0, -180.0, 0.0, 90.0]

        # 명령
        self.joint_pub = self.create_publisher(Float32MultiArray, '/joint_command', 10)
        self.coord_pub = self.create_publisher(Float32MultiArray, '/coord_command', 10)
        self.gripper_pub = self.create_publisher(Int32, '/gripper_command', 10)

        self.get_logger().info('pick_place_node 시작')
        self.run()


    def send_angles(self, angles):
        msg = Float32MultiArray()
        msg.data = angles
        self.joint_pub.publish(msg)

    def send_coords(self, coords):
        msg = Float32MultiArray()
        msg.data = coords
        self.coord_pub.publish(msg)

    def send_gripper(self, value):
        msg = Int32()
        msg.data = value
        self.gripper_pub.publish(msg)

    
    def go_home(self):
        self.get_logger().info('홈 이동')
        self.send_angles(self.home_angles)
        time.sleep(3)

    # gripper open
    # 숫자가 클수록 열고 작을수록 닫음
    def open_gripper(self):
        self.send_gripper(100)
        time.sleep(1)

    # gripper close
    def close_gripper(self):
        self.send_gripper(0)
        time.sleep(1)

    # 이동
    def move_to(self, coords, label=''):
        self.get_logger().info(f'{label} 이동 중')
        self.send_coords(coords)
        time.sleep(3)


    # 동작 전체 구현
    def run(self):
        self.get_logger().info('--- Pick and place start ---')

        self.go_home()
        self.open_gripper()

        self.move_to(self.pick_coords, 'pick')
        self.close_gripper()

        self.move_to(self.middle_coords, 'middle')

        # place
        self.move_to(self.place_coords, 'place')
        self.open_gripper()

        self.go_home()
        self.get_logger().info('--- 완료 ---')


def main(args=None):
    rp.init(args=args)
    node = PickPlaceNode()
    node.destroy_node()
    rp.shutdown()       


if __name__ == "__main__":
    main()
