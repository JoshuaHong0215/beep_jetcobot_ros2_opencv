import rclpy as rp
from rclpy.node import Node


from pymycobot.mycobot280 import MyCobot280
import time

# 클래스 생성
class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.speed = 30

        
        self.home_angles = [0, 0, 0, 0, 0, 0]
        self.pick_coords = [150.0, 0.0, 100.0, -180.0, 0.0, 90.0]
        self.place_coords = [0.0, 150.0, 100.0, -180.0, 0.0, 90.0]

        self.get_logger().info('pick_place_node 시작')
        self.run()

    # home pose 정의
    def go_home(self):
        self.get_logger().info('홈 이동')
        self.mc.send_angles(self.home_angles, self.speed)
        time.sleep(3)

    # gripper open
    # 숫자가 클수록 열고 작을수록 닫음
    def open_gripper(self):
        self.mc.set_gripper_value(100, self.speed)
        time.sleep(1)

    # gripper close
    def close_gripper(self):
        self.mc.set_gripper_value(0, self.speed)
        time.sleep(1)

    # 이동
    def move_to(self, coords, label=''):
        self.get_logger().info(f'{label} 이동 중')
        self.mc.send_coords(coords, self.speed, 0)
        time.sleep(3)


    # 동작 전체 구현
    def run(self):
        self.get_logger().info('--- Pick and place start ---')

        self.go_home()
        self.open_gripper()

        self.move_to(self.pick_coords, 'pick')
        self.close_gripper()

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
