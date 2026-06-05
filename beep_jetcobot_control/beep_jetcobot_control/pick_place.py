import rclpy as rp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32


import time

# 클래스 생성
class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        # 동작 정의
        self.speed = 30
        self.home_angles   = [0, 0, 0, 0, 0, 0]
        self.ready_coords  = [129.1, -62.8, 317.1, -162.04, -18.67, -42.35]
        self.pick_coords   = None
        self.middle_coords = [150.0, 60.0, 200.0, -180.0, 0.0, 90.0]
        self.place_coords  = [0.0, 150.0, 120.0, -180.0, 0.0, 90.0]

        self.PICK_Z     = 105.8   # 실측: 그리퍼가 물체에 닿는 TCP z (mm)
        self.APPROACH_Z = 200.0   # pick 전 접근 높이 (mm)

        # 명령
        self.joint_pub = self.create_publisher(Float32MultiArray, '/joint_command', 10)
        self.coord_pub = self.create_publisher(Float32MultiArray, '/coord_command', 10)
        self.gripper_pub = self.create_publisher(Int32, '/gripper_command', 10)

        # /marker_coord topic을 subscribe → 마커 좌표가 들어오면 marker_coord_cb 콜백 실행
        self.create_subscription(Float32MultiArray, '/marker_coord', self.marker_coord_cb, 10)

        self.get_logger().info('pick_place_node 시작 — 마커 감지 대기 중...')
        self.run()


    def marker_coord_cb(self, msg):
        # /marker_coord topic에서 6개 값(x,y,z,rx,ry,rz)을 받아 pick_coords에 저장
        self.pick_coords = list(msg.data)
        self.get_logger().info(f'마커 좌표 수신: {self.pick_coords}')

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

    def go_ready(self):
        self.get_logger().info('레디포즈 이동')
        self.send_coords(self.ready_coords)
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
        time.sleep(1.0)  # DDS publisher 연결 대기
        self.get_logger().info('--- Pick and place start ---')

        self.open_gripper()

        # 레디포즈로 이동 후 마커 감지 대기
        self.go_ready()

        # 레디포즈 도착 후 이전에 수신된 좌표 초기화 → 새로 감지된 것만 사용
        self.pick_coords = None
        self.get_logger().info('레디포즈 대기 중 — 마커 감지 기다리는 중...')
        while self.pick_coords is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'마커 감지 완료 → pick 좌표: {self.pick_coords}')

        # x, y만 마커에서 사용, z는 실측값 고정
        rx, ry, rz = self.pick_coords[3], self.pick_coords[4], self.pick_coords[5]

        approach_coords = [self.pick_coords[0], self.pick_coords[1], self.APPROACH_Z, rx, ry, rz]
        self.move_to(approach_coords, 'approach')

        pick_coords = [self.pick_coords[0], self.pick_coords[1], self.PICK_Z, rx, ry, rz]
        self.move_to(pick_coords, 'pick')
        self.close_gripper()

        self.move_to(self.middle_coords, 'middle')

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
