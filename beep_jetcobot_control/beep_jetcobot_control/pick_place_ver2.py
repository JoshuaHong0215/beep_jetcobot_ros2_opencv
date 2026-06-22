import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import time

# 서보 파라미터
LAMBDA    = 0.3   # 이동 게인
THRESHOLD = 0.05  # 수렴 기준
MAX_ITER  = 50    # 최대 반복 횟수
MAX_DELTA = 15.0  # 한 스텝 최대 이동량 (mm)

# ready z(317.1) - 지면까지(300) + 10cm 여유 = 117.1mm
PICK_Z    = 117.1
LIFT_Z    = 317.1  # 픽 후 복귀 높이 (ready 높이)

# 카메라-TCP 오프셋 (카메라 정렬 후 TCP를 물체 위로 이동)
# 부호는 실제 방향 확인 후 조정 필요 (+/-60.0)
CAM_TCP_X = 100
CAM_TCP_Y = 20.0


class PickPlaceVer2Node(Node):
    def __init__(self):
        super().__init__('pick_place_ver2_node')

        self.home_angles  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ready_coords = [129.1, -62.8, 317.1, -162.04, -18.67, -42.35]
        self.speed        = 30

        self.marker_error = None
        self.ee_coords    = None

        self.joint_pub   = self.create_publisher(Float32MultiArray, '/joint_command',  10)
        self.coord_pub   = self.create_publisher(Float32MultiArray, '/coord_command',  10)
        self.servo_pub   = self.create_publisher(Float32MultiArray, '/coord_servo',    10)
        self.gripper_pub = self.create_publisher(Int32,             '/gripper_command', 10)

        self.create_subscription(Float32MultiArray, '/marker_error', self.marker_error_cb, 10)
        self.create_subscription(Float32MultiArray, '/ee_coords',    self.ee_coords_cb,    10)

        self.get_logger().info('pick_place_ver2_node 시작')
        self.run()

    def marker_error_cb(self, msg):
        self.marker_error = list(msg.data)

    def ee_coords_cb(self, msg):
        self.ee_coords = list(msg.data)

    def send_angles(self, angles):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.joint_pub.publish(msg)

    def send_coords(self, coords):
        msg = Float32MultiArray()
        msg.data = [float(c) for c in coords]
        self.coord_pub.publish(msg)

    def send_servo_coords(self, coords):
        msg = Float32MultiArray()
        msg.data = [float(c) for c in coords]
        self.servo_pub.publish(msg)

    def send_gripper(self, value):
        msg = Int32()
        msg.data = value
        self.gripper_pub.publish(msg)

    def get_fresh_error(self):
        """감지된 최신 오차 반환. 미감지 메시지는 버림."""
        self.marker_error = None
        start = time.time()
        while time.time() - start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.marker_error is not None:
                if self.marker_error[2] > 0.5:
                    return self.marker_error
                self.marker_error = None
        return None

    def go_home(self):
        self.get_logger().info('홈 이동')
        self.send_angles(self.home_angles)
        time.sleep(3)

    def go_ready(self):
        self.get_logger().info('레디포즈 이동')
        self.send_coords(self.ready_coords)
        time.sleep(4)

    def open_gripper(self):
        self.send_gripper(100)
        time.sleep(1)

    def visual_servo(self):
        """물체가 카메라 십자선 중앙에 올 때까지 XY 서보잉"""
        self.get_logger().info('시각 서보 시작')

        for i in range(MAX_ITER):
            error = self.get_fresh_error()
            if error is None:
                self.get_logger().warn(f'[{i}] 물체 미감지')
                continue

            e_x, e_y, _ = error
            self.get_logger().info(f'[{i}] e_x={e_x:.4f}  e_y={e_y:.4f}')

            if abs(e_x) < THRESHOLD and abs(e_y) < THRESHOLD:
                self.get_logger().info('수렴 완료')
                return True

            if self.ee_coords is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            cur_x, cur_y, cur_z = self.ee_coords[0], self.ee_coords[1], self.ee_coords[2]
            rx,    ry,    rz    = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

            # 축 매핑: Robot +X → e_y 증가, Robot +Y → e_x 증가 (카메라 ~90° 회전)
            delta_x = max(-MAX_DELTA, min(MAX_DELTA, -LAMBDA * e_y * 1000.0))
            delta_y = max(-MAX_DELTA, min(MAX_DELTA, -LAMBDA * e_x * 1000.0))

            self.get_logger().info(f'  → delta_x={delta_x:.1f}  delta_y={delta_y:.1f}')
            self.send_servo_coords([cur_x + delta_x, cur_y + delta_y, cur_z, rx, ry, rz])
            time.sleep(0.5)

        self.get_logger().warn('최대 반복 도달 — 서보 종료')
        return False

    def pick(self):
        """카메라-TCP 오프셋 보정 후 Z 하강하여 물체 잡기"""
        if self.ee_coords is None:
            self.get_logger().error('ee_coords 없음 — pick 중단')
            return False

        # 카메라 정렬 위치에서 TCP 기준으로 오프셋 보정
        x = self.ee_coords[0] + CAM_TCP_X
        y = self.ee_coords[1] + CAM_TCP_Y
        z = self.ee_coords[2]
        rx, ry, rz = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

        self.get_logger().info(f'TCP 오프셋 보정: ({self.ee_coords[0]:.1f}, {self.ee_coords[1]:.1f}) → ({x:.1f}, {y:.1f})')
        self.send_coords([x, y, z, rx, ry, rz])
        time.sleep(2)

        self.get_logger().info(f'pick 접근: z {z:.1f} → {PICK_Z}')
        self.send_coords([x, y, PICK_Z, rx, ry, rz])
        time.sleep(3)

        self.get_logger().info('그리퍼 닫기')
        self.send_gripper(0)
        time.sleep(1.5)

        self.get_logger().info(f'lift: z {PICK_Z} → {LIFT_Z}')
        self.send_coords([x, y, LIFT_Z, rx, ry, rz])
        time.sleep(3)

        return True

    def run(self):
        while rclpy.ok() and (
            self.coord_pub.get_subscription_count() == 0
            or self.gripper_pub.get_subscription_count() == 0
        ):
            self.get_logger().info('joint_control 대기...')
            time.sleep(0.2)

        time.sleep(0.5)
        self.open_gripper()
        self.go_ready()
        self.marker_error = None

        converged = self.visual_servo()
        if not converged:
            self.get_logger().error('서보 실패 — 홈으로 복귀')
            self.go_home()
            return

        self.pick()
        self.go_ready()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceVer2Node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
