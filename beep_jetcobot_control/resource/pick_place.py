import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import time


# 서보 파라미터
LAMBDA     = 0.7    # 2~3스텝 수렴 목표 (캘리브레이션 지점 이탈 최소화)
THRESHOLD  = 0.05   # 수렴 판단 기준 (~50px)
MAX_ITER   = 20
MAX_DELTA  = 40.0   # 한 번에 최대 이동량 (mm)
CAM_Z_OFFSET = 30.0

# 픽 파라미터
PICK_Z     = 105.8
STEP_Z     = 20.0   # 서보 하강 시 한 스텝 거리 (mm)

# 카메라 → TCP 오프셋 (서보 수렴 후 TCP를 마커 위로 이동시키는 보정값, 단위 mm)
# 측정 방법: 서보 수렴 위치에서 TCP가 마커 위에 올 때까지 직접 이동 → 차이값 입력
CAM_TO_TCP_X = 10
CAM_TO_TCP_Y = 10

# 축 방향: 둘 다 +1 (테스트로 확인)
SIGN_X = 1
SIGN_Y = 1


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        self.speed = 30
        self.home_angles  = [0, 0, 0, 0, 0, 0]
        self.ready_coords = [129.1, -62.8, 317.1, -162.04, -18.67, -42.35]
        self.middle_coords = [150.0, 60.0, 200.0, -180.0, 0.0, 90.0]
        self.place_coords  = [0.0, 150.0, 120.0, -180.0, 0.0, 90.0]

        self.marker_error = None  # [e_x, e_y, detected]
        self.ee_coords    = None  # 현재 TCP 좌표

        self.joint_pub   = self.create_publisher(Float32MultiArray, '/joint_command', 10)
        self.coord_pub   = self.create_publisher(Float32MultiArray, '/coord_command', 10)
        self.servo_pub   = self.create_publisher(Float32MultiArray, '/coord_servo',   10)
        self.gripper_pub = self.create_publisher(Int32, '/gripper_command', 10)

        self.create_subscription(Float32MultiArray, '/marker_error',  self.marker_error_cb, 10)
        self.create_subscription(Float32MultiArray, '/ee_coords',     self.ee_coords_cb,    10)

        self.get_logger().info('pick_place_node 시작')
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
        """시각 서보 전용 — mode=1(직선 보간)으로 자세 유지"""
        msg = Float32MultiArray()
        msg.data = [float(c) for c in coords]
        self.servo_pub.publish(msg)

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
        time.sleep(4)

    def open_gripper(self):
        self.send_gripper(100)
        time.sleep(1)

    def close_gripper(self):
        self.send_gripper(0)
        time.sleep(1)

    def move_to(self, coords, label=''):
        self.get_logger().info(f'{label} 이동')
        self.send_coords(coords)
        time.sleep(3)

    def get_fresh_error(self):
        """마커가 실제로 감지된 최신 오차를 반환. sleep 중 큐에 쌓인 미감지 메시지는 버림."""
        self.marker_error = None
        timeout = 2.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.marker_error is not None:
                if self.marker_error[2] > 0.5:   # detected=1.0인 메시지만 반환
                    return self.marker_error
                self.marker_error = None          # 미감지 메시지는 버리고 계속 대기
        return None

    def visual_servo(self):
        """
        마커가 이미지 중앙에 올 때까지 로봇을 조금씩 이동.
        e_x, e_y가 THRESHOLD 이하가 되면 수렴 완료.
        """
        self.get_logger().info('시각 서보 시작')

        for i in range(MAX_ITER):
            error = self.get_fresh_error()

            if error is None:
                self.get_logger().warn(f'[{i}] 마커 미감지 (2초 타임아웃)')
                continue

            e_x, e_y, _ = error

            self.get_logger().info(f'[{i}] e_x={e_x:.4f} e_y={e_y:.4f}')

            # 수렴 확인
            if abs(e_x) < THRESHOLD and abs(e_y) < THRESHOLD:
                self.get_logger().info('수렴 완료')
                return True

            # 현재 TCP 좌표 읽기
            if self.ee_coords is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            cur_x, cur_y, cur_z = self.ee_coords[0], self.ee_coords[1], self.ee_coords[2]
            rx, ry, rz = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

            # 카메라 높이 (미터)
            Z = (cur_z + CAM_Z_OFFSET) / 1000.0

            # 실측: Robot +X → e_y 증가, Robot +Y → e_x 증가 (카메라 ~90° 회전)
            delta_x = -LAMBDA * Z * e_y * 1000.0
            delta_y = -LAMBDA * Z * e_x * 1000.0

            delta_x = max(-MAX_DELTA, min(MAX_DELTA, delta_x))
            delta_y = max(-MAX_DELTA, min(MAX_DELTA, delta_y))

            new_x = cur_x + delta_x
            new_y = cur_y + delta_y

            self.get_logger().info(f'  → 이동 delta_x={delta_x:.1f} delta_y={delta_y:.1f}')
            self.send_servo_coords([new_x, new_y, cur_z, rx, ry, rz])
            time.sleep(1.5)

        self.get_logger().warn('최대 반복 도달 — 서보 종료')
        return False

    def servo_descend(self):
        """마커를 추적하면서 PICK_Z까지 단계적 하강"""
        self.get_logger().info('서보 하강 시작')

        for i in range(MAX_ITER):
            if self.ee_coords is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            cur_x = self.ee_coords[0]
            cur_y = self.ee_coords[1]
            cur_z = self.ee_coords[2]
            rx, ry, rz = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

            if cur_z <= PICK_Z + 2.0:
                self.get_logger().info('PICK_Z 도달')
                return True

            error = self.get_fresh_error()
            if error is None:
                self.get_logger().warn(f'[{i}] 하강 중 마커 미감지')
                continue

            e_x, e_y, _ = error
            Z = (cur_z + CAM_Z_OFFSET) / 1000.0
            delta_x = -LAMBDA * Z * e_y * 1000.0
            delta_y = -LAMBDA * Z * e_x * 1000.0
            delta_x = max(-MAX_DELTA, min(MAX_DELTA, delta_x))
            delta_y = max(-MAX_DELTA, min(MAX_DELTA, delta_y))

            new_x = cur_x + delta_x
            new_y = cur_y + delta_y
            new_z = max(PICK_Z, cur_z - STEP_Z)

            self.get_logger().info(
                f'[{i}] z={cur_z:.1f} e_x={e_x:.4f} e_y={e_y:.4f}'
                f' → dxy({delta_x:.1f},{delta_y:.1f}) z→{new_z:.1f}'
            )
            self.send_servo_coords([new_x, new_y, new_z, rx, ry, rz])
            time.sleep(1.5)

        self.get_logger().warn('서보 하강 최대 반복 도달')
        return False

    def run(self):
        time.sleep(1.0)
        self.get_logger().info('--- Pick and place start ---')

        self.open_gripper()
        self.go_ready()

        # 레디포즈 도착 후 오차 초기화
        self.marker_error = None

        # 시각 서보: 마커를 이미지 중앙으로
        converged = self.visual_servo()

        if not converged:
            self.get_logger().error('서보 실패 — 중단')
            self.go_home()
            return

        # 수렴 후 현재 위치에서 CAM_TO_TCP 보정 → Z 직선 하강 → pick
        if self.ee_coords is None:
            self.get_logger().error('TCP 좌표 없음 — 중단')
            return

        cur_x = self.ee_coords[0]
        cur_y = self.ee_coords[1]
        rx, ry, rz = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

        pick_x = cur_x + CAM_TO_TCP_X
        pick_y = cur_y + CAM_TO_TCP_Y

        self.get_logger().info('pick 하강')
        self.send_servo_coords([pick_x, pick_y, PICK_Z, rx, ry, rz])
        time.sleep(3)
        self.close_gripper()

        self.move_to(self.middle_coords, 'middle')
        self.move_to(self.place_coords, 'place')
        self.open_gripper()

        self.go_home()
        self.get_logger().info('--- 완료 ---')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
