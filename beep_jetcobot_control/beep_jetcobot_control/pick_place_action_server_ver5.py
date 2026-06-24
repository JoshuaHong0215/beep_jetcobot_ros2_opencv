# ver5: mode 분기
# - mode 0: work → storage (기존)
# - mode 1: storage → loading (J1 -90도 회전)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import os
import math
import time

from beep_jetcobot_msgs.action import PickPlace

# visual servo 파라미터
LAMBDA    = 0.3
THRESHOLD = 16.0
MAX_ITER  = 50
MAX_DELTA = 15.0

LIFT_Z = 317.1     # 공통 ready 높이

# mode 0: work → storage
# CAM_TCP: 카메라 정렬 후 그리퍼를 박스 위로 이동할 오프셋 (base 프레임)
CAM_TCP_X_WORK       = 120
CAM_TCP_Y_WORK       = 5.0
PICK_Z_WORK          = 120.1
Z_MIN_WORK           = 100.0     # work 영역 안전 클램프 (지면 기준)
PLACE_ANGLES_STORAGE = [-84.99, -48.86, -23.81, -8.87, -1.05, -36.91]

# mode 1: storage → loading (J1을 -90° 돌린 자세 기준)
# work 오프셋(120, 5)을 -90° 회전한 값 ≈ (5, -120). 실측 후 조정 권장.
CAM_TCP_X_STORAGE    = 10
CAM_TCP_Y_STORAGE    = -100
PICK_Z_STORAGE       = 180.0                                            # storage 표면 60mm + 그리퍼 길이 + 여유
Z_MIN_STORAGE        = 180.0                                            # storage 영역 안전 클램프

# loading zone = mode 0의 작업 영역. ready 위치 XY에 work z 높이로 떨어뜨림.
LOADING_X            = 129.1
LOADING_Y            = -62.8
LOADING_Z            = 120.1
LOADING_RX           = -162.04
LOADING_RY           = -18.67
LOADING_RZ           = -42.35
J1_OFFSET_STORAGE    = -90.0                                            # 베이스 기준 우측 90도. 반대면 +90.0
PLACE_ANGLES_LOADING = [-84.99, -48.86, -23.81, -8.87, -1.05, -36.91]   # TODO: 측정

TASK_CLASS = {'0': 0, '1': 1, '2': 2}
CLASS_NAME = {0: 'large_blue_box', 1: 'medium_red_box', 2: 'small_yellow_box'}

MODE_WORK_TO_STORAGE    = 0
MODE_STORAGE_TO_LOADING = 1


class PickPlaceActionServerVer5(Node):
    def __init__(self):
        super().__init__('pick_place_action_server')

        self.home_angles  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ready_coords = [129.1, -62.8, 317.1, -162.04, -18.67, -42.35]
        self.speed        = 30

        config_path = os.path.join(
            get_package_share_directory('beep_jetcobot_control'),
            'config', 'camera_cali.yaml'
        )
        with open(config_path, 'r') as f:
            calib = yaml.safe_load(f)
        K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        self.cx_pp = float(K[0, 2])
        self.cy_pp = float(K[1, 2])
        self.get_logger().info(f'십자선 = ({self.cx_pp:.1f}, {self.cy_pp:.1f})')

        self.marker_error   = None
        self.ee_coords      = None
        self.current_angles = None
        self.target_class   = None
        self.z_min          = Z_MIN_WORK   # 현재 모드별 z 하한 (execute_cb에서 설정)

        cb = ReentrantCallbackGroup()

        self.joint_pub   = self.create_publisher(Float32MultiArray, '/joint_command',   10)
        self.coord_pub   = self.create_publisher(Float32MultiArray, '/coord_command',   10)
        self.servo_pub   = self.create_publisher(Float32MultiArray, '/coord_servo',     10)
        self.gripper_pub = self.create_publisher(Int32,             '/gripper_command', 10)

        self.create_subscription(Float32MultiArray, '/detection',    self.detection_cb,   10, callback_group=cb)
        self.create_subscription(Float32MultiArray, '/ee_coords',    self.ee_coords_cb,   10, callback_group=cb)
        self.create_subscription(JointState,        '/joint_states', self.joint_state_cb, 10, callback_group=cb)

        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=cb,
        )

        self.get_logger().info('pick_place_action_server_ver5 시작')

    def detection_cb(self, msg):
        if len(msg.data) < 6:
            return
        class_id, cx, cy, _, _, _ = msg.data[:6]
        if self.target_class is not None and int(class_id) != self.target_class:
            return
        e_x = cx - self.cx_pp
        e_y = cy - self.cy_pp
        self.marker_error = [e_x, e_y, 1.0]

    def ee_coords_cb(self, msg):
        self.ee_coords = list(msg.data)

    def joint_state_cb(self, msg):
        if len(msg.position) == 6:
            self.current_angles = [math.degrees(p) for p in msg.position]

    def goal_cb(self, goal_request):
        self.get_logger().info(
            f'Goal 수신: task_id={goal_request.task_id} mode={goal_request.mode}'
        )
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel 요청 수신')
        return CancelResponse.ACCEPT

    # ── publishers ──────────────────────────────────────────────
    def send_angles(self, angles):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.joint_pub.publish(msg)

    def send_coords(self, coords):
        coords = list(coords)
        if coords[2] < self.z_min:
            self.get_logger().warn(f'z={coords[2]:.1f} < z_min={self.z_min} — 클램프')
            coords[2] = self.z_min
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

    # ── helpers ──────────────────────────────────────────────────
    def get_fresh_error(self):
        self.marker_error = None
        start = time.time()
        while time.time() - start < 2.0:
            time.sleep(0.05)
            if self.marker_error is not None:
                if self.marker_error[2] > 0.5:
                    return self.marker_error
                self.marker_error = None
        return None

    def publish_feedback(self, goal_handle, state, e_x=0.0, e_y=0.0, iteration=0):
        fb = PickPlace.Feedback()
        fb.state     = state
        fb.e_x       = float(e_x)
        fb.e_y       = float(e_y)
        fb.iteration = iteration
        goal_handle.publish_feedback(fb)

    # ── motion ───────────────────────────────────────────────────
    def go_home(self):
        self.send_angles(self.home_angles)
        time.sleep(5)

    def go_ready(self):
        self.send_coords(self.ready_coords)
        time.sleep(4)

    def open_gripper(self):
        self.send_gripper(100)
        time.sleep(2)

    def rotate_j1_from_ready(self, j1_offset_deg):
        """work ready 자세에서 J1만 offset만큼 추가 회전."""
        if abs(j1_offset_deg) < 1e-3:
            return True
        if self.current_angles is None:
            self.get_logger().error('current_angles 없음 — 회전 불가')
            return False
        target = list(self.current_angles)
        target[0] += j1_offset_deg
        self.get_logger().info(f'>>> J1 회전: {self.current_angles[0]:.1f} → {target[0]:.1f}')
        self.send_angles(target)
        time.sleep(4)
        return True

    def visual_servo(self, goal_handle, mode):
        init_coords = self.ee_coords
        if init_coords is None:
            return False
        ready_rx, ready_ry, ready_rz = init_coords[3], init_coords[4], init_coords[5]
        self.get_logger().info(f'>>> servo 시작 자세각 고정: rpy=({ready_rx:.1f}, {ready_ry:.1f}, {ready_rz:.1f})')

        for i in range(MAX_ITER):
            if goal_handle.is_cancel_requested:
                return False

            error = self.get_fresh_error()
            if error is None:
                self.publish_feedback(goal_handle, 'SEARCHING', iteration=i)
                continue

            e_x, e_y, _ = error
            self.publish_feedback(goal_handle, 'SERVO', e_x, e_y, i)
            self.get_logger().info(f'[{i}] e_x={e_x:.4f}  e_y={e_y:.4f}')

            if abs(e_x) < THRESHOLD and abs(e_y) < THRESHOLD:
                self.get_logger().info('교차 완료')
                return True

            coords = self.ee_coords
            if coords is None:
                continue

            cur_x, cur_y, cur_z = coords[0], coords[1], coords[2]

            # mode별 축 매핑
            if mode == MODE_WORK_TO_STORAGE:
                # work: 카메라 ~90도 회전. Robot +X ↔ -e_y, Robot +Y ↔ -e_x
                delta_x = -LAMBDA * e_y
                delta_y = -LAMBDA * e_x
            else:  # MODE_STORAGE_TO_LOADING
                # storage: work에서 J1 -90도 추가 회전한 상태.
                # Robot +X ↔ -e_x, Robot +Y ↔ +e_y
                delta_x = -LAMBDA * e_x
                delta_y = +LAMBDA * e_y

            delta_x = max(-MAX_DELTA, min(MAX_DELTA, delta_x))
            delta_y = max(-MAX_DELTA, min(MAX_DELTA, delta_y))

            self.get_logger().info(
                f'  → delta=({delta_x:+.2f}, {delta_y:+.2f}) mm  cur=({cur_x:.1f}, {cur_y:.1f})'
            )
            self.send_coords([cur_x + delta_x, cur_y + delta_y, cur_z, ready_rx, ready_ry, ready_rz])
            time.sleep(0.5)

        return False

    def pick(self, goal_handle, pick_z, cam_tcp_x, cam_tcp_y):
        coords = self.ee_coords
        if coords is None:
            return False

        x  = coords[0] + cam_tcp_x
        y  = coords[1] + cam_tcp_y
        z  = coords[2]
        rx, ry, rz = coords[3], coords[4], coords[5]

        self.publish_feedback(goal_handle, 'TCP_ALIGN')
        self.get_logger().info(f'>>> TCP_ALIGN: ({x:.1f}, {y:.1f}, {z:.1f})')
        self.send_coords([x, y, z, rx, ry, rz])
        time.sleep(3)

        self.publish_feedback(goal_handle, 'DESCEND')
        self.get_logger().info(f'>>> DESCEND: ({x:.1f}, {y:.1f}, {pick_z:.1f})')
        self.send_coords([x, y, pick_z, rx, ry, rz])
        time.sleep(3)

        self.publish_feedback(goal_handle, 'GRIP')
        self.send_gripper(0)
        time.sleep(1.5)

        self.publish_feedback(goal_handle, 'LIFT')
        self.send_coords([x, y, LIFT_Z, rx, ry, rz])
        time.sleep(3)

        return True

    def place_at_angles(self, goal_handle, place_angles):
        self.publish_feedback(goal_handle, 'PLACE_MOVE')
        self.get_logger().info(f'>>> PLACE: angles={place_angles}')
        self.send_angles(place_angles)
        time.sleep(5)

        self.publish_feedback(goal_handle, 'RELEASE')
        self.send_gripper(100)
        time.sleep(1.5)

        return True

    def place_at_coords(self, goal_handle, x, y, descend_z, rx, ry, rz):
        # 1) 목표 위 LIFT_Z 높이로 이동
        self.publish_feedback(goal_handle, 'PLACE_MOVE')
        self.get_logger().info(f'>>> PLACE_MOVE: ({x:.1f}, {y:.1f}, {LIFT_Z:.1f})')
        self.send_coords([x, y, LIFT_Z, rx, ry, rz])
        time.sleep(4)

        # 2) descend
        self.publish_feedback(goal_handle, 'DESCEND')
        self.get_logger().info(f'>>> DESCEND: ({x:.1f}, {y:.1f}, {descend_z:.1f})')
        self.send_coords([x, y, descend_z, rx, ry, rz])
        time.sleep(3)

        # 3) release
        self.publish_feedback(goal_handle, 'RELEASE')
        self.send_gripper(100)
        time.sleep(1.5)

        # 4) lift
        self.publish_feedback(goal_handle, 'LIFT')
        self.send_coords([x, y, LIFT_Z, rx, ry, rz])
        time.sleep(3)

        return True

    # ── execute ──────────────────────────────────────────────────
    def execute_cb(self, goal_handle):
        self.get_logger().info('태스크 실행 시작')
        result = PickPlace.Result()

        task_id = goal_handle.request.task_id
        mode    = int(goal_handle.request.mode)

        # 검증
        class_id = TASK_CLASS.get(task_id, -1)
        if class_id == -1:
            self.get_logger().error(f'유효하지 않은 task_id: {task_id}')
            result.success = False
            result.message = f'유효하지 않은 task_id: {task_id} (0=파란, 1=빨간, 2=노란)'
            goal_handle.abort()
            return result

        if mode not in (MODE_WORK_TO_STORAGE, MODE_STORAGE_TO_LOADING):
            self.get_logger().error(f'유효하지 않은 mode: {mode}')
            result.success = False
            result.message = f'유효하지 않은 mode: {mode} (0=work→storage, 1=storage→loading)'
            goal_handle.abort()
            return result

        # mode별 파라미터
        if mode == MODE_WORK_TO_STORAGE:
            pick_z       = PICK_Z_WORK
            cam_tcp_x    = CAM_TCP_X_WORK
            cam_tcp_y    = CAM_TCP_Y_WORK
            j1_offset    = 0.0
            self.z_min   = Z_MIN_WORK
            place_angles = PLACE_ANGLES_STORAGE
        else:
            pick_z       = PICK_Z_STORAGE
            cam_tcp_x    = CAM_TCP_X_STORAGE
            cam_tcp_y    = CAM_TCP_Y_STORAGE
            j1_offset    = J1_OFFSET_STORAGE
            self.z_min   = Z_MIN_STORAGE
            place_angles = PLACE_ANGLES_LOADING

        class_name = CLASS_NAME.get(class_id, '알 수 없음')
        self.get_logger().info(
            f'타겟: {class_name}(id={class_id}) | mode={mode} | '
            f'pick_z={pick_z} | cam_tcp=({cam_tcp_x},{cam_tcp_y}) | '
            f'j1_offset={j1_offset} | z_min={self.z_min}'
        )
        self.target_class = class_id

        self.open_gripper()
        self.go_home()

        # 1) work ready
        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()

        # 2) mode 1이면 J1 -90도 회전
        if not self.rotate_j1_from_ready(j1_offset):
            self.target_class = None
            result.success = False
            result.message = 'J1 회전 실패'
            goal_handle.abort()
            return result

        # 3) visual servo
        self.marker_error = None
        converged = self.visual_servo(goal_handle, mode)

        if not converged:
            self.publish_feedback(goal_handle, 'SERVO_FAILED')
            self.go_home()
            self.target_class = None
            result.success = False
            result.message = '시각 서보 수렴 실패'
            goal_handle.abort()
            return result

        # 4) pick
        time.sleep(0.5)
        success = self.pick(goal_handle, pick_z, cam_tcp_x, cam_tcp_y)

        # 5) place
        if success:
            if mode == MODE_WORK_TO_STORAGE:
                self.place_at_angles(goal_handle, place_angles)
            else:
                # mode 1: 살짝 들어올린 뒤 work ready 거쳐서 loading zone에 떨어뜨림
                c = self.ee_coords
                self.send_coords([c[0], c[1], c[2] + 70, c[3], c[4], c[5]])
                time.sleep(2)
                self.z_min = Z_MIN_WORK
                self.go_ready()
                self.place_at_coords(
                    goal_handle,
                    LOADING_X, LOADING_Y, LOADING_Z,
                    LOADING_RX, LOADING_RY, LOADING_RZ,
                )

        # 6) work ready 복귀
        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()

        self.target_class = None

        if success:
            result.success = True
            result.message = f'{class_name} mode={mode} 완료'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = 'ee_coords 없음'
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceActionServerVer5()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
