"""
pick_place_action_server_ver.py

Step 1: 카메라와 객체 align만 한다.
- task_id 받음 → 해당 클래스를 yolo에 알림
- ready 자세로 이동
- visual_servo: 카메라 십자선에 객체가 오도록 robot XY 흔들기
- 끝. pick/descending/gripper 없음.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32

from beep_jetcobot_msgs.action import PickPlace


# ── visual servo 파라미터 (모두 픽셀 기준) ──────────────────────
LAMBDA         = 0.06   # mm / pixel
THRESHOLD      = 8.0    # pixel
MAX_DELTA      = 3.0    # mm / iter
MAX_ITER       = 80
CONVERGE_COUNT = 3
SERVO_SLEEP    = 0.8    # 초

# 축 매핑.
# SWAP_XY=True  → delta_x는 e_y로, delta_y는 e_x로 계산 (카메라 ~90° 회전 가정)
# SWAP_XY=False → delta_x는 e_x로, delta_y는 e_y로 (카메라가 robot 축과 같은 방향)
# AXIS_X_SIGN, AXIS_Y_SIGN → robot이 객체에서 멀어지면 부호 뒤집기.
SWAP_XY     = False
AXIS_X_SIGN = +1
AXIS_Y_SIGN = -1

# pick 파라미터
CAM_TCP_X = 122.0   # 카메라 정렬 후 TCP를 객체 위로 옮기는 base frame X 오프셋
CAM_TCP_Y = -20.0   # 동 base frame Y 오프셋
PICK_Z    = 130.1   # 그립 시 Z (mm)
LIFT_Z    = 317.1   # 그립 후 들어올릴 Z (mm)

TASK_CLASS = {'0': 0, '1': 1, '2': 2}
CLASS_NAME = {0: 'large_blue_box', 1: 'medium_red_box', 2: 'small_yellow_box'}


class PickPlaceActionServerVer(Node):
    def __init__(self):
        super().__init__('pick_place_action_server_ver')

        self.ready_coords = [153.5, -54.6, 265.9, -166.79, 1.59, -44.23]
        self.ready_angles = [0.43, 0.08, -50.53, -31.2, 10.45, -44.38]
        self.speed        = 30

        self.marker_error = None
        self.ee_coords    = None

        cb = ReentrantCallbackGroup()

        self.joint_pub        = self.create_publisher(Float32MultiArray, '/joint_command',   10)
        self.coord_pub        = self.create_publisher(Float32MultiArray, '/coord_command',   10)
        self.servo_pub        = self.create_publisher(Float32MultiArray, '/coord_servo',     10)
        self.gripper_pub      = self.create_publisher(Int32,             '/gripper_command', 10)
        self.target_class_pub = self.create_publisher(Int32,             '/target_class',    10)

        self.create_subscription(Float32MultiArray, '/marker_error', self.marker_error_cb, 10, callback_group=cb)
        self.create_subscription(Float32MultiArray, '/ee_coords',    self.ee_coords_cb,    10, callback_group=cb)

        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=cb,
        )

        self.get_logger().info('pick_place_action_server_ver 시작 (Step 1: 정렬만)')

    # ── ROS callbacks ──────────────────────────────────────────
    def marker_error_cb(self, msg):
        self.marker_error = list(msg.data)

    def ee_coords_cb(self, msg):
        self.ee_coords = list(msg.data)

    def goal_cb(self, goal_request):
        self.get_logger().info(f'Goal 수신: task_id={goal_request.task_id}')
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel 요청 수신')
        return CancelResponse.ACCEPT

    # ── publishers ─────────────────────────────────────────────
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

    def set_target_class(self, class_id):
        msg = Int32()
        msg.data = class_id
        self.target_class_pub.publish(msg)

    def send_gripper(self, value):
        msg = Int32()
        msg.data = int(value)
        self.gripper_pub.publish(msg)

    def publish_feedback(self, goal_handle, state, e_x=0.0, e_y=0.0, iteration=0):
        fb = PickPlace.Feedback()
        fb.state     = state
        fb.e_x       = float(e_x)
        fb.e_y       = float(e_y)
        fb.iteration = iteration
        goal_handle.publish_feedback(fb)

    # ── helpers ────────────────────────────────────────────────
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

    def go_ready(self):
        # joint 직접 명령 (IK 우회). 큰 이동에 mode=0 IK가 실패하던 문제 회피.
        self.send_angles(self.ready_angles)
        time.sleep(4)

    def open_gripper(self):
        self.send_gripper(100)
        time.sleep(1.5)

    def pick(self, goal_handle):
        if self.ee_coords is None:
            return False

        x = self.ee_coords[0] + CAM_TCP_X
        y = self.ee_coords[1] + CAM_TCP_Y
        z = self.ee_coords[2]
        rx, ry, rz = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

        self.publish_feedback(goal_handle, 'OFFSET_MOVE')
        self.get_logger().info(f'>>> OFFSET_MOVE: ({x:.1f}, {y:.1f}, {z:.1f})')
        self.send_coords([x, y, z, rx, ry, rz])
        time.sleep(2.5)

        self.publish_feedback(goal_handle, 'DESCENDING')
        self.get_logger().info(f'>>> DESCENDING: ({x:.1f}, {y:.1f}, {PICK_Z:.1f})')
        self.send_coords([x, y, PICK_Z, rx, ry, rz])
        time.sleep(3)

        self.publish_feedback(goal_handle, 'GRIPPING')
        self.send_gripper(0)
        time.sleep(1.5)

        self.publish_feedback(goal_handle, 'LIFTING')
        self.send_coords([x, y, LIFT_Z, rx, ry, rz])
        time.sleep(3)
        return True

    # ── visual servo ───────────────────────────────────────────
    def visual_servo(self, goal_handle):
        """카메라 십자선에 객체가 오도록 robot XY를 반복적으로 흔든다."""
        consec = 0
        for i in range(MAX_ITER):
            if goal_handle.is_cancel_requested:
                return False

            error = self.get_fresh_error()
            if error is None:
                self.publish_feedback(goal_handle, 'SEARCHING', iteration=i)
                self.get_logger().warn(f'[{i}] 객체 미감지')
                consec = 0
                continue

            e_x, e_y, _ = error

            if self.ee_coords is None:
                continue
            cur_x, cur_y, cur_z = self.ee_coords[0], self.ee_coords[1], self.ee_coords[2]
            rx, ry, rz         = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

            self.publish_feedback(goal_handle, 'SERVO', e_x, e_y, i)

            # 수렴 판정
            if abs(e_x) < THRESHOLD and abs(e_y) < THRESHOLD:
                consec += 1
                self.get_logger().info(f'[{i}] 수렴 근처 e=({e_x:+.1f},{e_y:+.1f}) consec={consec}')
                if consec >= CONVERGE_COUNT:
                    self.get_logger().info(f'★ 정렬 완료 @ ee=({cur_x:.1f},{cur_y:.1f},{cur_z:.1f})')
                    time.sleep(0.5)
                    return True
                time.sleep(SERVO_SLEEP)
                continue
            consec = 0

            # delta 계산 (픽셀 → mm)
            err_for_dx = e_y if SWAP_XY else e_x
            err_for_dy = e_x if SWAP_XY else e_y
            delta_x = AXIS_X_SIGN * LAMBDA * err_for_dx
            delta_y = AXIS_Y_SIGN * LAMBDA * err_for_dy
            delta_x = max(-MAX_DELTA, min(MAX_DELTA, delta_x))
            delta_y = max(-MAX_DELTA, min(MAX_DELTA, delta_y))

            tgt_x = cur_x + delta_x
            tgt_y = cur_y + delta_y

            self.get_logger().info(
                f'[{i}] e=({e_x:+6.1f},{e_y:+6.1f})px '
                f'cur=({cur_x:.1f},{cur_y:.1f}) '
                f'→ delta=({delta_x:+.2f},{delta_y:+.2f}) '
                f'tgt=({tgt_x:.1f},{tgt_y:.1f})'
            )

            self.send_servo_coords([tgt_x, tgt_y, cur_z, rx, ry, rz])
            time.sleep(SERVO_SLEEP)

        self.get_logger().warn('MAX_ITER 도달 — 수렴 실패')
        return False

    # ── execute ────────────────────────────────────────────────
    def execute_cb(self, goal_handle):
        result = PickPlace.Result()

        task_id  = goal_handle.request.task_id
        class_id = TASK_CLASS.get(task_id, -1)
        if class_id == -1:
            result.success = False
            result.message = f'잘못된 task_id: {task_id}'
            goal_handle.abort()
            return result

        self.get_logger().info(f'타겟: {CLASS_NAME[class_id]} (id={class_id})')
        self.set_target_class(class_id)

        self.open_gripper()
        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()

        self.marker_error = None
        converged = self.visual_servo(goal_handle)

        if not converged:
            self.publish_feedback(goal_handle, 'SERVO_FAILED')
            self.set_target_class(-1)
            result.success = False
            result.message = '시각 서보 수렴 실패'
            goal_handle.abort()
            return result

        success = self.pick(goal_handle)

        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()
        self.set_target_class(-1)

        if success:
            result.success = True
            result.message = f'{CLASS_NAME[class_id]} 피킹 완료'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = 'pick 실패'
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceActionServerVer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
