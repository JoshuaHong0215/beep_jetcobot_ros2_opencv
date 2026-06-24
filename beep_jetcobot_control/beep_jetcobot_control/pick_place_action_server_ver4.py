# J1의 관절 제한


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

# visual servo 파라미터 (ver3 동일)
LAMBDA    = 0.3    # mm/pixel
THRESHOLD = 16.0   # pixel
MAX_ITER  = 50
MAX_DELTA = 15.0   # mm/iter

PICK_Z = 120.1
LIFT_Z = 317.1

CAM_TCP_X = 120
CAM_TCP_Y = 5.0

PLACE_ANGLES = [-84.99, -48.86, -23.81, -8.87, -1.05, -36.91]

# J1 제한: 원점(0°) 기준 좌우 30도. picking 동안 IK 다중해 영역 차단용.
J1_LIMIT_MIN     = -30.0
J1_LIMIT_MAX     =  30.0
# 제한 해제 시 펌웨어 기본값
J1_DEFAULT_MIN   = -170.0
J1_DEFAULT_MAX   =  170.0

TASK_CLASS = {'0': 0, '1': 1, '2': 2}
CLASS_NAME = {0: 'large_blue_box', 1: 'medium_red_box', 2: 'small_yellow_box'}


class PickPlaceActionServerVer4(Node):
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

        cb = ReentrantCallbackGroup()

        self.joint_pub       = self.create_publisher(Float32MultiArray, '/joint_command',  10)
        self.coord_pub       = self.create_publisher(Float32MultiArray, '/coord_command',  10)
        self.servo_pub       = self.create_publisher(Float32MultiArray, '/coord_servo',    10)
        self.gripper_pub     = self.create_publisher(Int32,             '/gripper_command',10)
        # /joint_limit: [joint_id, min_deg, max_deg]
        self.joint_limit_pub = self.create_publisher(Float32MultiArray, '/joint_limit',    10)

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

        self.get_logger().info('pick_place_action_server_ver4 시작')

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
        self.get_logger().info(f'Goal 수신: task_id={goal_request.task_id}')
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

    def set_joint_limit(self, joint_id, mn, mx):
        msg = Float32MultiArray()
        msg.data = [float(joint_id), float(mn), float(mx)]
        self.joint_limit_pub.publish(msg)
        self.get_logger().info(f'>>> joint_limit J{joint_id}: [{mn:.1f}, {mx:.1f}]')
        time.sleep(0.3)

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

    def visual_servo(self, goal_handle):
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

            delta_x = max(-MAX_DELTA, min(MAX_DELTA, -LAMBDA * e_y))
            delta_y = max(-MAX_DELTA, min(MAX_DELTA, -LAMBDA * e_x))

            self.get_logger().info(
                f'  → delta=({delta_x:+.2f}, {delta_y:+.2f}) mm  cur=({cur_x:.1f}, {cur_y:.1f})'
            )
            self.send_coords([cur_x + delta_x, cur_y + delta_y, cur_z, ready_rx, ready_ry, ready_rz])
            time.sleep(0.5)

        return False

    def pick(self, goal_handle):
        coords = self.ee_coords
        if coords is None:
            return False

        x  = coords[0] + CAM_TCP_X
        y  = coords[1] + CAM_TCP_Y
        z  = coords[2]
        rx, ry, rz = coords[3], coords[4], coords[5]

        self.publish_feedback(goal_handle, 'TCP_ALIGN')
        self.get_logger().info(f'>>> TCP_ALIGN: ({x:.1f}, {y:.1f}, {z:.1f})')
        self.send_coords([x, y, z, rx, ry, rz])
        time.sleep(3)

        self.publish_feedback(goal_handle, 'DESCEND')
        self.get_logger().info(f'>>> DESCEND: ({x:.1f}, {y:.1f}, {PICK_Z:.1f})')
        self.send_coords([x, y, PICK_Z, rx, ry, rz])
        time.sleep(3)

        self.publish_feedback(goal_handle, 'GRIP')
        self.send_gripper(0)
        time.sleep(1.5)

        self.publish_feedback(goal_handle, 'LIFT')
        self.send_coords([x, y, LIFT_Z, rx, ry, rz])
        time.sleep(3)

        return True

    def place(self, goal_handle):
        self.publish_feedback(goal_handle, 'PLACE_MOVE')
        self.get_logger().info(f'>>> PLACE: angles={PLACE_ANGLES}')
        self.send_angles(PLACE_ANGLES)
        time.sleep(5)

        self.publish_feedback(goal_handle, 'RELEASE')
        self.send_gripper(100)
        time.sleep(1.5)

        return True

    # ── execute ──────────────────────────────────────────────────
    def execute_cb(self, goal_handle):
        self.get_logger().info('태스크 실행 시작')
        result = PickPlace.Result()

        task_id    = goal_handle.request.task_id
        class_id   = TASK_CLASS.get(task_id, -1)
        class_name = CLASS_NAME.get(class_id, '알 수 없음')

        if class_id == -1:
            self.get_logger().error(f'유효하지 않은 task_id: {task_id}')
            result.success = False
            result.message = f'유효하지 않은 task_id: {task_id} (0=파란, 1=빨간, 2=노란)'
            goal_handle.abort()
            return result

        self.get_logger().info(f'타겟 클래스: {class_name} (id={class_id})')
        self.target_class = class_id

        self.open_gripper()
        self.go_home()
        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()

        # picking 영역만 IK 풀 수 있게 J1 제한 ON
        self.set_joint_limit(1, J1_LIMIT_MIN, J1_LIMIT_MAX)

        self.marker_error = None
        converged = self.visual_servo(goal_handle)

        if not converged:
            self.publish_feedback(goal_handle, 'SERVO_FAILED')
            # 제한 풀고 복귀
            self.set_joint_limit(1, J1_DEFAULT_MIN, J1_DEFAULT_MAX)
            self.go_home()
            self.target_class = None
            result.success = False
            result.message = '시각 서보 수렴 실패'
            goal_handle.abort()
            return result

        time.sleep(0.5)
        success = self.pick(goal_handle)

        # pick 끝났으면 J1 제한 해제 (place 가야 함)
        self.set_joint_limit(1, J1_DEFAULT_MIN, J1_DEFAULT_MAX)

        if success:
            self.publish_feedback(goal_handle, 'GO_READY')
            self.go_ready()
            self.place(goal_handle)

        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()

        self.target_class = None

        if success:
            result.success = True
            result.message = f'{class_name} 피킹/플레이스 완료'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = 'ee_coords 없음'
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceActionServerVer4()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
