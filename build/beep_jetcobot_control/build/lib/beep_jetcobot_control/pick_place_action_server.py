import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32
import time

from beep_jetcobot_msgs.action import PickPlace

LAMBDA         = 0.3
THRESHOLD      = 0.08
MAX_ITER       = 150
MAX_DELTA      = 8.0
CONVERGE_COUNT = 3

PICK_Z    = 130.1
LIFT_Z    = 317.1

CAM_TCP_X = 122.0
CAM_TCP_Y = -20.0

# task_id 문자열 → class_id 매핑
TASK_CLASS = {
    '0': 0,  # large_blue_box
    '1': 1,  # medium_red_box
    '2': 2,  # small_yellow_box
}


class PickPlaceActionServer(Node):
    def __init__(self):
        super().__init__('pick_place_action_server')

        self.home_angles  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ready_coords = [40, -62.8, 317.1, -162.04, -18.67, -42.35]
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

        self.get_logger().info('pick_place_action_server 시작')

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
        time.sleep(3)

    def go_ready(self):
        self.send_coords(self.ready_coords)
        time.sleep(4)

    def open_gripper(self):
        self.send_gripper(100)
        time.sleep(2)

    def visual_servo(self, goal_handle):
        consec = 0
        for i in range(MAX_ITER):
            if goal_handle.is_cancel_requested:
                return False

            error = self.get_fresh_error()
            if error is None:
                self.publish_feedback(goal_handle, 'SEARCHING', iteration=i)
                consec = 0
                continue

            e_x, e_y, _ = error
            self.publish_feedback(goal_handle, 'SERVO', e_x, e_y, i)
            self.get_logger().info(f'[{i}] e_x={e_x:.4f}  e_y={e_y:.4f}  consec={consec}')

            if abs(e_x) < THRESHOLD and abs(e_y) < THRESHOLD:
                consec += 1
                if consec >= CONVERGE_COUNT:
                    self.get_logger().info('수렴 완료')
                    time.sleep(1.0)
                    return True
                continue

            consec = 0

            if self.ee_coords is None:
                continue

            cur_x, cur_y, cur_z = self.ee_coords[0], self.ee_coords[1], self.ee_coords[2]
            rx, ry, rz = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

            delta_x = max(-MAX_DELTA, min(MAX_DELTA, -LAMBDA * e_y * 1000.0))
            delta_y = max(-MAX_DELTA, min(MAX_DELTA, -LAMBDA * e_x * 1000.0))

            self.send_servo_coords([cur_x + delta_x, cur_y + delta_y, cur_z, rx, ry, rz])
            time.sleep(0.5)

        return False

    def pick(self, goal_handle):
        if self.ee_coords is None:
            return False

        x = self.ee_coords[0] + CAM_TCP_X
        y = self.ee_coords[1] + CAM_TCP_Y
        z = self.ee_coords[2]
        rx, ry, rz = self.ee_coords[3], self.ee_coords[4], self.ee_coords[5]

        self.publish_feedback(goal_handle, 'OFFSET_MOVE')
        self.send_coords([x, y, z, rx, ry, rz])
        time.sleep(2)

        self.publish_feedback(goal_handle, 'DESCENDING')
        self.send_coords([x, y, PICK_Z, rx, ry, rz])
        time.sleep(3)

        self.publish_feedback(goal_handle, 'GRIPPING')
        self.send_gripper(0)
        time.sleep(1.5)

        self.publish_feedback(goal_handle, 'LIFTING')
        self.send_coords([x, y, LIFT_Z, rx, ry, rz])
        time.sleep(3)

        return True

    def set_target_class(self, class_id):
        msg      = Int32()
        msg.data = class_id
        self.target_class_pub.publish(msg)

    # ── execute ──────────────────────────────────────────────────
    def execute_cb(self, goal_handle):
        self.get_logger().info('태스크 실행 시작')
        result = PickPlace.Result()

        task_id    = goal_handle.request.task_id
        class_id   = TASK_CLASS.get(task_id, -1)
        class_name = {0: 'large_blue_box', 1: 'medium_red_box', 2: 'small_yellow_box'}.get(class_id, '알 수 없음')

        if class_id == -1:
            self.get_logger().error(f'유효하지 않은 task_id: {task_id}')
            result.success = False
            result.message = f'유효하지 않은 task_id: {task_id} (0=파란, 1=빨간, 2=노란)'
            goal_handle.abort()
            return result

        self.get_logger().info(f'타겟 클래스: {class_name} (id={class_id})')
        self.set_target_class(class_id)

        self.open_gripper()
        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()

        self.marker_error = None
        converged = self.visual_servo(goal_handle)

        if not converged:
            self.publish_feedback(goal_handle, 'SERVO_FAILED')
            self.go_home()
            result.success = False
            result.message = '시각 서보 수렴 실패'
            goal_handle.abort()
            return result

        success = self.pick(goal_handle)
        self.publish_feedback(goal_handle, 'GO_READY')
        self.go_ready()

        self.set_target_class(-1)  # 타겟 해제

        if success:
            result.success = True
            result.message = f'{class_name} 피킹 완료'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = 'ee_coords 없음'
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
