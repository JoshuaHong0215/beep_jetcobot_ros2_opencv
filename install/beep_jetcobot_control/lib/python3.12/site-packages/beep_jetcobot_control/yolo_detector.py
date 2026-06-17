import asyncio
import threading
import json
import socket
import time
import cv2
import numpy as np
import websockets
import yaml
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from ament_index_python.packages import get_package_share_directory

SERVER_URI = "ws://100.106.128.93:8765/ws"
CAMERA_DEVICE = "/dev/jetcocam0"
JPEG_QUALITY = 80
MISS_THRESH = 5
UDP_PORT = 9998

# class name → int id (data.yaml names 순서와 동일)
CLASS_IDS = {
    'large_blue_box':   0,
    'medium_red_box':   1,
    'small_yellow_box': 2,
}

_latest_frame = None
_frame_lock   = threading.Lock()
_clients      = set()
_clients_lock = threading.Lock()


def _client_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))
    while True:
        _, addr = sock.recvfrom(16)
        with _clients_lock:
            _clients.add(addr[0])


def _stream_loop(get_dets_fn):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)

    while True:
        time.sleep(0.05)

        with _frame_lock:
            if _latest_frame is None:
                continue
            frame = _latest_frame.copy()

        small = cv2.resize(frame, (640, 480))
        dets  = get_dets_fn()

        if dets:
            best = max(dets, key=lambda d: d['confidence'])
            x1, y1, x2, y2 = best['bbox']
            sx1, sy1 = int(x1), int(y1)
            sx2, sy2 = int(x2), int(y2)
            cx, cy   = (sx1 + sx2) // 2, (sy1 + sy2) // 2
            cv2.rectangle(small, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
            cv2.circle(small, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(small, f'{best["class"]} {best["confidence"]:.2f}',
                        (sx1, max(sy1 - 5, 10)),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 0), 1)
            cv2.putText(small, 'DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 0), 2)
        else:
            cv2.putText(small, 'NOT DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)

        cv2.line(small, (320, 0),   (320, 480), (255, 255, 0), 1)
        cv2.line(small, (0,   240), (640, 240), (255, 255, 0), 1)

        _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 40])
        with _clients_lock:
            for ip in list(_clients):
                try:
                    sock.sendto(buf.tobytes(), (ip, UDP_PORT))
                except Exception:
                    pass


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        config_path = os.path.join(
            get_package_share_directory('beep_jetcobot_control'),
            'config', 'camera_cali.yaml'
        )
        with open(config_path, 'r') as f:
            calib = yaml.safe_load(f)

        self.K          = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        self.dist_coeffs = np.array(calib['distortion_coefficients']['data'])

        self.error_pub     = self.create_publisher(Float32MultiArray, '/marker_error',    10)
        self.detection_pub = self.create_publisher(Float32MultiArray, '/yolo_detection',  10)

        self._miss_count   = 0
        self._latest_dets  = None
        self._det_lock     = threading.Lock()
        self._target_class = None  # None=best 1개, 정수=해당 클래스만

        self.create_subscription(Int32, '/target_class', self._target_class_cb, 10)

        self.cap = cv2.VideoCapture(CAMERA_DEVICE)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다')
            return

        threading.Thread(target=_client_listener,                       daemon=True).start()
        threading.Thread(target=_stream_loop, args=(self._get_dets,),   daemon=True).start()
        threading.Thread(target=self._ws_thread_fn,                     daemon=True).start()

        self.timer = self.create_timer(0.1, self._publish_cb)
        self.get_logger().info('yolo_detector_node 시작')

    def _target_class_cb(self, msg):
        self._target_class = msg.data if msg.data >= 0 else None

    def _get_dets(self):
        with self._det_lock:
            return self._latest_dets

    def _ws_thread_fn(self):
        asyncio.run(self._ws_async())

    async def _ws_async(self):
        global _latest_frame
        while True:
            try:
                async with websockets.connect(SERVER_URI) as ws:
                    self.get_logger().info('YOLO 서버 연결됨')
                    while True:
                        ret, frame = self.cap.read()
                        if not ret:
                            continue

                        with _frame_lock:
                            _latest_frame = frame.copy()

                        _, buf = cv2.imencode('.jpg', frame,
                                             [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                        await ws.send(buf.tobytes())

                        raw  = await ws.recv()
                        dets = json.loads(raw)

                        with self._det_lock:
                            self._latest_dets = dets if dets else None

            except Exception as e:
                self.get_logger().warn(f'서버 연결 끊김: {e} — 2초 후 재연결')
                with self._det_lock:
                    self._latest_dets = None
                await asyncio.sleep(2.0)

    def _publish_cb(self):
        with self._det_lock:
            dets = self._latest_dets

        if not dets:
            self._miss_count += 1
            if self._miss_count >= MISS_THRESH:
                msg      = Float32MultiArray()
                msg.data = [0.0, 0.0, 0.0]
                self.error_pub.publish(msg)
            return

        self._miss_count = 0

        if self._target_class is not None:
            filtered = [d for d in dets if CLASS_IDS.get(d['class'], -1) == self._target_class]
            if not filtered:
                self._miss_count += 1
                if self._miss_count >= MISS_THRESH:
                    msg      = Float32MultiArray()
                    msg.data = [0.0, 0.0, 0.0]
                    self.error_pub.publish(msg)
                return
            best = max(filtered, key=lambda d: d['confidence'])
        else:
            best = max(dets, key=lambda d: d['confidence'])

        x1, y1, x2, y2 = best['bbox']
        cx     = (x1 + x2) / 2.0
        cy     = (y1 + y2) / 2.0
        bbox_w = x2 - x1
        bbox_h = y2 - y1

        pt   = np.array([[[cx, cy]]], dtype=np.float32)
        norm = cv2.undistortPoints(pt, self.K, self.dist_coeffs)
        e_x  = float(norm[0][0][0])
        e_y  = float(norm[0][0][1])

        class_id = float(CLASS_IDS.get(best['class'], -1))
        conf     = float(best['confidence'])

        error_msg      = Float32MultiArray()
        error_msg.data = [e_x, e_y, 1.0]
        self.error_pub.publish(error_msg)

        det_msg      = Float32MultiArray()
        det_msg.data = [class_id, float(bbox_w), float(bbox_h), conf]
        self.detection_pub.publish(det_msg)

        self.get_logger().info(
            f'{best["class"]}({conf:.2f}) | 중점({cx:.0f},{cy:.0f}) | '
            f'e_x={e_x:.4f} e_y={e_y:.4f} | bbox {bbox_w:.0f}x{bbox_h:.0f}'
        )

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
