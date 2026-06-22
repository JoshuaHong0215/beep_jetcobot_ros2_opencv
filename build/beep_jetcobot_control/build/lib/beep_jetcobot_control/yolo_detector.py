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
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

SERVER_URI = "ws://100.106.128.93:8765/ws"
CAMERA_DEVICE = "/dev/jetcocam0"
JPEG_QUALITY = 80
UDP_PORT = 9998

CLASS_IDS = {
    'large_blue_box':   0,
    'medium_red_box':   1,
    'small_yellow_box': 2,
}

_latest_frame = None
_frame_lock   = threading.Lock()
_clients      = set()
_clients_lock = threading.Lock()

_cross_x = 320
_cross_y = 240


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

        cv2.line(small, (_cross_x, 0),   (_cross_x, 480), (255, 255, 0), 1)
        cv2.line(small, (0,   _cross_y), (640, _cross_y), (255, 255, 0), 1)

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

        # 십자선 위치 캘리브 (UDP 영상 표시용)
        config_path = os.path.join(
            get_package_share_directory('beep_jetcobot_control'),
            'config', 'camera_cali.yaml'
        )
        with open(config_path, 'r') as f:
            calib = yaml.safe_load(f)
        K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        global _cross_x, _cross_y
        _cross_x = int(round(float(K[0, 2])))
        _cross_y = int(round(float(K[1, 2])))
        self.get_logger().info(f'십자선 = ({_cross_x}, {_cross_y})')

        # /detection: [class_id, cx, cy, w, h, conf]
        self.detection_pub = self.create_publisher(Float32MultiArray, '/detection', 10)

        self._latest_dets = None
        self._det_lock    = threading.Lock()

        self.cap = cv2.VideoCapture(CAMERA_DEVICE)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다')
            return

        threading.Thread(target=_client_listener,                     daemon=True).start()
        threading.Thread(target=_stream_loop, args=(self._get_dets,), daemon=True).start()
        threading.Thread(target=self._ws_thread_fn,                   daemon=True).start()

        self.timer = self.create_timer(0.1, self._publish_cb)
        self.get_logger().info('yolo_detector_node 시작')

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
            return

        # 모든 detection 발행: 필터링은 pick_place 쪽에서 target_class로.
        for det in dets:
            x1, y1, x2, y2 = det['bbox']
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w  = x2 - x1
            h  = y2 - y1
            class_id = float(CLASS_IDS.get(det['class'], -1))
            conf     = float(det['confidence'])

            msg      = Float32MultiArray()
            msg.data = [class_id, float(cx), float(cy), float(w), float(h), conf]
            self.detection_pub.publish(msg)

            self.get_logger().info(
                f'{det["class"]}({conf:.2f}) | 중점({cx:.0f},{cy:.0f}) | bbox {w:.0f}x{h:.0f}'
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
