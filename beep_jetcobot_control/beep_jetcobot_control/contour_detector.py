import threading
import socket
import rclpy
from rclpy.node import Node
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from std_msgs.msg import Float32MultiArray

UDP_PORT = 9998

_latest_frame  = None
_frame_lock    = threading.Lock()
_latest_result = None  # detect()가 쓰고 camera_loop()가 읽음
_result_lock   = threading.Lock()
_clients       = set()
_clients_lock  = threading.Lock()

DARK_OBJECT = True

MIN_AREA     = 500
MIN_SOLIDITY = 0.75
MIN_EXTENT   = 0.50

_morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


def _binarize(gray):
    thresh = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 31, 5
    )
    if not DARK_OBJECT:
        thresh = cv2.bitwise_not(thresh)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  _morph_kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, _morph_kernel)
    return thresh


def _pick_best_contour(contours):
    best = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue

        hull      = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        solidity  = area / hull_area if hull_area > 0 else 0
        if solidity < MIN_SOLIDITY:
            continue

        _, _, w, h = cv2.boundingRect(c)
        extent = area / (w * h) if w * h > 0 else 0
        if extent < MIN_EXTENT:
            continue

        if area > best_area:
            best_area = area
            best = c

    return best


def client_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))
    while True:
        _, addr = sock.recvfrom(16)
        with _clients_lock:
            _clients.add(addr[0])


def camera_loop(cap):
    global _latest_frame
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        with _frame_lock:
            _latest_frame = frame.copy()

        small = cv2.resize(frame, (320, 240))

        with _result_lock:
            result = _latest_result

        if result is not None:
            cv2.drawContours(small, [result['contour']], -1, (0, 255, 0), 2)
            cv2.circle(small, (result['cx'], result['cy']), 5, (0, 255, 0), -1)
            cv2.putText(small, 'DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(small, 'NOT DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.line(small, (160, 0), (160, 240), (255, 255, 0), 1)
        cv2.line(small, (0, 120), (320, 120), (255, 255, 0), 1)

        _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 40])
        with _clients_lock:
            for ip in list(_clients):
                try:
                    sock.sendto(buf.tobytes(), (ip, UDP_PORT))
                except Exception:
                    pass


class ContourDetectorNode(Node):
    def __init__(self):
        super().__init__('contour_detector_node')

        config_path = os.path.join(
            get_package_share_directory('beep_jetcobot_control'),
            'config', 'camera_cali.yaml'
        )
        with open(config_path, 'r') as f:
            calib = yaml.safe_load(f)

        K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        # 320x240 해상도에 맞게 스케일 조정
        self.K_half = np.array([
            [K[0, 0] * 0.5, 0.0,           K[0, 2] * 0.5],
            [0.0,           K[1, 1] * 0.5, K[1, 2] * 0.5],
            [0.0,           0.0,           1.0           ]
        ])
        self.dist_coeffs = np.array(calib['distortion_coefficients']['data'])

        self.error_pub    = self.create_publisher(Float32MultiArray, '/marker_error', 10)
        self._miss_count  = 0
        self._MISS_THRESH = 5

        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다')
            return

        threading.Thread(target=client_listener, daemon=True).start()
        threading.Thread(target=camera_loop, args=(self.cap,), daemon=True).start()

        self.timer = self.create_timer(0.05, self.detect)
        self.get_logger().info('contour_detector_node 시작')

    def detect(self):
        global _latest_result

        with _frame_lock:
            if _latest_frame is None:
                return
            frame = _latest_frame.copy()

        small  = cv2.resize(frame, (320, 240))
        gray   = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray   = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = _binarize(gray)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = _pick_best_contour(contours)

        msg = Float32MultiArray()

        if c is None:
            self._miss_count += 1
            with _result_lock:
                _latest_result = None
            if self._miss_count >= self._MISS_THRESH:
                msg.data = [0.0, 0.0, 0.0]
                self.error_pub.publish(msg)
            return

        self._miss_count = 0
        M = cv2.moments(c)
        if M['m00'] == 0:
            return

        u = M['m10'] / M['m00']
        v = M['m01'] / M['m00']

        # 중점 좌표만 왜곡 보정 → 정규화 좌표(e_x, e_y) 직접 반환
        pt   = np.array([[[u, v]]], dtype=np.float32)
        norm = cv2.undistortPoints(pt, self.K_half, self.dist_coeffs)
        e_x  = float(norm[0][0][0])
        e_y  = float(norm[0][0][1])

        with _result_lock:
            _latest_result = {'contour': c, 'cx': int(u), 'cy': int(v)}

        msg.data = [e_x, e_y, 1.0]
        self.error_pub.publish(msg)
        self.get_logger().info(f'물체 | 픽셀({u:.0f},{v:.0f}) | e_x={e_x:.4f} e_y={e_y:.4f}')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ContourDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
