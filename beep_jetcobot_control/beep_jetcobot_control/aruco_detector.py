import threading
import socket
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import os
import numpy as np

from std_msgs.msg import Float32MultiArray


ARUCO_DICT = aruco.DICT_6X6_250
UDP_PORT   = 9998

_cap_lock     = threading.Lock()
_clients      = set()
_clients_lock = threading.Lock()
_latest_frame = None
_frame_lock   = threading.Lock()


def client_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))
    while True:
        _, addr = sock.recvfrom(16)
        with _clients_lock:
            _clients.add(addr[0])
        print(f'클라이언트 등록: {addr[0]}')


def preview_loop(cap):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)

    aruco_dict   = aruco.Dictionary_get(ARUCO_DICT)
    aruco_params = aruco.DetectorParameters_create()

    while True:
        with _cap_lock:
            ret, frame = cap.read()
        if not ret:
            continue

        small = cv2.resize(frame, (320, 240))
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(small, corners, ids)
            cv2.putText(small, f'DETECTED id={ids.flatten()[0]}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(small, 'NOT DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 40])
        data = buf.tobytes()

        with _clients_lock:
            for ip in list(_clients):
                try:
                    sock.sendto(data, (ip, UDP_PORT))
                except Exception:
                    pass


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        config_path = os.path.join(
            get_package_share_directory('beep_jetcobot_control'),
            'config', 'camera_cali.yaml'
        )
        with open(config_path, 'r') as f:
            calib = yaml.safe_load(f)

        self.K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        self.D = np.array(calib['distortion_coefficients']['data'])

        # TCP → 카메라 렌즈 중심 오프셋 (단위: m, 실측값)
        # X: 60mm, Y: 0 (정중앙), Z: 30mm (카메라가 TCP 위)
        self.t_cam2ee = np.array([[0.060], [0.000], [0.030]])
        self.R_cam2ee = np.eye(3)

        self.ee_coords = None
        self.create_subscription(Float32MultiArray, '/ee_coords', self.ee_coords_cb, 10)

        self.marker_pub = self.create_publisher(Float32MultiArray, '/marker_coord', 10)

        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다')
            return

        self.aruco_dict   = aruco.Dictionary_get(ARUCO_DICT)
        self.aruco_params = aruco.DetectorParameters_create()

        threading.Thread(target=client_listener, daemon=True).start()
        threading.Thread(target=preview_loop, args=(self.cap,), daemon=True).start()

        self.timer = self.create_timer(0.03, self.detect)
        self.get_logger().info(f'aruco_detector_node 시작 — 로컬PC: python view_udp.py <로봇IP>')

    def ee_coords_cb(self, msg):
        self.ee_coords = list(msg.data)

    def detect(self):
        with _cap_lock:
            ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임을 읽을 수 없습니다')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, 0.030, self.K, self.D
        )

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]

            self.get_logger().info(
                f'ID: {marker_id} | 카메라 기준 x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f} (m)'
            )

            if self.ee_coords is None:
                self.get_logger().warn('EE 좌표 미수신 — 변환 스킵')
                continue

            x, y, z, rx, ry, rz = self.ee_coords
            R_base_ee = Rotation.from_euler('ZYX', [rz, ry, rx], degrees=True).as_matrix()
            t_base_ee = np.array([x, y, z]).reshape(3, 1) / 1000.0

            p_cam  = tvec.reshape(3, 1)
            p_ee   = self.R_cam2ee @ p_cam + self.t_cam2ee
            p_base = (R_base_ee @ p_ee + t_base_ee).flatten() * 1000.0

            msg = Float32MultiArray()
            msg.data = [
                float(p_base[0]), float(p_base[1]), float(p_base[2]),
                -180.0, 0.0, 90.0
            ]
            self.marker_pub.publish(msg)

            self.get_logger().info(
                f'ID: {marker_id} | 베이스 기준 x={p_base[0]:.1f} y={p_base[1]:.1f} z={p_base[2]:.1f} mm'
            )

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
