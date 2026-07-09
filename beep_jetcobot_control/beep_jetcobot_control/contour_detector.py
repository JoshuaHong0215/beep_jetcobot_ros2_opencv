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

# 모폴로지 연산(침식연산) 
_morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


# 이진화: 흑백이미지를 받아서 물체/배경을 흑백으로 딱 나눔
def _binarize(gray):
    thresh = cv2.adaptiveThreshold(
        gray,                               # 흑백 이미지만 들어감
        255,                                # 조건 만족하면 255(흰색)으로 칠함
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,     # 가우시안: 중심에 가까울수록 가중치를 더 줘서 평균계산(노이즈에 강함)
        cv2.THRESH_BINARY_INV, 31, 5        # 블록 크기 31, 상수 5를 빼서 임계값 계산
    )
    if not DARK_OBJECT:     # 물체가 밝은 때만 반전. 현재는 True라 이줄은 실행안됨
        thresh = cv2.bitwise_not(thresh)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  _morph_kernel)   # 작은 점 제거
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, _morph_kernel)   # 구멍 메우기
    return thresh

# contours 중에서 가장 좋은 contour를 선택
def _pick_best_contour(contours):
    best = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)

        # 500px이하의 작은 물체는 무시(노이즈로 판단)
        if area < MIN_AREA:              
            continue
        
        # 모양이 울퉁불퉁하면 탈락
        hull      = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        solidity  = area / hull_area if hull_area > 0 else 0
        if solidity < MIN_SOLIDITY:
            continue

        # 바운딩 박스 대비 너무 비어있으면 탈락
        _, _, w, h = cv2.boundingRect(c)
        extent = area / (w * h) if w * h > 0 else 0
        if extent < MIN_EXTENT:
            continue

        # 살아남은 것중 가장 큰 객체를 best로 선택
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


# 카메라 루프: 카메라에서 프레임을 읽고, 인식결과를 UDP로 전송하는 기능
def camera_loop(cap):
    global _latest_frame

    # 영상보낼 통로(소켓) 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)

    while True:
        # 카메라에서 사진 한장 가져오기(프레임 읽는 것)
        ret, frame = cap.read()
        if not ret:
            continue

        # 원본 사진 저장(다른 함수에서도 사용함)
        with _frame_lock:
            _latest_frame = frame.copy()
        # 사진 크기 줄이기(용량관리)
        small = cv2.resize(frame, (320, 240))
        # 인식 결과 가져오기, detect()에서 _latest_result에 저장한 결과를 가져옴
        with _result_lock:
            result = _latest_result

        # 화면에 인식 결과 표시
        if result is not None:
            cv2.drawContours(small, [result['contour']], -1, (0, 255, 0), 2)
            cv2.circle(small, (result['cx'], result['cy']), 5, (0, 255, 0), -1)
            cv2.putText(small, 'DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(small, 'NOT DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 화면 중앙에 십자선 그리기
        cv2.line(small, (160, 0), (160, 240), (255, 255, 0), 1)
        cv2.line(small, (0, 120), (320, 120), (255, 255, 0), 1)

        # 사진 압축하기
        _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 40])
        with _clients_lock:
            # 접속한 모든 컴퓨터에게 전송, 접속해 있는 IP목록 하나씩 돌면서 압축된 영상을 쏴준다
            for ip in list(_clients):
                try:
                    sock.sendto(buf.tobytes(), (ip, UDP_PORT))
                except Exception:
                    pass

# 카메라 설정을 불러오며, 통신 및 인식 루프를 시작하는 ROS2 노드
'''
__init__ 실행 순서:
1. 보정 파일 읽기
2. K행렬 절반 크기로 조정
3. 왜곡계수 저장
4. 에러 발행 채널 개설
5. 미검출 카운터 세팅
6. 카메라 연결
7. 백그라운드 스레드 2개 켜기 (영상송출용)
8. 타이머 켜기 (0.05초마다 detect 실행) ← 실제 물체 탐지는 여기서 반복됨
'''
class ContourDetectorNode(Node):
    def __init__(self):
        super().__init__('contour_detector_node')

        # calibration file 불러오기
        config_path = os.path.join(
            get_package_share_directory('beep_jetcobot_control'),
            'config', 'camera_cali.yaml'
        )
        # with를 사용해서 손상을 방지하며 파일을 load시킴
        with open(config_path, 'r') as f:
            calib = yaml.safe_load(f)


        K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        # 처리 속도를 위해 기존 640x480 -> 320x240낮추고 해상도에 맞게 스케일 조정
        self.K_half = np.array([
            [K[0, 0] * 0.5, 0.0,           K[0, 2] * 0.5],
            [0.0,           K[1, 1] * 0.5, K[1, 2] * 0.5],
            [0.0,           0.0,           1.0           ]
        ])
        self.dist_coeffs = np.array(calib['distortion_coefficients']['data'])               # 왜곡 계수 저장

        self.error_pub    = self.create_publisher(Float32MultiArray, '/marker_error', 10)   # 에러 발생자 생성 -> 나중에 물체 위치 오차값을 이 채널로 계속 쏴줌
        # 미검출 카운터 초기화
        self._miss_count  = 0   # 물체를 못 찾은 횟수 카운트
        self._MISS_THRESH = 5   # 몇번 연속으로 못찾으면 물체가 없는 것으로 판단할지 기준값


        # 카메라 열기
        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다')
            return

        # 쓰레드 시작
        threading.Thread(target=client_listener, daemon=True).start()                   # UDP로 접속한 클라이언트 IP를 수집하는 쓰레드
        threading.Thread(target=camera_loop, args=(self.cap,), daemon=True).start()     # 카메라에서 프레임을 읽고, 인식결과를 UDP로 전송하는 쓰레드

        self.timer = self.create_timer(0.05, self.detect)
        self.get_logger().info('contour_detector_node 시작')

    def detect(self):
        global _latest_result
        # 최신 프레임 가져오기
        with _frame_lock:
            if _latest_frame is None:
                return
            frame = _latest_frame.copy()

        # 흑백 변환 + 이진화
        small  = cv2.resize(frame, (320, 240))              # 크기를 줄이고
        gray   = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)    # 흑백 변환을 하고
        gray   = cv2.GaussianBlur(gray, (5, 5), 0)          # 노이즈 제거를 위해 블러 처리한 후
        thresh = _binarize(gray)                            # 이진화 처리하여 물체만 흰색으로 뽑아 낸다

        # 윤곽선 찾기 + 가장 좋은 contour 선택
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = _pick_best_contour(contours)

        msg = Float32MultiArray()

        # 예외 처리: contour가 없으면 miss_count 증가시키고, 일정 횟수 이상이면 에러 발행
        if c is None:
            self._miss_count += 1
            with _result_lock:
                _latest_result = None
            if self._miss_count >= self._MISS_THRESH:
                msg.data = [0.0, 0.0, 0.0]
                self.error_pub.publish(msg)
            return

        # 물체 찾았을 때의 중심점 계산
        self._miss_count = 0
        # moments함수에 베스트 contour인 c를 넣어주면, 넓이, x축 위치합, y축 위치합 등 다양한 정보를 담은 딕셔너리를 반환
        M = cv2.moments(c)
        # 예외 처리: contour의 면적(m00)이 0이면 무게중심 계산 불가 → return
        if M['m00'] == 0:
            return
        # moments에서 반환된 딕셔너리 M을 이용하여 contour의 무게중심 좌표(u, v)를 계산
        '''
        m
        '''
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
