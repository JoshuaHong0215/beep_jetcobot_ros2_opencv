import threading
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
from flask import Flask, Response


ARUCO_DICT = aruco.DICT_6X6_250
FLASK_PORT = 5000

_latest_frame = None
_frame_lock = threading.Lock()

flask_app = Flask(__name__)


@flask_app.route('/stream')
def stream():
    def generate():
        while True:
            with _frame_lock:
                frame = _latest_frame
            if frame is None:
                continue
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다')
            return

        self.aruco_dict = aruco.Dictionary_get(ARUCO_DICT)
        self.aruco_params = aruco.DetectorParameters_create()

        flask_thread = threading.Thread(
            target=lambda: flask_app.run(host='0.0.0.0', port=FLASK_PORT, threaded=True),
            daemon=True
        )
        flask_thread.start()

        self.timer = self.create_timer(0.03, self.detect)
        self.get_logger().info(f'aruco_detector_node 시작 — 스트림: http://0.0.0.0:{FLASK_PORT}/stream')

    def detect(self):
        global _latest_frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임을 읽을 수 없습니다')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                cx = int(corners[i][0][:, 0].mean())
                cy = int(corners[i][0][:, 1].mean())
                self.get_logger().info(f'마커 ID: {marker_id} | 중심: ({cx}, {cy})')

        _, buf = cv2.imencode('.jpg', frame)
        with _frame_lock:
            _latest_frame = buf.tobytes()

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
