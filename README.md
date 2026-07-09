# beep_jetcobot_ros2_opencv

OpenCV Contour 인식 + IBVS(Image-Based Visual Servoing)로 물체를 픽킹하는 Jetcobot(MyCobot 280 Pi) 제어 ROS2 패키지.

Addinedu × PinkLAB 팀 프로젝트 **"실버타운 로봇 어시스턴트"**의 세 파트(Open-RMF 기반 로봇 통합 제어 / AMR 자율주행 + Hand-Gesture 제어 / Robot Arm Pick & Place) 중 **Robot Arm Pick & Place**를 담당한 저장소입니다.

## 왜 IBVS인가

비교적 단순한 단일 객체를 대상으로 픽킹하는 조건상, 3D 좌표를 직접 추정하는 PBVS보다 카메라 픽셀 오차만으로 제어하는 **IBVS**가 개발 속도와 구현 난이도 면에서 더 적합하다고 판단해 채택했습니다.

## 전체 파이프라인

```
Camera (/dev/jetcocam0)
      │  320x240 프레임
      ▼
contour_detector  ── 적응형 이진화 → Contour 추출 → 최적 Contour 선택 → 중심점 계산
      │  undistortPoints로 왜곡 보정된 정규화 오차 (e_x, e_y)
      ▼  publish: /marker_error
pick_place_ver2   ── Visual Servo 루프 (오차가 THRESHOLD 이하로 수렴할 때까지 반복)
      │  publish: /coord_servo, /coord_command, /gripper_command
      ▼
joint_control     ── pymycobot SDK로 실제 로봇 제어
      ▼
MyCobot 280 Pi (Jetcobot)
```

세 노드는 `beep_jetcobot_bringup/launch/jetcobot_opencv.launch.py` 하나로 함께 실행됩니다.

## 패키지 구성

| 패키지 | 역할 |
|---|---|
| `beep_jetcobot_bringup` | 런치 파일 모음 (`jetcobot_opencv.launch.py` 등) |
| `beep_jetcobot_control` | 비전(Contour 인식) + 제어(Pick & Place, 관절 제어) 노드 |
| `beep_jetcobot_description` | MyCobot 280 Pi URDF |
| `beep_jetcobot_moveit_config` | MoveIt 설정 (현재 파이프라인에선 미사용) |
| `beep_jetcobot_msgs` | 커스텀 액션 정의 (`PickPlace.action`, 현재 파이프라인에선 미사용) |

## 노드 상세

### 1. `contour_detector` — 물체 인식

- 카메라 프레임을 320×240으로 축소 후 Gaussian Blur → Adaptive Threshold(`ADAPTIVE_THRESH_GAUSSIAN_C`) → Morphology Open/Close로 이진화
- `findContours`로 후보 윤곽선을 뽑은 뒤 다음 조건으로 최적 Contour를 선정
  - 넓이 500px² 이상 (노이즈 제거)
  - Solidity(면적/컨벡스헐 면적) 0.75 이상 (울퉁불퉁한 형태 제외)
  - Extent(면적/바운딩박스 면적) 0.5 이상 (바운딩박스 대비 빈 공간이 많은 후보 제외)
- `cv2.moments`로 무게중심 계산 → 카메라 캘리브레이션(`camera_cali.yaml`)의 K행렬/왜곡계수로 `undistortPoints` 보정 → 정규화된 오차 `(e_x, e_y)`를 `/marker_error`로 publish
- 디버깅용으로 인식 결과(윤곽선, 중심점, 중앙 십자선, DETECTED/NOT DETECTED 텍스트)를 오버레이한 프레임을 UDP(9998 포트)로 접속한 클라이언트에 스트리밍

### 2. `pick_place_ver2` — Visual Servo & Pick 시퀀스

1. 그리퍼 오픈 → Ready Pose 이동
2. **Visual Servo 루프**: `/marker_error`를 구독하며 P-제어 (`delta = -LAMBDA * error * 1000`, 최대 15mm/step)로 XY 좌표를 보정, `|e_x|, |e_y| < 0.05` 수렴 시 종료 (최대 50회 반복, 미수렴 시 Home 복귀 후 중단)
   - 카메라가 로봇 기준 ~90° 회전되어 있어 축을 교차 매핑 (`Robot +X → e_y`, `Robot +Y → e_x`)
3. **Pick**: 카메라-TCP 오프셋 보정 → Z축 하강(`PICK_Z`) → 그리퍼 닫기 → 상승(`LIFT_Z`)
4. Ready Pose로 복귀

### 3. `joint_control` — 하드웨어 인터페이스

`pymycobot` SDK로 실제 MyCobot 280 Pi(`/dev/ttyJETCOBOT`, 1,000,000bps)를 제어. `/joint_command`, `/coord_command`, `/coord_servo`, `/gripper_command`를 구독해 로봇에 명령을 전달하고, 현재 관절 각도·엔드이펙터 좌표를 주기적으로(`/joint_states`, `/ee_coords`, `/ee_angles`) publish.

## 요구사항

- ROS2 (colcon 빌드 환경)
- Python 3, `python3-opencv`
- [`pymycobot`](https://github.com/elephantrobotics/pymycobot) (MyCobot 280 Pi 제어 SDK)
- Jetcobot 카메라 디바이스(`/dev/jetcocam0`), 로봇 시리얼 디바이스(`/dev/ttyJETCOBOT`)

## 빌드 & 실행

```bash
cd ~/jetcobot_ws
colcon build --packages-select beep_jetcobot_bringup beep_jetcobot_control beep_jetcobot_description beep_jetcobot_msgs
source install/setup.bash

ros2 launch beep_jetcobot_bringup jetcobot_opencv.launch.py
```


## 관련 저장소

- [beep_jetcobot_ros2_yolo](https://github.com/JoshuaHong0215/beep_jetcobot_ros2_yolo) — YOLOv8 기반 물체 인식으로 전환한 후속 버전 (다중 물체 식별, 입/출고 통합 처리, Tailscale Mesh Network 구성 포함)
