# pick_place_action_server_ver5 코드 해설

## 개요
액션 서버. `task_id`(박스 종류), `pick_from`(픽 영역), `place_to`(놓을 영역) 받아서 visual servo 기반 픽앤플레이스 수행.

---

## 1. 상수

| 상수 | 값 | 설명 |
|---|---|---|
| `LAMBDA` | 0.3 | 픽셀 에러 → mm 변환 게인 (mm/pixel) |
| `THRESHOLD` | 16.0 | 픽셀 16 이내면 정렬 완료 |
| `MAX_ITER` | 50 | visual servo 최대 반복 횟수 |
| `MAX_DELTA` | 15.0 | 한 스텝당 최대 이동 mm |
| `LIFT_Z` | 317.1 | lift 높이 (ready 높이와 동일) |
| `CAM_TCP_X` | 120 | 카메라 정렬 후 그리퍼를 박스 위로 옮길 X 오프셋 |
| `CAM_TCP_Y` | 5.0 | 위와 동일 Y |
| `Z_MIN_SAFE` | 100.0 | send_coords 호출 시 이 값 이하로 못 감 (충돌 방지 클램프) |

### `PICK_AREAS`
pick 영역별 설정 딕셔너리.
- `'work'`: `pick_z=120.1`, `j1_offset_deg=0.0`
- `'storage'`: `pick_z=120.1`, `j1_offset_deg=-90.0` (베이스 기준 우측 90도)

### `PLACE_ANGLES`
place 영역별 6관절 자세각.
- `'storage'`, `'loading'` 각각 6개 float

### `TASK_CLASS` / `CLASS_NAME`
- `'0'→0`(blue), `'1'→1`(red), `'2'→2`(yellow)

---

## 2. `__init__`

- `ready_coords = [129.1, -62.8, 317.1, -162.04, -18.67, -42.35]` work ready 자세 (cartesian)
- `camera_cali.yaml` 읽어서 `cx_pp`, `cy_pp` 추출 (카메라 principal point = 화면 중심)
- 상태 변수:
  - `marker_error`: detection_cb가 채우는 픽셀 에러 [e_x, e_y, valid]
  - `ee_coords`: ee_coords_cb가 채우는 현재 그리퍼 카르테시안
  - `current_angles`: joint_state_cb가 채우는 현재 6관절 각도 (도)
  - `target_class`: 현재 잡으려는 박스 클래스 (필터링용)
- publisher: `/joint_command`, `/coord_command`, `/coord_servo`, `/gripper_command`
- subscription: `/detection`, `/ee_coords`, `/joint_states`
- ActionServer: `'pick_place'` 이름으로 등록

---

## 3. 콜백

### `detection_cb`
YOLO에서 `[class_id, cx, cy, w, h, conf]` 받음.
- `target_class`와 안 맞으면 무시
- 화면 중심 기준 픽셀 에러로 변환: `e_x = cx - cx_pp`, `e_y = cy - cy_pp`
- `marker_error`에 저장

### `ee_coords_cb`, `joint_state_cb`
받은 값 그대로 저장 (joint_state는 라디안→도 변환).

### `goal_cb` / `cancel_cb`
무조건 ACCEPT.

---

## 4. 퍼블리셔 래퍼

| 함수 | 동작 |
|---|---|
| `send_angles(angles)` | 6관절각을 /joint_command로 publish |
| `send_coords(coords)` | cartesian을 /coord_command로 publish. **z < Z_MIN_SAFE면 자동 클램프** |
| `send_servo_coords(coords)` | /coord_servo (mode 1 직선보간). ver5에선 안 씀 |
| `send_gripper(value)` | 0=완전닫힘, 100=완전열림 |

---

## 5. 헬퍼/모션

### `get_fresh_error()`
오래된 marker_error로 잘못된 서보 하지 않게 하기 위함.
1. `marker_error = None`으로 비움
2. 최대 2초 동안 새 메시지 기다림
3. 받으면 반환, 못 받으면 None

### `go_home()`, `go_ready()`, `open_gripper()`
각각 home 자세, ready 자세, 그리퍼 열기 + 안정화 sleep.

### `rotate_j1_from_ready(j1_offset_deg)`
ready 자세 그대로 두고 J1만 추가 회전.
1. offset이 거의 0이면 그냥 통과
2. 현재 6관절각(`current_angles`) 복사
3. 첫 값(J1)에 offset 더함
4. `send_angles(target)` + 4초 대기
- → storage 모드일 때 ready에서 -90도 돌리는 데 사용

### `visual_servo(goal_handle)` — 핵심 루프
1. 진입 시 자세각 (rx, ry, rz) 한 번 캡처 → 이후 고정 (자세 흔들림 방지)
2. `MAX_ITER`(50) 반복:
   - 취소 요청 시 종료
   - `get_fresh_error()`로 새 에러 받기
   - 에러 없으면 'SEARCHING' 피드백 후 continue
   - 'SERVO' 피드백 발행 (e_x, e_y, iter)
   - `|e_x| < THRESHOLD and |e_y| < THRESHOLD` → 수렴, return True
   - mm 변환:
     ```python
     delta_x = -LAMBDA * e_y   # 카메라 ~90° 회전 보정
     delta_y = -LAMBDA * e_x
     ```
   - `MAX_DELTA`로 클립
   - `send_coords([cur_x+delta_x, cur_y+delta_y, cur_z, rx, ry, rz])` + 0.5초 대기
3. 50회 다 돌고도 수렴 못 하면 return False

### `pick(goal_handle, pick_z)` — 4단계
1. **TCP_ALIGN**: 현재 좌표 + `(CAM_TCP_X, CAM_TCP_Y)` → 그리퍼를 박스 위로 이동
2. **DESCEND**: 같은 XY에서 z를 `pick_z`까지 하강
3. **GRIP**: 그리퍼 닫기 + 1.5초
4. **LIFT**: z를 `LIFT_Z`로 들기

### `place(goal_handle, place_angles)`
1. `send_angles(place_angles)` → 자세 이동
2. 그리퍼 열기

---

## 6. `execute_cb` — 메인 흐름

```
1. goal에서 task_id, pick_from, place_to 추출
2. 검증 (잘못된 값이면 abort)
3. PICK_AREAS[pick_from], PLACE_ANGLES[place_to] 가져오기
4. target_class 설정 (detection_cb 필터링 작동)
5. open_gripper → go_home → go_ready (work용 ready)
6. rotate_j1_from_ready(j1_offset_deg)
   - work면 0 → 그대로
   - storage면 -90 → 베이스만 우측 회전
7. visual_servo() → 박스 위로 카메라 정렬
   - 실패하면 go_home + abort
8. pick(pick_z) → TCP_ALIGN → DESCEND → GRIP → LIFT
9. place(place_angles) → 자세 이동 + 그리퍼 열기
10. go_ready (work 복귀)
11. succeed() 또는 abort()
```

### 시나리오별 흐름

**`pick_from='work', place_to='storage'`:**
```
home → ready → (회전 없음) → 박스 정렬 → 잡기 → storage 자세 → 놓기 → ready
```

**`pick_from='storage', place_to='loading'`:**
```
home → ready → J1 -90° 회전 → 박스 정렬 → 잡기 → loading 자세 → 놓기 → ready
```

---

## 7. 실행 명령

```bash
# 빌드
cd ~/jetcobot_ws
colcon build --packages-select beep_jetcobot_msgs
colcon build --packages-select beep_jetcobot_control
source install/setup.bash

# 런치
ros2 launch beep_jetcobot_bringup jetcobot_ver5.launch.py

# 액션 호출
ros2 action send_goal /pick_place beep_jetcobot_msgs/action/PickPlace \
  "{task_id: '1', pick_from: 'work', place_to: 'storage'}"

ros2 action send_goal /pick_place beep_jetcobot_msgs/action/PickPlace \
  "{task_id: '1', pick_from: 'storage', place_to: 'loading'}"
```

## 8. 새 영역 추가하는 법
1. `PICK_AREAS`에 항목 추가 (`pick_z`, `j1_offset_deg`)
2. 또는 `PLACE_ANGLES`에 항목 추가 (6관절각)
3. 실행 시 새 키 이름을 `pick_from`/`place_to`로 보내면 됨
- 코드 본문(execute_cb) 수정 불필요
