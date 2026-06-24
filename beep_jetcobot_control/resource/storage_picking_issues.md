# storage 영역 피킹 시 발생한 문제들

mode 1 (storage → loading) 구현하면서 만난 문제들 정리. 그냥 work 영역과 똑같이 하려고 했는데 로봇 팔이 90도 돌아간 상태라서 여러 가지가 어긋났음.

## 기본 전제

- 로봇은 책상 위에 고정돼 있고, 항상 같은 "베이스 좌표계"를 기준으로 좌표를 받음.
- work 영역 = 로봇 정면, storage 영역 = 로봇 오른쪽 (90도 회전 위치).
- 모든 자리 측정값(카메라 위치, 박스 잡는 위치 등)은 **work 영역에서 잰 값**.

핵심 비유: **고개를 정면에서 오른쪽으로 90도 돌리면 "내 앞"이 바뀌는데, 좌표는 "방의 북쪽"으로 고정돼 있어서 헷갈리는 것**. 로봇도 마찬가지.

---

## 문제 1. ready pose만 가지고는 storage 박스를 못 봄

### 증상
mode 0(work)에서 쓰던 ready_coords로 가면 카메라가 work 영역만 봄. storage 박스가 시야 밖이라 visual_servo가 박스를 못 찾고 SEARCHING만 반복.

### 원인
ready pose는 work 영역 정면을 내려다보는 자세로 정해놨음. storage 영역은 그 시야 밖에 있음.

### 해결
mode 1에서는 ready 자세에 도착한 다음에 J1(베이스 회전 관절)만 추가로 -90도 돌려서 카메라가 storage 쪽을 보게 함.

```python
self.go_ready()                       # 일단 work ready
self.rotate_j1_from_ready(-90.0)      # 그 자세에서 J1만 -90도 더 돌림
```

**중요**: 처음엔 `[J1=-90, 나머지 0]` 식으로 J1만 -90도이고 다른 관절은 home으로 보냈는데, 그러면 팔이 펴진 상태로 우측으로만 뻗어서 카메라가 아래를 안 보고 정면을 봄. 그래서 **반드시 work ready 자세를 먼저 만든 후에 J1만 추가 회전**해야 카메라가 아래를 본다.

---

## 문제 2. 카메라-TCP 오프셋(CAM_TCP)이 회전한 만큼 어긋남

### 증상
visual_servo가 박스를 카메라 중앙에 정렬해도, pick 단계에서 그리퍼가 박스 위가 아닌 엉뚱한 방향으로 이동.

### 원인
- 카메라는 그리퍼 옆에 살짝 떨어진 위치에 붙어 있음. work 영역에서 측정한 카메라→그리퍼 오프셋은 `(+120, +5)` (베이스 +X로 120mm, +Y로 5mm).
- 이건 "로봇이 정면을 보고 있을 때" 기준 방향임.
- mode 1은 J1을 -90도 돌렸기 때문에 그리퍼가 오른쪽을 보고 있음. 그래서 같은 `(+120, +5)`를 베이스 좌표에 더하면 storage가 아닌 다른 방향으로 가버림.

### 비유
정면을 볼 때 "내 오른쪽 어깨" 방향이 +Y였는데, 오른쪽으로 90도 돌면 "내 오른쪽 어깨"가 베이스 기준 -X 방향이 됨. 같은 "오른쪽으로 5cm"가 베이스 좌표에서는 다른 숫자가 되는 것.

### 해결
mode별로 CAM_TCP 오프셋을 따로 지정.

```python
# work 자세 기준
CAM_TCP_X_WORK    = 120
CAM_TCP_Y_WORK    = 5

# storage 자세 (J1 -90도 회전 후) 기준
# work의 (120, 5)를 -90도 회전시킨 값
CAM_TCP_X_STORAGE = 5
CAM_TCP_Y_STORAGE = -120
```

수학적으로는 점 `(120, 5)`를 -90도 회전시키면:
- 새 X = 120·cos(-90°) - 5·sin(-90°) = 0 + 5 = 5
- 새 Y = 120·sin(-90°) + 5·cos(-90°) = -120 + 0 = -120

---

## 문제 3. visual_servo의 축 매핑도 어긋남

### 증상
servo iteration에서 로봇이 박스 쪽으로 가야 하는데 엉뚱하게 멀어지거나 좌우로 발산. e_x가 -164 → -170 → -209 처럼 더 커지는 식.

### 원인
visual_servo 코드는 카메라 이미지의 픽셀 에러(e_x, e_y)를 로봇 좌표 이동량(delta_x, delta_y)으로 바꿔주는데, **카메라가 어느 방향으로 붙어있느냐**에 따라 매핑이 달라짐.

work 자세에서 카메라는 베이스에 대해 약 90도 회전된 채로 붙어 있음. 그래서 work 매핑은:
```
delta_x = -LAMBDA * e_y    # 이미지 세로 에러 ↔ 로봇 +X
delta_y = -LAMBDA * e_x    # 이미지 가로 에러 ↔ 로봇 +Y
```

mode 1은 J1을 -90도 더 돌렸으므로 카메라가 또 90도 회전된 셈. 이 추가 회전 때문에 위 매핑이 안 맞음.

### 비유
"앞에 있는 컵을 카메라로 보면서, 컵이 화면 오른쪽에 있으면 로봇을 왼쪽으로 움직여 가운데로 오게" 하는 규칙이 있다고 치자. 카메라를 90도 돌리면 같은 "화면 오른쪽"이 실제로는 위쪽 방향이 됨. 그러면 로봇은 왼쪽이 아니라 아래쪽으로 움직여야 함. 매핑 자체가 바뀌어야 한다는 뜻.

### 해결
mode별로 다른 매핑.

```python
if mode == MODE_WORK_TO_STORAGE:
    delta_x = -LAMBDA * e_y
    delta_y = -LAMBDA * e_x
else:  # MODE_STORAGE_TO_LOADING
    delta_x = -LAMBDA * e_x
    delta_y = +LAMBDA * e_y
```

---

## 문제 4. place 위치도 mode마다 달라야 함

### 증상
mode 1에서 박스를 잡았는데 storage 옆에다 떨어뜨림.

### 원인
PLACE_ANGLES가 mode 0에서 쓰던 storage 위치 자세각 하나만 있었음. mode 1은 loading 위치에 놔야 하는데 같은 값을 써서 엉뚱한 곳에 놓음.

### 해결
mode별로 PLACE_ANGLES 분리.

```python
PLACE_ANGLES_STORAGE = [-84.99, -48.86, -23.81, -8.87, -1.05, -36.91]   # mode 0용
PLACE_ANGLES_LOADING = [...]                                            # mode 1용 (측정 필요)
```

**아직 측정 안 한 placeholder**라 mode 1에서 정확한 적재공간 좌표는 직접 잰 후 코드에 박아야 함.

---

## 정리: mode 0과 mode 1의 차이

| 항목 | mode 0 (work→storage) | mode 1 (storage→loading) |
|---|---|---|
| 시작 자세 | work ready 그대로 | work ready + J1 -90° 회전 |
| CAM_TCP 오프셋 | (120, 5) | (5, -120) |
| visual_servo 매핑 | delta_x=-k·e_y, delta_y=-k·e_x | delta_x=-k·e_x, delta_y=+k·e_y |
| pick_z | 120.1 | 120.1 (TODO: 측정) |
| place 자세 | PLACE_ANGLES_STORAGE | PLACE_ANGLES_LOADING (TODO: 측정) |

## 왜 다 어긋났나 한 줄로

> 모든 측정값을 "정면을 보는 자세" 기준으로 잡아놨는데, mode 1은 그 자세를 90도 돌려서 시작하니까 방향에 관련된 모든 값이 같이 회전돼야 했음.

## 앞으로 새 영역(예: 좌측 storage) 추가하면

회전 각도만 다르고 똑같은 4가지 (시작 회전 / CAM_TCP / servo 매핑 / place 자세)를 그 영역에 맞게 다시 잡아줘야 함. 코드를 자동으로 회전 계산하게 짤 수도 있지만 직관성 떨어져서 mode별 상수 명시 방식으로 유지 중.
