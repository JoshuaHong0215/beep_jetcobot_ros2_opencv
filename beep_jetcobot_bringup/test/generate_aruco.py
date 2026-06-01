import cv2
import cv2.aruco as aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

for marker_id in range(5):
    img = aruco.drawMarker(aruco_dict, marker_id, 500)
    cv2.imwrite(f'marker_{marker_id}.png', img)
    print(f'marker_{marker_id}.png 생성완료')
