import cv2
import asyncio
import websockets
import json
import numpy as np

SERVER_URI = "ws://100.106.128.93:8765/ws"  # Local PC Tailscale IP로 교체
CAMERA_DEVICE = "/dev/jetcocam0"
JPEG_QUALITY = 80


async def run():
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    if not cap.isOpened():
        print("[!] 카메라 열기 실패")
        return

    print(f"[+] 서버 연결 중: {SERVER_URI}")
    async with websockets.connect(SERVER_URI) as ws:
        print("[+] 서버 연결됨")
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            await ws.send(buf.tobytes())

            result = json.loads(await ws.recv())
            print(result)

    cap.release()


if __name__ == "__main__":
    asyncio.run(run())
