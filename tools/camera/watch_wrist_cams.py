"""손목캠 2대의 끊김 감시. 카메라만 읽고 팔에는 아무 명령도 보내지 않는다.

조작 실행기와 카메라를 같이 열 수 없으므로 실행기를 끈 상태에서 쓴다. 토크를 끈 팔을 손으로 움직이며
케이블·커넥터를 흔들면 끊긴 시각과 복구 시간이 [camera ...] 줄로 남는다. 재연결은 실행기와 같은
camera_fallback.py를 쓴다. Windows 쪽 기록은 Kernel-PnP Device Management 이벤트 1010으로 맞춰 본다.

사용법:
    python tools/camera/watch_wrist_cams.py                 # 30분 뒤 스스로 끝남
    python tools/camera/watch_wrist_cams.py --seconds 300
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "camera"))
sys.path.insert(0, str(ROOT / "tools" / "rollout"))

import camera_fallback  # noqa: E402
from resolve_camera_indices import resolve  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=1800)
    a = ap.parse_args()

    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

    index = resolve()
    print("resolve", index, flush=True)
    camera_fallback.install({index["left"]: "left", index["right"]: "right"})
    cams = {side: OpenCVCamera(OpenCVCameraConfig(index_or_path=index[side], width=640, height=480, fps=30))
            for side in ("left", "right")}
    for cam in cams.values():
        cam.connect()
    print(f"[watch {time.strftime('%H:%M:%S')}] 감시 시작", flush=True)
    start = last_beat = time.monotonic()
    try:
        while time.monotonic() - start < a.seconds:
            for cam in cams.values():
                cam.read_latest()
            time.sleep(1 / 30)
            if time.monotonic() - last_beat >= 30:
                last_beat = time.monotonic()
                state = {side: "멈춘 화면" if cam._fb_down_since is not None else "정상" for side, cam in cams.items()}
                print(f"[watch {time.strftime('%H:%M:%S')}] {state}", flush=True)
    finally:
        print(f"[watch {time.strftime('%H:%M:%S')}] 감시 끝", flush=True)
        for cam in cams.values():
            cam.disconnect()


if __name__ == "__main__":
    main()
