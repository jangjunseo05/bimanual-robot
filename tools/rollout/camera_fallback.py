"""카메라가 순간적으로 끊겨도 롤아웃·상주 실행기가 멈추지 않게 하는 보호막.

lerobot OpenCVCamera.read_latest()는 최신 프레임이 0.5초보다 오래되면 TimeoutError를 낸다.
2026-10-30 실차 통합 시험에서 주행 진동으로 손목캠·Astra USB가 몇 차례 빠졌다 붙었고,
그때마다 관측 읽기가 실패해 팔 접기·정책 실행이 중단됐다.

install()은 read_latest를 감싸서
1. 새 프레임을 못 받으면 마지막으로 받은 정상 프레임을 그대로 돌려주고(끊긴 동안 화면은 멈춘 채로 보인다),
2. 뒤에서 그 카메라를 다시 연결한다(USB 재연결 뒤 장치 번호가 바뀌면 물리 경로로 번호를 다시 찾는다).
정상 프레임이 한 번도 없으면 원래대로 오류를 낸다.

한계: 끊긴 동안 정책은 멈춘 화면을 본다. 짧은 끊김은 버티지만, 오래 끊기면 정책 판단이 틀릴 수 있다.
카메라가 끝내 돌아오지 않아도 실행은 스스로 멈추지 않으니 그때는 STOP으로 끊는다.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

RECONNECT_EVERY_S = 2.0


def _log(msg: str) -> None:
    print(f"[camera {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _resolve_index(role: str | None):
    """역할(left/right/depth)로 지금의 장치 번호를 다시 찾는다. 실패하면 None."""
    if role is None:
        return None
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera"))
        from resolve_camera_indices import resolve

        return resolve().get(role)
    except Exception as e:  # noqa: BLE001
        _log(f"장치 번호 다시 찾기 실패({e})")
        return None


def install(role_by_index: dict[int, str]) -> None:
    """role_by_index: 시작할 때 찾은 {장치 번호: 'left'|'right'|'depth'}."""
    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

    original = OpenCVCamera.read_latest
    if getattr(original, "_fallback_installed", False):
        return

    def reconnect(cam) -> None:
        try:
            role = cam._fb_role
            new_index = _resolve_index(role)
            try:
                cam._cleanup_resources()
            except Exception:  # noqa: BLE001
                pass
            if new_index is not None and new_index != cam.index_or_path:
                _log(f"{role} 장치 번호 {cam.index_or_path} -> {new_index}")
                cam.index_or_path = new_index
            # 다시 연결하는 동안 is_connected를 True로 두므로(아래), 이미 연결됨 검사를 건너뛰는 원래 함수를 부른다
            plain = cam.__dict__.get("connect") or getattr(OpenCVCamera.connect, "__wrapped__", None)
            if plain is cam.__dict__.get("connect"):
                plain(warmup=False)
            elif plain is not None:
                plain(cam, warmup=False)
            else:
                cam.connect(warmup=False)
            _log(f"{cam} 다시 연결됨")
        except Exception as e:  # noqa: BLE001
            _log(f"{cam} 다시 연결 실패({e}) - {RECONNECT_EVERY_S:.0f}초 뒤 재시도")
        finally:
            # 연결이 끝난 시점부터 잰다. 시작 시점부터 재면 연결(1~2 s) 뒤 첫 프레임(0.3 s) 전에 다시 끊는다
            cam._fb_next_try = time.monotonic() + RECONNECT_EVERY_S
            cam._fb_reconnecting = False

    def read_latest(self, max_age_ms: int = 500):
        if not hasattr(self, "_fb_role"):
            self._fb_role = role_by_index.get(self.index_or_path)
            self._fb_last = None
            self._fb_down_since = None
            self._fb_next_try = 0.0
            self._fb_reconnecting = False
        try:
            frame = original(self, max_age_ms)
        except Exception as e:  # noqa: BLE001 - 끊김(TimeoutError·RuntimeError) 모두 같은 처리
            if self._fb_last is None:
                raise
            now = time.monotonic()
            if self._fb_down_since is None:
                self._fb_down_since = now
                _log(f"{self}({self._fb_role}) 프레임 끊김({e}) - 마지막 프레임으로 계속, 다시 연결 시도")
            if not self._fb_reconnecting and now >= self._fb_next_try:
                self._fb_reconnecting = True
                threading.Thread(target=reconnect, args=(self,), daemon=True, name=f"{self}_reconnect").start()
            return self._fb_last
        if self._fb_down_since is not None:
            _log(f"{self}({self._fb_role}) 복구 - {time.monotonic() - self._fb_down_since:.1f}초 동안 멈춘 화면 사용")
            self._fb_down_since = None
        self._fb_last = frame
        return frame

    read_latest._fallback_installed = True
    OpenCVCamera.read_latest = read_latest

    # 카메라 하나가 끊긴 동안 로봇 전체가 "연결 안 됨"으로 판정돼 관측·명령이 거부되지 않게,
    # 복구될 때까지(다시 연결이 실패해 다음 시도를 기다리는 동안 포함) 연결된 것으로 본다(그동안 read_latest는 마지막 프레임을 준다).
    original_is_connected = OpenCVCamera.is_connected

    def is_connected(self) -> bool:
        return getattr(self, "_fb_down_since", None) is not None or original_is_connected.fget(self)

    OpenCVCamera.is_connected = property(is_connected)
