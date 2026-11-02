"""카메라 끊김 보호: 카메라·lerobot 없이 가짜 카메라로 재연결 시점을 확인한다."""
import functools
import importlib.util
import sys
import time
from pathlib import Path
from types import ModuleType

import pytest

ROLLOUT_DIR = Path(__file__).resolve().parents[2] / "tools" / "rollout"


def check_if_already_connected(func):
    """lerobot 데코레이터 흉내: 연결된 것으로 보이면 connect를 거부한다."""

    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.is_connected:
            raise RuntimeError("already connected")
        return func(self, *args, **kwargs)

    return wrapper


class FakeCamera:
    """OpenCVCamera 흉내: 끊기면 읽기가 실패하고, 다시 연결해도 첫 프레임까지 시간이 걸린다."""

    connect_s = 0.0
    first_frame_s = 0.0

    def __init__(self):
        self.index_or_path = 2
        self.running = True
        self.fail_connect = False
        self.connects = 0
        self.first_frame_at = 0.0

    @property
    def is_connected(self):
        return self.running

    def read_latest(self, max_age_ms=500):
        if not self.running:
            raise RuntimeError("read thread is not running.")
        if time.monotonic() < self.first_frame_at:
            raise RuntimeError("has not captured any frames yet.")
        return f"frame-{self.connects}"

    def _cleanup_resources(self):
        self.running = False

    @check_if_already_connected
    def connect(self, warmup=True):
        self.connects += 1
        time.sleep(self.connect_s)
        if self.fail_connect:
            raise RuntimeError("failed to set fps=30 (actual_fps=0.0).")
        self.first_frame_at = time.monotonic() + self.first_frame_s
        self.running = True


@pytest.fixture
def fallback(monkeypatch):
    # install()이 클래스를 고치므로 시험마다 새 클래스를 쓴다
    camera_class = type("OpenCVCamera", (FakeCamera,), {"read_latest": FakeCamera.read_latest})
    module = ModuleType("lerobot.cameras.opencv.camera_opencv")
    module.OpenCVCamera = camera_class
    monkeypatch.setitem(sys.modules, module.__name__, module)
    spec = importlib.util.spec_from_file_location("camera_fallback_under_test", ROLLOUT_DIR / "camera_fallback.py")
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    loaded.RECONNECT_EVERY_S = 0.2
    loaded.install({})
    return loaded, camera_class


def wait_reconnect_done(cam):
    deadline = time.monotonic() + 2.0
    while cam._fb_reconnecting:
        assert time.monotonic() < deadline
        time.sleep(0.005)


def read_until(cam, frame, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cam.read_latest() == frame:
            return True
        time.sleep(0.005)
    return False


def test_failed_reconnect_does_not_disconnect_the_robot(fallback):
    _, camera_class = fallback
    cam = camera_class()
    assert cam.read_latest() == "frame-0"

    cam.running, cam.fail_connect = False, True
    assert cam.read_latest() == "frame-0"
    wait_reconnect_done(cam)
    assert cam.connects == 1
    assert cam.is_connected is True  # 팔 쪽 is_connected가 거짓이 되면 정책 실행이 끊긴다
    assert cam.read_latest() == "frame-0"

    cam.fail_connect = False
    assert read_until(cam, "frame-2")
    assert cam._fb_down_since is None


def test_reconnect_waits_for_the_first_frame(fallback):
    # 2026-10-31: 재연결이 간격보다 오래 걸려 첫 프레임 전에 다시 끊는 일이 10회 반복됐다
    _, camera_class = fallback
    camera_class.connect_s, camera_class.first_frame_s = 0.25, 0.05
    cam = camera_class()
    assert cam.read_latest() == "frame-0"

    cam.running = False
    assert read_until(cam, "frame-1")
    assert cam.connects == 1


def test_recovered_camera_reports_its_real_state(fallback):
    _, camera_class = fallback
    cam = camera_class()
    cam.read_latest()
    cam.running = False
    cam.read_latest()
    assert read_until(cam, "frame-1")
    cam._cleanup_resources()
    assert cam.is_connected is False
