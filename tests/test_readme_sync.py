from tools.ci.sync_readme_status import END, START, render, synchronize


def sample_spec() -> dict:
    return {
        "revision": "test_revision",
        "status": "test_status",
        "chassis": {
            "frame_outer_footprint": [340.0, 450.0],
            "plates": {"tabletop": {"z_top": 720.0}},
        },
        "left_cup_gripper": {"selected_attachment": None},
        "right_parallel_gripper": {"model": "right_test"},
        "camera": {"model": "camera_test", "height_above_tabletop": 250.0},
        "navigation": {
            "drive_type": "differential_test",
            "reuse_source": {"motor": "motor_test"},
        },
    }


def test_render_uses_single_sources() -> None:
    output = render(sample_spec())
    assert "450×340 mm" in output
    assert "SO-101 순정 회전식 죠, 무빙 죠만 TPU, 오버캡 없음" in output
    assert "FinRay 2개" not in output
    assert output.count(START) == 1
    assert output.count(END) == 1


def test_synchronize_inserts_and_replaces_generated_block() -> None:
    readme = "# title\n\n## 다음 세션 최우선 작업\nbody\n"
    first = synchronize(readme, render(sample_spec()))
    second = synchronize(first, render(sample_spec()))
    assert first == second
    assert first.index(START) < first.index("## 다음 세션 최우선 작업")


def test_render_rejects_left_attachment() -> None:
    spec = sample_spec()
    spec["left_cup_gripper"]["selected_attachment"] = "finray"
    try:
        render(spec)
    except ValueError as error:
        assert "추가 부착물" in str(error)
    else:
        raise AssertionError("왼손 추가 부착물이 자동 요약에 통과함")
