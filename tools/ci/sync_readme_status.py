#!/usr/bin/env python3
"""기계 단일 원본에서 README 최상단의 자동 상태 표를 생성한다."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


START = "<!-- AUTO:PROJECT-STATUS:START -->"
END = "<!-- AUTO:PROJECT-STATUS:END -->"


def render(spec: dict) -> str:
    footprint_x, footprint_y = spec["chassis"]["frame_outer_footprint"]
    left = spec["left_cup_gripper"]
    if left["selected_attachment"] is not None:
        raise ValueError("현행 왼손 순정 죠에 추가 부착물이 설정됐다")
    navigation = spec["navigation"]
    return "\n".join(
        [
            START,
            "## 단일 원본 자동 요약",
            "",
            "> 아래 표는 기계 YAML에서 생성한다. "
            "수정하려면 표가 아니라 연결된 원본 파일을 고친다.",
            "",
            "| 항목 | 현행 값 |",
            "|---|---|",
            f"| 기계 개정 | `{spec['revision']}` |",
            f"| 상·하부 프레임 | {footprint_y:.0f}×{footprint_x:.0f} mm |",
            f"| 상판 높이 | {spec['chassis']['plates']['tabletop']['z_top']:.0f} mm |",
            "| 왼손 | SO-101 순정 회전식 죠, 무빙 죠만 TPU, 오버캡 없음 |",
            f"| 오른손 | `{spec['right_parallel_gripper']['model']}` |",
            f"| RGB-D | {spec['camera']['model']}, 상판 위 {spec['camera']['height_above_tabletop']:.0f} mm |",
            f"| 베이스 | {navigation['drive_type']}, {navigation['motor']['model'] if 'motor' in navigation else navigation['reuse_source']['motor']} |",
            f"| 모델 상태 | `{spec['status']}` |",
            "",
            "원본: [`hold_flow_mechanical_v0_3.yaml`](design/mechanical/hold_flow_mechanical_v0_3.yaml)",
            "과거 후보: [`finray_80mm_lip3_selection.yaml`](design/gripper/finray_80mm_lip3_selection.yaml)",
            END,
        ]
    )


def synchronize(readme: str, generated: str) -> str:
    if START in readme or END in readme:
        if readme.count(START) != 1 or readme.count(END) != 1:
            raise ValueError("README 자동 요약 마커가 불완전하거나 중복됐다")
        prefix, remainder = readme.split(START, 1)
        _, suffix = remainder.split(END, 1)
        return prefix.rstrip() + "\n\n" + generated + suffix
    anchors = ("## 현재 최우선 작업", "## 다음 세션 최우선 작업")
    anchor = next((candidate for candidate in anchors if candidate in readme), None)
    if anchor is None:
        raise ValueError(f"README 삽입 기준 제목을 찾지 못했다: {anchors}")
    before, after = readme.split(anchor, 1)
    return before.rstrip() + "\n\n" + generated + "\n\n" + anchor + after


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    root = args.repo.resolve()
    readme_path = root / "README.md"
    spec_path = root / "design/mechanical/hold_flow_mechanical_v0_3.yaml"
    spec = yaml.safe_load(spec_path.read_text(encoding="utf-8"))
    current = readme_path.read_text(encoding="utf-8")
    expected = synchronize(current, render(spec))
    if current == expected:
        print("README automatic status OK")
        return 0
    if args.write:
        readme_path.write_text(expected, encoding="utf-8")
        print("README automatic status updated")
        return 0
    print("README automatic status is stale; run: python tools/ci/sync_readme_status.py --write")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
