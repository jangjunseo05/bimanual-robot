# 2026-11-03 급수대 접근 중 깊이 입력 지연

## 결론과 조사 상태

급수대 접근에서 입력 대기·재개와 느린 제어 경로 계산이 관측됐다. Wi-Fi 혼잡은 가능한 원인이지만, 당시 송수신 지연·무선 채널 점유율을 확보하지 못해 원인으로 확정하지 않았다. 최종 주행 종료 사유는 별도의 `table lateral alignment requires more clearance`였다.

이 문서는 기록과 현행 소스의 대조 결과다. 깊이 지연 해결 완료나 전체 시나리오 성공을 뜻하지 않는다. 이번 문서 작성에서는 실행 코드·센서 유효 기준을 변경하거나 로봇을 움직이지 않았다.

## 대상 회차와 근거

| 항목 | 값 |
| --- | --- |
| 주문 | `261103002` |
| PC 미션 | `20261103_153829_d723e714` |
| 급수대 실행 | `water_station_20261103_153855_e23ce186` |
| 당시 로컬 코드 HEAD | `f5d21a1` |
| PC 실행 로그 | `$HOME/.local/state/bimanual-robot/water-service/runs/20261103_153829_d723e714/controller.log` |
| Pi 상세 이벤트 원본 | `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/water_station_20261103_153855_e23ce186/cycle_events.jsonl` |
| PC 로그 SHA256 | `db2ef13eb484af90e60cfb9ab14068a3266833ae946f19c83371847e9a8b4df6` |

집계는 PC `controller.log`에서 추출한 JSON 이벤트 385건을 대상으로 했다. 원본 로그·영상은 저장소에 넣지 않는다. 조사 시점에 Pi SSH는 `No route to host`로 연결되지 않아 상세 이벤트의 수치 필드까지 회수하지 못했다. 이 접속 실패는 조사 시점의 상태이며 과거 주행 중 Wi-Fi 장애의 증거로 쓰지 않는다.

## 확인한 현상

| PC 로그 이벤트 | 건수 | 해석 범위 |
| --- | ---: | --- |
| `table_approach_waiting_for_input` / `table approach depth stale or future-dated` | 5 | 깊이 기반 관측의 시각 유효성 때문에 대기에 들어간 기록 |
| `table_approach_waiting_for_input` / `table approach tape reference pending` | 4 | 테이프 목표 확보·확정 대기. 깊이 지연과 구분 |
| `table_approach_latency_breakdown` | 24 | 관측 시각 판정의 진단 기록. 서로 다른 정지 24회를 뜻하지 않음 |
| `table_control_validation_slow` | 29 | 제어 경로 검증이 50ms 이상 걸린 기록 |
| `table_approach_late_frame_skipped` | 9 | 늦게 끝난 관측 결과를 건너뛴 기록. 네트워크 패킷 손실 건수가 아님 |
| `table_approach_input_recovered` | 7 | 입력 대기에서 복구한 기록. 특정 원인과 일대일 대응시키지 않음 |
| `table_lateral_restage` | 1 | 목표 재기준 뒤 측면 재정렬 |
| `failed` | 1 | `final_approach`: `table lateral alignment requires more clearance` |

PC 로그에는 15:40:49에 위 좌우 정렬 사유로 실행 종료가 보고됐다. 입력 지연 대기에서는 재개했지만, 이후 좌우 정렬 실패로 미션이 끝난 회차다. 같은 로컬 로그 배치에 찍힌 출력 시각을 개별 이벤트 발생 시각으로 간주해 대기 시간을 계산하지 않는다.

## ‘깊이 지연’이 의미하는 것

입력 경로는 다음과 같다.

```text
Windows Astra 촬영
  → RGB·깊이 압축 및 AST1 송신
  → Wi-Fi 전송
  → Pi 수신·시계 보정·복호화·ROS 발행
  → 책상 면 검출
  → 주차 제어기의 관측 사용·경로 검증
```

[`box_service.py`](../jdamr_cube_navigation/box_service.py)의 `_table_fine_approach`는 관측의 촬영 시각과 제어 시각을 비교한다. 기본 유효 범위는 `0 ≤ 관측 나이 ≤ 0.5초`다. 최신 깊이 입력과 유지 중인 면이 유효하면 이전 승인 관측을 이어 쓰거나 늦은 결과를 건너뛰는 예외가 있으므로, 0.5초 초과가 곧바로 모든 이동 종료를 뜻하지는 않는다.

`depth stale or future-dated`는 오래된 시각뿐 아니라 미래·무효 시각도 함께 나타내는 메시지다. PC 요약 로그에는 실제 `observation_age_s` 값이 없어, 5건 모두가 양의 0.5초 초과였다고 단정할 수 없다. 대화에서 사용한 ‘깊이 지연 5회’는 이 대기 사유의 약칭이다.

또한 이 판정에는 촬영·전송·검출·제어 단계의 누적 지연이 들어간다. 카메라 고장, Wi-Fi 지연, Pi 계산 지연 중 어느 하나를 가리키는 전용 오류가 아니다.

## 원인 후보와 확인 수준

| 후보 | 확보한 근거 | 아직 모르는 것 |
| --- | --- | --- |
| 무선망 혼잡·재전송 | 사용자가 당시 사람과 노트북·휴대폰의 Wi-Fi 사용량 증가를 보고. 영상 전송이 무선망을 경유 | 해당 회차의 RTT·재전송·채널 점유율·촬영→수신 지연. 혼잡의 실제 기여도는 미확정 |
| Pi 면 검출·제어 처리 지연 | 50ms 이상 제어 경로 검증 29건, 늦은 결과 건너뛰기 9건 | 검출·콜백·제어 계산 각각의 지연 비중과 CPU·메모리 병목 여부 |
| 시계 보정·타임스탬프 문제 | 오류 문구에 미래 시각도 포함됨 | 해당 시점의 관측 나이 부호, PONG RTT·시계 오프셋 변화. 문제 발생 증거는 아직 없음 |
| 좌우 목표·접근 경로의 불일치 | 재정렬 1회와 최종 공간 부족 오류 | 실제 테이프 좌우 오차와 목표 변화 원인. Wi-Fi 혼잡이 이 오류를 일으켰다고 연결할 근거는 없음 |

Wi-Fi는 같은 채널의 송수신이 많아지면 전송 대기와 지연이 늘 수 있다. 신호 세기가 좋아도 혼잡이 생길 수 있다는 일반적인 설명은 [Cisco의 무선 성능 진단 문서](https://www.cisco.com/c/en/us/support/docs/wireless/catalyst-9800-series-wireless-controllers/221766-validate-wi-fi-throughput-testing-and.html)와 [Ubiquiti의 airtime 혼잡 설명](https://help.ui.com/hc/en-us/articles/30266615758743-Resolving-Wi-Fi-Performance-Issues-Due-to-High-Airtime-Utilization)을 참고한다. 이 자료는 해당 회차의 원인 증명이 아니다. 앞선 대화에서 Ubiquiti 링크를 ‘Cisco 설명’으로 표기한 출처명도 여기서 정정한다.

## 다음에 원인을 분리할 때 볼 값

이미 구현된 진단 필드를 사용한다. 새 출발 차단 절차나 긴 검사를 추가하는 계획은 아니다.

| 관측 지점 | 기존 필드 | 구분할 내용 |
| --- | --- | --- |
| AST1 수신기 상태 | `receive_fps`, `accepted_fps`, `depth_capture_to_send_ms`, `depth_capture_to_receive_ms`, `decode_ms` | Windows 촬영·송신 대기, 전송·수신, 복호화 지연 |
| AST1 시계 보정 | `last_clock_rtt_ns`, `clock_rtt_ns`, `clock_offset_ns`, `clock_uncertainty_ns`, `clock_rejected` | RTT·보정 오차 또는 유효 동기화 표본 부족 |
| 주차 관측 진단 | `observation_age_s`, `observer_input_age_s`, `observer_processing_s`, `observer_calculation_s` | 촬영 시각부터의 나이와 관측 처리 구간 |
| 제어기 진단 | `status_callback_age_s`, `previous_control_validation_s`, `fresh_depth_available` | 콜백 전달 지연, 제어 계산 지연, 새 깊이의 도착 여부 |

수신기 필드는 [`astra_tcp_receiver.py`](../jdamr_cube_navigation/astra_tcp_receiver.py)의 `Receiver.current_status()`에, 주차 진단 필드는 [`box_service.py`](../jdamr_cube_navigation/box_service.py)에 있다. 수신기 통계는 최근 30초 창이므로 다음 회차의 현재 값으로 이번 과거 회차를 소급 설명하지 않는다.

같은 시각의 촬영→수신 지연이 증가하면 전송 경로를, 수신은 정상인데 관측·계산 시간이 증가하면 Pi 처리 경로를 먼저 조사한다. 촬영→수신 시간에도 Windows 송신 전 대기와 시계 보정 오차가 들어갈 수 있으므로 그 값 하나만으로 Wi-Fi를 확정하지 않는다. 원인 구분 후에 전송량·무선망 배치 또는 검출·계산 경로 중 해당 부분을 조정한다.

## 집계 재현

주행이나 서비스 조작 없이 기존 PC 로그만 읽는다.

```bash
python3 - <<'PY'
import collections
import json
from pathlib import Path

path = Path.home() / (
    '.local/state/bimanual-robot/water-service/runs/'
    '20261103_153829_d723e714/controller.log'
)
counts = collections.Counter()
for line in path.read_text().splitlines():
    start = line.find('{')
    if start < 0:
        continue
    try:
        event = json.loads(line[start:])
    except json.JSONDecodeError:
        continue
    if isinstance(event, dict) and 'event' in event:
        counts[event['event'], event.get('reason')] += 1
for (event, reason), count in sorted(counts.items(), key=lambda item: str(item[0])):
    if any(word in event for word in ('latency', 'waiting', 'recovered', 'slow', 'late_frame', 'restage', 'failed')):
        print(event, reason, count)
PY
```

## 완료한 일과 남은 문제

- 완료: PC 로그 재집계, 현행 소스의 판정과 대조, 원인 후보와 부족한 증거 정리.
- 미해결: 해당 회차의 전송·검출·제어 지연 비중, 마지막 좌우 정렬 실패의 근본 원인, 정지 없는 재주행의 실차 확인.
- 이 기록을 Wi-Fi 원인 확정, Pi CPU 부족 확정, 깊이 지연 수정 완료로 해석하지 않는다.
