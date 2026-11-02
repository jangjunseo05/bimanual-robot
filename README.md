# HOLD THE FLOW · 이동형 양팔 물 서빙 로봇

웹 주문을 받아 급수대에서 물을 따르고, 테이블에 컵을 놓은 뒤 도크로 돌아오는 로봇이다.
동국대 DAPIER 피지컬 AI 부트캠프 MS Team의 공용 저장소로, 실차 주행과 양팔 조작,
웹 관제를 함께 관리한다.

<p align="center">
  <img src="docs/assets/full_size_frame_20260910/product_concept_imagegen_v2.png" width="640" alt="이동형 양팔 물 서빙 로봇의 부품 배치 콘셉트">
</p>

위 이미지는 배치 콘셉트다. 제작 기준은 [기구 설계](design/mechanical/hold_flow_mechanical_v0_3.yaml)와
[URDF 설명](src/description/hold_flow_description/README.md)에 있다.

## 현재 동작과 남은 확인

| 영역 | 현재 상태 |
|---|---|
| 실차 주행 | 4WD 베이스, Cartographer 지도 생성, AMCL 위치추정, NavFn 경로·RPP 추종 |
| 정밀주차 | 급수대 전면 두 검은 테이프로 좌우 정렬하고 RGB-D 면으로 접근, 테이블1은 면 정렬, 도크는 후면 LiDAR 박스 도킹. 급수대·테이블1·도크 주차를 사용자가 수용한 기록이 있음 |
| 양팔 조작 | SmolVLA 80 물따르기·컵배치 모델과 Windows 실행기. 물따르기·컵배치까지 통합 2회 연속 통과(사용자 전달), 최종 후면 도킹 개선 중 |
| 주문 연동 | 주문 → 팔 접기 → 급수대 주차 → 물따르기 → 테이블1 주차 → 컵배치 → 도크 복귀 연결 |
| 장애물 대응 | 전방 차체 바깥 40cm 정지 구역과 정지·대기·동일 경로 재시도 설정. 변경 범위의 실물 끼어들기 확인은 남아 있음 |
| 센서 연결 | Windows가 Astra를 사용하며 RGB·정렬 깊이·CameraInfo를 AST1 TCP로 Pi에 전달 |
| 시뮬레이션 | Isaac Sim 5.1에서 파지·붓기·운반·배치·복귀 미션을 별도로 실행 |

실차 주차의 사용자 수용, 센서 내부 추정, 외부 실측은 구분한다. 테이프 주차 반복과
도크 출발 후 급수대·테이블1 주차와 물따르기·컵배치까지 2회 연속 통과한 결과를
전달받았다. 최종 후면 도킹은 아직 남아 있어 전체 왕복 성공과 구분한다.
상세 근거는 [PROGRESS](PROGRESS.md), 같은 구성으로 시작하는 방법은
[팀원 실행 절차](tools/jdamr/README.md#팀원-실행-절차)에 있다.

## 시연 구성

```text
웹 주문 / 주행 PC의 출발 명령
  → Windows 주문 서버: 팔 접기와 목적지 허용
  → Pi: 급수대 이동·정밀주차
  → 정렬 완료 알림 → Windows: 물따르기·이동 자세
  → Pi: 테이블1 이동·면 주차
  → 정렬 완료 알림 → Windows: 컵배치·팔 접기
  → Pi: 도크 복귀·후진 주차
```

| 장치 | 역할 | 연결 |
|---|---|---|
| OMEN 16 Windows | 주문 서버·SmolVLA 추론·Astra 송신 | `192.168.0.157:8765`, 조작 실행기 `127.0.0.1:8770` |
| Raspberry Pi | 센서·TF·SLAM/Nav2·정밀주차·베이스 제어 | `192.168.0.159`, AST1 수신 `5600`, ROS_DOMAIN_ID `12` |
| 주행 PC | 미션 실행 API·운영 명령·관측 | `192.168.0.217`, 로컬 미션 API `127.0.0.1:8771` |

노트북 주소는 DHCP이므로 바뀌면 주행 PC의 `--order-url`과 Pi 수신기의 `--peer`를 함께 바꾼다.
허용 IP 확인은 유지한다. 8770 조작 실행기는 Windows 내부에서만 호출하고, 주행 PC는 주문
서버의 `/api/state`와 `/api/align/confirm`을 사용한다.

주행에 필요한 팔 이동 자세 확인과 카메라가 필요한 정책 추론을 구분한다. 팔 접기는 모터
관절값을 읽으며 손목 영상 때문에 대기하지 않는다. 카메라 송신기·관측기는 시나리오 동안
유지한다. 이미 측정한 정지 상태는 새 입력으로 이어 확인하고, 정지 중 일시적인 주문 서버
조회 실패는 같은 단계에서 재조회한다. 실제 충돌·센서/통신 소실·팔 과부하 보호는 유지한다.

## 실행 진입점

실차 코드는 [src/navigation](src/navigation/README.md), 운영 명령은
[tools/jdamr](tools/jdamr/README.md)에 있다. 지도와 등록 위치는
`$HOME/jdamr_data/maps/current_manual_mapping.json`이 가리키는 실행 자산을 사용한다.
이전 지도·박스 주행은 `--legacy-map`으로 보존한다.

주문 서버 주소를 지정해 주행 PC의 미션 API를 켠다. 이 명령은 출발이나 팔 동작을 시작하지 않는다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/water_service_api.py serve --order-url http://192.168.0.157:8765
```

Windows에서는 충전기를 연결하고 팀 기준 저장소의 `tools/rollout/start_scenario.bat`을
일반 창에서 실행한다. `AMRArmService`·`AMRAstraCamera`·`AMROrderServer` 예약 작업을
SSH로 재기동하지 않는다. 두 정책의 로딩·워밍업과 노트북 담당자의 준비 완료 확인을
주행 전에 끝낸다. 첫 주문 이후 팔 접기가 끝나면 `arms_ready`가 활성화된다.

별도 터미널에서 상태를 확인하고, 실물 실행 요청이 있을 때 출발한다. Windows 장치 연결과
실행기 ready를 확인하기 전에는 `start`를 호출하지 않는다. 이미 systemd로 켜져 있으면
서버를 중복 실행하지 않는다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/water_service_api.py status
python3 tools/jdamr/water_service_api.py start --request-id demo-order-id
python3 tools/jdamr/water_service_api.py stop
```

| 필요한 내용 | 안내 |
|---|---|
| Pi 설치·배포·실차 운영 | [주행 사용법](tools/jdamr/README.md), [런타임 구성](docs/guides/JDAMR_RUNTIME.md) |
| Astra RGB-D 전송·시각 동기화 | [AST1 송수신 계약](tools/jdamr/ASTRA_TCP.md) |
| 지도·급수대·테이블·도크 좌표 | [강의실 지도와 위치](config/navigation/maps/classroom/README.md) |
| Windows 환경·장치 이전 | [새 노트북 인계](docs/guides/20261030_새노트북_작업인계.md) |
| 주문·주행·조작 API 순서 | [통합 구현 명세](docs/guides/20261029_주행_조작_연동_구현명세.md) |
| 팔 보정·모델·데이터 | [보정](calibration/README.md), [데이터 인덱스](data/README.md) |
| 시뮬레이션·실험·발표 | [문서 지도](docs/README.md), [Isaac 시뮬레이션](simulation/README.md) |

## 하드웨어

<!-- AUTO:PROJECT-STATUS:START -->
## 단일 원본 자동 요약

> 아래 표는 기계 YAML에서 생성한다. 수정하려면 표가 아니라 연결된 원본 파일을 고친다.

| 항목 | 현행 값 |
|---|---|
| 기계 개정 | `2026-10-14_final_four_wheel_drive_selected` |
| 상·하부 프레임 | 450×340 mm |
| 상판 높이 | 720 mm |
| 왼손 | SO-101 순정 회전식 죠, TPU·오버캡 없음 |
| 오른손 | `ggao50_SO101_Parallel_Gripper` |
| RGB-D | Orbbec_Astra_S, 상판 위 540 mm |
| 베이스 | skid_steer_four_wheel_drive, Feetech_STS3215_C018_12V |
| 모델 상태 | `simulation_model_ready_physical_measurements_pending` |

원본: [`hold_flow_mechanical_v0_3.yaml`](design/mechanical/hold_flow_mechanical_v0_3.yaml)
과거 후보: [`finray_80mm_lip3_selection.yaml`](design/gripper/finray_80mm_lip3_selection.yaml)
<!-- AUTO:PROJECT-STATUS:END -->

실차는 4륜 스키드스티어와 ESP32 구동, Raspberry Pi 4B, G4 LiDAR, Astra RGB-D,
SO-101 양팔을 사용한다. 주행 모델의 원점은 전륜축이며 설계 모델의 차체 중심 원점과
혼용하지 않는다. 카메라 장착·팔 보정은 실물 설정과 맞춰 사용한다.
뎁스 포인트클라우드 기반 후보 분리·추적은 [선택형 개발 경로](docs/guides/AMR_POINTCLOUD_DOCKING.md)이며
기본 주차 제어와 구분한다.

## 저장소 구조

| 경로 | 내용 |
|---|---|
| [src](src/README.md) | 주행·양팔 조작·인식·미션·URDF |
| [apps](apps/README.md) | 웹 주문·관제·카메라 화면 |
| [tools](tools/README.md) | Pi 운영·카메라·학습·추론·설치 도구 |
| [simulation](simulation/README.md) | Isaac 장면·접촉·왕복 실행 |
| [design](design/cad/README.md) | 기구 사양·CAD·출력물 |
| [data](data/README.md) | 외부 데이터·모델 인덱스·평가 규약 |
| [docs](docs/README.md) | 실행법·설계·실험·회의 기록 |

개발 기준은 Ubuntu 24.04·ROS 2 Jazzy·Python 3.12이며, Windows 조작 환경과 Isaac Sim 5.1은
각 실행 문서의 별도 환경을 따른다. 데이터셋·모델 가중치·rosbag은 Git에 넣지 않는다.
변경은 목적별 작업 브랜치에서 작성하고 PR로 `main`에 반영한다.
