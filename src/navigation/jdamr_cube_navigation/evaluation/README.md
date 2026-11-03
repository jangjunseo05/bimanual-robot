# SLAM 평가 Phase 0

이 디렉터리는 실제 로봇을 움직이기 전에 데이터와 실험 조건을 고정하는 오프라인 게이트다. 여기서 통과했다고 해서 센서 보정이나 실제 주행 안전이 검증된 것은 아니다.

## 구성

- [급수대 깊이 입력 지연 트러블슈팅](20261103_DEPTH_LATENCY_TROUBLESHOOTING.md):
  주문 261103002의 대기·재개와 계산 지연, Wi-Fi 혼잡 가설, 원인 분리에 필요한 기존 진단 필드
- [고정 사선 카메라 관제 메인 영상](media/gazebo_fixed_dashboard_20260909/README.md):
  실제 문에서의 횡단, 금지 구역·박스 우회, 동일 목표 재개 PASS와 동기화된 화이트 관제
- [고정 카메라 재촬영 인계](20260909_FIXED_CAMERA_HANDOFF.md):
  횡단 배치 시간 보정, 변경 범위와 검증 근거
- [금지 구역·장애물 대응 관제 영상](media/gazebo_keepout_dashboard_20260909/README.md):
  기존 추종 카메라의 Gazebo·라이다·위치·계획 경로·속도 기록 보관본,
  실제 마스크 격자와 회전 차체 외곽선의 이격 검증 및 원본 해시
- [20260908_PURPOSE_AND_RUNTIME_AUDIT.md](20260908_PURPOSE_AND_RUNTIME_AUDIT.md):
  현재 목적·완료 증거, 온보드 장애물 후보·goal UUID 기록·경량 후처리 구현,
  G005 오정지 수정과 제한시간 진단
- [20260921_NAVIGATION_JD_GAP_ROADMAP.md](20260921_NAVIGATION_JD_GAP_ROADMAP.md):
  물류 AMR 자율주행 SW 채용 요건 대조, 보유 증거 표, 고도화 항목 P1~P6·A1~A2의
  계약·구현 위치·완료 조건과 진행 현황
- [20260929_PARKING_FAILURES.md](20260929_PARKING_FAILURES.md):
  테이블 박스 정밀 주차의 실차 실패, 코드 경로 결함 수정, 파이 반영과 SHA 대조,
  소프트웨어·배치·실차를 구분한 남은 확인 항목
- [20260929_NAV2_PLATFORM_RESEARCH.md](20260929_NAV2_PLATFORM_RESEARCH.md):
  파이에서 Nav2·RGB-D·관제를 함께 돌릴 때의 부하·통신 구성 조사와 판단 한계
- [20260930_CLAUDE_HANDOFF.md](20260930_CLAUDE_HANDOFF.md):
  2026-09-30 도구 간 인계 시점의 미수정 문제·검증 빈틈·데이터 위치
- `depth_box_parking_provenance.yaml`: 박스 관측기 파라미터와 관측 status 간격 실측
  (`observer_status_timing_20260929`)의 출처
- [20260908_DYNAMIC_OBSTACLE_READINESS.md](20260908_DYNAMIC_OBSTACLE_READINESS.md):
  수납 팔 collision 외곽 기반 보호영역, 실제 온보드 후보의 정지·동일 목표 재개 PASS,
  compact MCAP과 공간 중심 미디어
- `experiment_manifest.schema.json`: 실험마다 남겨야 할 환경·버전·센서·TF·결과·산출물 계약
- `datasets.yaml`: rosbag의 해시, 토픽 수, 무결성, 사용 가능 범위
- `map_registry.yaml`: 지도 후보·게시·폐기 상태와 승격 조건
- `calibration_registry.yaml`: 센서 외부 파라미터와 시간 기준의 검증 상태
- `mcap_writer_options.yaml`: 다음 MCAP 기록에서 CRC와 인덱스를 보존하는 저장 설정
- `qos_overrides.yaml`: 기록·재생에 공통으로 사용할 명시적 QoS
- `phase0_status.yaml`: 현재 오프라인 게이트와 SO101 경계 감사 결과
- `20260907_SLAM_ADVANCEMENT_PLAN.md`: 완료된 증거와 G002 실패 원인 개선·G005
  구성요소 대조까지 연결한 실행 순서
- `20260907_G005_VERTICAL_READINESS.md`: 실제 Gazebo·Nav2 기반 G005 실행 경로의
  검증 결과, 주장 범위와 full15 실행 절차
- `inspect_mcap.py`: ROS 노드와 재생 없이 MCAP 전체 메시지와 CRC를 읽는 검사 도구
- `corridor_run_media.py`: 주행 이벤트·지표와 CSV·PNG·GIF·MP4를 MCAP에서 추출하는 도구.
  `--metrics-only`로 영상 재생성 없이 지표와 CSV만 저장 가능
- `make_sim_sensor_variant.py`: URDF 센서율·노이즈를 허용 목록 안에서 바꾸고 해시 manifest를 만드는 도구
- `run_sim_slam_experiment.py`: 격리 Gazebo, backend, MCAP, 왕복 경로와 ATE/RPE를 한 번에 실행하는 도구
- `compare_sim_slam_experiments.py`: 동일 조건의 backend·센서 stress 행렬을 검증하고 비교 자료를 만드는 도구
- `portfolio_capture_world.py`: 고정 월드 카메라와 촬영용 시각 요소를 추가하고,
  양쪽 복도 벽에 보행자 출입구를 만든다. 변경된 벽과 나머지 충돌 형상의 보존 여부를 별도로 기록한다.
- `record_simulator_camera.py`: Gazebo 카메라 센서 프레임을 메타데이터와 함께 MP4로 기록하는 도구
- `render_simulator_portfolio_media.py`: 검증 이벤트와 3D 영상을 결합해 방향별 하이라이트를 만드는 도구
- `render_bidirectional_reel.py`: 해시가 확인된 좌·우 하이라이트만 하나의 대표 영상으로 연결하는 도구
- `render_mujoco_nav2_evidence.py`: PASS MCAP의 Nav2 경로·실제 이동·장애물 이벤트를
  3D 추종 시점과 관제 HUD로 재생하는 자원 제한형 MuJoCo 렌더러
- `setup_mujoco_renderer.sh`, `run_mujoco_portfolio_render.sh`: 격리된 MuJoCo 환경 설치와
  재현 실행 진입점
- `navigation_dashboard.py`: LiDAR·Nav2·Collision Monitor·검증 영상을 localhost에서 보여주는 관측 전용 대시보드
- `export_dashboard_replay.py`: MCAP 관측을 영상의 wall-time 축으로 추출한다.
  영상 SHA-256을 확인하며 탐색·일시정지 시 미래 관측값을 사용하지 않는다.
- `../launch/offline_replay_guard.launch.py`: 저장 지도와 이동 명령을 재생하지 않고 AMCL `map -> odom`을 제거하는 launch

## 관제와 행동 로직의 증거 범위

실제 주행은 `navigate_to_pose_dynamic_obstacle_eval.xml`의 Nav2 행동 트리로
경로 계획·추종·재계획을 실행한다. 실패 시 costmap 정리와 대기를 수행하며,
Collision Monitor는 행동 트리 바깥에서 최종 속도를 감독한다.

데이터 흐름은 controller → `/cmd_vel_nav` → velocity smoother →
`/cmd_vel_smoothed` → Collision Monitor → `/cmd_vel`이다. 현재 bag에는
중간 두 속도 토픽이 없으므로 관제에서 해당 연결은 구조 설명이지 실시간 수신 증거가 아니다.
`/scan`, `/odom`, `/plan`, `/collision_monitor_state`, 목표 action 상태는 기록으로 확인한다.

관제의 관측 상태는 로봇을 제어하는 새 상태머신이 아니다. 목표 수락·주행·정지·재개·도착을
기록에서 보여주는 수동 관측 계층이다. 개별 BT 노드의 tick 상태는 아직 기록하지 않으므로
실행 중인 노드처럼 표시하지 않는다. 하역·충전·VDA 5050 order/action 연동도 현재 구현 범위 밖이다.

고정 카메라와 실제 출입구 설정의 코드 검증과 새 GPU 촬영은 별개다.
기존 추종 카메라 영상은 원본으로 보존하며 새 촬영 성공을 주장하는 데 사용하지 않는다.
GPU 촬영은 사용자에게 시작 전에 알리고 사용 보류가 해제된 뒤 진행한다.

### CPU 관제 재생 재현

`export_dashboard_replay.py --prepare-scene`는 원본 프레임 수와 카메라 시각을 확인한 뒤
CPU 인코더로 원래 시간 길이의 장면만 생성한다. 영상에 관제 UI를 다시 구워 넣지 않아 웹 화면과
중복되지 않는다. `.timing.json`의 원본·카메라 메타데이터·출력 해시와 실제 영상 길이를 확인하므로
속도를 줄인 하이라이트를 같은 시간축으로 연결할 수 없다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
python3 jdamr_cube_navigation/evaluation/navigation_dashboard.py \
  --no-ros --port 8765 \
  --summary jdamr_cube_navigation/evaluation/media/gazebo_keepout_dashboard_20260909/verified_run.json \
  --video "$HOME/jdamr_artifacts/gazebo_keepout_main_media_20260909_v02/dashboard_scene_walltime.mp4" \
  --replay "$HOME/jdamr_artifacts/gazebo_keepout_main_media_20260909_v02/dashboard_replay_walltime.json"
```

서버는 localhost만 열고 ROS 이동 명령을 발행하지 않는다. 실행 터미널의 `Ctrl+C`로 종료한다.
실시간 ROS를 선택했을 때 센서 관측이 끊기면 과거 목표·속도·STOP을 현재 상태로 표시하지 않는다.
브라우저 검증 스크립트는 `../test/dashboard_browser_check.mjs`이며, 별도로 실행한
GPU 비활성 headless Chrome의 localhost 디버깅 포트 9225를 사용한다.

## AMCL 재지역화 평가

G002는 실차 bag의 처리 비용을 비교하는 Axis A와, Gazebo ground truth가 있는 세 가지
재지역화 상황을 비교하는 Axis B로 나뉜다. `P0`는 현재 production 설정, `P1`은 입자 수를
고정한 대조군, `P2`는 recovery particle 주입 후보 설정이다. AMCL seed는 같은 입력에서
추정기 내부 난수 영향만 반복하기 위한 값이며 서로 다른 환경을 뜻하지 않는다.

최종 canonical 산출물은 다음과 같다.

- Axis A 15회: `/home/lim/jdamr_artifacts/amcl_axis_a_20260907_v06_full15`
- Axis B 입력 3개: `/home/lim/jdamr_artifacts/amcl_axis_b_inputs_20260907_v13`
- Axis B 45회: `/home/lim/jdamr_artifacts/amcl_axis_b_20260907_v02_full45`
- G009 실패 taxonomy: `/home/lim/jdamr_artifacts/amcl_failure_taxonomy_20260907_v01`

재검증은 용량을 중복하지 않기 위해 원본·정제 실차 bag, Axis B canonical 입력 3개와
production map·parameter 파일을 위 절대경로에서 다시 연다. 따라서 이 외부 입력을 경로와
SHA-256이 유지된 상태로 함께 보존해야 하며, 결과 디렉터리 하나만 옮기는 방식은 지원하지
않는다.

Axis B는 정상 초기화와 0.5m/15도 초기 오프셋을 세 프로필 모두 복구했지만, 8m 순간
이동 후 정답 초기 위치를 다시 주지 않는 kidnapped 상황은 모두 복구하지 못했다. 전체
45회에서 false convergence는 없었고 P2의 Axis A CPU 개별 최대 비율이 1.1543으로
1.10 상한을 넘었으므로 P0를 유지한다.

G009는 위 full45를 다시 검증한 뒤 15개 비복구, false convergence(실제 위치는
틀렸는데 추정 공분산만 낮아 맞다고 확신한 상태) 0개, 기존 1.10 자원 상한을 넘은 CPU
사례 2개를 taxonomy(원인 분류표)로 전수 분류했다. 비복구는 모두 kidnapped(초기 위치
정보를 다시 주지 않고 로봇 위치를 순간 이동시킨 상황)이다. 오위치 확신 수렴은 기록으로
배제됐고, 판정 기준 과보수는 이번 frozen 데이터에서 원인으로 뒷받침되지 않았다. 다만
frozen cloud에는 입자별
좌표·가중치와 재표본화 전후 후보 질량이 없으므로, 정답 주변 입자 부재와 정답 후보
소멸 중 하나를 원인으로 단정하지 않았다. CPU 비용 초과도 처리 지연의 인과 증거와
구분했다. 따라서 알고리즘 변경 gate를 열지 않고
`NO_FALSE_CONFIDENCE_OBSERVED_NO_CHANGE_JUSTIFIED`와 production P0(현재 운영 설정)를
동결했다.

G009 manifest SHA-256은
`3b373c56b66b537d938e05d4fe9e1f0bdf6647d1053c4d2d17309e2b7df096ce`, artifact tree
SHA-256은 `db4e9970c3b4148f0962de486b4d2f2c262f8af973ca99b2eed06e5369571b07`다.
다음 명령은 실차나 ROS graph를 기동하지 않고 기존 증거를 다시 검증한다.

```bash
cd $HOME/jdamr_cube_ws/src/jdamr_cube_ros
source /opt/ros/jazzy/setup.bash
PYTHONPATH="jdamr_cube_navigation/evaluation:$PYTHONPATH" \
  python3 jdamr_cube_navigation/evaluation/classify_amcl_axis_b_failures.py \
  --validate-manifest \
  $HOME/jdamr_artifacts/amcl_failure_taxonomy_20260907_v01/g009_manifest.json
```

각 실행은 다음을 hard gate로 검증한다.

1. 입력 bag에서 `map -> odom` 변환이 0건이다.
2. 격리 domain의 `/tf` 발행 endpoint는 `/amcl`과 `/rosbag2_player`뿐이다.
3. 관측된 `map -> odom` 변환은 1건 이상이다.
4. Gazebo Contact sensor의 ROS 발행자 1개를 확인하고 contact 메시지는 0건이다.
5. 실행 종료 시 최종 속도와 생존 프로세스가 0이다.
6. 평가 runner·evaluator·observer와 Axis B 입력 generator·driver의 source bytes를
   artifact의 `harness_sources/`에 보존한다.

여기서 kidnapped 결과는 자동 복구 행동을 시험한 것이 아니다. 미리 기록한 회전 관측을
동일하게 재생한 estimator-only 비교이며, 폐루프 정지·복구·목표 재개는 후속 단계에서
선택 후보가 있을 때만 별도로 검증한다.

## G005 프런티어 정책 평가

G005의 실제 실행 경로는 구현과 수직 스모크까지 완료했다. 고정 asset에서 실제 Gazebo와
Nav2 action을 기동하고, 지도·LiDAR·TF·lifecycle·`/cmd_vel` 권한·프로세스 자원을
run 전체에서 검증한 뒤 결과를 봉인한다. 짧은 수직 스모크에서는 Nav2 목표 수락, 비영
이동 명령, 10 Hz LiDAR 32/32 수락, runtime invalid 0, 잔존 프로세스 0을 확인했다.

정책 3개 × layout seed 5개의 full15 본평가는 아직 실행하지 않았다. 따라서 현재 결론은
`평가 실행 준비 완료`이며 정책 우승이나 production 승격이 아니다. 구현·검증·용어·실행
명령과 주장 경계는 `20260907_G005_VERTICAL_READINESS.md`에 정리했다. 2026-09-08에는
첫 목표 도착 뒤 45초에 발생하는 startup timeout 오적용을 실제 실행으로 재현·수정했다.
현재 기본 진단은 `--diagnostic --diagnostic-seconds 60` 한 조건이며 실패 로그도 보존한다.

`inspect_mcap.py`와 미디어 생성기는 `requirements.txt`에 고정한 MCAP reader, ROS 2
decoder, 수치·그림 라이브러리를 사용한다. 로봇 런타임의 전역 Python 환경이나 OS
패키지를 바꾸지 않도록 평가 전용 디렉터리에 설치한다. MP4 변환에는 시스템 `ffmpeg`를
사용하며, 없으면 GIF와 나머지 산출물만 만든다.

```bash
python3 -m pip install --disable-pip-version-check --no-input --upgrade \
  --requirement "$HOME/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/requirements.txt" \
  --target "$HOME/.local/share/jdamr-slam-eval/python"
PYTHONNOUSERSITE=1 PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
  python3 -c "import mcap; print(mcap.__version__)"
```

## 기존 G4 bag 판정

`g4_userloop_reset_20260824T175151_0.mcap`은 63,955개 메시지를 끝까지 읽었고 summary CRC와 인덱스가 있다. 그러나 39개 chunk의 CRC가 모두 0이며 data-section CRC도 0이다. 당시 기록·재생 QoS와 NIC·드라이버·커널 드롭 카운터가 없고, 로봇을 든 뒤의 꼬리 구간도 분리되지 않았다. 따라서 원인 탐색에는 사용할 수 있지만 지도나 성능 결과를 승격하는 입력으로는 사용할 수 없다.

`validate_crcs: PASS`는 파일에 존재하는 CRC가 일치했다는 뜻이다. `chunks_with_crc: 0`은 핵심 데이터 chunk에 검증할 CRC 자체가 없다는 뜻이므로 서로 모순되지 않는다.

검사 명령은 메시지를 발행하지 않는다.

```bash
cd $HOME/jdamr_cube_ws/src/jdamr_cube_ros
PYTHONNOUSERSITE=1 PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
  python3 jdamr_cube_navigation/evaluation/inspect_mcap.py \
  $HOME/jdamr_artifacts/g4_userloop_reset_20260824T175151/g4_userloop_reset_20260824T175151_0.mcap
```

## 다음 기록의 고정 조건

아래 명령은 운영자가 안전 구역과 비상 정지를 확인한 뒤 별도 주행 세션에서만 사용한다. `<run_id>`는 실행 전에 고유한 실제 값으로 바꿔야 한다. 이 문서는 이동 명령을 포함하지 않는다. 실차 Nav2가 파이에서 실행될 때 이 recorder도 반드시 파이에서 실행한다. 노트북에서 같은 고속 토픽을 추가 구독하면 약한 Wi-Fi 구간에서 TF·scan 지연이 제어 루프까지 전파될 수 있다. 저장지도 Keepout 주행은 recorder와 navigation을 함께 묶은 `onboard_keepout_navigation.launch.py`를 우선 사용한다. 파이에는 RViz, 온라인 mapping backend, bag 재생·분석을 추가로 띄우지 않는다.

```bash
cd $HOME/jdamr_cube_ws/src/jdamr_cube_ros
source /opt/ros/jazzy/setup.bash
ionice --class best-effort --classdata 7 \
nice --adjustment 10 \
ros2 bag record \
  --storage mcap \
  --storage-config-file jdamr_cube_navigation/evaluation/mcap_writer_options.yaml \
  --qos-profile-overrides-path jdamr_cube_navigation/evaluation/qos_overrides.yaml \
  --output $HOME/jdamr_artifacts/<run_id> \
  /scan /odom /tf /tf_static /imu/data_raw \
  /cmd_vel /cmd_vel_nav /amcl_pose /battery_state /plan \
  /collision_monitor_state
```

`/joint_states`는 RViz 바퀴 표현용이라 기본 온보드 기록에서 제외한다. MCAP 압축은 CPU를
아끼기 위해 사용하지 않고, 1MiB chunk의 CRC와 인덱스만 보존한다. 출발 전 1분 정적
소크에서 4코어 load average 4.0 미만과 thermal throttle 없음도 함께 확인한다.

프로토콜 적격 데이터는 다음을 모두 만족해야 한다.

1. bag SHA-256과 metadata SHA-256을 등록한다.
2. chunk CRC, summary CRC, chunk/message index를 보존한다.
3. 기록·재생 QoS 파일과 소프트웨어 버전을 manifest에 남긴다.
4. NIC·드라이버·커널 드롭 카운터와 rosbag 토픽 수를 남긴다.
5. 로봇을 들거나 센서를 가린 구간은 시간 경계로 제외한다.
6. `map -> odom` TF 권한자는 한 개만 허용한다.
7. `UNVERIFIED` 또는 `UNOBSERVABLE` 보정값은 센서 융합 성능 주장에 사용하지 않는다.

진단 전용 데이터에서는 미상 항목을 명시한 채 보존할 수 있다. 하지만 `protocol_qualified: true` 입력에 QoS, CRC, 드롭 또는 구간 경계가 미상이라면 승격을 차단한다.

## 완주 기록의 지표와 미디어 생성

`corridor_run_media.py`는 경로 실행 로그의 첫 목표 전송과 성공 시각을 분석 구간으로
사용한다. MCAP recorder timestamp로 토픽 rate, p99 gap과 최대 gap을 계산하고, 마지막
메시지 뒤 무응답 시간도 최대 gap에 포함한다. AMCL 누적 경로는 5cm보다 작은 연속 위치
변화를 제거한 뒤 계산한다. 이 숫자는 저장 지도 localization의 일관성 지표이며 외부
ground truth ATE가 아니다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
python3 jdamr_cube_navigation/evaluation/corridor_run_media.py \
  --run-dir "$HOME/jdamr_artifacts/corridor_localdds_armed_20260904T152036" \
  --compare-run-dir "$HOME/jdamr_artifacts/corridor_roundtrip_20260904T142436" \
  --output-dir \
    jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036
```

출력에는 집계 YAML/JSON, AMCL 궤적과 목표 진행 CSV, 지도 중첩 그림, stream gap 비교,
텔레메트리, GIF와 H.264 MP4가 포함된다. GIF는 실제 AMCL timestamp를 균등 시간축으로
압축한 시각화이며 카메라 촬영 영상이 아니다.

## 센서와 시간 프로파일

성공 주행의 route log가 있으면 실제 목표 전송부터 성공 시각까지만 분석한다. IMU 정적
노이즈는 주행 직전 60초를 별도 구간으로 잡아 이동 구간과 섞지 않는다. LiDAR의 `inf`는
토픽 손실이 아니라 최대 거리 안에서 반사가 없었던 ray로 기록한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
python3 jdamr_cube_navigation/evaluation/profile_sensor_streams.py \
  --bag "$HOME/jdamr_artifacts/corridor_localdds_armed_20260904T152036" \
  --output-dir \
    jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036
```

출력은 sensor profile YAML/JSON/Markdown과 센서·시간 분포 PNG다. 실차에 외부 ground
truth가 없으므로 LiDAR additive range noise와 odometry 정확도는 이 데이터만으로 만들지
않는다. 시뮬레이션 후보값은 관측 범위의 시작점이며 튜닝 완료값이 아니다.

## Gazebo 정답 궤적과 센서 노이즈 실험

실제 층 평면도에서는 긴 복도, 반복되는 양쪽 벽, 끝에서 회차해 돌아오는 구조만 가져왔다.
`slam_corridor.world`는 이를 통제 가능한 직선 복도로 단순화한 것이며 실제 축척·방 위치를
복원한 지도가 아니다. Gazebo PosePublisher의 `/ground_truth_pose`는 wheel odom과
SLAM TF를 사용하지 않으므로 ATE/RPE의 독립 기준으로 쓴다.

센서 variant는 원본 URDF를 덮어쓰지 않고 저장소 밖에 생성한다. 이번 기준선은 LiDAR
10Hz·가우시안 표준편차 0.01m이고, stress 조건은 주기는 유지한 채 표준편차만 0.05m로
바꿨다. 0.05m는 실차에서 측정한 노이즈가 아니라 강건성 확인용 합성 조건이다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
mkdir -p "$HOME/jdamr_artifacts/sim_slam_profiles_20260904"
python3 jdamr_cube_navigation/evaluation/make_sim_sensor_variant.py \
  --base jdamr_cube_description/urdf/jdamr_cube.urdf \
  --output "$HOME/jdamr_artifacts/sim_slam_profiles_20260904/baseline_10hz.urdf" \
  --label baseline_10hz \
  --lidar-update-rate-hz 10 --lidar-noise-stddev-m 0.01
python3 jdamr_cube_navigation/evaluation/make_sim_sensor_variant.py \
  --base jdamr_cube_description/urdf/jdamr_cube.urdf \
  --output "$HOME/jdamr_artifacts/sim_slam_profiles_20260904/lidar_noise_5x.urdf" \
  --label lidar_noise_5x \
  --lidar-update-rate-hz 10 --lidar-noise-stddev-m 0.05
```

같은 출력 경로가 이미 있으면 생성기가 덮어쓰지 않는다. 새 실험은 디렉터리 이름을
바꾸고, 이번 결과를 검증할 때는 보존된 `20260904` profile과 manifest를 그대로 쓴다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
export PYTHONNOUSERSITE=1
export PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python:$PWD/jdamr_cube_navigation/evaluation"

python3 jdamr_cube_navigation/evaluation/run_sim_slam_experiment.py \
  --backend cartographer \
  --profile-urdf "$HOME/jdamr_artifacts/sim_slam_profiles_20260904/baseline_10hz.urdf" \
  --profile-label baseline_10hz \
  --world "$PWD/jdamr_cube_gazebo/worlds/slam_corridor.world" \
  --out-root "$HOME/jdamr_artifacts/sim_slam_corridor_gt_20260904" \
  --seed 42 --route corridor --spawn-x -8.0 --spawn-y 0.0 \
  --corridor-distance-m 14.0
```

SLAM Toolbox 실행에는 검증할 설정을 명시해야 한다. 이번 비교는 실제 bag 원인분리에서
선택한 `slam_toolbox_range35_dense.yaml`을 사용했다. 실행 디렉터리가 이미 있으면
덮어쓰지 않고 중단하므로 각 결과가 한 조건에만 대응한다.

```bash
python3 jdamr_cube_navigation/evaluation/make_slam_toolbox_ablation.py \
  --base jdamr_cube_navigation/config/slam_toolbox_corridor.yaml \
  --output "$HOME/jdamr_artifacts/slam_toolbox_ablation_20260904/slam_toolbox_range35_dense.yaml" \
  --label range35_dense \
  --max-laser-range-m 3.5 \
  --minimum-travel-distance-m 0.1 \
  --minimum-travel-heading-rad 0.15 \
  --do-loop-closing true
```

네 실행이 끝난 뒤 비교 자료는 다음 명령으로 재생성한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python:$PWD/jdamr_cube_navigation/evaluation" \
python3 jdamr_cube_navigation/evaluation/compare_sim_slam_experiments.py \
  --results-root "$HOME/jdamr_artifacts/sim_slam_corridor_gt_20260904" \
  --output-dir \
    jdamr_cube_navigation/evaluation/media/sim_slam_corridor_gt_20260904
```

이번 seed 42 결과에서 Cartographer의 이동 ATE RMS는 기준선 0.645m, 5배 노이즈
0.752m였다. SLAM Toolbox는 각각 4.355m, 3.990m였다. 두 센서 조건의 이동·회전
ATE/RPE가 모두 낮아 Cartographer를 유지한다. 자세한 수치·입력 해시·단일 시드 한계는
`media/sim_slam_corridor_gt_20260904/sim_slam_robustness.md`에 있다.

### Cartographer 5-seed 단일 요인 강건성 행렬

`run_sim_slam_robustness.py`는 성공 실차 bag의 센서 프로파일을 입력으로 읽고, 고정된
Cartographer 설정·world·왕복 경로에서 정확히 5개 seed를 실행한다. 기준선 외 조건은
LiDAR 주기, timestamp jitter, 거리 노이즈, scan 메시지 dropout, wheel scale, wheel
slip, TF delay, scan transport delay 중 하나만 바꾼다. 거리·wheel·transport 값은 실차
bag만으로 측정할 수 없으므로 `measured_observation`으로 기록하지 않고
`derived_stress` 또는 `synthetic_stress`로 남긴다. 각 provenance에는 원본 JSON의
SHA-256과 JSON pointer가 포함된다.

먼저 실행 없는 dry-run으로 45개 명령과 단일 요인 계약을 검증한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
python3 jdamr_cube_navigation/evaluation/run_sim_slam_robustness.py \
  --sensor-profile \
    jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/sensor_profile.json \
  --base-urdf jdamr_cube_description/urdf/jdamr_cube.urdf \
  --base-bridge jdamr_cube_gazebo/params/bridge.yaml \
  --world jdamr_cube_gazebo/worlds/slam_corridor.world \
  --out-root "$HOME/jdamr_artifacts/sim_slam_robustness_<date>"
```

`matrix_manifest.json`의 조건·seed·명령을 검토한 뒤 같은 명령에 `--execute`를 붙이면
전체 행렬을 순차 실행한다. 기존 출력과 내용이 다른 generated 파일이나 run 디렉터리는
덮어쓰지 않는다. 조건마다 증거가 유효한 5개 seed가 모두 있어야 완전한 조건으로
집계한다. 그중 완주 기준을 충족하지 못한 실행은 성능 결과인 `FAIL`, 프로세스·schema·
입력/산출물 hash·동기화 coverage·fault realization·resource 증거가 불완전한 실행은
`INVALID`다. 따라서 `FAIL`은 5-seed 분포와 completion rate의 분모에 남고,
`INVALID`가 하나라도 있으면 그 조건은 `INCOMPLETE`다.

각 run에는 다음 증거가 남는다.

- `run_manifest.json`: `experiment_manifest.schema.json`을 실행 중 검증한 표준 manifest
- `execution_manifest.json`: 실제 명령, 프로세스 exit code, 입력 및 fault 해시
- `metrics.json`: GT total/outbound/return 완주, finite ATE/RPE, timestamp coverage
- `fault_stats.json`: 요청값이 실제 적용된 dropout·jitter·noise·delay·wheel 통계
- `resource_samples.jsonl`: 0.5초 CPU/RSS 원본 표본. CPU는 한 코어 100% 기준이다.
- `map_state.pbstream`, `map.yaml`, `map.pgm`: 비어 있지 않은 지도와 SHA-256

집계기는 matrix의 seed·backend·world·경로·Cartographer 설정·workspace fingerprint와
각 run identity를 다시 대조한다. 표준 manifest를 schema로 재검증하고, 모든 artifact와
원본 bag의 SHA-256, resource JSONL의 유한값과 요약 재현, PGM의 unknown/free/occupied
픽셀 존재까지 통과한 실행만 성능 분포에 포함한다.

강건성 실행은 카메라 bridge와 생성 URDF의 카메라 센서를 끈다. 따라서 CPU/RSS는
2D LiDAR SLAM 경로를 측정하며 Visual SLAM이나 Depth 카메라 성능을 포함하지 않는다.

## 저장 지도 주행 bag의 오프라인 SLAM 재생

저장 지도와 AMCL을 사용해 안전 경로로 수집한 bag도 새 지도 생성 입력으로 쓸 수 있다. 단, 새 mapping backend는 `use_sim_time=true`와 빈 상태로 먼저 실행하고, localization node와 저장 map server는 실행하지 않는다. 재생은 실차 domain 12가 아닌 격리 domain 199에서만 허용한다.

```bash
cd "$HOME/jdamr_cube_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=199
ros2 launch jdamr_cube_navigation offline_replay_guard.launch.py \
  bag:="$HOME/jdamr_artifacts/<run_id>"
```

이 launch는 `/scan`, `/odom`, `/tf`, `/tf_static`, `/imu/data_raw`, `/joint_states`만 재생한다. `/map`과 `/cmd_vel`은 재생 목록에 없으며, TF는 전용 토픽으로 우회한 뒤 기록된 `map -> odom`만 제거해서 원래 `/tf`로 전달한다. 따라서 `/tf`의 `odom -> base_footprint`와 센서 고정 TF는 보존되고, 새 mapping backend만 `map -> odom` 권한자가 된다. TF filter나 bag player가 종료되면 전체 재생도 종료한다. TF queue 유실을 막기 위해 `rate`는 실시간 1.0 이하만 허용한다.

두 backend 실행이 끝나면 같은 원본 기준의 비교 자료를 만든다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
python3 jdamr_cube_navigation/evaluation/compare_slam_runs.py \
  --results "$HOME/jdamr_artifacts/offline_slam_20260904" \
  --source-root "$HOME/jdamr_artifacts" \
  --output jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.json \
  --plot jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.png \
  --report jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.md
```

설정 필드와 QoS 형식은 [rosbag2 MCAP 저장소 공식 문서](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_storage_mcap/README.md)와 [rosbag2 QoS override 공식 문서](https://github.com/ros2/rosbag2/blob/rolling/README.md)를 기준으로 한다.

### 가변 LaserScan 격자 원인분리

스캔마다 빔 수나 시작 각도가 달라지는 bag은 원본을 수정하지 않고 고정 각도 격자의
파생 bag으로 만든다. `--beam-count`는 원본 분포를 확인한 뒤 실험 manifest에 기록한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
python3 jdamr_cube_navigation/evaluation/normalize_scan_bag.py \
  --bag "$HOME/jdamr_artifacts/<run_id>" \
  --output "$HOME/jdamr_artifacts/<run_id>_scan_fixed" \
  --beam-count <count> \
  --storage-config-file \
    jdamr_cube_navigation/evaluation/mcap_writer_options.yaml
```

출력 bag에는 원본·출력 SHA-256, 입력 빔 수 분포, 정규화 방식, 전수 검증 결과를 담은
manifest가 함께 생성된다. 검증은 메시지 순서와 timestamp, 비-scan payload, 정규화된
모든 scan을 대조한다.

파라미터 실험은 검토된 기본 YAML에서 허용된 값만 바꾼 파생 파일을 만들고 실행 라벨과
설정 경로를 로그에 남긴다.

```bash
python3 jdamr_cube_navigation/evaluation/make_slam_toolbox_ablation.py \
  --base jdamr_cube_navigation/config/slam_toolbox_corridor.yaml \
  --output "$HOME/jdamr_artifacts/<experiment>/params.yaml" \
  --label <label> \
  --max-laser-range-m <metres>

bash jdamr_cube_navigation/scripts/offline_slam_replay.sh \
  --bag "$HOME/jdamr_artifacts/<run_id>_scan_fixed" \
  --backend slam_toolbox \
  --label <unique_run_label> \
  --slam-params "$HOME/jdamr_artifacts/<experiment>/params.yaml" \
  --out "$HOME/jdamr_artifacts/<results>" \
  --rate 1.0
```

현재 복도 실험의 판정과 한계는 `20260904_SLAM_TOOLBOX_ROOT_CAUSE.md`에 기록했다.

## G004 Collision Monitor 시뮬레이션 평가

G004는 Gazebo 시뮬레이션에서 Nav2 주행 명령과 Collision Monitor의 정지·재개 동작을
검증한 결과다. Collision Monitor는 속도 명령 앞단에서 센서 입력을 감시해 정지 명령을
내리는 Nav2 구성 요소이고, StopZone은 장애물 점이 들어오면 즉시 정지시키는 전방
다각형 영역이다. `invalid source`는 설정한 시간 안에 새 센서 입력이 없어 해당 입력을
유효하지 않다고 판정한 상태다.

평가는 기준 주행, 갑작스러운 장애물, monitor scan timeout의 세 시나리오를 난수 초기값(seed)
11·23·42·67·89에서 각각 실행한 15회(3개 시나리오 × 5개 seed)로 구성했다. 15회 모두
각 실행에서 한 번 전송한 목표와 동일한 UUID의 상태가 `SUCCEEDED`로 끝났고 접촉·목표
취소·예상 밖 종료·잔류 프로세스는 각각 0건이었다. 표의 범위는 다섯 seed의
최소~중앙~최대값이다.

| 시나리오 | 핵심 결과 |
|---|---|
| 갑작스러운 장애물 | scan-gate 입력 발행부터 최종 속도 0 수신까지 0.0112~0.0337~0.0519 s, 정지 거리 0.0540~0.0540~0.0627 m, 로봇 footprint와 장애물 사이 최소 여유 0.0828~0.0882~0.0886 m |
| monitor scan timeout | 동결 승인부터 최종 속도 0 관측까지 0.2215~0.2453~0.2521 s, 정지 시 센서 나이 0.302~0.327~0.330 s, 정지 거리 0.0172~0.0194~0.0213 m |

scan-gate는 평가용 Collision Monitor scan 전달 노드다. 갑작스러운 장애물 반응 시간은
이 노드의 동일 프로세스 monotonic clock으로 잰 장애물 포함 scan 발행부터 최종 속도 0
수신까지이며, 센서 획득과 외부 전송 지연은 포함하지 않는다.

여기서 ground-truth는 시뮬레이터가 제공한 로봇의 정답 위치이며 localization 추정치와
구분한다. MCAP은 ROS 2 토픽 메시지를 보존한 bag 저장 형식이다. 대표 bag은 전체 15회를
사후 선별한 것이 아니라 미리 고정한 seed 11을 시나리오별로 다시 기록한 세 MCAP이다.
Keepout은 지도에서 진입 금지 영역을 경로 계획 단계에 반영하고, Collision Monitor는
실행 중 센서 입력을 바탕으로 속도 명령을 정지시키므로 역할과 증거가 다르다.

검증된 산출물은 다음 세 경로에 보존한다.

- 전체 15회: `$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_full15_cyclone`
  (3,452,814 B, 전체 행렬에서는 bag과 영상을 기록하지 않음)
- 대표 MCAP 3개: `$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_representatives_seed11_cyclone`
  (51,459,184 B, MCAP 합계 50,029,263 B)
- 최종 미디어: `$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_media`
  (346,213 B, 1280×720 H.264 MP4 3개와 요약 GIF 1개)

미디어는 위 MCAP의 경로와 상태 전환을 재생한 자료다. 모든 프레임에 Gazebo
simulation/MCAP replay임을 표시하며, 장애물·timeout 영상은 짧은 정지 구간을 알아볼 수
있도록 사건 중심으로 시간을 확대한 비실시간 재생이다. 이 결과는 실차 안전 인증이나
사람 인식을 검증하지 않으며, 화면 진행 비율도 실제 시간 비율로 해석하지 않는다.

전체 행렬, 대표 기록, 미디어는 다음 명령으로 원본 해시와 파생 결과를 다시 검증한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
export PYTHONPATH="$PWD/jdamr_cube_navigation/evaluation:$PYTHONPATH"

python3 - <<'PY'
from pathlib import Path
from run_sim_collision_monitor_eval import validate_source_matrix

root = Path.home() / 'jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_full15_cyclone'
result = validate_source_matrix(root)
assert len(result['evidence_manifest']) == 15, result
print('PASS', result['aggregate_sha256'])
PY

python3 jdamr_cube_navigation/evaluation/evaluate_sim_collision_monitor.py \
  --promotion-root \
  "$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_representatives_seed11_cyclone"

python3 jdamr_cube_navigation/evaluation/render_sim_collision_monitor_media.py \
  --matrix-root \
  "$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_full15_cyclone" \
  --representative-root \
  "$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_representatives_seed11_cyclone" \
  --output-dir \
  "$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_media" \
  --verify-existing
```

용량 정책은 전체 행렬 512 MiB 이하, 대표 bag 한 개당 128 MiB 이하, 대표 루트 512 MiB
이하다. 최종 v08 전체 행렬·대표 기록·미디어는 보존하고, 승인된 후속 결과로 대체된
v05·v07 대표 기록과 v08 미디어 후보·이전본만 삭제했다. 다른 실험 산출물은 이 정리
범위에 포함하지 않았다.

## 실제 온보드 프로필의 돌발 장애물 통합

G004 평가에서 확인한 정지·재개 동작을 실차용 `onboard_nav2_core.launch.py`의
`obstacle_candidate` 프로필에 연결했다. 차체 footprint만 쓰지 않고 production URDF의
고정 수납 자세에서 SO-101 collision 외곽을 계산해 StopZone 0.40 m와 SlowdownZone
0.50 m를 적용한다. `/joint_states`가 계약 자세에서 0.03 rad 이상 벗어나거나 stale이면
route가 fail-closed로 목표를 차단한다.

좌·우 진입 대표 실행은
`$HOME/jdamr_artifacts/onboard_candidate_sudden_20260908_v13_followcam`과
`$HOME/jdamr_artifacts/onboard_candidate_sudden_20260908_v15_followcam_right`에 있다.
두 실행 모두 설치된 Collision Monitor의 파라미터를 다시 읽었고 접촉 0회, 목표 전송
1회·취소 0회, 동일 UUID 재개와 최종 도착, 잔존 프로세스 0을 확인했다. 팔 포함 외곽
최소 여유는 각각 0.03960 m와 0.04503 m였고 compact MCAP은 약 1.39 MB와 1.37 MB다.

`evaluation/media/onboard_dynamic_obstacle_20260908`에는 모바일 베이스를 따라가는
Gazebo 3D 카메라 센서의 좌·우 진입 하이라이트, 이를 연결한 34초 대표 영상, GIF와
정지·도착 포스터가 있다. 하이라이트의 긴 순항 구간만 8배속이고 장애물 정지와 도착은
원래 프레임 속도다. 편집 전 원본과 전체 영상은 artifact에만 보존해 저장소 용량을
줄였다. `portfolio_media_manifest.json`은 두 원본과 공개 출력의 SHA-256을 보존한다.
자세한 주장 범위와 재현 명령은
[이동형 로봇팔 장애물 대응](20260908_DYNAMIC_OBSTACLE_READINESS.md)을 따른다.

`navigation_dashboard.py`는 `http://127.0.0.1:8765/`에서 현재 Nav2 목표 상태,
Collision Monitor 동작, `/cmd_vel`, LiDAR 최소 거리·방향과 최근 검증 결과를 함께
표시한다. 이 화면은 관측 전용이며 주행 명령을 발행하지 않는다. ROS 그래프가 없을
때도 `--no-ros`로 마지막 PASS 영상과 수치를 재생할 수 있다.

`detour_sudden_stop_resume`는 출발 전부터 직선 경로를 막은 0.50×0.40 m
고정 박스 우회와, 이후 보행자가 횡단할 때의 정지·동일 목표 재개·최종 도착을
한 번의 Nav2 목표에서 검증한다. 대표 PASS는
`$HOME/jdamr_artifacts/onboard_candidate_combined_20260908_v05`이며, 보행자는 통로
한쪽 `y=+1.0 m`에서 반대쪽 `y=-1.0 m`까지 완전히 횡단한다. 최대 횡방향 우회
0.57716 m, 고정 박스 최소 이격 0.13620 m, 접촉 0회, 목표 전송 1회·취소 0회를
기록했다. Gazebo 하이라이트는
`evaluation/media/gazebo_corridor_crossing_20260908`에 있다.

MuJoCo 미디어는 이 PASS 기록을 측면 사선의 3D 시점으로 재생한다. 카메라는
주행 거리의 75%만 따라가 로봇이 화면 왼쪽에서 오른쪽으로 이동하며, 보행자의
통로 완전 횡단도 원래 steady-clock 이벤트 순서로 보여준다. 주행 데이터의 원본은
Gazebo/ROS 2 MCAP이고, MuJoCo는 장면·LiDAR raycast·HUD 표현 계층이다. 자세한
증거 범위와 명령은
[20260908_DYNAMIC_OBSTACLE_READINESS.md](20260908_DYNAMIC_OBSTACLE_READINESS.md)에 있다.
