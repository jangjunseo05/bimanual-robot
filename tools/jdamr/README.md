# JD-AMR 저장소 내부 실행 도구

JD-AMR SLAM·주행 코드의 관리 기준은 이 저장소의 `src/navigation/`이다. 이후 수정은
`bimanual-robot`의 `main`에서 작업 브랜치를 만든 뒤 진행하며, 별도 JD-AMR 저장소나
`$HOME/jdamr_data/.../tools`의 실행 사본을 원본으로 수정하지 않는다.

## 로컬 준비와 빌드

저장소 루트에서 실행한다.

```bash
tools/jdamr/runtime.py bootstrap
tools/jdamr/runtime.py build-pc
```

`bootstrap`은 `$HOME/jdamr_data/maps/current_manual_mapping.json`이 선택한 현재 지도에서
map·keepout·목적지 등록부·station 프로파일을 만든다. 도크·급수대·테이블 원본 등록값과
`state.json`은 유지한다. 경유점 생성과 Pi 동기화는 출발 도구의 `prepare`를 사용한다.
다른 포인터는 `--map-session <전체 경로>` 또는 `JDAMR_MAP_SESSION`으로 선택한다.
이전 지도 번들은 `bootstrap --legacy-map`으로만 복원한다. 보존 번들의 기존 파일을
교체할 때는 `--legacy-map --refresh`를 사용하고 `.runtime/jdamr/asset_backups/`에 백업한다.

`build-pc`는 기본적으로 실차 런타임 9개 ROS 패키지를
`.runtime/jdamr/pc_ws/`에 빌드한다. 편입된 모든 ROS 패키지가 필요하면 `--all`을
추가한다. 실행하지 않고 명령만 확인하려면 `--dry-run`을 사용한다.
코드 빌드는 지도 생성·선택을 바꾸지 않으며 지도 포인터 없이도 수행할 수 있다.
현재 지도·등록·센서 참조가 없는 새 PC에서는 코드를 먼저 빌드하고, 해당 자료를 준비한
뒤 `bootstrap`과 출발 도구의 `prepare`를 사용한다.

## Pi 배포

```bash
tools/jdamr/runtime.py plan --host lim@192.168.0.159
tools/jdamr/runtime.py deploy-pi --host lim@192.168.0.159
tools/jdamr/runtime.py deploy-pi --host lim@192.168.0.159 --promote
```

`plan`은 PC 빌드와 Pi 배포 명령을 상태 변경 없이 출력한다. `deploy-pi`의 기본 동작은
Pi의 타임스탬프 stage에 전체 소스를 복사하고 빌드하는 것까지다. `--promote`를 붙여야
현재 `$HOME/jdamr_ws`에 반영한다. 승격은 검증된 stage의 `source/install/tools`를 그대로
보존하고, 안정 경로를 그 절대경로를 가리키는 링크로 교체한다. 기존 슬롯은 Pi의
`$HOME/jdamr_deploy/bimanual_backup_*`에 이동하며 중간 실패 시 원래 슬롯으로 복구한다.
live 경로에서 다시 빌드하지 않으며 서비스 재시작과 로봇 이동도 수행하지 않는다.
배포 자산도 현재 지도 포인터의 runtime에서 가져온다. 실행 상태·초기 위치 클릭·run 로그는
복사하지 않는다. 기존 원격 정적 자산과 카메라 설정은 기본적으로 유지하며,
선택한 자산으로 바꿀 때만 `--replace-assets`를 지정한다. 교체는 같은 승격 작업에서
백업·복구한다. 이전 지도 자산의 배포는 `--legacy-map`을 명시해야 한다.
지도 선택 포인터·원본 등록·센서 참조의 Pi 동기화는 출발 도구의 `prepare`를 사용한다.

## 출발 도구

```bash
tools/jdamr/jdamr_depart.py status
tools/jdamr/jdamr_depart.py prepare
tools/jdamr/jdamr_depart.py session-start
tools/jdamr/jdamr_depart.py init
tools/jdamr/jdamr_depart.py go water_station
```

기본 지도는 `$HOME/jdamr_data/maps/current_manual_mapping.json`에서 선택한다.
다른 포인터는 전역 옵션 `--map-session <전체 경로>` 또는 `JDAMR_MAP_SESSION`으로 지정한다.
`prepare`는 현재 등록 도크·급수대·테이블로 실행 자산을 만들고, 원본 지도·등록 파일·
도크 스캔·RGB-D 참조를 Pi에 동기화한 뒤 지도 포인터를 마지막에 반영한다.
`session-start`는 Windows Astra TCP 수신기와 관측기를 유지하고,
Nav2를 이동 전 준비 상태로 기동한다. Windows 연결 구성에서는 Pi 베이스의
`15-astra-usb-recover.conf` 의존성을 사용하지 않는다. Pi에 없는 카메라를 USB로
복구하려는 대기가 베이스 기동을 지연시키기 때문이다. `init`은 등록 도크를 다시 가르치지 않고 위치를 초기화한다.
도크 원본과 현재 스캔을 0.5° 간격으로 비교하고, 양쪽의 공통 지지점으로 정합한다.
대체 위치에는 같은 최적 지지점을 고정해 비교한다. 등록 도크 범위 안의 작은
평행이동 모호성은 초기 공분산에 반영하고 실제 AMCL 수렴·기존 자세 일치 조건을 확인한다.
회전 모호성·범위 밖·지지 부족은 구분하며 전체 지도 정합 기준은 낮추지 않는다.
각 스캔 80점 이상·서로 다른 방향의 지지·다른 후보와의 구별 조건을 유지한다. 초기화의
이동 확인은 연속 움직임과 캡처 마지막 정지를 확인해 단발 EKF 속도 표본과 구분한다.
충전 중에는 `go`를 실행하지 않는다.

Pi 베이스 서비스는 `jdamr_cube_navigation/real_bringup_imu.launch.py`로 바퀴 속도와
바이어스 보정 자이로를 합친다. 원시 바퀴 odom은 `/wheel/odom`, 보정 IMU는 `/imu/data`,
제어에 쓰는 odom은 EKF의 `/odom`이다. 드라이버의 odom TF 발행은 끄고 EKF만
`odom→base_footprint`를 발행한다. 가속도는 융합하지 않으며 AMCL의 `map→odom` 역할은 유지한다.
Pi의 Nav2·실행기·초기화 도구는 패키지 `config/fastdds_udp_loopback.xml`과
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`을 함께 사용한다. XML의 `127.0.0.1`
제한으로 Pi 내부 UDP만 사용하며, `LOCALHOST`가 추가하는 공유메모리 전송을 피한다.
베이스도 같은 XML과 `discovery_range:=SUBNET`을 사용한다. Pi에서는
`/etc/systemd/system/jdamr-base.service.d/99-loopback-udp.conf`에 적용돼 있다.
기존 `jdamr_cube_bringup/real_bringup.launch.py`는 바퀴 odom 단독 구성으로 남아 있다.
IMU는 회전 추정을 보완하며, 면을 따라 움직인 실제 좌우 위치를 독립 측정하지 않는다.

실행기는 새 지도·도크·급수대 참조의 해시를 대조한 뒤
`src/navigation/scripts/map_20260930_manual/jdamr_depart.py`를 불러온다. 주행·위치초기화
알고리즘의 관리 대상은 이 파일 하나다. `status`는 상태 조회, `init`은 위치초기화,
`go`는 실물 이동 명령이므로 운영 상황에 맞게 사용한다.
스크립트 폴더의 날짜는 관리 코드의 기존 위치이며 선택 지도 날짜를 뜻하지 않는다.
출발 도구를 함수로 불러도 현재 지도에 연결되며, 현재 지도에서의 면 탐색은 오래된
`state.json`의 영역 대신 현재 등록 좌표를 사용한다.

기본 `go water_station`은 도크 → 등록 급수대 → 도크를 수행한다.
급수대를 목적지와 경유점으로 중복 방문하지 않는다. 이전 지도의 박스·테이블 경로는
`--legacy-map`으로만 불러온다. 등록 위치가 우선이다. 급수대는 정답 자세에서 촬영됐다는
`goal_binding`이 있는 RGB-D 참조에 한해 `reference_mode: rgbd`를 선택할 수 있다.
반사 컵·투명병의 깊이값이 불안정한 급수대는
`rgbd_reference.observation_method: table_contact_edges`로 접촉 경계의 RGB 기울기를
책상 평면에 투영해 좌우 목표를 구한다. 접촉 영역·탐색 범위는 해시로 묶인
`reference_capture.json`의 `table_contact_reference`에 보관하며, 물체 몸통의 깊이는
사용하지 않는다. 검출한 두 경계 주변을 추적하고 놓치면 같은 영상에서 전체 영역을
다시 찾는다. 카메라 장착·책상 높이·물체 배치가 바뀌면 참조를 다시 등록한다.
컵·물병으로 추정한 목표는 기존 영상 처리 작업에서 접근 전부터 수집한다. 이동 중에도
최근 0.5초 안의 유효 관측 최대 5개 중앙값으로 제어 목표를 갱신하며 최초 영상 하나에
고정하지 않는다. 이동 제어와 도착 판정은 같은 필터를, 각 제어 주기와 경로 확인은
그 주기에서 선택한 같은 목표를 사용한다. 원본 odom·깊이 간격·차체 충돌 형상은 이동시키지 않는다.
최종 로그의 `lateral_goal_odom`·`lateral_actual_odom`은 제어 목표와 실제 odom 자세다.
비교용 최초 목표는 `lateral_anchor_odom`, 개별 영상 오차는 `rgbd_observed_lateral_error_m`,
최초 목표와 개별 영상 목표의 차이는 `rgbd_goal_lateral_difference_m`에 기록한다.
표본 수와 목표 산포는 `lateral_feedback_sample_count`·`lateral_feedback_xy_spread_m`에 남긴다.
현재 RGB-D 목표가 없으면 이전 지도 좌표를 정밀주차 좌우 목표로 대신 쓰지 않는다.
깊이 면으로 앞 간격·방향을 보정하고, RGB-D 목표 미확보 상태는 좌우 정밀주차 성공으로
판정하지 않으며 목표 미확보 상태에서 근접 면을 정렬 완료로 동결하지 않는다.
최초 유효 관측부터 사용하므로 별도 프레임 대기를 추가하지 않는다.
접촉점 검색에는 영상 시각의 odom 이동량을 반영한다. 추적이 끊기면 같은 영상을 반복
처리하지 않고 다음 최신 영상에서 재탐색한다. 처리가 늦어 신선도 조건을 넘긴 결과는
제어 목표나 다음 목표의 비교 기준으로 수집하지 않는다.
급수대 관측 위치에 도착하면 통로에서 수집된 영상 목표를 초기화한다. 좌우 보정 곡선이
불가능하거나 최종 좌우 확인이 끊기면, 원래 실행 제한시간 안에서 각 단계별 한 번의
후진·재관측·재접근을 수행한다. 새 유효 참조가 없는 상태를 주차 성공으로 처리하지 않는다.
접근 중 좌우 조향은 허용오차 경계에서 끊지 않는다. 정지 면 관측 중 움직임이 감지되면
기존 제한시간 안에서 정지를 한 번 다시 확인하고 이전 영상을 버린 뒤 새 관측으로 이어간다.
반복 움직임·사용자 정지·센서 소실을 무시하지 않는다.
정답 참조에 연결된 재접근 요청은 근접 책상 경계 검출을 기다리지 않고 기존 보호 직선 제어로
요청한 거리만큼 후진한 뒤 새 깊이 면을 관측한다. 충돌 감시와 입력·명령 timeout은 유지한다.
목표 연결이 없는 기존 설정의 `rgbd_auxiliary` 모드는 진단용이다. 표식 없는 테이블은
`map_pose_only`에서 등록 좌표로 좌우를 맞추거나, `surface_only`에서 좌우 미세보정 없이
관측 면의 방향과 앞 간격만 맞춘다. 현재 테이블1은 사용자 요청으로 `surface_only`를
사용하고 컵·병 참조를 요구하지 않는다. 급수대는 앞끝 1 cm·좌우 ±5 mm 목표를 유지한다.
최종 내부 추정과 사용자 수용은 외부 실측 정확도와 구분하며 `NOT_VALIDATED`를 유지한다.
등록된 도크 방향이 `reverse`이면
고정 도크 앞쪽에서 후진 진입하며, 테이블 주차 목표로 도크 위치를 다시 등록하지 않는다.

기본 책상 주차와 박스 주차는 주차 계약의 `target_front_gap_m`(현재 0.01 m)을
사용한다. 간격을 바꿀 때는 `go water_station --final-gap-m 0.03`처럼 지정한다.
책상 면 정렬 뒤에는 중간 5 cm 정지를 끼우지 않고 최종 간격까지 직진한다.
유효한 관측은 관측 시각의 TF로 갱신하며, 근접 가림 판정 뒤에는 마지막 면을 동결하고
odometry로 남은 거리를 계산한다. 이 경우 최종 값은 새 깊이 실측이 아닌 내부 추정이다.
주차 면을 측정할 때는 기존 정지 자세 측정으로 속도·자세가 1초 연속 안정됐는지
확인한다. 안정 범위를 벗어나면 연속 정지 확인을 다시 시작한다. 영상 특징 추적과
기존 접근 피드백은 유지하며 주차 목표와 참조 영상을 다시 등록하지 않는다.
RGB-D 매칭이 성공했더라도 완료 시각에 기존 영상 신선도 범위를 넘긴 결과는 좌우 목표
표본에 넣지 않는다. 이동 제어 계산 사이에는 같은 제어 주기 안에서 대기 입력을 처리한다.
이 처리와 소스 배포만으로 실차 연속 주행이 확인된 것은 아니며, 실행 결과는 run 로그에 남긴다.
준비된 세션에서는 상주 실행기에 요청을 제출하고 첫 상태를 즉시 조회한다.
로그의 `SUBMITTED`는 요청 제출이며 실제 이동 시작을 뜻하지 않는다.
`request_submit_s`와 첫 이벤트 경과시간으로 통신·실행기 대기를 구분한다.

이동 전 지도 수신 실패는 위치추정·센서·화면을 유지하고 실행기만 한 번 재시작한다.
재시도도 실패하면 원인을 보고하고 종료한다. `recover` 전체 복구는 별도 명령이며,
앞서 사용하던 화면만 복원하고 화면 실패가 Pi 복구를 중단시키지 않는다.
표시 중계도 `JDAMR_HOST`를 사용한다. PC 녹화가 필요하면
`display-start --record-diagnostics` 또는 `record-start`를 사용한다.

필요한 경로와 접속 정보는 환경변수로 바꿀 수 있다.

| 환경변수 | 기본값 | 용도 |
| --- | --- | --- |
| `JDAMR_HOST` | `lim@192.168.0.159` | Pi SSH 대상 |
| `JDAMR_PI_USER` | `JDAMR_HOST`의 사용자 | Pi systemd 사용자 |
| `JDAMR_PI_HOME` | `/home/lim` | Pi 홈 경로 |
| `JDAMR_PI_WS` | `$JDAMR_PI_HOME/jdamr_ws` | Pi ROS 워크스페이스 |
| `JDAMR_DATA_ROOT` | `$HOME/jdamr_data` | PC 지도·등록부·실행 상태 루트 |
| `JDAMR_MAP_SESSION` | `$HOME/jdamr_data/maps/current_manual_mapping.json` | 현재 수동 지도·도크·급수대 참조 포인터 |
| `JDAMR_RUNTIME_ROOT` | `<저장소>/.runtime/jdamr` | 로컬 빌드·백업 루트 |
| `JDAMR_PC_WS` | `$JDAMR_RUNTIME_ROOT/pc_ws` | PC ROS 빌드 워크스페이스 |

`state.json`, `rviz_initialpose.json`, 실행 로그, bag과 원시 스캔은 저장소 번들에 포함하지
않는다. 초기화와 배포는 기존 상태 및 목적지 등록부를 자동으로 덮어쓰지 않는다.

## 웹 버튼과 대화 명령의 공통 실행 API

기존 웹 또는 이 PC의 출발 명령이 같은 실행 API를 사용한다. 이 PC는 Pi에 SSH 주행 요청을,
Windows 주문 서버(8765)에 상태 조회·정렬 완료 알림을 보낸다. 노트북 내부 조작 실행기
8770을 호출하지 않는다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/water_service_api.py serve --order-url http://192.168.0.157:8765
```

출발 요청에서 진행 중인 테이블1 주문이 있으면 그 주문에 연결한다. 없으면 테이블1
`COLD_WATER` 주문을 한 번 등록한다. 준비·상태 조회만으로는 주문이나 팔 동작을 시작하지 않는다.
순서는 도크 → 급수대 주차 → `POST /api/align/confirm {"station":"kitchen"}` →
물따르기·오른팔 접기 완료 대기 → 테이블1 주차 → 같은 API의 `station=table` →
컵배치 완료 대기 → 도크 복귀다. 정차는 시간 만료로 풀지 않고 기존 `--park-only`와
`--resume-parked-log`로 구간을 연결한다. 테이블1은 표식 없이 면 정렬을 사용한다.

정지한 지점에서 다음 단계 허용이나 팔 실행 직전 상태를 읽을 때 일시적인 조회 실패는
같은 주문·단계에서 재조회한다. 준비 상태 확인 주기는 0.25초이며 기존 단계 제한시간은
유지한다. 실패·취소된 주문을 재시작하거나 정렬 완료 POST를 자동 재전송하지 않는다.
주행 중의 통신·팔 상태 감시와 이 정지 상태 재조회는 별개다.

`GET /api/state`의 동일 주문과 `execution_backend.drive.next`, `arms_ready=true`,
`running_policy=null`, 팔 자세를 확인한다. 주행 중에도 이 조건을 감시한다.
급수대·테이블 출발은 `waiting_alignment`의 주문·지점이 맞아야 하고, 도크 복귀는
해당 주문 `SUCCEEDED`와 `arms=stowed`가 필요하다. 주문 실패·취소·통신 오류·사용자 정지는
다음 주행을 시작하지 않는다. 완료 알림 POST의 결과가 불명확하면 자동 재전송하지 않는다.
주문 서버의 성공은 컵배치 완료이며, 실제 도크 복귀 완료는 이 API의 `state=succeeded`로 구분한다.

`--navigation-only`를 명시한 경우에만 팔 연동 없이 기존 전체 경로를 실행한다.

서버 기본 주소는 `http://127.0.0.1:8771`이다. 외부 웹 백엔드에서 접근할 때는 해당 PC의
주소로 접근할 수 있도록 `--host`를 지정한다. API는 Bearer 토큰을 사용하며 기본 토큰
파일은 `$HOME/.local/state/bimanual-robot/water-service/api-token`이다. 토큰을 저장소나
브라우저 공개 코드에 넣지 않는다. 웹 백엔드가 이 토큰을 읽어 요청 헤더에 넣는다.

| 요청 | 동작 |
| --- | --- |
| `POST /api/mission/start` | JSON `{"request_id":"고유한-요청-ID"}`로 시작 |
| `GET /api/mission/status` | 모드·현재 단계·실행 결과 조회 |
| `POST /api/mission/stop` | 진행 중인 단계 중지, 다음 단계 실행 취소 |

같은 `request_id`는 다시 실행하지 않는다. 이 PC의 대화 실행도 같은 API를 사용한다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/water_service_api.py status
# 사용자가 출발을 요청한 경우
python3 tools/jdamr/water_service_api.py start --request-id operator-request-id
python3 tools/jdamr/water_service_api.py stop
```

Windows에 카메라 USB를 연결하면 RGB·깊이·CameraInfo를 Pi로 보내는 별도 영상 연결이
필요하다. 이 API는 영상 송신기를 포함하지 않는다. 현재의 Pi USB 카메라 주행 준비와
Windows 카메라를 사용하는 추론 포함 준비는 구분한다.

### 관제 화면 START·STOP 따라가기

`serve`에 `--follow-web`을 붙이면 이 PC가 주문 서버를 0.5초마다 읽어 관제 화면을 따른다.
`start`를 따로 실행하지 않는다.

```bash
python3 tools/jdamr/water_service_api.py serve --order-url http://192.168.0.157:8765 --follow-web
```

- 출발 신호는 START 자체가 아니라 팔 접기가 끝난 상태다. 같은 주문의 `waiting_alignment.station=kitchen`,
  `drive.next=kitchen`, `arms_ready=true`가 모두 참일 때 `web-<주문 번호>` 요청으로 한 번만 시작한다.
- STOP(`emergency_stop.active`)은 진행 중 미션을 멈추고 새 출발을 막는다. 주행 중 감시도 같은 값을 읽어 도크 복귀 구간까지 멈춘다.
- 차를 멈추는 경로는 `jdamr_depart.py stop`이다. 정지까지의 실제 지연은 측정 전이다.
- `--follow-dry-run`을 더하면 판단만 `follow: would_start <주문>`, `follow: would_stop`으로 출력하고 아무것도 시작·정지하지 않는다.
- 이 PC의 상태는 `/api/drive/heartbeat`로 보고되어 관제 화면 제목 아래에 표시된다. 3초 넘게 보고가 없으면 미연결로 표시한다.
- 조작 실행기 없이 연결만 시험할 때는 Windows에서 `tools/rollout/start_scenario.bat` 대신
  `tools/rollout/rehearsal_service.py`와 주문 서버만 켠다. 가짜 실행기는 하드웨어를 열지 않는다.
- 주문 서버는 수락과 START 전에는 주문을 시작하지 않는다. `--follow-web` 없이 `start`만 실행하면
  진행 중 주문에 연결되고, 진행 중 주문이 없으면 새 주문이 `PENDING`에 머물러 300초 뒤 실패한다.

### Windows 공유 Astra 입력

현재 주행의 센서 입력 기본값은 `JDAMR_CAMERA_TRANSPORT=windows_tcp`다.
`session-start`에서 TCP 수신기와 면 관측기를 준비하고, 태스크 사이에도 유지해 카메라 재기동·시계 동기화 대기를 반복하지 않는다.
Windows 송신기·필드·지연 기록과 종료 명령은 [ASTRA_TCP.md](ASTRA_TCP.md)에 있다.
USB 복원 시에는 TCP 수신기를 먼저 중지하고 `JDAMR_CAMERA_TRANSPORT=usb`를 명시한다.

## 팀원 실행 절차

현재 기체와 같은 카메라 장착·테이프·지도·교시 자산을 사용하는 절차다. Git pull만으로
지도와 교시 RGB-D가 생기지는 않는다. 새 PC는 ROS 2 Jazzy와 아래 자산을 먼저 준비한다.

### 지도와 정렬 설정

- 선택 포인터: `$HOME/jdamr_data/maps/current_manual_mapping.json`
- 현재 자산 폴더: `$HOME/jdamr_data/maps/20261026_manual_160135/`
- 선택 지도: `map_table01_observed.yaml`과 해당 이미지. 도크: `home_dock_table01_observed.yaml`,
  `dock_scan_reference_table01_observed.json`, `dock_scan.json`.
- 급수대 원본: `water_station.yaml`, 테이블 원본: `table_01.yaml`.
  `stations/`의 참조 JSON·RGB·depth·보정 자료도 함께 필요하다.
- 기존 PC 또는 Pi에서 이 자산 폴더와 선택 포인터를 인계한다. 다른 홈 경로라면 포인터와
  등록 파일의 참조 경로도 새 자산 위치에 맞춰야 한다. 로그·이전 AMCL 상태를 현재 위치로 사용하지 않는다.
- 급수대 책상 앞 옆면에 두 검은 띠를 중앙에서 좌우5cm, 폭1.8cm로 붙인다.
  팔을 접었을 때 접근 구간에서 두 띠의 위끝이 보여야 한다. 카메라 장착과 기준 컵·병 위치는 유지한다.
- 설정 예제는 [config/water_station_tape.json](config/water_station_tape.json).
  현재 `water_station.yaml`의 `rgbd_reference`에 같은 두 키가 설정돼 있다.
  기존 참조 경로·해시·상판 보정은 보존하고 예제만으로 참조 전체를 교체하지 않는다.
  `prepare`가 원본 설정을 runtime/station_profiles.json과 Pi로 동기화한다.
- 목표 앞 간격1cm, 테이프 좌우 허용5mm는 제어 목표이며 실측 보증치가 아니다.
  테이블1은 `surface_only`로 면 정렬만 사용한다.

### 서비스 준비 후 출발

1. Windows OMEN에서 팔 전원·USB·손목캠·Astra를 연결하고 현재 COM 번호를 확인한다.
   모델은 저장소와 별도로 설치한다. 저장소 루트에서 `tools\rollout\start_scenario.bat`을
   일반 창으로 실행하고 ready를 기다린다. 오래된 AMRArmService 예약 작업과 중복 실행하지 않는다.
   Astra 송신은 별도 프로세스이며 이 bat만으로 켜지지 않는다. [ASTRA_TCP.md](ASTRA_TCP.md)를 따른다.
2. Windows 주문 서버는 `http://192.168.0.157:8765`, Pi 수신은 `192.168.0.159:5600`이다.
   IP가 바뀌면 주문 URL과 수신기 peer를 함께 변경한다. 같은 내부망에서 연결한다.
3. 주행 PC에서 아래 준비 명령을 실행한다. 현재 도크에 로봇을 놓고 앞은 통로를 보게 한다.
   충전선을 분리한 뒤 출발한다. init은 기존 도크를 재등록하지 않는다.

```bash
cd "$HOME/bimanual-robot"
git pull --ff-only origin main
python3 tools/jdamr/jdamr_depart.py prepare
python3 tools/jdamr/jdamr_depart.py session-start
python3 tools/jdamr/jdamr_depart.py init
python3 tools/jdamr/water_service_api.py serve --order-url http://192.168.0.157:8765
```

이미 미션 서버가 켜져 있으면 serve를 중복 실행하지 않는다. 서버는 시연 동안 유지한다.
다른 터미널에서 전체 시나리오를 시작한다(이 명령은 팔 접기와 실물 주행을 시작한다).

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/water_service_api.py start --request-id "serving-$(date +%Y%m%d-%H%M%S)"
python3 tools/jdamr/water_service_api.py status
```

중단이 필요할 때만 별도로 실행한다. 다음 구간을 시작하지 않으며 자동 복귀하지 않는다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/water_service_api.py stop
```

순서는 도크 → 급수대 테이프·면 주차 → 물따르기 → 테이블1 면 주차 → 컵배치 → 후진 도킹이다.
기존 웹 주문이 있으면 start가 그 주문에 연결하고, 없으면 새 주문을 생성한다.
정렬 완료는 미션 실행기가 보내므로 관제에서 같은 단계 버튼을 중복 누르지 않는다.
수동 조종 화면은 `http://192.168.0.159:8080/`이며 자동주행과 동시에 조작하지 않는다.

### 전원 종료 후 재개 주의점

마지막 주문261030002는 급수대 주차 후 pour 실행 중 Windows가 꺼졌다. 로컬 미션은
stopped이며 Windows 취소 요청은 전달되지 않았다. 재부팅 뒤 `/control`에서 그 주문의
상태를 확인하고 미완료 주문을 취소한 다음 새 회차를 시작한다. 물따르기 완료를 추정해
테이블로 보내거나 align/confirm을 재전송하지 않는다. 기존 모델 로딩은 출발 전에 마친다.
전원 꺼짐 이전 주행·추론 진행은 사용자 확인이며, 이번 회차 전체 완료를 뜻하지 않는다.
