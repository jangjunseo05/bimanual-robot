# 2026-09-29 테이블 주차 실패와 재발 방지

상태: 충전 중 수정. 테이블 관측 위치까지 이동했으나 박스 앞 5cm 주차는 미완료다. 이후 수정본의 실차 성공으로 승격하지 않는다.

## 1. 관측 지점에 정밀주차를 적용해 발생한 Nav2 105

`home_exit`는 성공했다. 다음 `table_01_observation`의 목표는 (1.45, 0.25, yaw 0)이었는데, `ServiceRoute.execute()`가 마지막 waypoint를 무조건 주차 트리로 선택했다. 이 waypoint는 실제 박스 면을 찾기 전 관측 지점이다.

주행 MCAP에서 확인한 연쇄:

| 항목 | 근거 |
|---|---|
| 17:01:47.194 위치 | (1.445658, 0.242186), 목표와 8.94mm |
| 같은 시점 방향 오차 | 1.0798rad, 약 61.87도 |
| 회전 요청 | `/cmd_vel_nav` 약 -0.15rad/s |
| 충돌 방지 이후 | `/cmd_vel` 39개 모두 (0, 0) |
| 4.196초 동안 실제 변화 | odom 0.479mm, 방향 -0.000099rad |
| 17:01:51.390 종료 | `FAILED_TO_MAKE_PROGRESS`, 코드 105 |

오류 직전 10초 이동은 34.1mm, 회전은 0.0230rad였다. 진행 기준 50mm 또는 0.10rad를 충족하지 못했다. 충돌 방지가 회전을 차단한 것은 실제 기록으로 확인됐다. 전체 105 오류가 같은 원인이라고 일반화하지 않는다.

조치: `execute(final_parking=False)`를 관측 transit에만 적용한다. 관측 지점에서는 일반 재계획/위치 도착 판정을 사용하고, 이후 `face_alignment`와 `final_approach`에는 기존 주차 컨트롤러와 엄격한 방향·거리 확인을 유지한다. 회전 StopZone·진행 timeout·주차 허용치는 완화하지 않는다.

검증: 변경 전 회귀 테스트 6개 실패, 변경 후 관련 135개 통과. 실제 재주행은 충전 이후에 확인해야 한다.

## 2. 이동할 필요 없는 앵커를 정밀주차로 판정

현재 측정 좌표·방향으로 앵커 두 개를 만들었던 재개는 Nav2 목표 둘 모두 성공했지만 `parking odometry or final command missing`으로 종료됐다. 실제 이동이 없어 command가 발행되지 않은 상황에 최종 주차 확인을 요구한 운영 실수였다.

조치: 관측 위치 재개는 현재 정지 자세와 기존 관측 waypoint 거리 범위를 확인한 뒤 transit만 생략한다. 박스 관측·45cm 정렬·5cm 접근·최종 gap/corner/yaw 판정은 그대로 수행한다. 속도 0 메시지를 만들어 채우지 않는다. 수정 커밋 `9cbcefe`.

## 3. 박스 앞면 후보가 바뀌어 관측 실패

앞면 관측 거리 0.638~0.656m와 1.04~1.125m가 섞였고, 각도 약 +32~35도와 -15~-21도가 번갈아 나타났다. 8프레임 안정성이 확보되지 않아 `stable detected front surface is required`로 끝났다. 이 단계에서는 정렬/최종 접근 goal을 보내지 않았다.

최대 거리 0.85m와 평면 잔차 허용 0.015m으로 후보 영역을 좁힌 한 차례 시도도 실패했다. 임계값 변경만으로 해결된 문제가 아니며 원래 관측 설정으로 복원했다. 당시 원시 RGB-D 이미지가 없어 후보 전환의 기하학적 원인을 확정하지 않는다. 배경 평면/박스의 다른 면이 섞였다는 것은 후보 추정이며, 특정 물체 분류 결과가 아니다.

남은 수정/검증: 목표 영역에 결합된 일관된 면 선택을 구현하고 원시 RGB-D와 LiDAR 정합으로 검증해야 한다. 안정성·LiDAR 지지점 조건을 해제하거나 임의 박스를 목표로 삼지 않는다.

## 4. 후속 기록 유닛은 생성됐지만 rosbag 시작 실패

후속 두 유닛은 실행 사용자를 지정하지 않아 `rcutils_expand_user failed`와 `Failed to get logging directory`로 종료됐다. 첫 주행 MCAP은 존재하며 정상 마감됐지만, 후속 두 MCAP은 없다. 후속 JSONL은 보존했다.

재발 방지: 관리되는 기록 명령에 `User=<운영 사용자>`와 운영 사용자의 `ROS_LOG_DIR`를 명시하고 실제 recorder의 기록 시작과 metadata를 확인한다. `systemd-run`의 Started 응답을 기록 성공이라고 보고하지 않는다. 주행 요청 이후 이 준비 작업을 새로 작성하지 않는다.

## 5. PC RViz에 현재 로봇 표시 없음

PC에는 저장 지도와 목적지 마커가 있었으나 Current Robot Error가 남았다. 파이는 LOCALHOST 제어 그래프를 사용하고 PC RViz에는 실시간 TF/robot_description이 전달되지 않았다. 지도와 과거 표시가 보인다는 이유로 현재 로봇 위치를 확인했다고 판단한 운영상 누락이다.

조치 중: ROS 제어 경로를 바꾸지 않는 SSH 표시 전용 중계. 원래 필드와 `TransformStamped` timestamp를 보존하며 /tf, /tf_static, /robot_description, /scan, /plan, /amcl_pose만 전달한다. /tf는 자식 프레임별 최신 변환을 합쳐 전달하므로 메시지 경계 자체는 유지하지 않는다. /cmd_vel, action, initialpose, 서비스는 전달하지 않는다. 중계의 실제 화면·TF 연결 검증은 별도로 기록한다.

## 6. 백그라운드 실행 구분

파이의 베이스·RGB-D·관측기·Nav2 세션은 각각 하나였다. `lifecycle_manager_keepout`, `lifecycle_manager_localization`, `lifecycle_manager_navigation`은 서로 다른 역할로, 같은 주행의 3중 실행이 아니다. 관측기의 `ros2 run` 부모와 Python 자식도 서로 다른 검출기가 아니다.

종료 확인 당시 이동 스크립트와 rosbag 기록기는 남아 있지 않았다. 종료 뒤 3초 동안 odom 83개에서 선속도/각속도 최대 모두 0.0이었다. GUI의 백그라운드 숫자가 어느 목록을 뜻하는지는 이 파이 프로세스 목록만으로 확정하지 않는다.

## 7. 출발 요청을 수정/리뷰 단계로 바꾼 운영 실수

출발·재출발 도중 새 코드 작성, 앵커 재주행, 반복 관측과 검증을 이어가 실제 주행을 늦췄다. 해결되지 않은 센서 목표 문제를 남겨 둔 상태에서 접근을 이어가겠다고 예고한 것도 부정확했다.

재발 방지: 저장소 `AGENTS.md`에 출발/재출발을 기존 실행 경로만 사용하는 단계로 명시한다. 실패는 즉시 로그에 남기고 기존 기능으로 재개 가능한 구간만 이어간다. 코드·리뷰·빌드는 주행 종료 후 또는 요청된 충전 중 수정 단계에서 수행한다. 배치 확인을 다시 묻지 않고, 무의미한 같은 실패 반복을 하지 않는다.

## 8. 회전 탐색이 앞뒤 이동으로 바뀐 문제

관측 위치에서 동일 XY와 변경된 yaw를 NavigateToPose 주차 목표로 보냈다. RPP는 XY 허용 범위 밖에서는 목표 방향보다 경로 추종을 선택하므로, 이를 제자리 탐색 명령으로 쓰면 앞뒤 이동이 섞일 수 있다. 당시 behavior_server는 Wait만 로드해 Spin action을 사용할 수 없었다. 이것은 탐색 명령 선택과 실행 설정의 결함이며, 모든 왕복 현상이 이 원인 하나였다고 단정하지 않는다.

수정: 정밀 서비스에서만 Spin을 로드하고 `box_service --search`로 최대 30도씩 12회 탐색한다. Spin 전후 새 정지 자세로 회전량과 XY 이동량을 확인하며, 박스 면이 확보되면 45cm 정렬→5cm 접근으로 같은 실행기 안에서 이어진다. 관측 지점에 이미 도착했을 때는 `--resume-at-observation`으로 transit만 건너뛴다. 가까운 최종 접근 단계에는 탐색 회전을 넣지 않는다. 명령은 기존 감속기와 충돌 방지 경로를 통과한다.

센서 지연·비정상 scan metadata·geometry 오류를 박스 미검출과 구분한다. 회전으로 회복 가능한 면 관측 실패만 명시적 목록으로 재시도한다. 리뷰에서 발견한 모든 witness ValueError를 회전 재시도로 처리하던 결함도 수정했다.

제약: 회전 공간이 막혔을 때 전후진해 공간을 확보하는 자동 복구는 이번 구현에 포함하지 않았다. 당일 약 0.106m 전진과 약 0.115m 후진 기록은 개별 회피 동작이지 통합 복구 또는 주차 성공 근거가 아니다.

## 9. 후진 goal 수락 후 예외와 속도 설정 불일치

후진 실행기는 waypoint가 하나여도 이벤트 기록에 인덱스 1을 사용했다. goal 수락 후 IndexError가 날 수 있었고, handle을 기록 함수 이후 등록해 예외 시 취소 추적도 불완전했다.

수정: 마지막 waypoint 인덱스를 사용하고 빈 경로는 goal 전송 전에 거부한다. 수락한 handle은 기록보다 먼저 보관한다. NavigateToPose·FollowPath·Spin을 각각의 UUID와 action 취소/결과 서비스로 종료 확인한다.

별도로 박스 주차 계약의 0.04m/s와 서비스 launch의 smoother 상한 0.08m/s가 달랐다. 런타임 임시 변경은 재기동 시 유지되지 않았다. 후면 주차 서비스의 전진/후진 smoother 제한을 계약에 맞춰 생성하도록 수정했다. 회전 탐색 속도 역시 주차 계약에서 가져온다.

## 10. 18:29 센서·제어 지연 및 연쇄 비활성화

보존 journal에서 scan 입력 2.637초 지연, 10Hz controller 주기 미달, TF 미래 외삽 오류, keepout heartbeat 10초 미수신에 따른 lifecycle 비활성화가 확인됐다. 후속 재접근도 collision ahead와 controller patience 초과로 실패했다. 이를 모두 105로 묶지 않는다. 정밀주차는 완료되지 않았다.

높은 시스템 부하가 관측됐지만 CPU 부하만을 확정 원인으로 삼을 근거는 부족하다. 표시 중계는 구독 큐를 최신 1개로 줄이고, TF를 callback마다 재조합하지 않고 송신 시 조합하도록 바꿨다. 새 exporter는 낮은 CPU 우선순위로 실행된다. 이는 표시 부하 완화이며 온보드 scan 지연의 근본 해결 실측은 아니다. 충돌 방지 입력 timeout이나 lifecycle timeout을 늘려 오류를 숨기지 않았다.

충전 중 `jdamr-restaurant-navigation`을 종료했고 inactive를 확인했다. 센서 서비스는 유지했다. 수정 검증 중 실차 목표나 속도 명령을 전송하지 않았다.

19:07 정지 상태에서 기존 읽기 전용 probe로 15초를 측정했다. scan 142개, 9.638Hz, stamp age 최대 0.118초, 0.2초 초과 수신 간격 0회였다. odom 740개, 50.007Hz이며 선속도·각속도·변위 최대 모두 0이었다. Nav2가 꺼진 상태의 기준선이므로 주행 부하에서의 지연 해결 증거는 아니다. 파이 소스에 probe 파일이 없어 첫 호출은 실행되지 않았고, 로컬의 기존 probe를 SSH 표준입력으로 전달해 측정했다. 새 이동 코드는 만들지 않았다.

## 11. 반복 설정 조회에서 ROS service client 누적

관측용 지도 확인·탐색 회전 설정·후진 속도·costmap footprint 조회마다 `AsyncParameterClient`를 새로 만들었다. 설치된 Jazzy 구현은 인스턴스마다 6개 service client를 만들며, 지역 변수만 사라져도 Node가 보유한 client는 남는다. 회전 설정 조회 12회를 실제 rclpy Node에서 재현했을 때 72개가 남았다. 응답은 대체했지만 client 생성과 개수는 실제 rclpy 객체로 측정했다.

수정: `ServiceRoute._read_parameters()`가 대상 노드별 `GetParameters` client 하나를 재사용한다. 반환된 설정값은 캐시하지 않고 매번 서버에서 새로 읽는다. 사용하지 않는 다른 5개 parameter service의 발견을 기다리지 않는다. 기존 서비스 발견 2초·응답 2초 상한을 유지하고, 중단·timeout 때 미완료 요청을 client에서 제거한다. 지도 식별·footprint·속도·회전 설정 비교와 충돌 방지 조건은 유지했다.

검증 결과:

- 수정 전 회귀 실패: 12회 조회 뒤 client 72개. 수정 후 같은 반복에서 1개 유지.
- 새 설정값 수신, 서비스 부재 시 요청 금지, 중단 시 pending request 제거를 검증했다. GetParameters만 제공하는 실제 ROS 서버에서도 두 번의 서로 다른 응답을 받았다.
- restaurant service·box service·activate navigation 관련 229개 테스트 통과. 변경 Python 2개 파일 ament_flake8, py_compile, diff 검사 통과.
- 파이의 실행 모듈 SHA-256은 로컬과 같은 `4087e2cd05c87e18c85cfc8980143cf22758ea0ded0cc9a016071592beae8998`이다. 파이 관측 노드의 `processing_hz`를 12번 읽어 모두 2.0을 받았고 client는 1개였다. 해당 조회 구간은 0.283초이며, 서비스 시작·Nav2 활성화·실차 태스크 시간을 뜻하지 않는다.
- 파이 원본은 `$HOME/jdamr_data/restaurant_service.before_parameter_clients_20260929.py`에 보존했다. Python 모듈은 빌드 경로에서 소스를 참조하므로 다음 실행부터 적용된다. Nav2·모터 서비스를 재기동하거나 이동 goal/cmd_vel을 보내지 않았다.
- 독립 리뷰 에이전트는 런타임 thread limit으로 시작하지 못했다. 작성과 분리한 검토 패스에서 호출부·값의 신선도·실패 전파·pending request 정리·수명 종료를 확인했다. 독립 승인으로 표시하지 않는다.

연결 누적 결함은 재현·수정했다. 하지만 이것이 18:29 heartbeat 상실의 원인이었다는 인과는 아직 확인하지 못했다. 당시 생성 client 수와 장애 시각의 대응 기록이 없으므로 모든 지연의 해결이라고 보고하지 않는다. 기존 `BoxService.precision_parameters`와 `CorridorRoute.parking_parameters`는 인스턴스를 재사용하므로 이번 누적 결함과 구분해 그대로 두었다.

파이 조회 출력은 `$HOME/jdamr_data/nav2_architecture_20260929_tF5td9/parameter_reader_pi_check.json`에 보존했다. 구성 판단과 남은 계측 조건은 [Nav2 실행 위치 조사](20260929_NAV2_PLATFORM_RESEARCH.md)를 따른다.

## 12. 개발 우선 운영과 불필요한 태스크 종료 축소

사용자 요청: 교실·복도의 저속 학습용 시제품에서 실제 태스크 구현·실행을 최우선으로 한다. 반복 출발 검수·코딩·리뷰로 배터리를 소모하거나, 일시적인 문제마다 처음부터 재주행하는 과정을 줄인다. 저장소 `AGENTS.md`에 이후 세션의 우선순위로 기록했다. 차체 손상 감수 의사를 사람·물건과의 충돌 허용 또는 센서 없이 이동하는 권한으로 확대하지 않는다.

변경 범위:

| 기존 처리 | 반영한 처리 |
|---|---|
| 입력 지연 시 전체 경로 종료 | 일반 CorridorRoute와 ServiceRoute는 이전 action의 terminal 확인 후 새 입력·새 정지 odom을 기다려 현재 waypoint부터 1회 재개. 대기는 최대 8초이며 정상 출발에 고정 대기를 추가하지 않음 |
| 탐색·후진의 입력 지연 후 처음부터 재시작 | Spin은 남은 각도만, 후진은 현재 실제 pose부터 경로를 다시 만들고 충돌 확인 후 재개 |
| 매 이동·회전에 출발 예비전압 적용 | 첫 action 수락 전 10.8V, 같은 실행기의 후속 단계는 기존 진행 중 10.5V. 저전압·비정상 값 차단은 유지 |
| 45cm 중간 정렬에도 1cm/1도 최종 판정 | 저속 Parking 제어기는 유지하고 중간 정렬용 5cm/3도 goal checker 분리. 최종 5cm 간격·양쪽 모서리·1도 판정은 유지 |
| 최종 접근의 단발 경로 계산 | 1Hz 재계획 및 복구 가능한 계획/제어 오류에 한해 1초 대기 후 1회 재시도. 가까운 물체 앞 임의 Spin/BackUp은 넣지 않음 |
| 탐색 Spin 충돌 차단 즉시 태스크 종료 | COLLISION_AHEAD가 확인된 경우에만 앞·뒤 20cm 관측 후보를 계획하고 가능한 첫 위치로 1회 이동 시도. 속도 명령을 우회 발행하지 않음 |
| 같은 지도 파일·서버 설정 조회 반복 | 동일 실행기에서 확인한 지도 자산을 재사용. 실제 grid 변경과 명령 경로 검사는 유지 |
| 준비 시간까지 240초 태스크 예산에 포함 | 준비가 끝나고 이동을 시작하는 시점부터 계산 |
| ROS graph 누락으로 전체 Nav2 종료 | graph 감시기는 진단 알림으로 한정. 일반 주행·자율 매핑 launch에서 감시기 종료를 전체 shutdown 조건에서 제외. 핵심 프로세스 종료·lifecycle bond 보호는 유지 |
| 기동 완료 후 후속 조회 실패에도 RESET | 기동·ACTIVE 확인이 끝난 서버는 유지하고 준비 미완료를 보고. STARTUP 자체 실패·부분 활성화는 기존 rollback 유지 |
| 세션 스크립트 기본 SUBNET | 현재 온보드 제어 기본 LOCALHOST와 일치. 명시한 discovery-range 옵션은 유지 |

스킬 확인:

- 설치된 개인 스킬에서 주행 진단에 해당하는 것은 `ros-graph-triage`다. 모터 제어 코드는 없으며 진단 지침이다. 출발/재출발 자동 적용을 제외하고 Codex `allow_implicit_invocation: false`로 바꿨다. PKM 원본을 수정하고 설치본을 동기화한다.
- `arm-motion-gate`·`arm-motion-guard` 설치 스킬과 해당 Codex 훅은 이번 조회에서 발견되지 않았다. 없는 스킬을 제거했다고 기록하지 않는다.
- `servo-register-triage`는 팔 서보 진단용이며 이번 베이스 주행 차단 원인으로 확인되지 않아 변경하지 않았다. OMX·리뷰 스킬 전체를 삭제하지 않고 출발 단계의 새 필수 절차로 사용하지 않도록 프로젝트 규칙에 명시했다.

유지한 조건: 실제 footprint·keepout, CollisionMonitor, 유효 입력, 명령 watchdog, 저전압 cutoff, 사용자 정지. 정밀 접근 앞 여유 5cm·costmap 차체 패딩 2cm 및 회전 swept 영역은 변경하지 않았다. 정지거리·입력 지연의 실측 근거 없이 수치를 낮추지 않는다. 수동 조종에는 사용자가 버튼을 놓은 뒤 자동으로 다시 움직이는 기능을 넣지 않았다.

검증: 관련 629개 테스트 통과, 변경 핵심 Python 9개 파일 ament_flake8 통과, 셸 구문·diff 확인 및 로컬 navigation 패키지 빌드 통과. 입력 공백 시 취소 terminal 뒤 현재 waypoint만 재전송하는 시나리오, 남은 Spin 각도와 후진 경로 재계산, 지도 자산 재사용 후 grid 변경 거부를 검사했다. 독립 리뷰 에이전트는 thread limit으로 시작되지 않아 작성 후 별도 로컬 검토 패스를 수행했으며 독립 승인으로 표시하지 않는다.

이번 변경은 18:29 센서·제어 지연의 근본 원인이 사라졌다는 증거가 아니다. 박스 면 후보가 바뀌는 인식 문제도 별도이며, 수정본의 실제 주차 성공·소요시간 단축·충돌 회피 성능은 아직 측정하지 않았다. 새 거리 완화나 실물 주행은 실행하지 않았다.

파이 반영: 변경 파일을 기존 소스에 반영하고 navigation 패키지 빌드, `box_service --help`, 실행 모듈 구문 확인을 완료했다. 원본 백업은 파이 `$HOME/jdamr_data/development_flow_20260929_T8eNdh/before.tar.gz`, 반영 목록은 같은 디렉터리의 `changed_paths.txt`다. 주행 세션은 inactive를 유지했고 베이스 MainPID 17643은 변경되지 않았다. 새 정렬 goal checker·BT 설정은 다음 Nav2 세션 시작 때 로드된다. 실행 중 센서·모터 서비스 재시작이나 이동 명령은 없었다.

## 13. 개발 우선 변경의 실행 경로 추가 검증

사용자 요청에 따라 cb21ce1의 구현을 다시 확인했다. 앞선 629개 테스트 통과만으로 반영이 충분했다고 볼 수 없었다. 다음 네 가지 누락을 확인해 수정했다.

| 확인된 문제와 근거 | 수정 및 확인 |
|---|---|
| 지도 검사 캐시 키 문자열을 서버 확인 반복문의 지도 dict가 덮어썼다. 첫 호출부터 두 번 실행하니 parameter 조회가 기대 2회가 아니라 4회였다. 기존 테스트는 캐시를 미리 넣어 이 결함을 놓쳤다. | 캐시 키 변수를 분리. 첫 호출에서 생성한 캐시를 두 번째 호출이 재사용하는 검사를 추가했다. 실제 grid 변경과 명령 경로 확인은 유지한다. |
| action 수행 중 입력 공백은 재개했지만 다음 waypoint 발행 직전 공백은 재개 이유를 기록하지 않고 종료했다. 첫 waypoint 성공 후 다음 입력만 늦추면 전체 실행이 실패했다. | 일반 경로·테이블 경로·Spin·후진의 action 시작 경계에도 기존 입력 회복 분기를 연결. 완료 waypoint를 건너뛰는 전환과 사용자 정지·저전압 미재개를 확인했다. |
| 탐색 Spin의 COLLISION_AHEAD만 관측 위치 변경 대상이었다. 로컬 20초 제한으로 취소하면 오류 코드가 None이어서 이 분기에 진입하지 못했다. | 정상 입력·사용자 정지 아님을 확인한 로컬 만료를 TIMEOUT으로 기록한다. 취소 terminal 확인 후 COLLISION_AHEAD 또는 TIMEOUT에 한해 기존 앞·뒤 20cm 후보 계획을 1회 시도한다. TIMEOUT을 충돌 원인 확정으로 해석하지 않는다. |
| 세션 셸은 LOCALHOST였으나 별도 restaurant_service.launch.py 기본값은 SUBNET이었다. | 현재 온보드 센서와 같은 LOCALHOST로 통일. 명시적인 SUBNET 옵션이 보존되는 것도 검사했다. |

검증 근거:

- 수정 전 새 재현 검사에서 지도 캐시·waypoint 경계 두 실패를 확인하고 수정 후 통과했다.
- 관련 14개 테스트 파일에서 651개 통과. 변경 Python/launch/test 6개 파일 ament_flake8 통과, 로컬 navigation 빌드 통과.
- 로컬 변경과 기존 파이 실행 소스가 cb21ce1 기준으로 일치하는지 대조한 후 여섯 파일만 백업·반영했다. 파이 navigation 빌드와 설치된 Python 모듈 import가 통과했다. 설치된 Spin TIMEOUT 값은 701이다.
- 파이 백업: `$HOME/jdamr_data/development_verify_20260929_ENEC2Z/before.tar.gz`. 같은 디렉터리에 반영 목록·SHA-256·빌드 로그를 보존했다.
- 반영 전후 주행 서비스는 system/user 모두 inactive, 베이스 MainPID는 17643으로 같았다. 센서·모터 서비스 재시작, action 실행, 속도 명령 발행은 하지 않았다.
- 독립 리뷰 에이전트는 thread limit으로 생성되지 않았다. 작성 후 별도 로컬 diff·회귀 검증 패스를 수행했으며 독립 승인으로 표기하지 않는다.

`ros-graph-triage`는 이번에 진단 절차로 실행하지 않고 지침 내용을 검토했다. 모터 제어·제동 실행부가 아닌 TF·통신·Nav2 최초 장애를 좁히는 읽기 전용 지침이며, 안전 차단만을 목적으로 하는 스킬은 아니므로 삭제하지 않았다. PKM 원본과 Codex 설치본의 `allow_implicit_invocation: false`, 저장소의 출발 자동 적용 금지를 확인했다. 이번 변경에서 새 스킬·검수 단계·출발 조건을 추가하지 않았다.

남은 검증 범위: 실차 주차 완료·주행 지연 감소는 측정하지 않았다. 반복 센서 지연의 근본 원인, Nav2 실행 중 비활성화 재발 여부, 박스 면 후보 변경 문제까지 해결됐다는 증거는 아니다. 위 네 코드 결함의 수정 및 반영과 실차 태스크 성공을 구분한다.

## 14. 정지 캡처·관측 입력 공백·계획 끝점·박스 이탈 복귀 (2026-09-30, Claude 인계 후)

인계 문서 `20260930_CLAUDE_HANDOFF.md` §4의 미수정 두 경로와, 이어받으며 보존 기록에서 새로 확인한 실행 경로 결함을 수정했다. 계획은 OMC ralplan 합의(Planner·Architect·Critic 3회차, 정오표 반영 v4.1)를 거쳤다. 이 절은 소프트웨어 수정과 파이 반영까지의 기록이며 **실차 주행은 하지 않았다.** 5 cm 주차·복귀 성공은 주장하지 않는다.

### 14.1 확정 사실과 근거

| 사실 | 근거 |
|---|---|
| AMCL은 odom 축별 변위가 0.05 m/0.05 rad를 넘을 때만 갱신·발행한다. 정지 중에는 `/amcl_pose` 없이 map→odom TF만 scan마다 재발행한다. 그래서 같은 프로세스에서 15 s 넘게 정지한 뒤 `capture_stationary_pose()`와 `wait_until_ready()`의 guard(True)가 TF가 정상이어도 `AMCL pose stale`로 막혔다 | nav2_amcl 1.3.12 소스, `new_base_nav2_params.yaml` update_min_d/a, 합성 재현(인계 §4-A) |
| 새 프로세스는 TRANSIENT_LOCAL `/amcl_pose`를 받는 순간 나이가 0이다. 막히는 것은 같은 프로세스 안의 긴 정지 뒤다 | `_amcl_callback` 수신 시각 기준 |
| `observe_target()`은 입력 공백 1회에 회복 없이 종료했다 | 인계 §4-B 재현 |
| `observe_target()`은 같은 관측 status를 매 반복 다시 평가했다. 수신 뒤 나이가 0.5 s를 넘으면 신선도 실패로 판정이 덮여, 보존 bag 기준 창 끝의 약 27 %가 재시도 불가로 끝났다 | 보존 MCAP status 563건 재계산 |
| 관측 transit BT의 기본 goal checker는 방향을 보지 않는 `position_goal_checker`다. 9/29 네 관측 실행은 모두 (1.446, 0.243)에서 실제 yaw 61.7°였고, 영역 표지 (1.896, 0.303)는 카메라 반시야각 약 29° 밖이었다. 번갈아 나온 앞면 후보는 다른 면이었을 가능성이 크다(지정 박스 식별은 미확정) | 보존 MCAP 재투영, BT XML |
| 플래너 tolerance 0.5에서 `plan_pose()`는 목표에서 떨어진 곳에서 끝나는 경로도 성공으로 받았다(플래너 비교 범위 0.15–0.26 m, 현재 NavFn은 0.15 m) | 실제 지도 오프라인 planner 비교 |
| KeepoutFilter 출력은 팽창되지 않고 unknown 셀을 FREE로 덮어쓴다. 점 로봇 플래너가 keepout 경계에 중심을 붙여 계획한다. 깊이로 계산한 박스 면 후보는 keepout 띠(y ≥ 0.619) 안이라 5 cm 목표가 keepout 안에 떨어진다 | 같은 오프라인 비교 |
| 박스 앞 5 cm 정지 자세에서는 LiDAR(range_min 0.28 m)와 깊이(최소 0.35 m) 모두 면을 보지 못한다. 그 자리의 제자리 회전은 collision monitor가 막지 못한다 | provenance, `new_base_geometry.yaml` |
| 정밀 세션은 box 계약으로 Parking·ParkingReverse를 띄우는데 `restaurant_service`는 일반 계약 파일을 고정으로 읽어, 같은 세션의 `home --execute`가 이동 전에 결정적으로 실패했다 | launch·CLI 코드 |
| 9/29 실행 중 한 번은 관측 위치로 가던 중 AMCL x 공분산이 0.0102 m²(한계 0.01)로 잠깐 넘어 재개 경로 없이 끝났다. 주행 중 공분산은 수렴 뒤 0.002–0.009 m²였다 | 파이 events.jsonl, 보존 MCAP `/amcl_pose` |

보존 로그에는 `AMCL pose stale`·`stationary teaching unavailable` 종료 기록이 없다. 위 AMCL 나이 결함을 과거 출발 실패의 원인이라고 단정하지 않는다.

### 14.2 조치 (커밋 `2fbed38`)

| 결정 | 내용 |
|---|---|
| D1 | `ServiceRoute._guard_failure(True)`는 마지막 AMCL pose 이후 odom 축별 변위가 AMCL 갱신 임계 이하이고 map→base_link TF가 0.5 s 이내일 때만 메시지 나이 조건을 면제한다. 공분산·지도·배터리 등 나머지 검사는 그대로다. 초기화 전·이동 후·TF 정지·미래 시각은 계속 거부한다 |
| D2 | 관측 중 입력 공백은 함수당 1회 기존 회복으로 기다린 뒤 새 정지 pose와 새 cutoff로 다시 관측한다. 실패는 plain `RuntimeError`라 탐색 회전으로 이어지지 않는다. status는 stamp별로 한 번만 평가한다. 판정을 만든 지연은 회전 대신 무이동 재관측 1회로 처리한다 |
| D3 | `_verify_parking_stop()`은 입력 공백에 명령 없이 1회 재확인한다. box 계획 시점 공백은 공백 전 목표를 버리고 다시 관측한다 |
| D4 | (이 커밋 시점) 이동 중 공분산 초과는 코드를 바꾸지 않았다. 사용자 답변(불필요하면 제거)에 따라 §15.2에서 중간 단계 한도로 바꿨다 |
| D5 | `box_service --return-home --return-timeout-s <s>`: 최종 접근 성공 뒤 검증된 5 s 대기 → 박스 이탈 → 충전소 복귀를 한 실행기에서 이어간다. 복귀 도킹은 충전소 계약(5 cm/3°)으로 판정한다. `--task-timeout-s`로 테이블 구간 예산을 받는다(기본 240 s) |
| D6 | 관측 전 카메라 자세·영역 방위·시야 포함 여부·거리를 기록하는 무이동 진단 이벤트를 남긴다 |
| D7 | `plan_pose()`는 경로 끝이 goal checker 허용오차 밖이면 `goal_not_reachable_within_tolerance`로 실패한다. box transit preflight도 같은 검사를 한다 |
| D8 | 이탈은 박스 면에서 0.565 m(StopZone 회전 반경 0.43 + 여유)까지 직선 후진한다. 검증은 현재 차체 앞쪽 띠를 뺀 꼬리로 한다(직선 후진은 그 띠를 다시 쓰지 않는다). 재시도는 이미 후진한 거리만큼 띠를 줄인다. 이탈 전 방향·거리 타당성을 확인하고, 실패 사유는 `box_escape_*`로 남긴다 |
| D9 | 관측 실패 시 최근 status 16개·정지 pose·scan·mount를 묶은 증거 이벤트를 남긴다. 새 이벤트는 `allow_nan=False`로 직렬화할 수 있고, 증거 기록 실패가 제어 흐름 예외를 바꾸지 않는다 |

`restaurant_service home`만 `--parking-contract`를 받는다. 박스 앞 약 0.6 m 안에서는 이 단독 복귀를 쓰지 말라는 안내를 도움말에 넣었다(센서가 그 거리의 면을 보지 못한다).

### 14.3 검증

- 테스트를 먼저 작성했다. 수정 전 HEAD `5ab15ee`에서 회귀 테스트 49건이 계획한 사유(`HEADFAIL[Tn]`)로 실패했고, 하네스 오류·수집 오류는 0건이었다. 기존 테스트는 모두 통과했다.
- 수정 후: 핵심 5개 파일 434건, 관련 24개 스위트 934건이 통과했다.
- 변이 M1–M12가 모두 지정 테스트에서 검출됐다. M2에서 살아남은 세 사례(지도 불일치·사용자 정지)는 앞선 `stop_requested` 경로 때문에 도달할 수 없는 등가 사례다.
- 독립 코드 리뷰에서 차단 결함은 없었다. 지적된 이탈 재시도 검증 구멍, 이탈 전 타당성, 복귀 checker 확인 순서 등은 테스트를 먼저 실패시킨 뒤 고쳤다.
- 변경 파일의 flake8·pep257 위반은 0건이다. 패키지 전체의 기존 flake8 위반 29건은 HEAD와 같다.
- PC와 파이에서 `colcon build --packages-select jdamr_cube_navigation`이 통과했다.

### 14.4 파이 반영

- 반영 전 파이 런타임 3파일은 `5ab15ee`와 같았다. 백업·반영 기록: 파이 `$HOME/jdamr_data/development_verify_20260930_ad77a4/`(`before.tar.gz`, `changed_paths.txt`, `build.log`, `deployed_sha256.txt`, `no_motion_cli_check.txt`).
- PC 커밋, 파이 소스, 파이 설치 site-packages의 런타임 3파일 SHA-256이 모두 같다.
- 베이스 MainPID는 반영 전후 44342로 같고, 주행 서비스는 inactive를 유지했다. Nav2 기동·action·속도 명령은 없었다.
- 무이동 점검: 새 CLI 옵션, `--return-timeout-s` 없는 `--return-home` 거부, departure_mask_v2 registry·경로의 지도 식별 일치, 계약·카메라 장착 로드를 확인했다.
- 운영 관찰: 2026-09-30 06:52–06:56에 무인 보안 업데이트(openssl·curl·expat·sudo 등, ROS 패키지 아님)가 라이브러리 교체 뒤 `jdamr-base`·`jdamr-box-observer`·`jdamr-box-rgbd`·ssh·네트워크 서비스를 재시작했다. 새 커널 6.8.0-1065가 설치되어 재부팅을 기다린다(실행 중 1064). 주행 중 같은 재시작이 일어나면 태스크가 끊긴다. 업데이트 정책은 사용자 결정 사항으로 남긴다.

### 14.5 남은 항목 (출발 전 Phase 2, 사용자 답변 필요)

- 지정 박스의 실제 위치와 붙을 면: 현재 데이터로는 면 후보가 keepout 안이라 최종 접근 계획이 끝점 검사에서 실패한다. 답변에 따라 region·관측 waypoint·keepout 사본을 만들고, 실측 면 선분·LiDAR 잡음 0–0.02 m·줄자 ±0.01 m 조합을 오프라인으로 검증한 뒤 출발한다.
- 관측 지점 옆 지도에 없는 물체(폭 약 0.32 m)의 유지 여부, 이동 중 공분산 정책, 실제 충전소와 registry home_dock(route 시작점과 0.2 m 차이)의 일치 여부.
- 첫 주행은 진단 성격이 크다. 5 cm 간격과 0.05 m costmap 격자가 겹쳐 최종 접근이 목표 직전에 멈출 수 있다. 면 셀과 2 cm 잡음을 둔 격자 모델(추론, 실측 아님)에서 최종 자세 footprint 외곽선이 면 셀과 겹쳐 목표 직전에 멈출 비율은 면이 지도 축과 평행해도 약 80 %였다(잡음 없이 면 셀만이면 0° 40 %, 30°·45° 80–96 %). 최종 목표 셀 자체가 inscribed가 되어 끝점 검사가 이동 전에 `goal_not_reachable_within_tolerance`로 거부할 비율은 0° 약 20 %, 15° 36 %, 30° 80 %, 45° 60 %였다.
- 이탈 시작 자세에서 RPP 현재 자세 충돌 검사가 접근 중 남은 costmap 면 셀과 겹치면 `box_escape_failed`로 멈춘다(fail-closed). ObstacleLayer footprint clearing이 이 셀을 지우는지는 실차에서 확인하지 않았다.
- 활성화 단계의 공분산 통과는 `/request_nomotion_update` 반복 뒤의 값이라 위치추정 품질의 독립 증거가 아니다.
- status를 stamp별로 한 번만 평가하면서, HEAD가 같은 status를 0.5 s 안의 새 scan으로 LiDAR witness에 다시 넣던 암묵적 재시도도 사라졌다. 첫 주행에서 `box_lidar_witness_rejected` 빈도를 본다.
- 리뷰에서 보류한 알려진 한계(회복 뒤 cutoff의 시간축 혼합, D1 참조 시점 서술, 잘못된 관측기 JSON의 침묵 판정, 정밀 세션 전진 home 미지원 등)는 보존 폴더의 `review/code_review_phase1.md`에 있다.
- 권고(설정 변경 없음): inflation_radius 0.42 m 이상은 오프라인에서 NavFn 여유를 늘렸고 불가능해진 목표가 없었다. keepout 마스크를 차체 여유만큼 팽창하는 것도 권한다. 플래너는 NavFn을 유지한다(Smac Hybrid는 Ackermann용, 비원형 차동의 공식 권장인 Lattice는 이 지도에서 큰 루프·keepout 침범을 보였다). 정밀 세션 footprint 앞변(물리 0.065 m) 또는 costmap 해상도 0.025 m는 사용자 선택 항목이다. 파이는 대기 중에도 74–78 °C라 냉각을 권한다.
- 근거 자료 보존(PC): `$HOME/jdamr_data/claude_phase1_20260930/`(합의 계획·검토, 박스 면·지연 분석, 오프라인 planner 비교, HEAD 실패 증명, 변이·리뷰 기록).

## 15. 중간 단계 공분산 한도, 파이 DDS 공유메모리 삭제, 9/29 초기 자세 오차 (2026-09-30 오후)

### 15.1 확정 사실과 근거

| 사실 | 근거 |
|---|---|
| 09:0x 파이에서 새로 뜬 ROS 참가자는 3 s 안에 `/scan`·`/odom`·`/tf`와 발행자를 발견하면서도 20 s 동안 메시지를 0개 받았다. 새 참가자끼리는 정상 수신(12 s 38개) | 읽기 전용 probe 3종 |
| 원인: 센서 서비스 3개는 시스템 유닛이지만 `User=lim`이고, logind `RemoveIPC`는 기본 yes, `Linger=no`였다. 08:52:38 PC의 `jdamr-rviz-display-relay`·`robot-dashboard-link` 정지로 파이 lim의 마지막 ssh 세션이 끝나 08:52:49에 `user-1000.slice`가 제거되면서 `/dev/shm/fastrtps_*`가 지워졌다. 기존 노드의 매핑은 `(deleted)`로 남아 기존 노드끼리만 통신했다 | `/proc/<pid>/maps`, `/dev/shm` 0개, 파이·PC journal 시각 대조 |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`를 준 LOCALHOST 모드 프로세스도 공유메모리 포트를 매핑하고 있었다 | 같은 maps |
| 조치(사용자 실행): `sudo loginctl enable-linger lim` 뒤 `jdamr-base`·`jdamr-box-rgbd`·`jdamr-box-observer` 재시작. 이후 새 참가자 수신 19 s에 `/scan` 156·`/odom` 774·`/tf` 469. 베이스 드라이버는 시작 시 속도 명령 없이 정지 명령만 보낸다(`base_driver_node.cpp:168-172`) | probe 재실행, `loginctl show-user` |
| 9/29 출발 대기 구간(0–54 s)의 AMCL 자세 (−0.622, −0.878, 87.7°)는 같은 스캔 40개를 지도에 정합한 자세 (−0.533, −0.896, 93.3°)와 9 cm·5.5° 달랐다. AMCL 자세로 투영하면 남쪽 벽이 약 5° 기울고 5 cm 안 점은 17 %, 정합 자세는 64 %였다. 관측 지점 정지 구간도 8.5 cm 차이였다 | 보존 MCAP `/scan`·`/tf`, 오프라인 likelihood-field 정합(전역 탐색에서 유일해) |
| 9/29 출발 위치(정합 자세)는 registry `home_dock`에서 0.30 m 떨어져 있었다 | 같은 정합 |
| 정합 자세로 다시 투영하면 앞서 박스 후보로 보던 L자 형태는 지도의 큰 장애물 서쪽 벽(x≈2.33)과 겹친다. 9/29 출발 위치에서 LiDAR 높이로 table_1·table_2 표지 주변의 지도에 없는 박스 크기 물체는 확인되지 않았다 | 같은 투영 |
| 9/29 관측 transit 도착 시점 공분산 5건은 x 0.0033–0.0071, y 0.0017–0.0062였다. 정지 뒤 재개 실행 3건은 같은 값을 그대로 기록했다(정지 중 AMCL 미발행) | 파이 events.jsonl(독립 리뷰가 집계) |

9/29 AMCL 오차가 생긴 과정(시드 근처 nomotion 반복으로 입자가 실제 자세에 닿기 전에 모였을 가능성)은 추론이며 재현하지 않았다.

### 15.2 조치

| 결정 | 내용 |
|---|---|
| D10 | AMCL 공분산 한도를 단계별로 나눴다. 중간 단계(관측 경로 transit, 관측·탐색 회전·면 정렬, 관측 지점 재개 준비, 정밀 세션 복귀 staging의 계획·주행)는 위치추정 상실 한도만 본다. x·y 0.04 m²(0.2 m)는 9/29 수렴 상태 최대 0.010213과 기록된 미수렴 최소 0.17 사이 값이고, yaw는 미수렴 기록이 없어 초기 pose 시드 분산 (15°)²를 쓴다(근거는 계약 파일 `intermediate_bound_provenance`). 새 출발 준비, 최종 접근, staging 정지 확인, 최종 캡처, 박스 이탈, 도킹 후진, 활성화는 0.01 m²·(10°)²를 유지한다. 비유한·음수 검사는 모든 단계에 남는다 |
| D10 구현 | 계약 키 `intermediate_max_*`와 로더 검증(유한·수치·엄격 한도 이상). `ServiceRoute._localization_bound()`는 단계 값을 정확히 설정하고 finally에서 복원한다. `execute()`는 transit이면 중간, alignment는 호출 단계 상속, final은 항상 엄격이며 입력 공백 회복 대기도 같은 한도를 쓴다. 공분산 실패 메시지에 x·y 값(소수 6자리)과 `bound=intermediate/strict`를, yaw 메시지에 값·한계를 넣고, 정지 캡처 실패에는 마지막 가드 사유를 붙였다. 표준 세션 경로는 바뀌지 않는다 |
| D11 | 파이 lim linger 활성화(사용자 실행). 저장소 AGENTS.md "주행·실험 뒤 정리"에 linger 전제를 추가했다 |

### 15.3 검증

- 새 테스트 25건(T31a–g, T32a–c, T33, T34a–e, T35)과 계약 로더 테스트 7건을 먼저 작성했다. 수정 전 코드에서 20건이 계획한 사유로 실패했다(`HEADFAIL[Tn]`, `AMCL x covariance high: value=0.010 limit=0.010`, `localization or sensor data unavailable`, 단계 한도 기능 없음). 수정 전에도 통과한 5건(T31c 2건, T34a, T35 2건)은 수정 전후 모두 0.01·미수렴에서 멈춰야 하는 음성 대조군이다. 로더 테스트 7건은 수정 전 모두 실패했다.
- 변이 6종(final의 중간 한도 상속, 새 출발 준비의 중간 한도, final 단계의 중간 한도, finally 복원 제거, staging 정지 확인의 중간 한도, 엄격 한도 상향)이 모두 지정 테스트에서 검출됐다.
- 수정 후 핵심 7개 파일 568건 통과. 패키지 전체를 같은 조건의 HEAD 사본과 비교해 새 실패 0건. 변경 파일 ament_flake8·pep257 0건.
- 독립 코드 리뷰 1차(transit만 완화한 초안): APPROVE, 권고 반영(T31f·T31g, 기록 갱신, 도착 뒤 잠긴 공분산 → 단계 확장). 2차(단계 확장본): APPROVE, 권고 반영(엄격 유지 테스트 T34, 미수렴을 통과시키던 0.25 → 0.04, staging 정지 확인 엄격화, 메시지 정밀도, HEADFAIL 규약).

### 15.4 남은 항목

- 중간 한도는 발산 감지일 뿐 정확도 보증이 아니다. keepout은 map 좌표로 적용되므로 LiDAR로 보이지 않는 금지 영역은 위치추정 정확도에 기댄다.
- 9/29처럼 초기 자세가 틀린 채 수렴하면 공분산 한도는 이를 잡지 못한다. 출발 초기화에서 스캔 정합 자세와 AMCL 자세를 대조한다(운영 절차, 코드 아님).
- 도착 뒤 AMCL 값이 0.04를 넘은 채 잠기면 관측 단계에서, 정렬 뒤 0.01을 넘은 채 잠기면 최종 접근에서, staging 뒤 0.01을 넘으면 도킹 전에 멈춘다. 실패 메시지에 값과 적용 한도가 남는다. 운영자 조치는 충전소 이동 뒤 재초기화다.
- 데이터 폴더의 `resume_table_trial.py`는 `covariance high` transit 중단을 복구 가능으로 보고 nomotion을 반복한다. D10 뒤에는 그 사유가 위치추정 상실을 뜻하므로 쓰지 않는다.

## 16. table_01 박스 탐색 실패와 충전소 복귀 (2026-09-30 11:36–12:14)

데이터 폴더: `$HOME/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930/` (파이 같은 경로). 도킹 성공 자료는 `$HOME/jdamr_data/dock_return_success_20260930/`에 따로 묶었다([포트폴리오 인계](20260930_DOCK_RETURN_PORTFOLIO_HANDOFF.md)).

### 16.1 관찰

| 시각 | 실행 | 결과 |
|---|---|---|
| 11:36–11:41 | `runs/table_01_20260930_113650` (box_service `--search`, 박스 경로 `table_01_route_box.yaml`) | 관측점 (1.0, 0.05) 도착 뒤 박스 면 안정 판정(8프레임·영역 안)을 한 번도 통과하지 못하고 +33° 탐색 회전을 이어 갔다. 박스 쪽 방위에서는 면이 카메라에서 0.42 m라 화면을 넘쳐 대부분 `no_box_surface_candidate`, 다른 방위는 비스듬한 면의 흔들림이나 다른 평면을 골랐다. 탐색 회전 6회는 Nav2 성공(4), 7번째가 703으로 끝났고, 11:41:07 운영자 정지(`jdamr_depart.py stop`)로 종료 |
| 11:46 | `runs/home_20260930_114600` (`restaurant_service home --execute`, 박스 계약) | 움직이기 전 도크 후진 경로 정적 검사 `static_obstacle_unknown_keepout_or_map_boundary`로 종료. 로봇 이동 없음 |
| 11:58–12:01 | `runs/home_20260930_115853` (같은 명령, 16.3 수정 뒤) | 11:59:11 (0.94, 0.07)에서 staging (−0.64, −0.14)로 출발, 12:00:57 도착(106 s, Nav2 4). staging 정지 확인 통과(staging 목표 대비 2.6 cm·1.7°, 1.0 s 유지). 후진 경로 검사 통과, 12:01:00 후진 시작, 12:01:52 FollowPath 중단 `Failed to make progress`(Nav2 6, 오류 105). 서비스 판정 `failed`, 종료 코드 1 |
| 12:01 이후 | 사용자 현장 확인 | 도크 정위치에 도달했다. 끝 구간에서 앞뒤 왕복과 좌우 반복 회전이 있었다 |
| 12:13–12:14 | 무이동 측정 (`final_pose/`) | 도크 목표 대비: map→base_link TF 0.25 cm·+3.83°, 래치된 `/amcl_pose` 4.0 cm·+3.76°, 독립 스캔 정합 3.9 cm(도크 축 2.2 cm·측방 3.2 cm)·−1.17°(inlier 0.61, 전역 탐색 일치). 줄자 실측 없음 |

### 16.2 정정 (커밋 `5462b62` 기록)

`5462b62`의 "중단 직후 정지 확인은 도크 목표 대비 2.6 cm·1.7°"는 틀렸다. `failed` 이벤트의 `confirmation`은 staging 정지 확인 결과가 그대로 남은 값이다(hold 1.0139 s·오차가 staging 이벤트와 같다). 후진 중단 뒤에는 도크 정지 확인이 실행되지 않았다. 도크 최종 자세 수치는 12:14 무이동 측정값뿐이다.

### 16.3 도크 경로 차단 원인과 조치

- 정적 검사(`static_corridor_clear`, 런타임 footprint 0.38×0.58 m, 미확인 = 막힘)가 세 곳에서 막혔다.
  1. staging 자세 차체 왼쪽이 도크 서쪽 keepout 블록 동쪽 열(x −0.93~−0.88)과 수 mm 겹침.
  2. 도크 자세 차체 뒤쪽(y −1.138)이 지도 점유 2칸(x −0.63~−0.58, y −1.08~−0.98)과 겹침. 이 칸은 스캔 정합·AMCL이 1 cm 안에서 일치한 도크 자세에서 차체 안(base_link 뒤 0.19–0.24 m)에 들어가고, 같은 자리 초기화 스캔에서 뒤쪽 0.77 m 안에 반사가 없었다 → 지도 작성 때의 일시 물체로 판단.
  3. 통로 가운데 미확인 1칸(−0.70, −0.51): 도크 초기화 스캔 빔 287개가 통과, 반사 0.
- 사용자 지시("킵아웃 해제하고 도킹 진행해", "킵아웃만들지말고 그냥해")로 keepout을 전부 비운 마스크를 먼저 썼으나, 세션 런치가 `keepout mask contains no blocked cells`로 거부했다(`keepout_mask.py`, 11:53:18). 그래서 원래 keepout에서 staging 회전 원(0.414+0.10 m)과 도크 통로(반폭 0.29+0.10 m)에 걸리는 46칸만 열었다.
- 새 자산 `map_dockfix.yaml`(3칸 수정), `keepout_dockfix.yaml`(46칸 해제), 근거 `dockfix_provenance.json`. 원본 `local_updated.yaml`·`keepout.yaml`과 이전 registry `service_destinations.before_dockfix.yaml`은 보존. registry 해시 교체 → Nav2 세션 재시작 → 현재 자리에서 스캔 정합 초기화(0.933, 0.074, 185.2°; AMCL과 0.4 cm·0.1°, inlier 0.50). 도구 `jdamr_depart.py`의 `MAP_YAML`도 새 지도로 바꿨다. 수정 뒤 staging 회전(185°→92.3°)과 후진 통로가 같은 정적 검사를 통과했다.
- 새 지도에서는 PC 표시 유닛 `jdamr-p2-markers`가 `table annotation belongs to a different map`으로 시작하지 않는다(표시 전용, 주행 무관).

### 16.4 도크 후진 끝의 105 해석

| 구분 | 내용 |
|---|---|
| 확인된 근거 | 정지 뒤 컨트롤러가 보는 자세(map→base_link TF)는 위치 0.25 cm, yaw +3.83°로 도크 계약 yaw 3°를 넘는다. 진행 검사는 `PoseProgressChecker` 5 cm·0.10 rad / 10 s다. 후진 수락 시 AMCL yaw 분산 0.00489 rad²(1σ 4.0°), 종료 시 0.00788 rad²(1σ 5.1°)로 계약 yaw 3°보다 크다. 사용자가 끝 구간 왕복·좌우 회전을 봤다 |
| 추론(미검증) | 위치는 허용 안인데 yaw가 허용 밖이라 goal checker가 도착을 인정하지 않았고, 컨트롤러가 방향을 고치려 왕복·회전하는 동안 10 s에 5 cm·0.1 rad 넘는 진전을 만들지 못해 중단했다. cmd_vel·컨트롤러 내부 오차는 기록하지 않아 확정하지 못한다 |
| 가리지 못한 것 | TF yaw(+3.8°)와 스캔 정합 yaw(−1.2°)가 5° 다르다. 도크 목표 yaw 92.3°도 10:24 teach-home 때의 TF 값이다. 외부 실측이 없어 어느 쪽이 실물에 가까운지 이 자료로 정할 수 없다 |

### 16.5 남은 항목

- 후진 중단 뒤 서비스는 도크 정지 확인 없이 실패로 끝나고, 실패 이벤트에 이전 단계(staging) 확인값을 싣는다. 충전 중 수정 후보: 중단 뒤 기존 `_verify_parking_stop` 경로로 도크 계약 정지 확인 1회, 이벤트에 확인 단계 표시.
- 도크 yaw 허용 3°가 AMCL yaw 1σ(4–5°)보다 작다. 허용을 넓힐지, 도크 끝 구간에서 yaw를 스캔 정합으로 보정할지는 다음 도킹에서 cmd_vel·odom·goal checker 판정을 함께 기록한 뒤 정한다.
- 사용자가 지도 재작성, 박스 재배치, 목적지 재설정을 진행할 예정이다. 이번 흔적 칸과 keepout 경계는 새 지도 기준으로 다시 정한다([지도·박스 설치 계획](20260930_MAP_BOX_LAYOUT_PLAN.md)).

## 17. 2026-09-30 전체 실행·변경 대장

오전 인계부터 12:14까지. 로봇을 움직인 실행은 사용자 요청("출발", "주차좀 진행해", "충전소 도킹 진행해봐") 범위다.

### 17.1 실행 대장

| 시각 | run (`phase2_20260930/runs/`) | 경로·영역 | 결과 | 기록된 사유·근거 |
|---|---|---|---|---|
| 10:22 | `init_20260930_102257` | — | 중단 | 정합 단계 도구 오류(당시 수정: 빔 수가 다른 scan의 각도 binning, map origin 부호 파싱). `scan.json`만 남음 |
| 10:23–10:53 | `init_…102327`, `…103424`, `…104502`, `…104729`, `…105342` | — | 활성화 | 반복 초기화(세션 재시작·교착 조사 중). 각 `activation.jsonl` |
| 10:30 | `table_01_20260930_103006` | `table_01_route.yaml`, (1.896, 0.303) | 이동 없음 | `live map data unavailable: keepout, map`. Nav2 합성 컨테이너 교착(`container_hang_evidence/`: futex 대기, `dds.*` 스레드) |
| 10:35 | `table_01_20260930_103516` | 같음 | 이동 없음 | `stationary teaching unavailable`. 같은 교착 |
| 10:45–11:03 | `churn_test_*.log` | — | 진단 | 참가자 반복 접속: 합성+LOCALHOST 2회째 교착, 중계 없이 4회째 교착, 루프백 UDP 2회째 교착, 비합성 10/10 통과 → 세션을 `--use-composition false`로 운영 |
| 10:57 | `init_20260930_105723` | — | 중단 | 정합 통과(inlier 0.61) 뒤 도구 JSON 직렬화 오류(numpy bool). 수정 뒤 재실행 |
| 10:58 | `init_20260930_105836` | — | 활성화 | 스캔 정합 (−0.613, −0.834, 92.2°)와 AMCL 1 cm 이내. home_dock 유지 |
| 11:03 | `table_01_20260930_110340` | `table_01_route.yaml`, (1.896, 0.303) | 관측점 도착, 관측 실패 | `box observation unavailable: stable detected front surface is required`(표지 영역에 박스 없음) |
| 11:08 | `table_01_20260930_110834` | `table_01_route_forward.yaml` | 관측점 도착 | 탐색 회전 1회 뒤 `nav2_spin_failed` |
| 11:18, 11:20 | `…111834`, `…112042` | `table_01_route_box.yaml`, (1.13, 0.50) | transit 실패 | `table_01_box_pre` Nav2 104(PATIENCE_EXCEEDED). keepout이 회전 원 안 |
| 11:22 | `…112240` | 같음 | 관측점·search_view 도착 | 탐색 회전 703(COLLISION_AHEAD) ×2 → `nav2_spin_failed` |
| 11:25 | `…112549` | 같음 | transit 실패 | `table_01_box_back_off` 104 |
| 11:28 | `…112806` | 같음 | 관측점·search_view 도착 | 안정 앞면 없음(`box_observation_unavailable_evidence`) |
| 11:31 | `…113143` | 같음 | transit 실패 | `table_01_box_front` 104 |
| 11:35 | `…113541` | 같음 | 이동 없음 | `parking controller parameters unavailable`(부하 중 일시, 다음 실행 통과) |
| 11:36 | `…113650` | 같음 | 관측점 도착, 운영자 정지 | §16.1 |
| 11:46 | `home_20260930_114600` | 도크 | 이동 없음 | §16.3 정적 검사 |
| 11:53 | 세션 재시작 1차 | — | 기동 거부 | 전부 비운 keepout(`keepout mask contains no blocked cells`) |
| 11:57 | `init_20260930_115717` | — | 활성화 | 재시작 뒤 현재 자리 스캔 정합(inlier 0.50, 전역 일치), AMCL 0.4 cm·0.1° |
| 11:58 | `home_20260930_115853` | 도크 | 도크 도달(사용자 확인) | 서비스 판정 105 실패, §16.4 |

박스 정밀 주차(정렬 → 5 cm → 5 s 대기 → 이탈)는 이날 한 번도 정렬 단계에 들어가지 못했다. 막힌 이유는 세 가지였다. 박스 자리가 keepout과 지도 장애물에 가까워 회전이 막혔고(104·703), 관측 거리와 방향 때문에 앞면이 안정되지 않았으며, 가까운 transit waypoint는 BT에서 곧바로 도착 처리됐다.

### 17.2 변경 대장

| 구분 | 변경 | 위치·근거 | 반영 |
|---|---|---|---|
| 코드 | 중간 단계 AMCL 공분산 한도(x·y 0.04 m², yaw (15°)²) | 커밋 `e3b00ca`, §15 | 파이 런타임 3파일 해시 일치 |
| 설정 | 박스 앞면 법선 하한 `minimum_front_normal_z` 0.80 → 0.90(앞면·윗면 혼합 평면 제외) | 커밋 `6f4764f` | 파이 설치본 해시 일치, 관측기 재시작 |
| 규칙 | 파이 lim linger 전제 | 커밋 `523bec1` | 사용자 `loginctl enable-linger lim` |
| 운영 | Nav2 세션 비합성(`--use-composition false`), LOCALHOST | `jdamr_depart.py` `SESSION_COMPOSITION` | 10:59 이후 모든 세션 |
| 운영 도구 | `jdamr_depart.py`(RViz 클릭 → 스캔 정합 → 활성화 → teach-home, `go`·`stop`·`status`·세션·표시 제어), `scan_match.py`, `dock_survey.py`, `capture_scan2.py`, `initialpose_capture.py`, `geometry_check.py` | 데이터 폴더 `tools/`, 저장소 사본 `scripts/phase2_20260930/` | PC에서 실행, 파이는 ssh |
| 지도 자산 | Phase 2 keepout(표 1 앞 띠 개방, `keepout_provenance.json`), home_dock 재교시(10:24, (−0.608, −0.843, 92.3°)), table_01 경로 3종 | 데이터 폴더 | registry 해시 |
| 지도 자산 | `map_dockfix.yaml`(흔적 3칸), `keepout_dockfix.yaml`(46칸 해제), registry 교체 | §16.3, `dockfix_provenance.json` | 11:57 세션부터 |
| 기록 | 이 §16·§17, 포트폴리오 인계, 그림 3장, 지도·박스 설치 계획 | `evaluation/`, `portfolio_data/figures/` | — |

### 17.3 기록 부재

- 이날 어떤 실행도 rosbag을 남기지 않았다. 도킹 구간의 cmd_vel·odom 궤적, Nav2 계획 경로, 카메라 영상이 없다.
- 도크 최종 자세는 내부 추정(TF·AMCL·스캔 정합)과 사용자 육안 확인뿐이다. 줄자·외부 촬영 실측이 없다.
- 박스 관측 실패 run은 최근 status 16개 증거 이벤트만 있고 RGB-D 원본 프레임은 없다.

### 17.4 정리

12:23에 PC 표시 유닛 5개(`jdamr-p2-mapserver`·`relay`·`markers`·`rviz`·`click`)와 파이 Nav2 세션(`jdamr-restaurant-navigation`)을 `jdamr_depart.py display-stop`·`session-stop`으로 끄고 inactive를 확인했다. 파이에는 상시 센서 서비스 `jdamr-base`·`jdamr-box-rgbd`·`jdamr-box-observer`만 남았다(이 작업이 띄운 것이 아니다). 같은 지도로 다시 주행한다면 `display-start` → `session-start` → 도크에서 RViz 초기화 → `init --keep-home` 순서다(표시는 Nav2 세션 기동 전에 띄운다). 새 지도로 바꾸면 설치 계획의 등록 절차를 따른다.

## 18. 새 지도, 목적지 재지정, 물 받는 곳 경유 흐름 (2026-09-30 14:10–14:50)

데이터: `$HOME/jdamr_data/map_20260930_manual/` (파이 같은 경로). 로봇 이동은 사용자 수동 매핑 주행뿐이다.

### 18.1 지도와 도크

| 항목 | 내용 | 근거 |
|---|---|---|
| 매핑 | 파이 `jdamr-operator-mapping.service`(Cartographer 2D + map_saver + 웹 조종기 8080)로 사용자가 14:10–14:23 주행. 14:23:47 `/finish_trajectory` 뒤 저장해 이후 스캔을 넣지 않았다 | `map_provenance.json`, `map.pbstream` |
| 매핑 중 표시 | 기존 중계는 `/map`을 옮기지 않아, 설치본 중계에 `/map`만 더한 임시 스크립트를 ssh 표준입력으로 실행하고 PC RViz를 띄웠다(파이 설치·빌드 없음) | `scripts/map_20260930_manual/mapping_display_relay.py` |
| 도크 | 사용자 지정("마지막 차량 자리 부근을 충전소로"). 정지 스캔 정합 inlier 0.80, 전역 탐색 일치, Cartographer 추정과 0.7 cm·0.75°. 도크 차체 영역의 점유·미확인 0칸, 0.7 m 대기점 회전 여유 0.67 m | `dock_pose.json` |
| 벽 정렬 | 사용자 요청으로 벽 방향(원 지도 축 −5.7°, 두 방법 일치)에 맞춰 원점 기준 +5.7° 회전한 격자 `aligned/map.yaml`(최근접 재표본, 벽 잔여 0.0°). 도크 재정합 (−0.185, 0.191) inlier 0.72. 도크 목표 방향은 사용자 요청으로 0°(실제 정지 6.7°) | `aligned/alignment.json`, `aligned/dock_pose.json` |
| keepout | 지도 미확인 칸만 막았다(22,485칸). 세션의 비어 있지 않은 마스크 조건을 만족하고 KeepoutFilter가 미확인을 빈 칸으로 덮는 문제(§14.1)를 막는다 | `aligned/keepout.yaml`, `keepout_mask validate` 통과 |

### 18.2 목적지와 배치 검사

RViz 2D Pose Estimate 클릭(화살표 = 박스 앞면 바깥 법선, 지도 축 0/90/180/270°로 고정)으로 지정했다. 박스 0.40×0.30 m 가정, 회전 여유 기준 0.514 m.

| 이름 | 앞면 가운데(정렬 지도) | 법선 | 결과 |
|---|---|---|---|
| `water_station` (P2) | (−0.145, −1.531) | 0°(동쪽에서 접근) | 통과 |
| `table_01` (P1) | (1.496, −1.487) | 180°(서쪽에서 접근) | 통과. 두 박스가 1.64 m 간격으로 마주 본다 |
| `table_02` (P3) | 클릭 (0.518, −3.578) | 90°(북쪽에서 접근) | 불통과: 박스 자리에 지도 점유 17칸, 접근 자세 회전 여유 0.37–0.46 m. 제안 자리 (0.62, −2.83) 북향(북쪽 75 cm·동쪽 10 cm)은 통과. 사용자 확인 전 |

경로는 앞면 법선 위 한 직선에 둔다(사전 1.10 m, 관측 0.60 m, 간격 0.50 m. 이후 `jdamr_depart.py`에서 사전 1.00 m·관측 0.58 m·간격 0.42 m로 바꿨고, 간격은 BT 목표 허용 0.15 m보다 크다). 박스 세 개를 넣은 지도에서 회전 지점 13곳 모두 0.514 m 이상, 세 접근 통로와 도크 후진 통로는 `static_corridor_clear` 통과. `water_station` 경로는 도크에서, 테이블 경로는 `water_station` 이탈 자세(앞면 + 0.565 m)에서 시작한다. `table_01`의 사전 지점은 이탈 자세와 5 cm라 곧바로 끝나고 관측 지점(0.48 m 앞)에서 방향이 맞춰진다.

### 18.3 새 흐름 (커밋 `a34b230`)

- 사용자 결정: 웹 호출은 만들지 않고 사용자의 "출발 table_0N"을 호출로 본다. 흐름은 도크 → `water_station` 정밀 주차·5 s·후진 이탈 → 호출 테이블 정밀 주차·5 s·후진 이탈 → 도크 후면 도킹.
- `box_service --via-id water_station --via-route --via-region-xy [--via-region-radius-m]`: 경유지를 기존 `visit_observed_box`로 방문하고 `dwell_and_leave`(검증된 정지 유지 뒤 `_leave_parked_pose` 이탈)를 거쳐 테이블 방문과 기존 `dwell_and_return_home`으로 이어진다. 경유 인자는 함께만 받고 `--return-home`이 필요하며 관측 지점 재개와 함께 쓰지 않는다. 경유 실패·이탈 실패면 테이블로 가지 않는다.
- 테스트 T36 7건은 수정 전 모두 실패, 수정 뒤 통과. 핵심 7개 파일 605건 통과, 변경 파일 flake8·pep257 0건.
- 파이 반영: `scripts/deploy_navigation_to_pi.sh`는 파이 사전 검증에서 멈췄다(검증 사본에 `jdamr_cube_description`이 없어 `new_base_geometry.yaml`을 읽는 기존 테스트 2건 실패, 주 워크스페이스 미변경). 백업(`$HOME/jdamr_data/deploy_backup_20260930_via/`) 뒤 `box_service.py` 한 파일을 복사하고 `colcon build --packages-select jdamr_cube_navigation` 통과. PC·파이 소스·파이 설치본 SHA-256 `7f954a6c…` 일치, `--help`에 경유 옵션 표시.
- 출발 도구: `$HOME/jdamr_data/map_20260930_manual/tools/jdamr_depart.py`(저장소 사본 `scripts/map_20260930_manual/`). 새 지도 폴더, `destinations.json`, 세 경로 생성, init 때 세 목적지 영역 등록, `go table_0N`에 경유 인자 추가. 14:49 새 registry로 정밀 준비 세션 기동(localization·keepout active) 확인 뒤 종료.

### 18.4 최종 배치 (14:55, 사용자 조정)

사용자 요청: "워터스테이션은 조금 위로, 테이블1은 내릴 수 있을 만큼 내리고, 테이블2는 제안한 거로", "주행 중 멈추지 않을 정도로". 세 목적지를 함께 넣은 검사기(`scripts/map_20260930_manual/layout_eval.py`: 박스 자리, 박스 간격 0.30 m, 회전 지점 13곳 0.514 m, 직선 접근 통로, 도크 후진 통로, 0.30 m 측면 여유 연결성)로 정했다.

| 이름 | 앞면 가운데 | 법선 | 조정과 한계 |
|---|---|---|---|
| `water_station` | (−0.145, −1.331) | 0° | 북쪽 0.20 m. +0.30 m까지 통과, +0.40 m 접근 통로 차단 |
| `table_01` | (1.596, −1.687) | 180° | 남쪽 0.20 m(−0.35 m부터 사전 지점 회전 여유 부족), 동쪽 0.10 m. 두 박스 앞면 간격 1.64 → 1.74 m로 넓혀 서로의 사전 지점 여유를 0.03 m에서 0.13 m로 늘렸다 |
| `table_02` | (0.67, −2.83) | 90° | 제안 자리에서 동쪽 0.05 m(서쪽 설치 오차 대비) |

설치 오차 ±0.10 m(9방향 × 3곳) 검사에서 남은 실패는 `water_station`을 서쪽으로 0.10 m 놓을 때(뒤쪽 지도 물체와 0.10 m 이내)와 `table_02`를 서쪽으로 0.10 m 놓을 때(0.510 m < 0.514 m)뿐이다. 카메라 시야 1.5 m 안 점유 칸: `water_station` 19칸(최근접 1.20 m), `table_01` 15칸(1.28 m), `table_02` 0칸. 경로·주석·표시를 다시 만들고 파이에 동기화했다.

### 18.5 남은 항목

- 박스 세 개 설치, 로봇을 도크에 벽과 나란히 두고 `display-start` → `session-start` → RViz 초기화 → `init`(teach-home). 그 뒤 "출발 table_0N"에 `go table_0N`.
- 실차 미검증: 두 박스가 마주 본 1.64 m 공간에서의 회전 탐색, 테이블 경로 첫 사전 지점 즉시 완료, 도크 끝 yaw(§16.4)는 첫 주행에서 본다.
- 표시: `service_visualization`은 테이블 두 곳만 그리고 `water_station`은 그리지 않는다.

## 19. 새 지도 첫 "2번 출발"과 반복 중단 정리 (2026-09-30 15:02–16:24)

데이터: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/` (파이 원본을 PC로 복사). 모든 실행은 사용자의 "2번 출발"(물 받는 곳 경유 → table_02 → 도크)과 이어가기 요청 범위다. 정확도 값은 모두 내부 추정(AMCL·TF)이고 외부 실측은 없다.

### 19.1 실행 대장

| 시작 | 실행 | 끝난 단계 | 중단 사유(로그 근거) | 조치 |
|---|---|---|---|---|
| 15:02 | `table_02_150238` 경유 포함 | 물 받는 곳 최종 접근 | 첫 관측에서 앞면 미검출 → 탐색 회전 1회 뒤 검출, 면 정렬 완료. 최종 접근이 목표 8 cm 앞에서 `nav2_error_code` 105(진행 검사). Parking 최저 0.01 m/s에 SlowdownZone 0.6 감속이 겹쳐 0.004 m/s로 진행 요구(0.05 m/10 s) 미달 | `5449400` 정밀 세션 SlowdownZone 끔, 최종 접근 최저 속도 0.02 m/s |
| 15:22 | `table_02_152226` 경유 포함 | 물 받는 곳 탐색 회전 | 요청 30° 회전에 관측 38.2°(변위 1.5 cm), 확인 허용 ±5° 초과로 `nav2_spin_failed` | `33e960f` 확인 허용 ±15° |
| 15:27 | `home_152750` 도크 복귀 | 대기점 이동 중 사용자 중단 | Claude가 물 받는 곳 정밀 주차·대기와 테이블 방문을 마치지 않은 채 복귀를 시작했다. 사용자 지적("아직 물 받는 데 앞인데") 뒤 즉시 취소(`operator_or_timeout`) | 남은 정차를 건너뛰지 않는다 |
| 15:29 | `table_02_152914` 경유 포함 | 물 받는 곳 사전 지점 | 박스 앞에 남은 자리에서 제자리 회전이 105로 멈춤(회전 영역 안 박스). BackUp 서버가 없어 사용자가 로봇을 뒤로 옮김 | `box_escape_once.py`(실행기 후진 이탈만 실행) 작성 |
| 15:38 | `table_02_153849` 경유 포함 | 물 받는 곳 최종 접근 | `AMCL x covariance high: value=0.010 limit=0.010 (bound=strict)`로 중단 | 사용자 지시("공분산이나 한도 쓰지 말라")로 `87ff341` 운영 계약 공분산 정지 6항목을 1e6으로(사실상 끔) |
| 15:44 | `escape_154451` 이탈 단독 | 후진 완료, 확인 실패 | 앞면 0.512 m(기준 0.515), 위치 5.3 cm, 방향 2.9° | 16:23 `e32cc3a`(19.2) |
| 15:46 | `table_02_154603` 경유 포함 | 물 받는 곳 최종 접근 도착 | 위치 오차 0.65 cm, 방향 1.83° > 당시 허용 1°로 `parking_not_confirmed` | `86581fb` 방향 허용 3°(계약 상한). 실시간 파라미터를 계약과 대조하므로 세션 재시작 |
| 15:51 | `water_dwell_escape_155149` | 대기 시작 직후 | 새 프로세스에는 최종 명령이 없어 `wait_parked`가 `parking odometry or final command missing` | 정지 자세 두 번 캡처(5 s 간격, 2 cm 이내)로 대기 확인. `b3687d9`의 `--resume-parked-from-log`에 반영 |
| 15:52 | `water_dwell_escape_155235` | 대기 완료(이동 0.0 m), 후진 완료, 확인 실패 | 앞면 0.526 m로 충분히 물러났는데 방향 6.0° > 3°로 `box_escape_not_confirmed` | 16:23 `e32cc3a`(19.2) |
| 15:53, 15:54 | `table_02_155334`, `155424` 테이블만 | 시작 직후 | `live map data unavailable: map`(실행기 대기 10 s) | 19.3 |
| 16:00 | `table_02_160052` 테이블만 | table_02 사전 지점 이동 13 s | 104. 파이 journal 16:01:23에 RPP `detected collision ahead!` 반복과 collision_monitor `Robot to stop due to StopZone polygon`이 같은 순간에 나옴. 앞 물체는 기록이 없어 특정하지 못했다 | 미해결(19.5) |
| 16:03 | `table_02_160312` 테이블만 | 출발 전 | `battery_departure_reserve` 10.784 V < 출발 하한 10.8 V | 사용자가 충전소로 옮겨 충전 |

초기화(`init`)는 14:57–15:59에 13회 시도해 5회 활성화했다(14:57, 15:21:28, 15:37, 15:50, 15:59:49). 실패 중 사유가 남은 것은 15:32 정합 inlier 0.19(로봇을 옮긴 직후), 15:33 활성화 중 `map_server lifecycle service unavailable`, 15:56 스캔 0개, 15:59:21 전역 탐색 불일치다. 15:14·15:15(스캔 40개, 정합 결과 없음)와 15:20·15:20:56(정합 0.49·0.48 통과, 활성화 기록 없음)은 도구 출력이 남지 않아 사유를 모른다. 15:37·15:59:49는 `--local-only`(inlier 0.43·0.38, 사용자 배치 기준)로 활성화했다.

도달한 것: 물 받는 곳 5 cm 정밀 주차 한 번(15:46, 내부 추정 0.65 cm·1.83°, 외부 실측 없음)과 그 자리 5 s 대기(이동 0.0 m)·후진 이탈 동작. 테이블 정밀 주차와 도크 복귀를 포함한 "2번 출발" 한 사이클은 아직 끝까지 가지 못했다.

### 19.2 후진 이탈 확인 (커밋 `e32cc3a`)

- 두 번의 이탈 모두 후진 자체는 Nav2 성공(상태 4)인데 확인 단계가 막았다. 확인은 위치 5 cm·방향 3°·앞면 거리 0.515 m 이상을 모두 요구했다.
- 이탈의 목적은 다음 회전 여유다(`ESCAPE_CLEARANCE_M` 0.565 m ≥ 회전 정지 영역 최대 반지름 + 0.1 m, `test_t29`). 이제 앞면 거리만 판정하고 방향·측면 오차는 `box_escape_finished`에 기록만 한다. 다음 구간은 멈춘 자리에서 Nav2가 다시 계획한다.
- T44(방향 6°·측면 4 cm 표류에도 staging으로 이어짐)는 수정 전 실패, 수정 뒤 통과. 이탈 전 정렬 조건(앞면을 3° 안으로 바라봄)과 앞면 거리 미달 시 중단(`test_m3_escape_not_confirmed_never_stages`)은 그대로다.

### 19.3 새 프로세스의 DDS 수신 실패

관찰:
- 실행 중인 베이스·세션과 무관하게, 새로 띄운 프로세스가 기존 노드의 데이터를 받지 못한 경우가 이어졌다. robot_state_publisher TF 없음, map_server 수명주기 서비스 없음, `/map` 없음, 스캔 0개.
- 매번 `jdamr-base` 재시작이나 세션 재시작으로 풀렸다. 당시 파이 부하 평균은 9–15였다.

확인한 것:
- 참가자 수 가설(LOCALHOST 모드의 `maxInitialPeersRange=32`)은 기각했다. 세션과 표시 중계를 모두 띄운 16:10 측정에서 참가자 21개(id 0–20)였다. 번호를 고정해도 풀리지 않는다.
- 한가한 상태에서 새 프로세스 18회(SIGKILL 뒤 포함)는 모두 지도·TF·스캔을 받았다. 재현하지 못했으므로 원인은 확정하지 않았다.

조치(원인 대신 증상 대응):
- `e82d45c` 실행기의 지도 수신 대기를 10 s → 30 s. 한가할 때도 1–6 s가 걸렸고 주행 중에는 10 s를 넘겼다.
- `aa454e8`은 `go`가 출발 직전 `dds_probe.py`로 수신을 확인하고, 실패하면 `recover`를 거친 뒤 출발했다. 코드 리뷰(19.6)에서 주행 중 재입력 시 세션을 끄는 순서 결함과 저장소 AGENTS.md "새로운 출발 차단 계층을 덧붙이지 않는다"와의 충돌이 나와 바꿨다.
- 현재 방식: 정상 출발에는 추가 확인이 없다. 실행기 첫 확인(`verify_live_maps` 30 s, 이동 명령 전)이 `live map data unavailable`로 끝나고 다른 이벤트가 없을 때만 `recover`를 거쳐 한 번 더 출발한다(1회 한정).
  - `recover` 순서: 사이클 없음·linger 확인 → 위치추정 무효화 → 표시·세션 종료 → `jdamr-base`·`jdamr-box-rgbd`·`jdamr-box-observer` 재시작(시작 시각이 바뀐 것까지 확인) → 표시·세션 기동 → 직전 init 자세로 재초기화(전역 정합 필수) → 수신 점검.
  - 재개 옵션으로 출발했거나 직전 init이 `--local-only`면 자동 복구 대신 `recover --seed X Y YAW_DEG [--local-only]`를 안내하고 멈춘다.
- `health`(수동 점검)는 대기를 실행기와 같은 30 s로 맞췄고 `/keepout_filter_mask`도 본다. 박스 관측 상태는 수만 기록한다.
- 검증(로봇 정지·충전 중): 16:16–16:20 `health` 4회 통과(0.8–4.6 s). 다른 도메인(99)에서 같은 점검은 `MISSING`(당시 대기 20 s). 바뀐 흐름은 ssh를 가짜로 바꾼 `scripts/map_20260930_manual/test_jdamr_depart_flow.py` 22건으로 확인했다. `recover`의 실제 재시작 경로는 아직 실행하지 않았다.

### 19.4 그 밖의 변경

- `c5b066f` 탐색 회전을 영역 방위 쪽으로 먼저 돌림(8° 이상 벗어났을 때, 한 번에 30° 이하, 전체 예산 유지).
- `b3687d9` 중단 지점에서 이어가기: 관측 지점부터 재개가 경유와 함께 첫 정차에만 적용, `--resume-parked-from-log`(로그의 마지막 앞면으로 정지 확인 대기 → 이탈 → 경유면 테이블, 아니면 도크).
- `8c5b92d` RViz 서비스 표지에 `water_station` 영역(청록)을 그린다(§18.5의 표시 항목 해결).
- 배포: 모든 파이 반영은 백업(`$HOME/jdamr_data/deploy_backup_20260930_*`) → 파일 복사 → `colcon build --packages-select jdamr_cube_navigation` → PC·파이 소스·설치본 SHA-256 대조 순서다. `scripts/deploy_navigation_to_pi.sh`는 §18.3의 사전 검증 문제로 쓰지 않았다.

### 19.5 남은 항목

- "2번 출발" 한 사이클 끝까지: 물 받는 곳 → table_02 정밀 주차·대기·이탈 → 도크 후면 도킹.
- table_02 사전 지점 이동 중 104: 센서가 정지 영역 안 물체를 보았다. 사람인지, 박스나 지도 물체인지 가리려면 다음 발생 때 로컬 코스트맵과 스캔을 함께 남겨야 한다.
- 도크 끝 방향 허용 3°와 AMCL 방향 잡음(§16.4).
- DDS 수신 실패의 원인. 부하 상태 재현과 `recover` 실제 경로(지도 미수신 뒤 자동 복구·재출발)는 다음 주행에서 본다.
- 새로 추가한 조건: 이번 판부터 `go`는 완료된 init이 있어야 출발한다(`state.json`의 `localized`). 충전소로 옮긴 지금은 `display-start` → `session-start` → RViz 클릭 → `init`이 먼저다.
- 이탈 전 정렬 3°는 그대로다. 새 프로세스 재개(`--resume-parked-log`)에서 AMCL 방향 잡음(§16.4)이 `heading_not_facing_box_face`를 반복시키는지 본다.

### 19.6 코드 리뷰 반영 (충전 중, 로봇 이동 없음)

`e82d45c`·`e32cc3a`·`aa454e8`을 별도 리뷰 에이전트가 검토했다(REQUEST CHANGES, HIGH 1·MEDIUM 6·LOW 13·열린 질문 2). 앞의 두 커밋은 승인이었다.

| 항목 | 지적 | 처리 |
|---|---|---|
| HIGH | 사이클 주행 중 `go` 재입력 시 탐지 실패 → `recover`가 세션을 끄고 베이스를 재시작. 단독 `recover`·`session-stop`에는 사이클 확인이 없음 | `go`·`recover`·`session-stop` 맨 앞에서 `jdamr-table-cycle-*` 확인, 실행 중이면 아무것도 끄지 않고 멈춤 |
| M1 | init·recover가 실패해도 이전 `regions`로 다음 `go`가 출발 | `localized` 표지: init 시작·세션 종료·새 세션 기동 때 false, init 완료 때만 true. `go`가 요구 |
| M2 | 재개 경로에서 자동 복구가 전역 정합으로 늘 실패, 안내 문구 없음 | 재개 옵션·`--local-only` init이면 자동 복구 대신 안내 후 멈춤. init 실패 시 안내 출력 |
| M3 | 복구가 `jdamr-base`만 재시작, linger 미확인, 탐지에 keepout 없음 | 센서 서비스 3개 재시작, linger가 yes가 아니면 멈춤, 탐지에 keepout 추가 |
| M4 | `recover --seed`가 전역 정합 생략을 강제 | `--local-only`를 따로 줄 때만 생략 |
| M5 | 출발 직전 탐지·자동 복구가 저장소 규칙("새로운 출발 차단 계층")과 충돌 | 출발 전 탐지 제거. 실행기 첫 확인 실패 때만 복구·재출발(19.3) |
| M6 | `box_escape_once.py` face 모드가 법선을 검증·정규화하지 않음(\|n\| 1.118이면 정렬 허용 약 26.7°) | \|n\|이 1±0.01이 아니면 거부, 정규화해 사용 |
| L1·L2 | 탐지 실행 불가와 MISSING 미구분, `' ok '` 문자열 파싱 | ok·missing·error 구분(ssh 시간 초과는 error), 세 번째 토큰으로 판정 |
| L3 | 재시작 실패가 active로 보임 | `ActiveEnterTimestampMonotonic`이 바뀌어야 통과 |
| L4 | 표시 유닛 일부만 떠 있으면 나머지를 띄우지 않음 | 없는 유닛만 기동 |
| L5·L6 | 탐지에 keepout 없음, 탐지 20 s와 실행기 30 s 불일치 | keepout 추가, 30 s로 맞춤 |
| L7 | `restaurant_service`의 `visit`·`go_home`이 기한을 먼저 잡아 최대 30 s 지도 대기가 예산을 씀 | 대기 동안 기한 해제, 대기 뒤 설정. 테스트 2건(수정 전 실패·수정 뒤 통과). 파이 반영(백업 `deploy_backup_20260930_legbudget`, PC·파이 소스·설치본 SHA-256 `ee624af8…` 일치) |
| L8 | 새 프로세스 재개에서 이탈 전 3°가 AMCL 방향 잡음에 걸릴 수 있음 | 설계대로 둠, 남은 항목(19.5) |
| L9 | 큰 표류·경로 시작점 검사 연동 테스트 없음 | 추가하지 않음. 경로 시작점 0.3 m 검사는 `test_box_service`의 `starting place` 테스트가 다루고, 이탈 뒤 표류는 기록만 하는 설계다 |
| L10 | 주석·문서의 경로 간격이 옛 값 | 주석과 §18.2 갱신 |
| L11 | `--resume-parked-log` 경로 미인용, `--skip-via`에도 물 받는 곳 경유 문구 | `shlex.quote`, 문구 분기 |
| L12 | `box_escape_once.py`에 SIGTERM·SIGHUP 처리 없음, 정류지 id 고정 | 두 신호와 SIGINT에서 정지 요청, `--stop-id` |
| L13 | PC 로컬 설치본이 옛 이탈 규칙 | PC 재빌드, 소스·설치본 해시 일치 |
| 열린 질문 1 | 이탈 뒤 큰 AMCL 도약을 잡을 느슨한 상한 | 추가하지 않음. 사용자 지시("공분산이나 한도 쓰지 말라")와 충돌 |
| 열린 질문 2 | 출발 직전 탐지 참가자의 입퇴장이 교착 방아쇠일 가능성 | 출발 경로에서 탐지를 없앴다. `health`·`recover`에서만 돈다 |

검증: 패키지 테스트 2347 통과. 실패 28·오류 9는 frontier·AMCL 평가·G006·저장소 전체 린트 파일로, 같은 파일을 HEAD 임시 worktree에서 돌려도 같은 수가 나와 이번 변경과 무관하다. 수정 파일 `ament_flake8`·`ament_pep257` 문제 없음. 스크립트는 flake8 F·E9 통과.

### 19.7 저녁 "2번 출발"과 후진 문제 (2026-09-30 16:56–18:20)

데이터: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/` (`table_02_20260930_1656*`–`1736*`, `home_20260930_173307`, `go_table_02_*_monitor.log`). 정확도 값은 모두 내부 추정이고 외부 실측은 없다.

| 시작 | 실행 | 결과 | 원인(근거) | 조치 |
|---|---|---|---|---|
| 16:56 | 도크 출발 | 이동 전 `live map data unavailable: map` | 실행기 시작 12 s 뒤 16:57:00 lifecycle_manager `controller_server IS DOWN after not receiving a heartbeat for 10000 ms` → 스택 종료. 15:54에도 같은 기록. 파이 UDP `RcvbufErrors` 누적 3,664,856(수신 데이터그램의 약 10 %), `rmem_max` 212992 | 파이 `/etc/sysctl.d/60-ros2-dds-udp.conf`(rmem_default 4 MB, rmem_max 16 MB), `f4f375d` bond_timeout 0. 이후 주행 중 폐기 0 |
| 16:57 | 자동 복구 | 재초기화 활성화 실패(`navigation rollback unconfirmed`) | 기동 요청 45 s 무응답(부하 평균 8–10) | 17:07 버퍼 적용 뒤 `recover` 성공 |
| 17:11 | 도크 출발 | 물 받는 곳 5 cm 주차(0.31 cm, 1.39°), 5 s 대기. 이탈 0.526 m에서 105 | 아래 후진 문제 | 그 자리에서 `--skip-via`로 이어감 |
| 17:15 | table_02 | 이동 성공(16:00의 104 구간 통과). 관측 13회 실패로 탐색 소진 | 라이다 앞면 직선(지지점 45–51, 잔차 1.3 mm, 방향 차 약 1°)이 깊이 앞면보다 일정하게 2.4–2.7 cm 뒤. 같은 날 통과 관측 8건은 −0.43~+0.19 cm | `5bd2708` 2–3.5 cm 차이는 가까운 면 기준 |
| 17:28 | table_02 관측 지점부터 | 5 cm 주차(중앙 5.7 cm, 2.74°) 뒤 `box_gap_not_confirmed` | 모서리 4.4·7.0 cm가 ±1 cm 밖. 방향 허용을 3°로 넓힐 때(`86581fb`) 모서리 기준을 그대로 둔 결함 | `665f891` 중앙 ±1 cm와 계약 방향으로 판정, 모서리는 기록 |
| 17:30 | 주차 자리 재개 | 5 s 대기 뒤 이탈 0.554 m에서 105, AMCL 방향 33° 틀어짐 | 아래 후진 문제 | `665f891` 회전 여유(0.515 m)면 이탈 완료·생략 |
| 17:33 | `restaurant_service home` | 이동 전 거부 | 도크 계약 파라미터를 요구하는데 세션은 박스 계약 | `9017be0` `--dock-only`(box_service `--home-only`) |
| 17:36 | 도크 복귀 | 대기점 1.5 cm·1.4°, 후면 도킹 뒤 105. AMCL 방향 18.8° | 사용자 확인: 로봇은 충전기에 평행, 충전 중(12.1–12.2 V). 18:13 스캔 정합 3.5°·AMCL 3.1°. AMCL 방향 오차 약 15° | 아래 |

후진 문제(세 번 모두 105): 목표 몇 cm 앞에서 좌우로 흔들리다 진행 검사에 걸렸다(사용자 관찰: 후진·회전 중 좌우 왕복). 확정 사실은 AMCL 방향이 도크 근처에서 약 15° 틀렸다는 것이다. RPP 끝단에서 추종점 거리가 줄어 곡률이 폭증하는 현상이 이를 키웠다는 것은 가설이다. `c1770a2` 주차·후진 RPP에 `use_fixed_curvature_lookahead`·`curvature_lookahead_dist` 0.6·`interpolate_curvature_after_goal`을 켰다(Nav2 1.3.12 지원 확인, 파이 반영·세션 재시작 적용). 실차 확인은 다음 주행에서 한다.

남은 항목:
- 짧은 직선 후진(이탈 0.45 m, 도킹 마지막 0.7 m)을 odom 좌표 경로로 보내 AMCL 방향 튐의 영향을 없애는 방안. 도킹 확인이 AMCL에 기대는 부분(물리적으로 도킹됐는데 105)도 같이 다뤄야 한다.
- 도크·박스 근처 AMCL 방향 오차(15°)의 원인.
- 출발 순간 발견 트래픽 소켓(104xx 짝수 포트)의 폐기는 버퍼 확대 뒤에도 기동 중에 남았다(주행 중에는 0).

### 19.8 odom 고정 후진·직선 최종 접근과 table_02 무중단 구간 (2026-09-30 18:25–18:52)

사용자 확인: 17:39 도킹 뒤 로봇은 충전기에 평행했고(충전 12.1–12.2 V) RViz만 삐뚤게 보였다. 18:13 스캔 정합 3.5°·AMCL 3.1°로, 도킹 직후 AMCL 18.8°는 약 15° 방향 오차였다. 18:32·18:41 최종 접근은 11°·23° 기울어진 채 도착해 RPP 충돌 예상으로 104가 났다(사용자 확인: 두 번 모두 기울어진 채 정밀 주차).

| 커밋 | 내용 |
|---|---|
| `64bb05c` | 박스 이탈·도킹 마지막 후진: 지도에서 만들고 검증한 경로를 보내기 직전의 map-odom 관계로 odom에 고정해 보낸다. 정밀 도킹은 대기점을 지도에서, 마지막 구간을 같은 도크 계약으로 odom에서 확인 |
| `f9af87f` | `c1770a2` 곡률 설정 되돌림. 켠 뒤 첫 최종 접근이 앞면 2.5 cm까지 들어갔다. (정정 2026-09-30 20:10: 커밋 메시지의 "속도 조절용 곡률에만 쓰인다"는 틀렸다. Nav2 1.3.12 소스에서 켜면 조향 곡률에도 쓰인다. 되돌린 근거는 켠 직후 접근이 더 들어간 시점 대응뿐이고, 원인은 확인하지 않았다) |
| `093f559` | odom 고정 이탈의 회전 여유를 odom에서 판정(18:37 AMCL 0.5146 m로 0.4 mm 미달 판정) |
| `a681f62` | 최종 접근: 면 정렬 방향을 유지한 채 앞면 간격 5 cm까지 Parking 제어기로 odom 고정 직선 경로. 간격·방향 판정도 odom, 대기는 도착 자세 유지. 이탈 전 정면 조건 15° |

18:47 table_02 관측 지점부터(`table_02_20260930_184711`) 한 실행으로 끝까지 갔다(`parked=True dwell=True escape=True home=True`). 모두 내부 추정, 외부 실측 없음.

| 단계 | 결과 |
|---|---|
| 직선 최종 접근 | 이동 0.397 m, 앞면 대비 방향 1.1°. 정지 확인 0.42 cm·0.17°(odom) |
| 간격 | 중앙 5.42 cm, 모서리 5.83·5.02 cm, 앞면 방향 0.87° |
| 5 s 대기 | 이동 0 |
| 후진 이탈 | Nav2 성공, 앞면 거리 0.520 m(odom) |
| 도크 대기점 | 3.57 cm·0.78°(AMCL), 이동 2분 7초 |
| 후면 도킹 | Nav2 성공, 4.44 cm·1.2°(odom, 도크 계약 안) |

- 같은 날 물 받는 곳은 18:37 odom 직선 이탈이 Nav2 성공(끝 방향 0.28°)이었다. 그 전 최종 접근은 곡률 설정 상태의 104였다.
- 충전기는 사용자가 연결하는 방식이다("충전소"는 위치 이름). 도킹 뒤 전압(18:52 11.4 V대)은 도킹 판정에 쓰지 않는다.

남은 항목:
- 물 받는 곳부터 도크까지 새 방식으로 끊김 없는 한 사이클.
- 도크 대기점 이동 2분 7초(평소보다 길다), 사용자가 본 앞뒤 왕복과 같은 구간인지.

### 19.9 퇴근 전 정리 (2026-09-30 18:52–)

- `b5edc5a` 사용자 요청으로 이동 속도 +50 %(0.08→0.12 m/s)와 이동 감속 완화. 바꾼 값은 최저 접근 0.075 m/s, 곡선 감속 반경 0.6 m, 목표 앞 감속 0.4 m, 비용 감속 0.2 m, SlowdownZone 비율 0.8이다. 주차·후진 제어기는 0.04 m/s와 기존 감속값(0.6·0.9·0.3)을 명시해 유지했다. 회전 0.3 rad/s는 탐색 회전 넘침(15:22 30°에 38.2°) 때문에 그대로 두었다. 파이 반영 완료, 다음 세션 기동부터 적용, 실차 확인 전.
- 도크 대기점 앞 큰 회전(사용자 관찰: 한쪽으로 조금만 돌면 되는데 반대로 크게 돈다): 18:48:54–18:51:01(2분 7초) 동안 controller_server가 `Passing new path to controller`를 113회 남겼다. 내비게이터가 약 1초마다 경로를 새로 계획했다는 기록이다. AMCL 자세가 튈 때마다 경로의 돌아 들어가는 쪽이 바뀌어 회전 방향이 뒤집혔다는 것은 가설이고, 경로를 기록하지 않아 확인하지 못했다. 개선안은 두 가지다. (1) 대기점 구간도 한 번 계획한 경로를 odom에 고정해 FollowPath로 보내기, (2) 마지막 방향 맞춤은 정지 자세에서 계산한 최단 각도로 Spin 한 번. 다음 주행에서 `/plan`·`/cmd_vel`·`/odom`·`/amcl_pose`를 기록해 확인한 뒤 적용한다.
- 보존: 파이 run 로그 전체를 PC `aligned/runs/`로 회수했다. Nav2 세션 journal 16:50–18:55와 실행 유닛 journal 14개는 `aligned/runs/journal_20260930_evening/`에 있다.
- 포트폴리오 데이터 점검: `20260930_EVENING_E2E_PORTFOLIO_NOTES.md`.
- 주행 기록기: 파이 디스크가 98 %(여유 844 MB, `~/jdamr_data/vslam` 14 GB·`~/jdamr_artifacts` 2.5 GB)라 파이에서 기록하지 않는다. 표시 중계가 이미 PC로 넘기는 `/tf`·`/tf_static`·`/scan`·`/plan`·`/amcl_pose`를 PC에서 mcap으로 기록한다(`display-start`가 켜고 `display-stop`이 끈다, `record-start`·`record-stop`도 있음). `/tf`에 odom 궤적과 AMCL 보정이 들어 있다. 중계는 설계상 명령 토픽을 싣지 않으므로 `/cmd_vel`은 없다. 19:06 중계만 켠 시험에서 10 s에 `/scan` 139·`/tf` 235·`/tf_static` 2를 기록했다(시간당 약 120 MB). 도크 대기점 회전 수정은 이 기록으로 원인을 확인한 뒤 한다.
- 파이 디스크 정리(`vslam` 14 GB 보관·삭제)는 사용자 결정 사항이다.
- 코드 리뷰 LOW 반영(출발 도구): 사이클 종료 직후 로그 재읽기, ssh 실패와 사이클 실행 구분, session-start 시간 초과 시 위치추정 무효화, 도크에 닿지 못한 `go`는 종료 코드 1, `recover --local-only`는 `--seed` 필요, 상태 파일 덮어쓰기 방지. `box_escape_once.py`는 대기 중 2 cm 넘게 움직이면 중단, 잘못된 모드 입력은 인자 오류. 모의 흐름 테스트 26건 통과.
- `scripts/deploy_navigation_to_pi.sh`: 검증 사본에 `jdamr_cube_description/config/new_base_geometry.yaml`을 넣었다(§18.3의 테스트 2건 실패 원인). 스크립트는 `jdamr-base`를 멈추고 다시 켜므로 퇴근 전에는 실행하지 않았다. 다음 배포 때 확인한다.

## 20. 2026-10-01 오전 "2번 출발" — 첫 무중단 구간과 막힌 원인 다섯 가지

수치는 모두 로봇 내부 추정(AMCL·odom)이고 외부 실측은 없다. 런 로그는 PC `aligned/runs/table_02_20261001_*`, 궤적은 PC bag `aligned/runs/bag_20261001_092908`·`bag_20261001_095541`(`/tf`·`/scan`·`/plan`·`/amcl_pose`).

### 20.1 결과 (10:12–10:28 실행)

| KST | 구간 | 결과 |
|---|---|---|
| 10:11:54 | 도크 init(등록 도크 자세 기준) | 스캔 정합과 AMCL 0.7 cm·0.3° |
| 10:13:54 | 물 받는 곳 정밀 주차·5 s 대기 | 정지 0.42 cm·0.18°, 간격 5.42 cm(모서리 5.40·5.44), 면 0.04° |
| 10:14:19 | 물 받는 곳 후진 이탈 | Nav2 성공 |
| 10:14:32 | 이탈 → table_02 접근 지점 | Nav2 성공 13 s. 회전을 먼저 하고 출발, 꼬리~박스 면 최소 0.129 m(패딩 포함) |
| 10:15:09 | table_02 최종 접근 | 시작 전 실패(§20.2 4) → 수정 후 관측 위치에서 재개 |
| 10:20:10 | table_02 정밀 주차·5 s 대기 | 정지 0.53 cm·0.23°, 간격 5.53 cm(모서리 6.23·4.83), 면 1.48° |
| 10:20:47 | table_02 후진 이탈 | Nav2 성공 |
| 10:21:37 | 도크 대기점 이동 | Nav2 성공 50 s(9/30 같은 구간 118 s) → 회전에서 실패(§20.2 5) |
| 10:27:09 | 대기점 한 방향 회전 | 필요 −122.8°, 실제 −125.8°(odom), 이동 0.1 cm |
| 10:28:10 | 후면 도킹 | Nav2 성공, 도착 확인 4.6 cm·1.42° |

물 받는 곳 → table_02 → 도크의 모든 단계를 같은 오전에 실차로 지났다. 수정 배포 때문에 세 번의 실행(10:12, 10:19 재개, 10:26 도크만)으로 나뉘었고, 한 실행으로 끊김 없이 끝난 사이클은 아직이다.

### 20.2 막힌 원인과 조치

1. **PC 절전 뒤 Wi-Fi 변경(08:56).** 깨어나며 `aicampus_291_5G`에 붙어 파이(`aicampus_286`)에 닿지 않았다. 파이 문제로 오진해 불필요한 전원 재시작을 요청했다. 그 재부팅으로 임시 유닛이던 `jdamr-box-observer`가 사라져 이전 부팅 journal의 `systemd-run` 명령으로 다시 만들었고, `jdamr-box-rgbd`는 disabled라 수동 시작했다. 조치: PC를 `aicampus_286`으로 되돌림.
2. **Graceful 최종 접근 105(09:43).** Nav2 1.3.12 Graceful은 목표까지 궤적을 costmap으로 충돌 검사하고 끌 수 없다. 박스 5 cm 앞 목표에서 "Collision detected in trajectory" 뒤 105. 로봇은 목표 2.8 cm 안(앞면 약 4.8 cm, 0.14°)까지 갔다. 조치 `311a23d`: RPP `Parking`·`ParkingReverse`를 기본으로, Graceful은 `--graceful-final`.
3. **이탈 뒤 전진 원호로 꼬리가 박스 쪽으로(09:45, 9/30 16:01과 같은 104).** `rotate_to_heading_min_angle` 1.57이라 90° 미만 방향 차이는 제자리 회전 없이 전진하며 돌았다. bag 궤적에서 차체 중심이 박스 쪽으로 7 cm 들어가며 꼬리 여유가 0.405→0.082 m(패딩 포함, 실제 약 10 cm)로 줄었고, RPP 1.5 s 예측과 StopZone 정지 영역(옆 5 cm 띠)이 그 자리에서 진행을 막았다. 실제 접촉은 없었다(사용자 확인). 1.57은 회전을 진행으로 세지 않던 SimpleProgressChecker 시절(9/15 `2d7a7d8`) 값이다. 조치 `311a23d`: 0.785. `1641650`(사용자 지시, 모의 주행이라 박스 접촉 허용): RPP 예측 충돌 검사 끔(FollowPath·Parking·ParkingReverse), StopZone 정지·후진 영역의 옆·뒤를 패딩 footprint로, FootprintApproach 2.0→0.5 s. 남긴 것은 패딩 footprint StopZone, 회전 반경 원(0.43 m 16각형), 정밀 주차 앞면 5 cm, 센서 timeout 1 s.
4. **table_02 최종 접근 시작 전 실패(10:15).** 정렬은 관측 지점에서 본 면 기준 3° 안이었지만 가까이서 다시 본 면과 3.52° 달라 직선 접근 조건(3°)에 걸렸다. 조치 `abb3eef`: 3°를 넘으면 직선 경로를 다시 본 면 법선 방향으로 만들고 Parking이 그 방향으로 돌며 접근.
5. **도크 대기점 회전 AttributeError(10:21).** 한 방향 회전이 박스 탐색 Spin 함수를 공유하는데 그 함수의 `_search_target_yaw`가 탐색에서만 만들어졌다. 탐색을 거치지 않은 재개 실행에서 종료. 단위 테스트는 회전 함수를 모의로 바꿔 잡지 못했고, 테스트 하네스는 생성자를 거치지 않는다. 조치 `50bdc1e`: 생성자에서 초기화, 탐색 목표는 `search_rotation`일 때만 기록.
6. **출발 전 `/cmd_vel` 발행자 2.5 s 탐색 실패(10:18, 10:24, 이동 없음).** 파이 UDP RcvbufErrors가 부팅 직후 20,175, 10:18까지 84,538로 늘었고 새 프로세스의 탐색이 늦었다. 같은 원인으로 09:26 첫 세션 기동 때 keepout lifecycle과 09:36 내비게이션 lifecycle 관리자·planner_server 서비스가 새 프로세스에서 보이지 않아 세션을 다시 띄웠다. 조치 `35a54eb`: 발행자가 아직 안 보일 때만 30 s까지 기다림(다른 발행자는 즉시 거부 유지). UDP 넘침의 근본 원인(어느 소켓·어느 트래픽)은 확인하지 않았다.

### 20.3 9/30 이후 104·105 전체 (Nav2 journal 기준)

| 시각 | 구간 | 코드 | 사유 | 상태 |
|---|---|---|---|---|
| 9/30 15:04 | 최종 접근 | 105 | 진행 없음 | 직선 접근으로 해결 |
| 9/30 15:29 | 물 받는 곳 접근 | 105 | 진행 없음 + StopZone | 원인 미확정 |
| 9/30 16:01 | 물 이탈 → table_02 | 104 | collision ahead + StopZone | §20.2 3 |
| 9/30 17:14·17:31 | 박스 이탈 | 105 | 진행 없음 | odom 고정 이탈로 해결 |
| 9/30 17:39 | 도킹 | 105 | 진행 없음 | odom 도크 레그로 해결 |
| 9/30 18:31·18:41 | 최종 접근 | 104 | collision ahead | 직선 접근으로 해결 |
| 10/01 09:43 | 최종 접근 | 105 | Graceful 궤적 충돌 | §20.2 2 |
| 10/01 09:45 | 물 이탈 → table_02 | 104 | collision ahead ×45 + StopZone | §20.2 3 |

### 20.4 남은 확인

- 한 실행으로 끊김 없는 사이클(이번 수정이 모두 들어간 상태로).
- 파이 UDP 수신 넘침의 원인 트래픽.
- 충돌 검사 완화 뒤 박스 근접 거동(접촉 여부는 영상·사용자 관찰로).
- 3·4번 브랜치 AMCL 비교: 이번 bag 두 개를 `amcl_replay.py`에 넣는다.

### 20.5 파이 UDP 수신 넘침 측정 (2026-10-01 11:20–11:35, 이동 없음)

| 상황 | 측정 | 넘침(RcvbufErrors 증가) |
|---|---|---|
| 정지 상태 60 s | 수신 15,639건 | 0 |
| 새 참가자 접속 3회 | `/cmd_vel` 발행자 발견 0.97–1.91 s, 자기 소켓 대기열 최대 818 KB(버퍼 4 MB) | 0 |
| Nav2 세션 재기동 | 기동 중 부하 평균 6.26 | 0 |
| 부팅 직후(09:15–09:28) | — | 20,175 |
| 09:28–10:18(세션 재기동 3회, 주행 2회, 진단 프로브) | — | 64,363 |
| 10:18 이후 주행 2회 | — | 0 |

- 지금 살아 있는 소켓 중 넘침이 있는 것은 카메라 프로세스(1,355건)뿐이다. 84,538건의 대부분은 이미 종료된 프로세스의 소켓에서 났고, `/proc/net/udp`의 소켓별 계수는 소켓과 함께 사라져 사후에 주인을 가릴 수 없다.
- 오늘 2.5 s 발행자 탐색 실패(10:18, 10:24)는 조용한 상태의 발견 시간 1–2 s와 겹치는 짧은 한도 문제였다. 30 s 대기(`35a54eb`)와 상주 실행기(`4c5adcf`)로 출발마다의 탐색 자체를 없앴다.
- 원인 귀속을 위해 `go`가 실행 동안 파이에서 5 s마다 프로세스별 넘침을 `runs/<run>/udp_drops.jsonl`에 남긴다(`udp_drop_monitor.py`). 다음 주행에서 넘침이 다시 나면 그 파일로 프로세스를 가린다.

### 20.6 사이클 정지 시간과 시간 상수 (오늘 bag·이벤트, 11:40)

정지는 odom 기준 0.5 cm/s·0.02 rad/s 미만이 0.5 s 이상인 구간이다. 10:12 실행은 167 s 중 정지 7회 40 s, 10:19 재개는 116 s 중 5회 21 s, 10:26 도크만은 80 s 중 4회 9 s였다.

| 정지 | 시간 | 정하는 값 | 판단·조치 |
|---|---|---|---|
| 박스 관측(정렬 전·접근 전, 사이클당 4회) | 5.4–7.8 s | 관측기 `processing_hz` 2.0 × 안정 프레임 8 = 최소 4 s | 2 Hz는 9/29 임시 유닛 명령값이고 근거 기록이 없다. 설정값 5 Hz로 되돌림(최소 1.6 s). CPU 관측기 23 → 35 %, 전체 27 → 30 %(4코어). 관측기를 정식 유닛 파일로 설치 |
| 주차 확인 + 5 s 대기 | 8.6 s | 정지 확인 `hold_s` 1.0 s, 대기 5 s(과제) | 유지 |
| 이탈 뒤 다음 출발 | 2.2–3.2 s | 정지 확인 1.0 s + 계획 | 유지(정지 확인이 다음 경로의 출발 자세) |
| 대기점 방향 측정 | 2.0 s | 정지 확인 1.0 s | 유지 |
| 도킹 전·후 확인 | 2–3 s | 정지 확인 1.0 s | 유지(도크 판정 근거) |

상한으로만 쓰이는 시간(실패할 때만 기다림): 발행자 탐색 30 s, 지도 수신 30 s, 정지 포착 10 s, 관측 대기 5 s, 과제 600 s. Nav2: 경로 만료 10 s(대기점·이동 BT), 진행 판정 10 s 안에 5 cm 또는 0.1 rad, 제어 실패 허용 2 s. `session-start`는 재부팅 뒤 꺼져 있는 RGB-D·관측기 유닛을 먼저 켠다.

### 20.7 11:52 한 번의 실행으로 끝난 사이클 (run `table_02_20261001_115239`, bag `bag_20261001_115154`)

§20.2–§20.6 수정(`311a23d`·`1641650`·`abb3eef`·`50bdc1e`·`35a54eb`·`e6b1e3e`·`869b8d0`·`4c5adcf`·`ff85980`, 관측기 5 Hz)이 모두 들어간 첫 주행이다. 상주 실행기로 출발했고 run 이름 시각 11:52:39, 첫 Nav2 목표 수락 11:52:49였다. 시간은 첫 목표 수락부터 센 값이다.

| s | 구간 | 결과 |
|---|---|---|
| 0.1–23.0 | 도크 → 물 받는 곳 관측 지점 | 경유점 3개를 NavigateThroughPoses 한 목표로 23.0 s(오전 목표 3개 44.0 s) |
| 23.0–33.4 | 관측 2.7 s → 면 정렬 4.5 s → 재관측 2.1 s | 정렬 잔차 2.1°라 직선 접근은 로봇 방향 기준(`heading_basis: robot`) |
| 33.4–41.4 | 최종 접근 0.447 m | 8.0 s(오전 22.4–22.7 s) |
| 42.6 | 물 받는 곳 주차 확인, 5 s 대기 | 정지 0.44 cm·0.25°, 간격 5.45 cm(모서리 6.32·4.57), 면 1.86° |
| 50.0–55.9 | 후진 이탈 | 5.9 s(오전 22.3–22.9 s) |
| 59.3–69.5 | table_02 관측 지점 | 경유점 2개 한 목표로 10.3 s(오전 16.5 s) |
| 69.5–83.0 | 관측 2.0 s → 면 정렬 7.5 s → 재관측 2.9 s | 정렬 잔차 1.9°, 로봇 방향 기준 |
| 83.0–91.5 | 최종 접근 0.493 m | 8.5 s |
| 92.7 | table_02 주차 확인, 5 s 대기 | 정지 0.54 cm·0.38°, 간격 5.54 cm(모서리 6.23·4.84), 면 1.47° |
| 100.1–105.8 | 후진 이탈 | 5.7 s |
| 107.6–151.2 | 도크 대기점 이동 | Nav2 성공 43.6 s. 막바지 대기점 5–7 cm 앞에서 좌우 회전 약 14 s(아래) |
| 151.6–175.4 | 대기점 방향 맞춤 | Spin 요청 −132.8° → odom −136.0°, 보정 +3.9° → +7.1°, 정지 확인 |
| 175.4–185.2 | 곡선 후진 도킹 | 정렬 이동 없이 9.8 s(오전 28.4 s), 도착 확인 4.24 cm·1.19° |

- 사이클 186.5 s, 이동 139.7 s. Nav2 목표 10개 모두 error_code 0. 0.5 s 이상 정지 11회 45.0 s(관측 4회 1.7–3.8 s — 이 중 Nav2 결과부터 박스 확정까지 2.0–2.9 s, 주차 확인 + 5 s 대기 8.4·8.7 s, 이탈 뒤 2.2·3.2 s, 대기점 방향 맞춤 전후 2.9·3.2·4.7 s). 오전 세 실행 합은 363.3 s(정지 55.5 s).
- 파이 UDP: 실행 동안 5 s 간격 39회 기록에서 RcvbufErrors 84,538, 카메라 소켓 1,355가 그대로였다(증가 0, `runs/table_02_20261001_115239/udp_drops.jsonl`).
- 면 방향 1.5–1.9°는 정렬 잔차(2.1°·1.9°)가 직선 접근 동안 그대로 남은 값이다. 정렬 허용 안이면 로봇 방향으로 곧게 들어가도록 한 설계(`abb3eef`)의 결과이며, 오전 물 받는 곳 0.04°는 정렬 잔차가 작았던 경우다.

**대기점 앞 좌우 회전(사용자 관찰 "도킹 지점에서 좌우로 회전").** bag에서 대기점 5–7 cm 앞, 우 65° → 좌 105° → 우 35°를 약 14 s 돌았다. 대기점 도착 허용이 5 cm라 남은 몇 cm 경로를 따라야 했고, 그 짧은 경로의 방향이 차체 옆을 가리켜 45°(`rotate_to_heading_min_angle` 0.785) 넘는 차이로 제자리 회전이 반복됐다. 곡선 후진(`869b8d0`, 최소 회전 반경 0.3 m)이 대기점 위치 오차를 흡수하므로 5 cm 도착 정밀도는 필요 없다. 조치 `4621399`: 대기점 도착 허용 15 cm(PositionGoalChecker), 큰 회전 뒤 10° 이하 잔차는 보정 Spin 없이 곡선 후진에 맡김. 파이 반영·세션 재기동·init까지 마쳤고 실차 확인 전이다.

**Spin 초과 회전.** 두 번 모두 요청보다 3.2° 더 돌았다(−132.8° → −136.0°, +3.9° → +7.1°). 원인은 확인하지 않았다. 다음 주행에서는 10° 이하 보정 Spin이 빠져 잔차가 곡선 후진으로 넘어간다.

남은 확인:

- `4621399`: 대기점 앞 좌우 회전이 사라지는지, 15 cm 안의 대기점 오차에서 곡선 후진이 성립하는지(`dock_curve_unavailable` 이벤트가 없어야 함).
- Spin 3.2° 초과 회전의 원인(Spin 정지 판정과 odom 갱신 시점 비교).
- 남은 정지 중 줄일 수 있는 것: 주차 확인 1.0 s, 이탈 뒤 다음 목표까지 2.2–3.2 s, 관측 2.0–2.9 s. 5 s 대기는 과제 조건이다.
- `jdamr_depart.py record-stop`이 이번 bag(`bag_20261001_115154`) 대신 이전 이름 `bag_20261001_095541`을 출력했다. `record_start`가 mcap 크기 4 KB 초과를 30 s 안에 보지 못하면 상태 파일에 새 bag 경로를 남기지 않았기 때문이다(rosbag2는 메시지를 캐시에 모았다가 쓴다). 조치: 기록기 시작 직후 경로를 저장하고 쓰기 확인은 로그로만 남김. 또 115154 bag에 `metadata.yaml`이 없었다(mcap 끝 표식은 정상, 원인 미확인). `ros2 bag reindex`로 만들었다(275.1 s, 메시지 3,109개).
- UDP 넘침이 다시 늘면 `udp_drops.jsonl`로 프로세스를 가린다. 이번 실행에서는 늘지 않았다.
- 포트폴리오 데이터 묶음: `$HOME/jdamr_data/portfolio_20261001/`(지표 `metrics.json`, 궤적·타임라인 그림, run 이벤트 사본, `make_figures.py`, `MANIFEST.sha256`).

### 20.8 IMU 장착 방향 확정과 주행마다 파이 원시 기록 (2026-10-01 12:10–12:45, 이동 없음)

지금까지 주행은 바퀴 odom과 AMCL만 썼고 IMU(`/imu/data_raw`, 50 Hz)는 어디에도 연결되지 않았다. 3·4번 브랜치 분석에서 "자이로 z 부호 반대"만 확인하고 x·y 방향은 정하지 못했다.

| 확인 | 결과 |
|---|---|
| 자이로 z / 바퀴 회전율 기울기 | −0.966 ~ −0.996 (9/15–16 bag 5개) → 보드 z 아래 |
| 보드 y / 바퀴 전진 가속 | +1.15 ~ +1.25, 상관 0.87–0.93 (bag 4개) → 보드 y가 로봇 앞 |
| 보드 x / 바퀴 전진 가속 | −0.06 ~ −0.26, 원심 가속 v·ω에는 4개 모두 양수 → 보드 x가 로봇 왼쪽 |
| 정지 가속도 z | −9.35 ~ −9.45 m/s² |

- 보드는 뒤집힌 채 90° 돌아가 있다(base_link 기준 roll 180°, yaw 90°). 그런데 드라이버는 `frame_id: base_link`로 내고 있었다. z축만 쓰는 회전율 융합에는 영향이 없지만, 가속도나 x·y 회전율을 쓰면 축이 틀린다. 앞선 분석의 정적 TF(roll 180°만)는 x·y가 틀렸다.
- 조치: URDF `imu_joint`(`rpy 3.1416 0 1.5708`, 위치는 재지 않아 0)를 추가했고, 근거는 `new_base_geometry.yaml` `imu_rotation_rpy`에 남겼다. 드라이버 `imu_frame`은 `imu_link`로 바꿨다. 보정 대장도 `esp32_imu_base_link_to_imu_link_v2`로 바꿨다(UNVERIFIED, 융합 불허 유지). 파이에 반영(백업 `$HOME/jdamr_data/deploy_backup_20261001_1236_imu`)한 뒤 `jdamr-base`를 재시작했다. 확인 결과 `/imu/data_raw` frame `imu_link`, TF base_link→imu_link RPY (180°, 0°, 90°), `/odom`·`/imu/data_raw` 50 Hz, `/scan` 9.7 Hz.
- odom+IMU EKF(`imu_bias_relay`, `ekf_odom_imu.yaml`, 오프라인 `ekf_replay.sh`, `rotation_truth.py`)를 메인 브랜치로 가져왔다. 실행 구성에는 아직 넣지 않았다. 9/15 bag 재생 결과(제자리 회전 4건, 스캔 기준 오차 중앙값)는 이전과 같다. odom 0.50°, 자이로 원값 0.40°, EKF 0.10°(최대 0.18°)다.
- 표본을 늘리려고 `go`가 출발 요청 직전 파이에서 `/odom`·`/imu/data_raw`·`/scan`·`/tf`·`/tf_static`·`/amcl_pose`·속도 명령(`/cmd_vel_nav`·`/cmd_vel_smoothed`·`/cmd_vel`)을 기록한다. 유닛은 `jdamr-onboard-bag-<run>`이고 캐시 1 MB, 최대 1시간·400 MB로 묶었다. 사이클이 끝나면 닫고(메타데이터가 없으면 reindex) run 폴더째 PC로 복사한다. 이동 없는 26.5 s 시험에서 IMU·odom 1,326건, 스캔 257건이 기록됐고 static TF에 imu_link가 들어 있었다. 약 8 MB/분이다. 기록기가 첫 메시지를 쓰기까지 약 8 s 걸리므로 출발 전 정지 구간은 짧다. 바이어스는 주행 중 정지 구간(주차 확인·대기)에서 배운다.
- 주행 뒤 분석: `scripts/map_20260930_manual/imu_check.sh <run_dir>` → `runs/<run>/imu_check/`(보드 축, EKF 재생, 정지 구간으로 감싼 회전의 odom·자이로·EKF 오차). 도크 대기점 Spin(약 133°)과 박스 앞 회전이 표본이 된다. 같은 기록의 속도 명령으로 Spin 3.2° 초과 회전도 본다.

남은 확인:

- 다음 주행 bag으로 보드 축이 지금 장착에서도 같은지(`imu_check.sh`의 `axes.jsonl`)와 큰 회전에서 EKF가 odom보다 나은지 확인. 나으면 드라이버 `publish_tf:=false` + EKF가 odom TF를 내도록 연결하고, Nav2가 읽는 `/odom` 속도를 바꿀지는 따로 정한다.
- 보드 위치와 가속도 스케일(정지 0.95 g)은 재지 않았다. 가속도는 융합하지 않는다.

### 20.9 14:42 주행: 대기점 좌우 회전 해소, 도크 끝 방향 4.4°에서 105 (run `table_02_20261001_144234`)

`4621399`(대기점 허용 15 cm·작은 보정 회전 생략)와 §20.8 파이 원시 기록이 들어간 첫 주행이다. 사용자 관찰: 도착까지 깔끔했고, 도착 뒤 앞뒤로 조금 왔다 갔다 했다.

| s (첫 목표 수락 기준) | 구간 | 결과 |
|---|---|---|
| 0.4–23.8 | 도크 → 물 받는 곳 관측 지점 | 23.4 s |
| 26.5–43.6 | 면 정렬 5.1 s, 최종 접근 7.9 s | 정지 0.56 cm·0.34°, 간격 5.56 cm(모서리 5.56·5.56), 면 0.00° |
| 52.2–58.4 | 후진 이탈 | 6.2 s |
| 61.2–92.2 | table_02 관측 10.5 s, 면 정렬 6.2 s, 최종 접근 7.8 s | 정지 0.48 cm·2.38°, 간격 5.49 cm(모서리 4.36·6.61), 면 2.38° |
| 100.8–106.8 | 후진 이탈 | 6.0 s |
| 109.0–137.5 | 도크 대기점 이동 | 28.6 s, 좌우 회전 없음(11:52는 14 s 좌우 회전 포함 43.6 s) |
| 138.9–153.6 | 대기점 Spin | 요청 −121.9°, odom −125.7°. 남은 2.3°는 보정 회전 없이 곡선 후진으로 |
| 156.1–179.4 | 곡선 후진 도킹 | 12.8 s에 도크 위치 0.5 cm 안, 방향 4.4° 남음 → 10 s 앞뒤 왕복 → 105 |

- 도크 위치 도달까지 약 169 s. 주행 중 파이 UDP 넘침 증가 0.
- **105 원인(파이 bag `/odom`·`/cmd_vel_nav`·`/cmd_vel`).** 후진은 0.08 m/s로 경로를 따라 12.8 s에 목표 0.5 cm 안까지 왔다. 방향은 곡선 추종 지연으로 4.4° 남았다(도크 허용 3°). 그 뒤 컨트롤러 명령이 −0.020 ↔ +0.020 m/s로 바뀌며 0.1–0.5 cm 안에서 앞뒤로 움직였다. 명령은 Collision Monitor를 그대로 통과했다(충돌 정지 아님). 10 s 진행 없음으로 105. Nav2 RPP는 `use_rotate_to_heading`과 `allow_reversing`을 함께 켤 수 없어(설치 라이브러리 문구 "Both use_rotate_to_heading and allow_reversing parameter cannot be set to true") 후진 컨트롤러는 목표에서 돌지 못한다. 목표 판정기는 위치와 방향을 같이 요구해 끝나지 않았다. 11:52에는 끝 방향이 1.19°라 이 상황이 생기지 않았다.
- **조치 `b7bc384`.** 도크 후진은 `dock_position_checker`(PositionGoalChecker, 2 cm)로 위치에서 끝낸다. 남은 방향이 도크 허용(3°)을 넘으면 전진 `Parking` 컨트롤러(제자리 회전 가능)에 같은 위치·도크 방향의 두 점 경로를 보내 돌린다. 박스 면 정렬과 같은 방식이고, 종료는 도크 판정기가 허용 안에서 끊는다. 이후 odom 기준 정지 확인. 도크에서 가장 가까운 장애물은 왼쪽 벽 0.448 m라 회전 정지 영역(0.43 m) 밖이다(14:50 스캔). 단위 테스트(4.4°면 회전 1회, 2°면 없음)와 관련 612건 통과. 파이 반영(백업 `$HOME/jdamr_data/deploy_backup_20261001_1453_dockturn`), 세션 재기동·init 완료, 실차 확인 전.
- **Spin 초과 회전 원인(같은 bag).** Spin은 끝까지 0.2 rad/s로 돌았다(감속 기준 `rotational_acc_lim` 1.5 rad/s²라 목표 0.8° 전부터만 감속). 정지 명령이 나간 순간 odom은 이미 목표를 2.6° 지나 있었고(동작 서버 10 Hz·TF 20 Hz·속도 평활기 지연, 약 0.2 s), 정지 명령 뒤 0.23 s 동안 1.3° 더 돌았다. 스캔 정합 기준 실제 회전은 −123.85°라 실제 초과는 약 2.0°다. odom이 3.8°로 보인 것은 바퀴 odom이 회전을 1.5 % 크게 재기 때문이다. 감속 설정은 바꾸지 않았다(대기점 잔차는 곡선 후진이, 도크 방향은 위 제자리 회전이 받는다).
- **IMU(`imu_check.sh`).** 지금 장착에서도 보드 축이 같다. 자이로 z/바퀴 회전율 −0.994, 보드 y/전진 가속 +0.96(상관 0.87), 보드 x는 왼쪽이다. 대기점 Spin 1건을 스캔 정합(잔차 3.0 cm)과 비교하면 오차는 odom 1.88°, 바이어스 뺀 자이로 0.25°, EKF 0.19°다. 이번 자이로 바이어스는 −0.0114 rad/s(9/15–16은 ±0.002)라 빼지 않으면 분당 약 39° 흐른다.
- **정지 판정 수정.** 정지 중 odom 속도에 한쪽 바퀴 엔코더 한 칸(0.00126 m/s, 0.00495 rad/s)이 섞여 "정확히 0"이 1 s를 못 넘었다. 그래서 회전 비교가 대기점 Spin 앞 정지를 못 잡았고, `imu_bias_relay`도 바이어스를 거의 배우지 못했을 것이다. 두 곳 모두 0.002 m/s·0.006 rad/s 안을 정지로 본다.

남은 확인:

- 다음 주행에서 도크 끝 제자리 회전(`dock_heading_measured` → `turn_in_place`)이 도크 허용 안에서 끝나는지.
- EKF 연결: 큰 회전 표본은 1건(EKF 0.19° vs odom 1.88°)이다. 표본을 더 모은 뒤 드라이버 `publish_tf:=false` + EKF odom TF로 바꾼다.
- 14:55 init에서 table_02 영역이 표지(0.689, −2.591) 대신 지도에 없는 라이다 덩어리(0.823, −2.721)로 잡혔다. 다음 출발 전 박스 위치를 확인한다.

### 20.10 table_01 첫 실차 (15:35–15:56, run `table_01_20261001_153549`, `_154936`, `_155418`)

| 실행 | 결과 |
|---|---|
| 15:35 1번 출발 | 물 받는 곳 주차 5.65 cm(모서리 5.64·5.67), 면 0.04°, 정지 0.65 cm·0.32°. table_01 관측 지점 도착 뒤 박스 관측 실패 → 탐색 회전 12번 소진 |
| 15:49 재개(`--resume-at-observation`) | 경유지가 남아 물 받는 곳 관측 지점 확인에서 즉시 종료(이동 없음). 재개에는 `--skip-via`를 함께 줘야 한다 |
| 15:49 재개(`--skip-via --resume-at-observation`) | 박스 관측 성공, 면 정렬 후 최종 접근 사전 계획이 목표 0.05 m 앞에서 끝나(허용 0.01) 실패 |
| 15:54 재개 | table_01 주차 5.63 cm(모서리 5.63·5.64), 면 0.01°, 정지 0.63 cm·0.37°, 5 s 대기, 이탈 6.0 s, 대기점 21.6 s, Spin −129.3° → −131.7°(잔차 −0.29°는 보정 없이), 곡선 후진 12.4 s, 도크 방향 0.87°(3° 안이라 제자리 회전 없음), 도크 확인 1.48 cm·0.88° |

**관측 실패 원인(실시간 깊이 영상으로 확인).** 라이다는 박스 앞면을 x ≈ 1.50 m, y −1.84 ~ −2.01 m에 보고 있었다(사용자 RViz 확인과 같음, 등록 앞면 (1.438, −1.817)에서 동쪽 6 cm·남쪽 10 cm). 깊이 영상에도 박스가 0.65 m에 10,135픽셀로 찍혔다. 그러나 검출기는 다음 두 가지 때문에 박스를 고르지 못했다.

1. 화면 전체에서 가장 큰 평면 하나를 고르는데, 박스 뒤 1.33 m에 폭 1 m 면이 있었다(9/30 배치 검사의 "카메라 시야 1.5 m 안 15칸, 1.28 m" 경고와 같은 곳).
2. 범위를 좁혀도 박스 면(평면 잔차 1.9 mm, nz 0.98)의 폭이 0.158 m(5–95 %)라 최소 폭 0.18 m에 걸렸다.

같은 실시간 20프레임 비교:

| 설정 | 박스 검출 | 뒤쪽 면 검출 |
|---|---|---|
| 범위 2.0 m, 최소 폭 0.18 m | 1/20 | 대부분 |
| 범위 2.0 m, 최소 폭 0.12 m | 6/20 | 14/20 |
| 범위 1.117 m, 최소 폭 0.12 m | 20/20 | 0/20 |

조치 `ad09a74`: 실행기가 관측마다 관측기 `maximum_depth_m`을 "영역까지 거리 + 영역 반경"(설정 2.0 m 이하)으로 정한다(`box_depth_window` 이벤트). 관측기는 실행 중 이 값만 받는다. 최소 폭은 0.12 m이고, 라이다 교차 확인은 그대로 둔다.

**최종 접근 사전 계획(조치 `552b818`).** 정적 지도·keepout은 목표 주변이 비어 있었고, 계획은 만들어졌지만(error_code 0) 정확히 한 칸(0.05 m) 앞에서 끝났다. 라이다로 본 박스가 전역 코스트맵 장애물로 올라가 칸 경계에 따라 목표 칸이 팽창 안쪽에 걸린 것으로 판단한다(코스트맵 값은 발행이 없어 직접 읽지 못함, 추정). 이 계획은 길이 비었는지만 보는 확인이라 허용을 한 칸 + 1 cm(0.06 m)로 했다. 실제 접근은 직선 경로와 1 cm 정지 확인이 결정한다.

**UDP 넘침(`udp_drops.jsonl`).** 15:35 실행 329 s 동안 +3,182(box_service 2,876, planner_server 306), 15:54 실행 101 s 동안 +655(파이 원시 기록기 `ros2 bag`).

남은 확인:

- 도크 끝 제자리 회전은 이번에 방향이 0.87°라 쓰이지 않았다. 3°를 넘는 경우는 아직 실차로 보지 않았다.
- table_01 등록 앞면을 실제 위치(약 (1.50, −1.92))로 갱신할지. 관측이 실제 면을 쓰므로 주행에는 지장이 없었다.
- 원시 기록기와 실행기의 UDP 수신 넘침이 주행에 영향을 주는지.

### 20.11 16:06 1번 출발: 한 번의 실행으로 끝난 table_01 사이클 (run `table_01_20261001_160621`)

`ad09a74`·`552b818`·`ffc8970`이 들어간 첫 1번 출발이다. 탐색 회전·재시도·실패 없이 첫 목표 수락부터 `home_arrived`까지 173.8 s.

| s | 구간 | 결과 |
|---|---|---|
| 0.0–31.7 | 도크 → 물 받는 곳 관측 지점 | 31.7 s |
| 35.5–53.6 | 면 정렬 6.0 s, 최종 접근 7.8 s | 간격 5.52 cm(모서리 4.72·6.33), 면 1.71°, 정지 0.52 cm·0.21° |
| 62.2–68.2 | 후진 이탈 | 6.0 s |
| 70.8–102.4 | table_01 관측 12.7 s, 면 정렬 3.4 s, 최종 접근 7.8 s | 간격 5.50 cm(모서리 6.67·4.33), 면 2.49°, 정지 0.50 cm·2.49° |
| 111.0–117.0 | 후진 이탈 | 6.0 s |
| 118.7–140.9 | 도크 대기점 | 22.2 s, 좌우 회전 없음 |
| 142.6–160.5 | 대기점 방향 맞춤 | Spin −121.9° → −125.1°(11.0 s), 잔차 0.98°는 보정 없이. 회전 외 정지 확인·경로 확인 합 약 8 s |
| 160.5–172.7 | 곡선 후진 도킹 | 12.2 s, 끝 방향 2.40°(3° 안), 도크 확인 1.63 cm·2.38° |

- table_01 관측은 `box_depth_window`(영역 거리 + 반경)로 매번 첫 시도에 확정됐다.
- 파이 UDP 넘침 증가 0. 파이 원시 기록 확인(25 s)은 4.0 MB로 정상 표시됐다.
- 줄일 수 있는 시간: 대기점 방향 맞춤 중 회전 외 대기 약 8 s(정지 포착 2.1 s, 재측정 1.0 s, 정지 확인 1.1 s, 경로 확인 1.5 s 등). 도크 → 물 받는 곳 31.7 s는 오전 23 s보다 길다(원인 미확인).

### 20.12 연속 주행(1번 → 충전 → 2번 → 충전) 가능 여부 (16:14, 이동 없음)

| 확인 | 결과 |
|---|---|
| 사이클 뒤 출발 조건 | `go`는 `localized` 상태만 본다. 사이클이 끝나도 유지돼 init 없이도 다음 `go`가 가능하다 |
| 첫 구간 출발 위치 | 1번·2번 모두 도크 → 물 받는 곳 경로로 시작하고, 출발 조건은 도크 0.3 m 안이다. 16:09 도킹 뒤 상태에서 상주 실행기에 계획만 요청(`--execute` 없음, run `dryrun_chain_*`): 결과 코드 0, `transit_planned_only` |
| 배터리 | 12.24 V(출발 기준 10.8 V, 주행 하한 10.5 V) |
| 충전 | 충전 케이블은 손으로 꽂고 시스템은 감지하지 않는다(전압으로 판정하지 않음). 케이블을 꽂은 채 다음 사이클이 출발하면 안 된다 |

조치 `jdamr_depart.py go table_01 table_02`(테이블 순서대로, 반대도 가능): 사이클마다 도크에서 확정돼야 다음으로 간다. 사이에 `--dock-wait-s`(기본 10 s) 정차 뒤 도크 init(스캔 정합·AMCL 확인, 이동 없음)을 다시 한다. `stop`은 대기 중에도 듣는다. 실제로 충전을 하려면 사이클마다 `go`를 따로 실행하고 사이에 케이블을 꽂았다 뺀다. 모의 테스트 3건(순서, 첫 실패에서 멈춤, 단일 사이클 옵션 거부)을 추가했다. 실차 연속 주행은 아직 하지 않았다.

### 20.13 16:22 연속 주행 2번 → 도크 → 1번 → 도크 (run `table_02_20261001_162204`, `table_01_20261001_162642`)

`jdamr_depart.py go table_02 table_01` 한 명령으로 두 사이클을 이어 마쳤다. 사이클 사이에는 도크 10 s 정차와 init(이미 활성인 스택에서 STARTUP 없이, `252cb6e`)을 했다.

| 사이클 | 시간 | 물 받는 곳 | 테이블 | 도크 끝 방향 → 제자리 회전 | 도크 확인 |
|---|---|---|---|---|---|
| 1 (table_02) | 198.2 s | 5.61 cm, 면 0.15° | 5.46 cm(모서리 4.36·6.57), 면 2.35° | 4.27° → 0.51 s | 1.46 cm·0.92° |
| 2 (table_01) | 167.9 s | 5.44 cm, 면 0.06° | 5.64 cm(모서리 5.24·6.03), 면 0.84° | 5.93° → 0.71 s | 1.33 cm·1.56° |

- **도크 끝 제자리 회전(`b7bc384`) 첫 실차 사용.** 두 번 모두 0.5–0.7 s 만에 허용 3° 안으로 들어왔다.
- **사이클 사이 간격:** 도크 도착 16:25:35 → 사이클 종료 16:25:51 → init 완료 16:26:40 → 다음 출발 16:26:50, 약 75 s.
- **두 번째 사이클 끝 대기:** 도크 도착부터 종료까지 110 s 걸렸다. 기록기는 16:29:56에 바로 멈췄고, 나머지는 bag 정리와 PC 복사로 보인다(원인 미확인).
- **UDP 넘침:** 파이 원시 기록기에서만 +587, +791.
- **table_02 면 정렬 불필요한 회전(사용자 관찰).** 정렬 목표가 관측 위치에서 앞 0.27 m, 옆 0.119 m였다(다른 실행은 옆 0.006–0.067 m).
  - 0–9 s: 곡선으로 목표 5.6 cm까지 왔다.
  - 9–21 s: 5 cm 판정을 채우려고 남은 몇 cm 경로를 따랐는데, 경로 방향이 옆을 가리켜 제자리 회전이 좌우로 반복됐다(1 Hz 재계획·AMCL 잡음).
  - 21–31 s: 3 cm 전진 뒤 103°를 되돌아 돌았다.
  - 합계 30.5 s, 왼쪽 124.8°·오른쪽 129.4°, 방향 전환 5번. 도크 대기점 좌우 회전(`4621399`)과 같은 현상이다.
- **조치 `46e50d2`.** 박스 면 정렬은 `face_alignment_checker`(위치 0.10 m, 방향 3°)와 전용 BT `navigate_to_pose_face_alignment.xml`을 쓴다. 이 기록이었다면 6.5 s에 위치를 채우고 약 35°만 돌았을 것이다. 도크 대기점 정렬 대체 경로는 직선 후진이 이어지므로 5 cm `alignment_goal_checker`를 유지한다. 관련 635건 통과, 파이 반영·세션 재기동·init 완료(16:38), 실차 확인 전.

### 20.14 Lattice 시험 (브랜치 `feat/lattice-planner-trial`, 18:30–18:40, 이동 없음)

파이의 살아 있는 세션에서 `scripts/map_20260930_manual/plan_compare.py`로 같은 시작·목표를 `GridBased`(NavFn)와 `Lattice`(SmacPlannerLattice, 차동 0.5 m 프리미티브, 전역 팽창 0.45)로 계획만 했다.

| 구간 | NavFn 길이 비 | Lattice 회전 패널티 1.5 | 0.5 | 0.0 |
|---|---|---|---|---|
| A 도크 → 물 받는 곳(경유 3) | 1.11 | 실패(도크 출발, NO_VALID_PATH) | 2.00 | 1.99 |
| B 물 받는 곳 → table_02(경유 2) | 1.04 | 1.00 | 3.91 | 1.02 |
| C 물 받는 곳 → table_01(경유 2) | 1.03 | 5.18 | 5.30 | 3.83 |
| D table_01 → 도크 대기점 | 1.04 | 1.34 | 1.03 | 1.02 |
| E table_02 → 도크 대기점 | 1.05 | 1.05 | 1.04 | 1.01 |

- 계획 시간은 파이에서 0.02–0.8 s로, 9/30 PC 비교의 수십 초 문제는 재현되지 않았다.
- 경유점 구간은 Lattice가 중간 경유점의 방향까지 맞추려고 2–5배 돌아간다. 도크 안 출발은 차체 외곽 충돌 검사에 걸릴 때가 있다.
- 도크 대기점 구간은 1.01–1.04에 도크 방향으로 도착한다. 대기점에서의 약 −122° Spin을 줄일 수 있는 후보다.
- 브랜치 마지막 설정: 대기점 BT만 Lattice(회전 패널티 0.5), 경유점 이동은 NavFn. 실차 주행은 하지 않았다. 메인은 Lattice 이전 상태로 두었다.

### 20.15 Lattice 대기점 구간 첫 실차 (18:51, run `table_02_20261001_185143`)

브랜치 `feat/lattice-planner-trial` 설정(대기점 BT만 Lattice, 회전 패널티 0.5, 전역 팽창 0.45)으로 2번 사이클을 한 번에 마쳤다. 같은 table_02 사이클의 이전 실행과 비교한다.

| 항목 | 18:51 Lattice | 16:22 NavFn | 14:42 NavFn | 11:52 NavFn |
|---|---|---|---|---|
| 사이클 | **171.0 s** | 198.2 s | 도킹 실패 | 186.4 s |
| 이탈 뒤 대기점 도착 | 34.0 s | 29.2 s | 28.6 s | 43.6 s |
| 대기점 도착 방향 오차 | **0.4°** | −114.6° | −121.9° | −132.8° |
| 대기점 Spin | **없음** | −117.9° | −125.7° | −136.0°, +7.1° |
| 대기점 도착 → 도크 후진 시작 | **15.8 s** | 30.6 s | 18.6 s | 24.2 s |
| 이탈 → 도크 후진 시작 합 | **49.8 s** | 59.8 s | – | – |
| 도크 끝 방향 → 제자리 회전 | 6.94° → 회전 | 4.27° → 회전 | – | – |
| 도크 확인 | 1.54 cm·0.38° | 1.46 cm·0.92° | – | 4.24 cm·1.19° |
| table_02 면 정렬 | 10.7 s | 30.6 s | 6.2 s | 7.5 s |
| 주차(물 받는 곳 / table_02) | 5.62 cm·0.83° / 5.69 cm·0.54° | 5.61·0.15 / 5.46·2.35 | 5.56·0.00 / 5.49·2.38 | 5.45·1.86 / 5.54·1.47 |

- 이탈부터 도크 후진 시작까지 odom 회전 합은 비슷하다(Lattice 348°, 16:22 NavFn 361°, 이동 3.04 m vs 3.15 m). 다만 Lattice는 회전을 주행 중에 나눠 하고 도크 방향으로 도착해 대기점 Spin(약 12 s)과 그 앞뒤 정지 확인이 없어졌다.
- table_02 면 정렬 10.7 s는 `46e50d2`(면 정렬 10 cm 판정기, main 포함)의 첫 실차 확인이다(16:22 30.6 s).
- 표본 1건. 다음은 1번(table_01 → 대기점 구간)과 연속 주행으로 확인한다.

### 20.16 NavFn과 Lattice 대기점 구간 비교 (18:51–19:10, 설정 고정)

같은 날 설정이 다른 실행이 섞여 있어, 실행마다 `run_config.json`에 대기점 계획기와 그때 반영된 수정을 남겼다(`scripts/map_20260930_manual/compare_runs.py`로 집계). 18:51 이후 네 실행은 모두 같은 설정이다(Lattice 대기점, 면 정렬 10 cm, 도크 위치 + 제자리 회전, 깊이 범위).

| run | 대기점 계획기 | 사이클 s | 대기점 도착 방향 오차 | Spin | 대기점 도착 → 도크 후진 s | 이탈 → 도크 후진 s | 도크 확인 |
|---|---|---|---|---|---|---|---|
| table_02 11:52 | NavFn | 186.4 | −132.8° | −136.0°, +7.1° | 24.2 | 69.6 | 4.24 cm·1.19° |
| table_02 16:22 | NavFn | 198.2 | −114.6° | −117.9° | 30.6 | 61.4 | 1.46 cm·0.92° |
| table_02 18:51 | Lattice | 171.0 | 0.4° | 없음 | 15.8 | 51.6 | 1.54 cm·0.38° |
| table_02 19:02 | Lattice | 171.0 | −90.1° | −92.8° | 16.6 | 50.2 | 1.49 cm·2.00° |
| table_01 16:06 | NavFn | 173.8 | −121.9° | −125.1° | 19.6 | 43.5 | 1.63 cm·2.38° |
| table_01 16:26 | NavFn | 167.9 | −123.3° | −126.4° | 31.1 | 54.6 | 1.33 cm·1.56° |
| table_01 18:58 | Lattice | 160.4 | −88.9° | −92.2° | 16.1 | 41.4 | 1.49 cm·2.99° |
| table_01 19:07 | Lattice | 172.4 | −108.4° | −112.0° | 29.2 | 53.5 | 1.30 cm·0.80° |

- table_02: 이탈 → 도크 후진이 Lattice 50.2–51.6 s, NavFn 61.4–69.6 s. table_01: 41.4–53.5 s 대 43.5–54.6 s로 차이가 뚜렷하지 않다.
- 대기점에 도크 방향으로 도착한 것은 4번 중 1번(18:51)이다. 나머지는 위치 판정(15 cm)이 Lattice 경로의 마지막 꺾임보다 먼저 끝났다. 18:58 table_01은 북쪽(90°)을 보고 0.12 m/s로 직진하다 대기점 12 cm 앞에서 끝났다(odom·TF 확인). Spin은 92–112°로 NavFn의 115–136°보다 작다.
- 사이클 시간은 면 정렬 수정(16:36 반영) 전후가 섞여 있다. NavFn 실행은 모두 수정 전이라 사이클 차이를 Lattice 효과로만 볼 수 없다. 16:22 table_02는 면 정렬 30.6 s가 포함됐다.
- 표본은 테이블별로 NavFn 2건, Lattice 2건이다.

비교가 끝난 뒤 처리할 항목(설정을 바꾸면 비교가 깨지므로 지금은 기록만 함):

1. 도크 끝 제자리 회전 대신 방향을 맞춘 뒤 후진할 것(사용자 지적). 지금은 도크 위치에 도달한 뒤 남은 방향을 제자리에서 돈다.
2. Lattice 대기점 구간은 위치 판정(15 cm)이 마지막 꺾임 전에 끝나지 않도록 할 것(방향도 보는 판정, 또는 꺾임이 판정 밖에서 끝나는 경로).
3. 연속 주행에서 사이클 사이 도크 대기를 없앨 것(지금 `--dock-wait-s` 기본 10 s).
4. 각 정차 지점(물 받는 곳·테이블)의 5 s 대기를 2 s로 줄일 것.

### 20.17 비교 뒤 수정과 계획 탐침 (2026-10-01 19:45 – 10-02 00:12, 이동 없음)

§20.16의 처리 항목 네 건을 반영했다. 계획기 선택은 파이 실세션에서 계획만 해서 정했다. 로봇은 움직이지 않았다.

**계획기 선택: 구간별 선택 유지 (이동 구간 NavFn, 대기점 구간 Lattice)**

| 구간 (경유점 없이 시작 → 목표) | NavFn 길이 비 | Lattice 길이 비 | Lattice 계획 시간 |
|---|---|---|---|
| A 도크 → 물 받는 곳 관측 | 1.08 | 2.78 | 0.109 s |
| B 물 받는 곳 → table_02 관측 | 1.05 | 3.14 | 0.004 s |
| C 물 받는 곳 → table_01 관측 | 1.05 | 2.64 | 0.008 s |
| D table_01 → 도크 대기점 | 1.00 | 1.03 | 0.200 s |
| E table_02 → 도크 대기점 | 1.02 | 1.04 | 0.175 s |

- 경유점을 포함한 같은 날 19:45 계획에서는 A 2.05, C 2.23이었다(NavFn 1.12, 1.03). 경유점을 빼면 Lattice가 더 길어지므로, Lattice 통일 판정 기준(길이 비 1.1 이하)은 계획 단계에서 통과하지 못했다. 통일 판정용 주행 3회는 하지 않는다.
- 계획 시간은 NavFn 0.001–0.039 s, Lattice 0.004–0.543 s. 두 계획기를 올린 `planner_server` 메모리는 77 MB(파이 사용 1.1 / 3.8 GB).
- Smac Hybrid-A*는 후보에서 뺀다. 설치본 `constants.hpp`의 모션 모델은 `TWOD`, `DUBIN`, `REEDS_SHEPP`, `STATE_LATTICE`이고 Hybrid-A*는 DUBIN·REEDS_SHEPP(최소 회전 반경 곡선)만 쓰므로 제자리 회전을 쓰지 못한다.
- Jazzy 1.3.12 Smac 헤더에 `goal_heading_mode`가 없다. 경유점마다 정해진 방향으로 도착해야 하는 것이 이동 구간 우회의 원인으로 보인다(추론).
- NavFn 헤더 주석은 "ROS values of 253 are obstacles"다. 우리 내접 반경이 0.085 m여서 NavFn은 약 0.25 m 이상 틈을 지나갈 수 있다고 본다(차체 폭 0.58 m, 회전 지름 0.83 m). 이동 구간에 NavFn을 남긴 대가이고, 지금은 Collision Monitor 정지 구역이 막는다. 좁아진 통로 실측은 하지 않았다.
- 원본: `$HOME/jdamr_data/map_20260930_manual/aligned/plan_compare_20261001_1945.jsonl`, `plan_probe_direct_staging_20261001_2356.jsonl`, `plan_probe_rotation_penalty_20261001_2357.jsonl`, 탐침 `plan_probe_20261001.py` (보관본 `/data/lim/jdamr_artifacts/jdamr_runs_20261001/plan_probes/`).

**처리 항목 2 (Lattice 대기점의 방향 판정): 효과 없음으로 보류**

- 대기점 구간 Lattice 경로는 직진한 뒤 목표점에서 제자리 회전으로 끝난다. 마지막 0.3 m 안의 제자리 회전이 D 63.4°, E 90.0°다. 회전 패널티를 1.5·3.0·5.0으로 올려도 D·E 모두 제자리 회전 63.4°로 같았다(길이 비 1.03–1.14).
- 판정에 방향을 넣으면 그 회전을 Spin 대신 RPP가 하게 된다. RPP의 끝 회전은 AMCL 방향을 따르며, 2026-09-30 도크 근처에서 AMCL이 약 15° 튀어 먼 쪽으로 돌았다가 돌아왔다(§`_reach_staging` 설명). 그래서 odom Spin을 유지한다.
- 18:51의 방향 정렬 도착(0.4°)은 그 실행의 시작 자세에서 나온 경로 모양으로 본다(추론). Lattice의 대기점 이점은 Spin이 92–112°로 NavFn의 115–136°보다 작은 것이다.
- 탐침 중 사고: 회전 패널티 원래 값 읽기가 "Node not found"로 비어 복원 명령이 실패했고, 실행 중 `planner_server`의 `Lattice.rotation_penalty`가 23:57부터 복원 명령까지 약 1–2분 5.0으로 남았다. 설정 파일 값 0.5로 다시 설정하고 `Double value is: 0.5`를 확인했다. 그 사이 주행은 없었다.

**처리 항목 1 (도크 안 제자리 회전 없애기): 방향을 도크 밖에서 맞춘 뒤 직선 후진**

도크 끝 방향 오차는 대기점의 옆 어긋남에서 나왔다. 대기점 Spin 뒤 방향은 0.3–2.3°로 정확했지만, 곡선 후진이 옆 어긋남을 도크까지 끌고 가 바로잡으면서 방향이 그만큼 남았다(odom, onboard bag).

| run | 대기점 옆 어긋남 | 도크 끝 방향 오차 |
|---|---|---|
| table_01 15:54 | −0.4 cm | −0.87° |
| table_01 16:06 | −4.5 cm | −2.39° |
| table_02 19:02 | −4.1 cm | −2.00° |
| table_01 18:58 | −4.4 cm | −2.99° |
| table_01 19:07 | +4.6 cm | +3.32° |
| table_02 16:22 | −7.5 cm | −4.27° |
| table_01 16:26 | −8.3 cm | −5.93° |
| table_02 18:51 | −9.6 cm | −6.93° |

옆 어긋남 1 cm마다 약 0.7°이고, 도크 끝 위치는 모두 0.2 cm 안·1.7–1.9 cm 앞에서 멈췄다. 수정(`restaurant_service.py` `_dock_straight`):

1. 대기점 옆 어긋남이 1 cm를 넘으면 도크 앞 0.25 m의 도크 축 위 지점(pre-dock)까지만 곡선 후진한다. 0.25 m는 10 cm 어긋남에서도 곡선 반경이 0.3 m보다 크게 남는 거리다. 곡선이 안 맞으면 기존 정렬 구간(turn-move-turn)으로 간다.
2. 직선 시작점(pre-dock 또는 대기점)에서 방향이 3°를 넘으면 그 자리에서 Parking 컨트롤러로 제자리 회전한다. 도크 밖이라 회전 원(반경 0.414 m)이 도크 위치의 차체 뒤 끝보다 0.13 m 앞에 머문다.
3. 도크까지는 odom 직선 후진(`reverse_waypoints`, 5 cm·3°)이고 위치 판정 2 cm로 끝낸다.
4. 도크 끝 방향이 3°를 넘으면 도크 안에서 돌지 않는다. 직선 시작점까지 직선 전진으로 빠져나와 방향을 맞추고 한 번 더 들어간다. 그래도 넘으면 `dock_heading_out_of_tolerance`로 실패한다.
5. 모든 구간은 대기점에서 고정한 odom 기준이다. 이벤트: `dock_leg_frozen_in_odom`(대기점 옆 어긋남·pre-dock 자세), `dock_entry_heading_measured`, `dock_heading_measured`(시도 번호), `dock_heading_out_of_tolerance`.

**처리 항목 3·4**

- 3: `jdamr_depart.py` `DOCK_WAIT_S` 10 → 0 s. 연속 주행 사이의 init(스캔 정합 재위치추정, 이동 없음)은 남겼다. 19:02 연속 주행의 사이클 사이 61 s는 대기 10 s, init 41 s(클릭 5 s, 스캔 정합 18 s, 활성화 확인 14 s, 지역 확인 3 s), 출발 준비 10 s였다. 이 init에서 스캔 정합이 클릭 기준 0.10 m·1.2° 차이를 잡았다.
- 4: `box_service.py` `STOP_DWELL_S` 2.0 s. 물 받는 곳, 테이블, 정차 재개(`resume_parked`) 모두 같은 값이다. restaurant_service의 `serve` 흐름(지금 쓰지 않음)의 5 s는 그대로다.

**검증과 배치**

- 테스트: `test_input_recovery.py`·`test_restaurant_service.py`·`test_box_service.py` 390건, `test_jdamr_depart_flow.py` 36건 통과. 도크 테스트 t27(2.0°는 한 번에, 4.4°는 빠져나와 다시 진입, 회전은 직선 시작점에서만), t27b(두 번째에도 넘으면 실패, 도크 안 회전 없음), t27c(6 cm 어긋남은 pre-dock 곡선 후 직선). 변경 Python 4개 `ament_flake8` 통과.
- 파이: 기존 두 파일을 `~/jdamr_data/deploy_backup_20261002_000927/`에 백업하고 `box_service.py`·`restaurant_service.py`만 반영, `colcon build --packages-select jdamr_cube_navigation` 통과. 설치본 SHA-256이 로컬과 같다. Nav2 세션은 설정 변화가 없어 그대로 두고 `jdamr-box-executor`만 00:10:43에 재시작했다(설치 00:09:30 이후).
- 파이 소스의 `service_visualization.py`는 로컬과 다르지만 이번 변경과 무관해 반영하지 않았다.
- PC 기록 `bag_20261001_160232`를 00:11에 닫고(906 MB) 보관본에 맞췄다(해시 255건 확인). 다음 주행용으로 PC 표시 유닛 다섯 개와 기록 `bag_20261002_001136`을 다시 띄워 두었다. 유휴 중에도 bag이 커지므로(16:02–00:11에 906 MB) 주행하지 않으면 `python3 $HOME/jdamr_data/map_20260930_manual/tools/jdamr_depart.py display-stop`으로 끈다.
- 실차 미확인: pre-dock 곡선, 직선 진입 뒤 도크 방향 오차, 빠져나와 다시 진입하는 경로, 정차 2 s, 연속 주행 대기 제거.

### 20.18 돌발상황 대처 로직 (2026-10-02 00:12–00:46, 이동 없음)

현업 조사(표준·VDA 5050·상용 서빙로봇·Nav2 기본값)와 기존 처리 전수 조사를 대조해 우선순위를 정하고, 위에서부터 구현했다. 조사·우선순위 표·구현 내용·남은 공백은 [20261002_ANOMALY_HANDLING_RESEARCH.md](20261002_ANOMALY_HANDLING_RESEARCH.md)에 있다.

- 구현:
  - 드라이버 래치 비상정지(`/emergency_stop`, `emergency_stop_reset`, `/emergency_stop_state`)
  - 실행기 가드 연동과 취소 미확인 시 래치
  - 막힘 20 s 대기 후 재시도 3회
  - 등급별 `operator_call`과 PC 알림
  - 운영자 연결 하트비트 30 s
  - 정차 사이 저전압 도크 복귀
  - PC ssh 끊김을 "알 수 없음"으로 처리
  - PC `estop`·`estop-reset`
- 파이 반영:
  - 백업 `~/jdamr_data/deploy_backup_20261002_003758/`
  - 드라이버·실행기 빌드, `recover`로 재시작과 초기화(AMCL과 스캔 정합 0.006 m, DDS OK)
  - 드라이버는 8월 25일 빌드였어서 `geometry.hpp` 범위 검사도 함께 반영됨
- 정지 상태 확인: `estop` → engaged, `estop-reset` → released. 드라이버 로그도 둘 다 확인했다.
- init에서 table_02 영역에 박스 크기 LiDAR 클러스터가 잡히지 않았다(표지 위치 사용). 19:06 init에서는 잡혔다. 박스가 넘어졌거나 옮겨졌을 수 있다(현장 미확인).
- 실차 미확인: 주행 중 비상정지, 막힘 대기·재시도, 하트비트 끊김 정지, 저전압 복귀, PC 알림 표시.
- 켜 둔 것 (다음 주행 준비):
  - 파이: Nav2 세션, `jdamr-box-executor`
  - PC: 표시 유닛 다섯 개와 기록 `bag_20261002_004248`. `recover`가 앞의 `bag_20261002_001136`을 닫았다.
  - 주행하지 않으면 `python3 $HOME/jdamr_data/map_20260930_manual/tools/jdamr_depart.py display-stop`, 이어서 `session-stop`으로 끈다.

### 20.19 수정본 첫 실차와 도크 진입 앞뒤 반복 (2026-10-02 08:43–09:01)

08:43에 table_02를 한 번, 08:47에 table_02 → table_01을 연속으로 주행했다. 세 사이클 모두 주차·정차(2 s)·이탈·도킹을 확인했다. 이상 이벤트(막힘, 호출, 하트비트, 비상정지)는 없었다.

| run | 사이클 s | 대기점 옆 어긋남 | 도크 앞 방향 → 회전 | 도크 끝 방향 | 도크 확인 |
|---|---|---|---|---|---|
| table_02 08:43 | **164.4** | 3.3 cm | 2.79° (회전 없음) | −1.71° | 1.48 cm·1.71° |
| table_02 08:47 | 191.3 | 10.8 cm | 15.67° → 회전 | 3.97° → 빠져나와 −4.0° → 회전 → **−0.84°** | 1.48 cm·0.84° |
| table_01 08:51 | **162.0** | 4.3 cm | −9.54° → 회전 | −2.92° | 1.45 cm·2.92° |

- 연속 주행의 사이클 사이 공백은 50 s였다(도크 대기 없이 init 38 s, 출발 준비). 19:02 연속 주행에서는 61 s였다.
- 08:47 도크에서 로봇이 앞뒤로 움직였다(사용자 관찰). 도크 밖 회전이 3° 판정(alignment_goal_checker)에 들자마자 멈췄고, 0.25 m 직선 후진이 약 1°를 더해 도크 끝에서 3°를 넘었다. 그래서 빠져나와 다시 들어갔다(도크 구간 26.4 s, 08:43은 15.0 s). 08:51도 −2.92°로 기준을 겨우 넘지 않았다.
- 조치(사용자 지적: 그 자리에서 좌우로 돌아 맞추면 된다):
  1. 도크 밖 회전을 1.5° 판정 `entry_heading_checker`로 바꿨다. 10 Hz·0.2 rad/s에서 한 주기에 1.15°씩 돌기 때문에 1.5°를 골랐다.
  2. 도크 끝 방향이 3°를 넘고 5° 이하면 그 자리에서 회전해 맞춘다. 5°에서 뒤 모서리(회전 중심에서 0.414 m)가 약 3.6 cm 쓸린다. 5°를 넘을 때만 빠져나와 다시 들어간다.
  3. 실행기는 출발 전에 실제 컨트롤러의 `entry_heading_checker` 값까지 확인한다.
- Lattice 후진: Lattice는 대기점까지만 쓰고, 설정도 `allow_reverse_expansion: false`다. 도크 진입은 별도 후진 경로를 RPP 후진 컨트롤러가 따라간다. 끝 방향 오차는 컨트롤러 추종에서 생기므로, 계획기를 바꿔도 남는다고 본다(추론).
- 대기점 옆 어긋남(최대 10.8 cm)은 대기점 위치 판정 15 cm에서 나온다. 기록만 하고 바꾸지 않았다.
- 검증: 관련 6개 테스트 파일 514건 통과(도크 3종: 2.0°는 그대로, 4.4°는 도크에서 회전, 7.0°는 재진입 / 7.0° 지속은 실패 / 판정기 누락 시 출발 전 실패), `ament_flake8` 통과.
- 파이 반영: 백업 `~/jdamr_data/deploy_backup_20261002_085731/`. `recover`로 세션을 다시 올리고 초기화했다(AMCL과 스캔 정합 0.005 m). 실제 컨트롤러에서 `entry_heading_checker.yaw_goal_tolerance` = 0.02618 rad(1.5°)를 확인했다.
- 실차 미확인: 1.5° 도크 밖 회전 뒤 도크 끝 방향, 도크 안 작은 회전.
- 켜 둔 것: 파이 Nav2 세션과 `jdamr-box-executor`, PC 표시 유닛과 기록 `bag_20261002_085904`.

### 20.20 장애물 시험: 책상 다리 옆에서 움직이지 못함 (2026-10-02 10:05–10:33)

물 받는 곳으로 가는 통로에서 장애물 시험을 시작하자마자 정지했다. run `table_02_20261002_100530`.

**결과**
- 10:05:57 Collision Monitor가 "StopZone 때문에 정지"를 남겼다. 이어서 Nav2 105가 4번 나왔다(대기 20 s × 3).
- 10:08:23 `path_blocked_give_up` → `operator_call` CRITICAL `path_blocked`가 나오고, PC에 `OPERATOR CALL` 알림이 떴다. 막힘 처리와 운영자 호출의 첫 실차 확인이다.

**원인 (onboard bag과 차체 제원 대조)**
- 정지 시점 로봇은 지도 (0.313, −0.701), −74°였다. 지도에 있는 책상 다리가 차체 기준 x −0.11~−0.15 m, 옆 y −0.286~−0.298 m에 있었다(지도 점유 셀과 10 cm 안에서 일치). 왼쪽은 0.48 m까지 비어 있었다(지도에 없는 물체).
- 차체 외곽과 정지 구역(전진·후진·정지)은 바퀴 폭 + 2 cm(±0.29 m) 직사각형을 차체 길이 전체에 적용하고 있었다. 실제로는 바퀴(바깥 폭 0.54 m)가 축 앞뒤 0.033 m에만 있고, 나머지는 폭 0.45 m 프레임이다.
- 다리 위치에서 프레임까지 실제 여유는 약 6.5 cm였다(사용자 확인: 5 cm 이상, 회전·전진·후진 모두 가능). 그래도 다리가 세 정지 구역 모두에 들었고, 회전 구역(반지름 0.43 m 원)에도 들어 어느 방향으로도 움직일 수 없었다. 같은 경로 재시도로는 풀리지 않는다.
- 경로가 다리 쪽으로 붙은 이유: 같은 출발점에서 계획하면 이 경로는 다리에서 0.81–0.82 m 떨어진다(10:33 탐침, `route_clearance_probe_20261002_1033.jsonl`). 시험용 장애물로 평소 경로가 막혀 다리 옆 틈으로 다시 계획된 것으로 본다(추론). NavFn은 내접 반경 0.085 m만 보므로 이런 틈을 지나갈 수 있다고 판단한다.

**조치**
1. 코스트맵 차체 외곽과 정지 구역(전진·후진·정지)을 실측 계단형 다각형으로 바꿨다. 프레임은 ±0.245 m(0.225 + 0.02), 바퀴 축 ±0.053 m 구간만 ±0.29 m다. 회전 구역은 그대로 둔다. 충돌 예측(FootprintApproach)도 코스트맵 외곽을 쓰므로 함께 바뀐다.
2. `new_base_contract.py`와 `docking_stop_profile.py`가 축 대칭 직각 다각형을 받게 했다. 대신 외곽이 실측 프레임·바퀴 꼭짓점을 모두 감싸는지, 정지 구역 세 개가 외곽을 감싸는지 새로 검증한다.
3. 막힘 처리를 개선했다.
   - 컨트롤러 정지(104·105·106)이고 차체 앞 끝에서 0.30 m 안에 물체가 보이면, 기다리기 전에 0.10 m 직선 후진한다. 후진 경로는 지도·코스트맵으로 검증한다.
   - 옆에 있는 물체로는 후진하지 않는다. 이번처럼 다리가 축 뒤쪽 옆에 있으면 후진이 바퀴를 다리 쪽으로 보내기 때문이다.
   - 경로가 없는 정지(205·206·208·305·306·308)는 2 s마다 다시 계획해 보고, 경로가 생기면 20 s를 채우지 않고 바로 재시도한다.
4. 계획 비용 기울기(`cost_scaling_factor`) 3.0/2.0/1.5 비교: 다리까지 거리가 0.822/0.812/0.807 m로 차이가 없어 바꾸지 않았다. 좁은 틈을 계획 단계에서 거르는 일(차체 폭 기준 계획)은 남은 과제다.

**확인과 반영**
- 테스트: 관련 9개 파일 689건 통과. 신규로 정지 구역이 다리 위치를 제외하는지, 프레임보다 좁은 외곽을 거부하는지, 앞쪽 띠 판정, 경로가 생기면 대기 조기 종료, 컨트롤러 정지 때 후진 호출을 확인한다.
- 패키지 전체 테스트의 나머지 실패는 이번 변경과 무관한 기존 실패다. frontier·G005 자산, G006 번들(HEAD에서도 2건 실패), 다른 파일의 flake8·pep257이 해당한다.
- 파이: 백업 `~/jdamr_data/deploy_backup_20261002_102755/`. 설치본 해시가 로컬과 일치한다. `recover`로 재시작·초기화했다(AMCL과 스캔 정합 0.006 m). 실제 `collision_monitor`의 `StopZone.stopped.points`와 `local_costmap` `footprint`가 계단형으로 바뀐 것을 읽어 확인했다(정밀 세션 앞 끝 0.115 m).
- RViz가 비어 있던 이유: PC 표시 중계(`jdamr-p2-relay`)가 09:08:33 "jdamr.local not responding"으로 종료되고 다시 켜지지 않았다. 10:10에 `display-start`로 다시 띄웠다. 09:08–10:10 PC bag에는 로봇 데이터가 없다(파이 onboard bag에는 있음). 중계 자동 재시작은 남은 과제다.
- 실차 미확인: 계단형 정지 구역에서 책상 다리 옆 통과, 앞이 막혔을 때 후진, 경로가 생기면 조기 재출발.

### 20.21 장애물 시험 2회차와 후속 수정 (2026-10-02 10:36–10:58)

run `table_02_20261002_103556`(물 받는 곳·table_02 주차 완료, 도크 복귀 단계에서 정지), 이어서 `--dock-only` 두 번(`104333`, `104455`).

**사용자 관찰과 기록 대조**

| 관찰 | 기록 |
|---|---|
| 막았을 때 멈추고 알림 | 10:36:28–10:36:50 Collision Monitor가 접근 감속과 StopZone 정지를 0.1–0.3 s 간격으로 번갈아 남겼다(장애물이 정지 구역 경계에 걸려 깜빡임). 105 → 대기 20 s. 10:39:25와 10:44:08에 운영자 호출 → PC 알림 |
| 치웠을 때 다시 출발 | 10:37:18 재시도 → 10:37:25 관측 지점 도착. 다만 치운 뒤에도 20 s를 다 기다렸다 |
| 앞을 지나갈 때 감속 | 10:38:04–10:38:13 짧은 정지 5번(각 0.04–0.4 s) 뒤 진행, 10:38:17 table_02 관측 지점 도착 |

**드러난 문제와 조치**
1. 대기점 구간: Lattice가 table_02 옆 (0.47, −1.42)에서 "no valid path found"를 반복했다. 시험용 장애물 때문에 통로가 차체보다 좁아졌다(사용자 확인). Lattice의 판단은 맞았다.
   - 그런데 경로 재확인은 NavFn으로 해서 매번 "열림"이 나왔다. 그래서 대기 없이 재시도 4회를 2–3 s 만에 써 버렸다.
   - 조치(`6e6ba4d`): 대기점 BT를 Lattice → NavFn Fallback으로 바꿨다. 이제 대기점 208은 NavFn도 실패할 때만 나와서 재확인과 판단이 같다.
2. 105(컨트롤러 정지) 대기는 막은 물체가 사라져도 20 s를 채웠다.
   - 조치: 2 s마다 차체 앞 0.30 m 띠를 최신 스캔으로 보고, 두 번 연속 비면 바로 재시도한다(`PATH_BLOCKED_CLEAR_PROBES`).
   - 옆에 걸린 물체로 막힌 경우는 앞이 비어 있으니 약 4 s 만에 재시도한다. 계속 막히면 운영자 호출까지 걸리는 시간이 짧아진다.
3. 두 번째 `--dock-only`가 출발 5 s 만에 `request interrupted, failed or timed out`로 끝났다(실행기의 ROS 요청 시간 초과). 원인은 확정하지 못했다.
   - 이어진 `recover`에서는 Nav2 세션 기동이 `lifecycle_manager_keepout`의 `get_state` 시간 초과로 중단됐다. 이때 파이 부하 평균은 5.7이었고, 09:01부터 떠 있던 ros2 데몬(제가 `ros2 param`으로 띄움)이 같은 도메인에 있었다.
   - 데몬을 멈추고 세션만 다시 띄우니 정상 기동했다.
   - 조치: PC 도구의 `estop`·`estop-reset`·`status`가 파이에서 ros2 CLI를 쓴 직후 데몬을 정리한다.
4. 사용자가 로봇을 옮긴 자리에서는 시작 위치 초기화가 일치율 0.36, 전역 불일치로 멈췄다(안전 동작). 사용자가 도크로 옮긴 뒤 초기화했다(10:58, AMCL과 스캔 정합 0.003 m).
5. PC 표시 중계에 `Restart=on-failure`(5 s)를 붙였다(09:08 중계 종료 재발 방지).

**확인**
- 테스트 425건(실행기)과 38건(PC 도구) 통과. 신규: 앞쪽 띠가 두 번 연속 비면 105 대기 조기 종료. `ament_flake8` 통과.
- 파이: 백업 `~/jdamr_data/deploy_backup_20261002_105639/`(실행기 2개)와 `..._1044xx`(대기점 BT). 설치본 해시가 일치한다.
- 대기점 BT는 10:52 세션 재기동으로 반영됐다. 실행기는 10:57:05에 다시 띄웠다. 중계는 `Restart=on-failure`로 실행 중이다.
- 실차 미확인: 105 대기 조기 출발, 대기점 NavFn 대체 경로, 정지 구역 경계 깜빡임은 그대로(Collision Monitor에 유지 시간이 없음, 남은 과제).

### 20.22 이동 중 교착의 실제 경로와 갇힘 탈출 (2026-10-02 11:08–12:05)

run `table_02_20261002_110821`. 물 받는 곳으로 가는 이동 구간에서 멈췄고 운영자 호출로 끝났다.

**기록**
- 11:08:31 출발. 11:09:07 Collision Monitor StopZone 정지, 11:09:09 105.
- 105 → 대기 → 앞쪽 띠가 두 번 비어 약 13 s 뒤 재시도(§20.21 조기 재출발이 동작함). 같은 정지가 세 번 더 나왔고 11:10:41에 포기, CRITICAL 운영자 호출. 재시도마다 "approach 0.5 s" 직후 0.1 s 안에 StopZone 정지가 찍혔다.
- 정지 위치 map (0.413, −1.073), yaw −56.7°. 왼쪽 가장 가까운 점은 0.48 m였다.
- 사용자 관찰: 왼쪽에 공간이 충분했는데 회전하다 멈춘 것으로 보였다.

**처음 판단과 정정**
- 처음 판단(11:15): RPP가 45° 넘는 방향 차이에서 제자리 회전을 냈고, 오른쪽 뒤 다리가 회전 원(r 0.43) 안이라 막혔다고 봤다. 이 판단으로 전진 탈출(`6c7015a`)을 만들었다.
- bag으로 확인하니 틀렸다(`cmd_at_stops.py`, `stop_polygon.py`).
  - `/cmd_vel_nav`는 11:08:50–11:10:38 동안 v 0.12, w 최대 +0.32인 좌회전 호였다. 제자리 회전 명령은 한 건도 없었다.
  - 정지 순간 CM 입력(`/cmd_vel_smoothed`)도 여섯 번 모두 (0.12, 0.30)이었다. 따라서 전진 다각형이 선택됐다.
  - 그 다각형 안에 점이 5–7개 있었다: x −0.257~−0.275, y −0.238~−0.245. 패딩 외곽(y −0.245) 안쪽 0–7 mm이고, 측정 프레임 옆면(±0.225)에서는 1.3–2.0 cm다. 사용자가 본 여유(5 cm 이상)와 다르며, LiDAR 오차는 따로 확인하지 않았다.
- 경로:
  1. 좌회전 호에서 오른쪽 뒤 모서리가 오른쪽으로 쓸리며 다리에 다가갔다.
  2. FootprintApproach가 감속했지만 다리 점이 패딩 안으로 들어왔고, StopZone이 정지시켰다.
  3. 그 뒤로는 정지·전진·후진·회전 다각형이 모두 외곽 전체를 담는다. FootprintApproach는 외곽 안 점을 t=0 충돌로 계산해 모든 속도를 0으로 만든다. 그래서 어느 방향으로도 움직일 수 없었다.
  4. §20.20(10:06)과 같은 종류다. 위치만 바퀴 옆에서 뒤 모서리로 바뀌었다.
- `6c7015a` 전진 탈출도 이 상태에서는 전진 다각형에 막힌다. 처음 재계산(`escape_replay.py`)은 회전 원과 앞쪽 띠만 보고 CM 다각형을 빠뜨렸다.
- "회전할 때는 Lattice를 쓰지 않나"에 대한 답은 다음과 같다.
  - Lattice는 경로 모양을 정하는 전역 계획기이고, 이 이동 구간은 NavFn이 계획한다.
  - 회전과 호를 실제로 내는 것은 컨트롤러 RPP다. RPP의 예측 충돌 검사는 2026-10-01 운영자 요청으로 꺼져 있다(실제 10 cm 여유 회전을 멈췄기 때문).
  - 따라서 주행 중 차체 외곽을 보는 것은 CM뿐이다.

**수정**
1. `6c7015a`: 축 뒤 회전 원(0.45 m) 안에 반사점이 있으면, 그 점이 원 밖으로 나가는 거리 + 3 cm(0.10–0.35 m)만큼 직진 전진한다. 외곽 밖에서 회전 원에 막힌 경우에 쓴다.
2. `8e90746`: 갇힘 탈출.
   - StopZone에 순수 직진(|w| ≤ 0.005) 다각형 2개를 일반 이동 다각형 앞에 둔다. 진행 방향 쪽 외곽 절반만 담고(축에서 자름), 앞 끝은 일반 전진 다각형과 같다(정밀 세션 0.115).
   - 탈출은 `cmd_vel_nav`에 0.05 m/s, w = 0을 내고 odom으로 거리를 잰다. smoother와 CM은 그대로 거친다. 그동안만 FootprintApproach를 끄고 `finally`에서 다시 켜며, 다시 켜지지 않으면 래치 비상정지를 건다.
   - 외곽 앞 절반 침범이면 0.10 m 후진(뒤 0.15 m 띠가 비어야 함), 뒤 절반 침범이면 회전 원 탈출 거리만큼 전진, 양쪽이면 대기한다.
   - 계약은 직진 다각형이 축에서 끊기고 진행 쪽 외곽 절반을 담는지 검사한다. 도킹 프로파일은 직진 다각형의 앞 끝도 함께 옮긴다.
3. `be44451`, `87652f2`: MPPI 적재와 `--mppi-transit` A/B 스위치. 이동 구간과 대기점 구간만 MPPI로 바꾼다. 차체 외곽으로 호를 평가하므로 패딩 침범을 예방할 후보다.

**확인 (이동 없음)**
- bag 재계산(`escape_replay.py`): 여섯 정지 모두 앞쪽 띠가 비었다. 회전 원 탈출 거리는 0.177–0.191 m이고 전진 거리는 0.207–0.221 m다.
- PC 로컬 CM(도메인 77, 저장소 설정, `cm_escape_check/driver.py`)
  - 현재 설정: 패딩 안에 점이 있으면 FootprintApproach를 꺼도 직진 전진이 막힌다.
  - 새 설정: 직진 전진만 통과한다(0.032 m/s = 0.04 × 감속 0.8). w 0.1, 직진 후진, 제자리 회전, 앞쪽 띠 점은 정지한다.
  - 하위 다각형은 목록 순서상 첫 번째로 맞는 것이 선택되고, `FootprintApproach.enabled`는 실행 중에 바꿀 수 있다.
- PC 종단(`cm_escape_check/e2e.py`: velocity smoother + CM + 간이 odom 시뮬, 실행기 `_drive_straight` 그대로)
  - 11:09 호 명령 3 s: 이동 0.000 m.
  - 탈출: 0.225 m, 방향 변화 0.00°, 출력 w 최대 0. 이후 FootprintApproach가 다시 켜졌다.
  - 다리–축 거리 0.517 m.
- MPPI CPU(`mppi_bench/`): `/bench` 네임스페이스 controller_server로 쟀다. 출력 `/bench/cmd_vel` 구독자는 0이었고, 실제 이동 경로 2.79 m를 썼다.

  | 구간 | controller_server CPU (1코어 기준) | 시스템 전체 (4코어) | 주기 | 최대 간격 | 주기 미달 경고 |
  |---|---|---|---|---|---|
  | 목표 없음 | 7.9 % | 44.8 % | – | – | – |
  | RPP | 9.5 % | 44.6 % | 10.40 Hz | 0.108 s | 0 |
  | MPPI | 18.9 % | 47.8 % | 10.27 Hz | 0.106 s | 0 |

  첫 측정(11:33)은 timeout 래퍼 PID를 재서 CPU가 0.0으로 나와 무효다.
- 테스트: 패키지 2213건 통과(기존 frontier·G005·G006 30건 실패는 변경과 무관), PC 도구 39건 통과, `ament_flake8` 통과.
- 파이
  - 백업: `~/jdamr_data/deploy_backup_20261002_112729`, `_114215`, `_120057`. 설치본 해시가 로컬과 일치한다.
  - 12:04 `recover`: AMCL 대 스캔 0.005 m.
  - 살아 있는 CM 파라미터: 새 순서, 직진 앞 끝 0.115, FootprintApproach 켜짐.
  - MPPI 적재 로그: "Critic will collision check based on footprint cost".
- 실차 미확인: 갇힘 탈출, 전진 탈출, MPPI 주행.

**남은 것**
- 예방: RPP의 호가 뒤 모서리를 장애물 쪽으로 쓸어 넣는 것은 그대로다. MPPI A/B가 이 질문에 답한다.
- 탈출 뒤 RPP가 같은 호로 다시 들어갈 수 있다. 이 경우는 재시도 3회 한도 안에서 운영자 호출로 끝난다.
- 로봇이 움직이지 않았는데 도크의 스캔 정합 y가 11:31 0.104에서 12:04 0.149로 달라졌다. 원인은 확인하지 않았다.

### 20.23 RPP/MPPI A/B 1·2회차와 회전 정지 구역 완화 (2026-10-02 13:42–14:15)

계획은 table_02 코스(도크 → 물 받는 곳 → table_02 → 도크)를 RPP(A)와 `--mppi-transit`(B)로 번갈아 두 번씩 도는 것이었다. 2회차가 운영자 호출로 끝나서 3·4회차는 보류했다. 지표는 이동 세 구간(물 받는 곳, table_02, 도크 대기점)에서 뽑았다(`ab_metrics.py`, `ab_commands.py`, 파이 controller_server CPU 1 s 기록).

| | 1회차 RPP `table_02_20261002_134203` | 2회차 MPPI `table_02_20261002_134626` |
|---|---|---|
| 결과 | 한 사이클 완주 208.9 s | 물 받는 곳 이동 중 13:49:44 CRITICAL 운영자 호출 |
| 막힘 | 13:42:39–13:43:00 11:09와 같은 외곽 침범(최소 거리 0 m, CM 감속·정지 53.9 %) → 105 | 외곽 침범 없음(최소 7.3–10.9 cm). 105가 4번 |
| 탈출 | 13:43:01 직진 전진 0.26 m(`done`, FootprintApproach 복구) → 4 s 대기 → 13:43:37 도착. 첫 실차 확인 | 13:47:30 전진 0.19 m 성공. 그 뒤 두 번은 "뒤 막힘"으로 후진 거부 |
| 명령 | 첫 구간 중앙값 v 0.12, w 0.30 | 첫 구간 중앙값 v 0.015, 후진 66건·저속 146건. 재시도는 제자리 회전(w 0.02–0.07) 위주이고 CM이 87–92 % 차단 |
| controller CPU (1코어 기준) | 9.6–9.9 % | 20.5–41.1 %, 제어 주기 5.8–9.9 Hz(미달 경고 반복) |
| 시스템 CPU (4코어) | 75–77 % | 76–85 % |

**2회차 원인**
- 멈춘 자리 map (0.438, −0.695), −37.6°. 왼쪽 앞에 물체가 있었다: 축에서 0.29 m, 앞면에서 약 14 cm. 오른쪽 뒤에도 물체가 있었다: 축에서 0.45 m, 뒤 모서리 옆 5 mm 밖.
- MPPI는 차체 외곽(회전 반경 0.383 m) 기준으로 제자리 회전을 골랐다. CM 회전 원(0.43 m)은 왼쪽 앞 물체 때문에 그 회전을 막았다. 그런데 이 물체는 어느 방향 회전에도 닿지 않는 자리였다(계산: 반시계·시계 모두 외곽 밖).
- 후진 탈출은 외곽을 감싼 직사각형(±0.29)으로 뒤 띠를 봐서, 뒤 모서리 옆 5 mm 밖의 반사점을 "뒤 막힘"으로 판정했다. 실제 계단형 외곽으로 보면 직진 후진은 그 점에 닿지 않는다.
- MPPI 1000×40은 정지 벤치에서 10.27 Hz였지만, 주행 중에는 10 Hz를 지키지 못했다.

**결정과 조치** (`d12a000`)
1. 회전 정지 구역: 0.43 m 원 → 15°(운영자 선택) → "더 완화"(운영자) → 패딩 외곽을 회전 방향으로 6° 쓸린 영역으로 정했다(`evaluation/rotation_sweep_polygon.py`, 점 20개, 시계 방향은 거울상).
   - 근거: smoother 회전 상한 0.3 rad/s에서 스캔 0.1 s + 감속 1.5 rad/s² ≈ 5.2°. 10-01 Spin bag에서 0.2 rad/s일 때 정지 명령 뒤 1.3° 더 돌았다.
   - 계약 검사: 0.5° 간격 쓸림 포함, 거울 대칭, 변 교차 없음, smoother와 MPPI 회전 상한 ≤ 0.3 rad/s.
2. 탈출 띠: 실제 계단형 외곽을 진행 방향으로 쓸어 판정한다(`straight_sweep_hit`). 외곽 안 반사점은 진행 쪽 절반만 막힘으로 센다.
3. MPPI: 500×30, 이동 중 후진 금지(vx_min 0), wz_max 0.3.

**확인 (이동 없음)**
- 로컬 CM(도메인 77, 저장소 설정, 제자리 회전 ±0.3 rad/s, FootprintApproach 켬)

  | 장면 | 반시계 | 시계 |
  |---|---|---|
  | 13:48 물체 | 통과 0.24 | 통과 −0.24 |
  | 11:09 패딩 안 다리 | 정지 | 정지 |
  | 뒤 옆 2 cm 다리 | 정지 | 통과 −0.24 |

  첫 시도는 합성 스캔 0.5° 간격 때문에 점이 같은 빔에 겹쳐 3점 미만이 되어 무효였다. 점을 서로 다른 빔으로 다시 놓고 쟀다.
- 테스트: 패키지 2224건 통과(frontier·G005·G006 30건은 기존 실패), lint 통과.
- 파이: 백업 `~/jdamr_data/deploy_backup_20261002_141244`, 설치본 해시 일치, 실행기 14:13:11 재시작. Nav2 세션은 다시 띄우지 않았다(로봇이 도크 밖에 끼어 있음). 새 CM·MPPI 설정은 다음 `recover`부터 적용된다.
- MPPI 500×30 정지 벤치(14:13, 끼인 자리, 장애물 근처): 20.7 %(1코어 기준), 10.27 Hz, 주기 미달 0. 1000×40 벤치(11:36, 도크)와 장소가 달라 직접 비교할 수 없다.

**남은 것**
- 재측정 전에 로봇을 도크로 옮기고 `recover`로 세션을 다시 띄워야 한다.
- 이어서 A/B를 다시 돈다. 확인할 것:
  - RPP의 외곽 침범 → 탈출 반복 여부
  - MPPI의 주행 중 제어 주기
  - 회전 완화 뒤 MPPI 통과 여부

### 20.24 재측정 중 접촉·갇힘과 여유 축소 (2026-10-02 17:34–18:51)

**경과**

| 시각 | run | 결과 |
|---|---|---|
| 17:34 | – | Wi-Fi 끊김(파이는 계속 켜져 있었음) 복구 뒤 `recover`. 새 회전 정지 구역·MPPI 500×30이 살아 있는 세션에 올라간 것을 확인 |
| 17:45 | `table_02_20261002_174547` (RPP) | 물 받는 곳에서 후진 0.10 m → 전진 탈출 0.211 m로 통과, 두 곳 주차. 도크 대기점 구간에서 Lattice "경로 없음" → NavFn 경로(0.64 m 틈 오른쪽 물체에 차체가 1–2 cm 걸림)를 RPP가 4번 따라가다 운영자 호출 |
| 18:01 | `table_02_20261002_180130` (복귀) | 전진 탈출(FootprintApproach 끔) 중 물체를 밀었다(사용자 확인). 스캔상 프레임 옆면 약 1 cm, 직진(odom −0.15°)이었다 → 비상정지 |
| 18:05 | `…180520` (복귀) | 도킹(내부 추정 1.28 cm·1.82°) |
| 18:11 | 2회차 MPPI | 출발 전 `/cmd_vel` 발행자 없음으로 거부: collision_monitor 프로세스는 살아 있었는데 DDS 그래프에서 사라짐(18:11:27). `recover`로 복구 |
| 18:18 | `table_02_20261002_181824` (MPPI) | MPPI가 v 0.000, w 0.017 rad/s로 기었다(CM 입력=출력, 막지 않음). 물체 18 cm 칸 비용 ~190 때문으로 판단 → 실행 중 local inflation 감쇠 3→10, 외곽 검사 간격 2→4 |
| 18:36 | `…183613` (복귀, 새 설정) | 막힘 없이 47 s 도킹(1.47 cm·2.66°) |
| 18:40 | `table_02_20261002_184010` (RPP, 새 설정) | 물 받는 곳 구간에서 외곽(5 mm) 안 접촉으로 4번 대기 후 운영자 호출 |
| 18:48, 18:50 | 복귀 | 배터리 10.52 V → 출발 예비 10.8 V에 걸림(복귀도) → 수정 뒤 10.24 V로 주행 하한 10.5 V 미달. 충전 필요 |

**조치 (운영자: "부딪혀도 되니까", "1 cm 정밀 주차", "들어갔으면 나올 수도", "호출 알림 보내지 말라")**
- `d1b7e25` 컨트롤러 정지 뒤 재시도는 MPPI 트리로, 후진 탈출은 대기마다 0.10→0.20→0.30 m.
- `eb0ece5` 패딩 안 물체 옆 직진 탈출 금지(18:02 접촉). 이어서 `819ffaf`에서 대체했다.
- `d623b3f` 외곽 패딩 2 cm → 5 mm(StopZone 전부·회전 쓸림 재생성, 앞 정지 여유 5 cm 유지), local inflation 감쇠 10, MPPI CostCritic 가중 1.5, 외곽 검사 간격 4.
- `819ffaf` odom 궤적(2 cm마다, 4 m)을 남겨, 접촉이거나 뒤로 똑바로 못 빼면 그 궤적을 ParkingReverse로 0.30–0.60 m 되짚고 MPPI로 다시 들어간다. 되짚는 동안만 Collision Monitor `~/toggle`을 끈다(로컬 CM으로 끄면 그대로 통과·켜면 복귀 확인, 못 켜면 비상정지). PC `alerts off|on`(지금 off), `init --min-inlier`(국소 초기화, 하한 0.25).
- `1a2171f` `--home-only` 복귀는 주행 하한 전압만 본다.

**확인과 남은 것**
- 관련 테스트 459건(실행기)·40건(PC 도구), 패키지 전체 2224건(기존 frontier·G005·G006 30건 제외) 통과. 파이 설치본 해시 일치.
- 실차 미확인: 되짚기 탈출, 새 설정의 MPPI 주행.
- 위치추정: 혼잡 구역에서 스캔 정합 점수가 평평해 ±5–10 cm를 가려내지 못했다(`amcl_vs_scanmatch.py`). 도크에서는 AMCL 도킹 위치와 스캔 정합이 두 번 약 9–11 cm 달랐다(18:16, 18:38). 원인 미확정.
- 다음: 충전(10.8 V 이상) 뒤 도크에서 `recover` → A/B(RPP·MPPI 번갈아).

## 앞선 충전 중 수정본 검증

- 로컬: box service, restaurant service, relay, new-base 설정, keepout, reverse parking, parking contract/integration, depth target, LiDAR witness, session, stop profile 관련 570개 테스트 통과.
- 변경 Python 9개 파일 ament_flake8 통과, `git diff --check` 통과.
- 별도 code-reviewer: 차단 결함 수정 후 코드 finding 0, 관련 358개 테스트 통과. LSP 도구 부재로 형식상 판정은 COMMENT이며 실차 승인으로 해석하지 않는다.
- PC와 파이에서 navigation 패키지 빌드 통과. 파이 `box_service --help`에 탐색·관측 위치 재개 옵션이 표시되고 소스 3개 파일 SHA-256이 로컬과 일치한다. 주행 서비스는 inactive 상태를 유지했다.
- 파이 반영 전 백업: `$HOME/jdamr_data/charging_code_backup_20260929_3fDZK3/navigation_before_fix.tar.gz`. 파이 배치 경로는 Git 저장소가 아니므로 파일별 비교·백업 후 여섯 실행 파일만 반영했다. 기존 실행 중 표시 중계는 재시작하지 않았으므로 부하 완화 변경은 다음 중계 시작부터 적용된다.
- 실차 5cm 거리·박스 면 평행 정렬·복귀, 장시간 센서 지연 재발 여부는 미검증이다.

## 보존 데이터

- 데이터 루트: `$HOME/jdamr_data/parking_attempt_20260929_preserved/`
- 최초 주행: `table_departure_20260929_latest_bag/`
- MCAP SHA-256: `d40f57c88818b68f326616aaf8b5d0c49f4bdf8dc431dd532cc6cf11a5e21d3e` (PC와 파이 일치)
- 실행 이벤트: `table_departure_20260929_zkmrDR`, `table_face_resume_20260929_lsuar_qj`, `table_park_resume_20260929__9ovfvaf`, `table_near_face_20260929_qrq4gvlf`
- 충전 중 추가 보존: `$HOME/jdamr_data/parking_charging_fix_20260929_hwT9Pe/`의 탐색·회피·재접근 JSONL 다섯 run과 `nav2_latency_journal.log`. 이 후속 구간의 원시 RGB-D rosbag은 확보하지 못했다.
- 정지 기준선: 같은 폴더의 `stationary_sensor_baseline.json` (파이 원본은 위 백업 폴더). 지도 위치추정은 종료 상태라 map→odom 자료는 없으며 센서/베이스 기준선만 제공한다.

기록과 코드 검증은 주행 성공 증거가 아니다. 테이블 앞 5cm 정밀주차는 아직 성공하지 않았다. 물 받는 곳 앞 정밀 주차는 2026-09-30 15:46에 내부 추정 0.65 cm·1.83°로 도달했다(외부 실측 없음, 당시 방향 허용 1°라 실행기 판정은 미확인, §19). 충전소 후진 도킹은 2026-09-30 12:01에 도크 정위치에 도달했다(사용자 현장 확인, 내부 추정 TF 0.25 cm·3.8°, 스캔 정합 3.9 cm·1.2°, 외부 실측 없음). 끝 구간 yaw 보정 중 Nav2 진행 검사 중단(105)으로 서비스 판정은 실패였다(§16).

## 21. 출발 지연·RViz 정적 TF 누락과 근접 주차 실패 — 수정 보류 (2026-10-16)

사용자 요청: "기록해둬 나중에 고치게". 초기 진단에서는 이동 목표를 보내지 않았고, 이후 출발 요청으로 실행한 주차 결과를 §21.6에 추가했다. §21.1–21.7에서는 제품 코드와 주행 기준을 바꾸지 않았고, 이후 수정 요청에 따른 변경과 실행은 §21.8에 기록했다. 새 카메라 높이에서 1 cm 주차 성공을 주장하지 않는다. 앞 절의 날짜별 과거 결과는 그대로 보존한다.

### 21.1 실행 조건과 보존 근거

- PC는 `aicampus_286`, Pi는 `lim@192.168.0.159`에서 접속됐다. USB를 여는 중 Astra 드라이버가 `-11`로 종료됐고 카메라 포트 소프트웨어 복구 1회도 실패했다. 사용자 USB 재연결 뒤 뎁스 `320×240`, 약 30 fps 수신이 회복됐다. 이것만으로 케이블·전원·SDK 중 물리 원인을 확정하지 않는다.
- 사용자가 책상 앞에 배치한 뒤 뎁스 10장 모두에서 `table_edge`가 검출됐다. 마지막 상태는 `stable=true`, `stable_frame_count=8`, 폭 약 1.006 m, 경계 RMS 약 5.15 mm였다. 이 값은 관측 결과이며 최종 주차 실측이 아니다.
- 현재 실행 근거: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/precision_1cm_20261016_092352/`. `scan.json`, `global_pose.json`, `result.json`, `rviz_initial_pose.yaml`, `rviz_activation.jsonl`, `rviz_localization_diagnostic.jsonl`, `rviz_pc_tf_diagnostic.json`, `rviz_amcl_initialized_static_tf_missing.png`, `runtime_units_after_record_request.json`을 보존했다. TF JSON은 완료된 진단 도구 출력에서 옮긴 요약이며 원시 ROS bag으로 취급하지 않는다.
- 비교 근거: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/init_20261013_110701/scan.json`과 `refine.json`.

### 21.2 스캔 정합 비율과 위치초기화 안내 정정

| 항목 | 이전 초기화 | 이번 출발 전 관측 |
|---|---|---|
| 지도 점유점에서 5 cm 안에 든 스캔 점 비율 | 33.55% | 26.06% |
| 실행에 사용한 초기화 기준 | `--local-only --min-inlier 0.30` | 기본 `MATCH_MIN_INLIER=0.45` |
| 같은 처리기로 남은 스캔 점 수 | 155 | 165 |
| 라이다 좌표·방향 | `(-0.01, 0, 0.15) m`, yaw 180° | 동일 |
| 스캔 frame·각도 간격·최소 거리 | `laser_link`, 약 -0.00676339 rad, 0.28 m | 동일 |

PC·Pi의 지도 YAML과 이미지 SHA-256은 각각 `3f35f4528f68c26344c8ca1f7547c0fb238cc60cb97006ed8a7c44f12e8ea7bd`, `f97acff6cbb5d3cc449b44dcb98b75b7f3dac089bf268c4f0d7f975d0628c226`으로 일치했다. 저장 지도 원점은 `(-3.7, -5.55, 0)`, 해상도는 0.05 m/cell이다.

이전 운영 옵션을 재사용하지 않고 기본 기준으로 진입한 절차 차이가 있었다. 다만 이번 비율은 기존 30%에도 못 미치므로 그 차이만으로 전체 원인을 설명하지 않는다. 비율이 낮아진 물리적 이유는 미확정이다. 이 수치는 위치 정확도나 위치추정 성공 확률이 아니므로, 26%만으로 위치초기화가 불가능하거나 지도 없는 새 주차 구현이 필수라고 안내했던 판단을 정정한다.

RViz 확인을 위해 계산된 위치 후보 `(0.555, -0.250, 3.211406 rad)`를 기존 비이동 초기화에 넣었다. 이후 `navigation_activated_without_motion` 이벤트가 기록됐으며 AMCL은 `(0.558395, -0.254856, -3.079350 rad)`였다. 후보와의 차이는 약 5.9 mm·0.43°다. 정지 확인은 약 1.02 s, AMCL XY covariance는 약 `(0.002319, 0.002426) m²`, yaw covariance는 약 `0.006014 rad²`였다. 좁은 초기 후보를 넣은 후의 내부 일치이며 실제 위치를 외부에서 측정한 정확도가 아니다. 이 진단으로 PC의 `state.json`을 출발 준비 완료로 덮어쓰지 않았고 `localized=false`를 유지했다.

### 21.3 RViz에서 스캔·모델이 보이지 않는 정적 TF 문제

PC 표시 그래프에서 수행한 TF2 조회 결과:

- `/scan` 수신 정상.
- `map→odom→base_footprint`는 최신 시점과 스캔 stamp 모두에서 조회 성공.
- `map→base_link`는 최신·스캔 stamp 모두에서 "two or more unconnected trees" 오류.
- `laser_link`는 TF 버퍼에 존재하지 않아 조회 실패.
- `base_link→left_wheel_link/right_wheel_link`만 별도 트리에 남았다. 카메라 광학 프레임 연결은 수신됐지만 차체·라이다까지 연결하는 정적 프레임이 PC 진단에 없었다.
- RViz 로그는 `laser_link` 메시지를 "queue is full"로 버렸고, 초기화 후에도 스캔·로봇 모델 표시 오류가 남았다. 이 큐 메시지만으로 USB·CPU·라이다 고장을 단정하지 않는다.

**확인된 범위:** AMCL의 `map→odom`이 생긴 뒤에도 PC의 정적 TF 일부가 누락됐다. **미확정:** Pi 원본 정적 TF, export 수신, 직렬화·집계, import 재발행, late subscriber QoS 중 어느 경계에서 누락되는지 아직 분리하지 않았다. 표시용 가짜 TF는 만들지 않았다.

**나중에 할 일:** Pi 원본과 PC의 부모·자식 프레임 목록을 비교하고 `/tf_static`의 여러 발행자·늦은 구독자·재접속 조건에서 각 중계 경계의 보존 여부를 확인한다. 누락 경계를 고친 뒤 같은 RViz 창에서 실제 `/scan`과 모델의 표시 회복을 확인한다. 화면 상태를 새로운 출발 승인 조건으로 추가하지 않는다.

### 21.4 세션 재기동과 초기화 순서 오류

1. 기존 `session-stop` 뒤 `jdamr-restaurant-navigation`은 `inactive/dead`, `Transient=yes`, `Result=success`였지만 `/run/systemd/transient/jdamr-restaurant-navigation.service`가 로드된 상태로 남았다. 다음 `systemd-run`이 "already loaded or has a fragment file"로 거부돼 `session-start`가 code 3으로 끝났다. `reset-failed`만으로는 이 상태가 해소되지 않았다. 같은 registry·params·LOCALHOST·precision·prepare-only 인자를 확인한 뒤 기존 유닛을 `systemctl start`로 시작하는 것은 성공했다. 유닛이 유지된 정확한 참조 원인은 미확정이다.
2. 진단 과정에서 `systemctl active` 직후 초기화를 호출했을 때 `map_server lifecycle state unconfirmed`가 발생했다. 그 뒤 journal에서 `map_server/amcl`과 keepout의 "Managed nodes are active"를 확인한 다음 기존 초기화를 적용한 시도는 성공했다. 프로세스 active와 lifecycle 준비 완료를 같은 상태로 취급한 호출 순서 문제를 기록한다.

**나중에 할 일:** 세션 식별자가 같은 inactive transient 유닛은 기존 유닛 재사용 또는 정확한 범위의 정리 후 생성으로 수렴하게 한다. 다른 유닛·세션을 일괄 삭제하지 않는다. 준비 완료 확인을 기존 session-start 경로에서 일관되게 처리하고, 진단·운영 경로가 서로 다른 초기화 상태를 남기는지 점검한다. 기본 기준과 명시된 기존 운영 옵션의 적용도 로그에 남겨 반복 출발 때 숨은 절차 차이가 생기지 않게 한다. 수치 기준을 낮추는 것만으로 해결됐다고 처리하지 않는다.

### 21.5 남겨 둔 서비스와 종료 방법

사용자가 요청한 RViz·뎁스·대시보드 화면을 유지하기 위해 PC의 `jdamr-p2-mapserver/relay/markers/rviz/click`, `robot-dashboard`, `robot-dashboard-link`와 Pi의 Nav2 세션·뎁스·관측기·대시보드를 유지했다. 초기 진단 종료 시점의 상태와 PID는 위 run의 `runtime_units_after_record_request.json`에 기록했다. 진단 노드는 종료됐다. 이 스냅샷 이후 §21.6의 주차 요청으로 Pi의 `jdamr-box-executor`도 활성화됐다. 해당 run은 실패 후 정지 상태이며 자동 도크 복귀는 요청하지 않았다. 요청한 화면을 유지하기 위해 표시·센서·주행 세션은 켜 두었고 도크 등록은 바꾸지 않았다.

PC에서 표시와 주행 세션을 종료할 때:

```bash
python3 "$HOME/bimanual-robot/tools/jdamr/jdamr_depart.py" display-stop
python3 "$HOME/bimanual-robot/tools/jdamr/jdamr_depart.py" session-stop
systemctl --user stop robot-dashboard robot-dashboard-link
```

Pi에서 카메라·관측기·대시보드도 종료할 때:

```bash
sudo systemctl stop jdamr-box-observer jdamr-box-rgbd jdamr-ammr-dashboard
```

### 21.6 후속 1 cm 주차 실행과 사용자 좌우 실측

초기 진단 뒤 사용자가 출발을 요청해 기존 `box_service --serve` 상주 실행기에 책상 주차를 제출했다. `--resume-at-observation --surface-mode table_edge --final-gap-m 0.01`을 사용했고 자동 도크 복귀는 요청하지 않았다. 별도 속도 발행 코드나 가짜 TF·위치값은 만들지 않았다.

- 첫 요청 `parking_20261016_094811_739acb9b`는 작성한 route에 waypoint가 하나뿐이어서 `route requires at least two waypoints`로 이동 없이 종료됐다. 운영 자료 작성 오류이며 제품 코드 오류로 귀속하지 않는다. 같은 현재 위치에 시작·관측 waypoint를 분리한 자료로 다음 요청을 제출했다.
- 실제 이동 run: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/precision_1cm_20261016_092352/parking_20261016_094932_b52bc7ba/`. `command.json`, `current_table_route.yaml`, `cycle_events.jsonl`, `result.json`, `operator_measurement.json`을 보존했다.
- 면 정렬 뒤 `box_phase: final_approach`에는 요청 목표 `0.01 m`가 기록됐다. 이어진 `table_visible_alignment_stage`와 `final_approach_straight`의 목표 `0.05 m`는 중간 정렬 단계다. 최초 추정 최근접 간격은 약 0.530 m, 직진 명령의 계획 이동량은 약 0.487 m였다. 계획값을 실제 이동량으로 취급하지 않는다.
- 내부 최종 방향 보정 로그는 보정 전 -2.130°, 회전 -2.347°, 보정 후 -0.157°였다. 이 값은 내부 관측이며 주차 종료 시점의 외부 각도 실측이 아니다.
- 전진 뒤 새 안정 관측을 얻지 못해 `final_approach`에서 `table_alignment_observation_lost`로 종료됐고 `result.json`은 `code: 1`이다. 관측 근거는 `no_table_edge_candidate`, `stable=false`, `stable_frame_count=0`이다. 오류 시점의 `latency_limited=false`, `fresh_since_latency=24`이므로 단순히 오래된 프레임만 기다리다 끝났다고 설명할 수 없다. 10 mm 마무리 단계의 완료 근거는 없다.
- 종료 후 대시보드 odometry와 마지막 속도 명령은 선속도·각속도 모두 0이었다. 추가 이동이나 자동 복귀는 수행하지 않았다.

사용자는 종료 후 “오른쪽 1.4 왼쫒ㄱ 2cm인데”라고 보고했다. 앞선 측정과 같은 차체 앞쪽 좌우 간격으로 해석했으며 기준점은 다시 확인하지 않았다.

| 항목 | 오른쪽 | 왼쪽 |
|---|---:|---:|
| 사용자 실측 | 14 mm | 20 mm |
| 요청 목표 | 10 mm | 10 mm |
| 목표보다 남은 간격 | +4 mm | +10 mm |

평균 간격은 17 mm, 좌우 차이는 6 mm다. 1 cm 정밀주차 성공으로 판정하지 않는다. 사용자 실측은 별도 JSON으로 남겼고 원시 실행 로그의 `physical_accuracy: NOT_MEASURED`는 소급해서 바꾸지 않았다.

**확정된 현상:** 접근은 수행됐고, 중간 50 mm 목표를 가진 직진 이후 근접 관측이 소실됐다. 실제 종료 위치는 사용자 실측 14/20 mm였으며 방향 차이도 남았다. 중간 목표와 실측의 차이를 검출 소실 하나만으로 설명할 수 없다.

**미확정 가설과 나중에 할 일:** 근접 시야·가림·면 후보 필터 중 검출 소실 지점을 분리하고, 마지막 유효 면을 사용한 접근에서 거리·방향 오차가 어디서 생겼는지 같은 run의 관측·계획·odometry를 대조한다. 높이 1.26 m로 바꾼 뒤 유지한 이전 장착각 추정, 앞끝–렌즈 수평 거리 0.34 m의 적용, 이전 근거리 보정 해제의 영향도 확인 대상이다. 아직 어느 항목도 원인으로 확정하지 않았다. 가까이서 검출되지 않는 면을 있다고 꾸미거나, 실측 근거 없이 보정값을 더하지 않는다. 코드 수정은 보류했다. 이후 별도 재주행 요청의 경과는 §21.7에 기록한다.

### 21.7 후진·한쪽 회전 후 재주차의 잘못된 실행 선택과 정지

사용자는 후진한 뒤 좌우 중 한쪽으로 방향을 바꾸고, 면을 정렬해 다시 정밀주차할 것을 요청했다. 보존 위치는 같은 데이터 루트의 `repark_20261016_095710/`이다.

1. 기존 `box_escape_once.py`에 이전 run의 관측 면을 넣었다. `reverse_path_checked`의 `valid=false`, `reason=static_obstacle_unknown_keepout_or_map_boundary`로 `box_escape_blocked`가 발생해 이동하지 않았다. 사용자는 후진 경로에 실제 장애물이 없다고 알렸다. 이 로그는 지도 검사 결과이며 실제 물체 존재의 확인으로 취급하지 않는다. 어느 셀·외곽 검사가 막았는지는 아직 확인하지 않았다.
2. 일반 Nav2 `/backup`을 시도했지만 action server가 없어 대기 뒤 종료됐다. 현행 기본 `behavior_plugins`에 일반 후진 서버가 없는 것을 먼저 확인하지 않은 실행 선택 오류였다. 후진 목표는 수락되지 않았고 이동하지 않았다. `backup_console.txt`와 `backup_result.json`을 보존했다.
3. 기존 웹 조종기 `/cmd`를 통해 `cmd_vel_nav` 경로로 후진과 한쪽 회전을 수행했다. 충돌 감지·조종기 데드맨을 유지했다. `manual_reset_evidence.json`의 odometry 이동은 약 0.24765 m 후진이며, 후진 전후 yaw는 2.568°→2.602°, 이어서 무작위로 선택한 오른쪽 회전 후 yaw는 0.499°였다. 회전 변화 약 -2.10°는 엔코더 기반 관측이며 외부 각도 실측이 아니다. 각 동작 후 0속도 입력을 보냈다.
4. `parking_20261016_100039_f7c85b46`에 최종 목표 10 mm와 `--resume-at-observation --zero-turn-final`을 제출했으나 **`--resume-at-final-approach`를 선택하지 않았다.** 이에 기존 `face_alignment`의 45 cm 중간 위치 이동이 실행됐다. 당시 시작 map pose는 `(0.136993, -0.291476, -3.031284 rad)`, 보낸 목표는 `(0.286381, -0.287610, -2.997573 rad)`였다. 시작 자세 기준 목표는 뒤쪽 약 0.14891 m·왼쪽 약 0.01260 m, 목표 yaw 차이는 약 +1.93°다. 이 뒤쪽 위치를 일반 Nav2 위치 이동으로 제출한 뒤 큰 회전이 발생했다. 면과 평행을 맞추기 위한 작은 각도 보정과 구분해야 한다. Codex가 현재 근접 배치에 맞는 재개 옵션을 선택하지 않은 오류로 기록한다. 실제 Nav2의 세부 회전 분기까지 원인 분석이 끝난 것은 아니다.
5. 불필요 회전을 확인해 기존 `cmd_stop`으로 목표를 취소했다. 해당 run은 `face_alignment / nav2_or_stop_confirmation`, 종료 코드 1로 끝났다. 이후 `final_only_20261016_100249_b2c17270`은 기존 `--resume-at-final-approach`로 중간 위치 이동을 생략했으나, 시작 면 관측을 얻지 못해 `stable detected table_edge surface is required`, 종료 코드 1로 이동 없이 끝났다. `--search`도 제출했지만 이 재개 경로에서 실제 탐색 회전이 실행된 로그는 없다.
6. 사용자가 “전혀 잘못된 행동을했다”고 지적한 뒤 추가 정지를 보냈다. `post_stop_dashboard_state.json`에서 현재 odometry와 마지막 속도 명령의 선속도·각속도가 모두 0이었다. 두 추적 프로세스는 종료됐고 추가 주차·복귀·회전을 보내지 않았다.

**결과:** 재주차 실패. 후진과 작은 오른쪽 회전은 수행했으나, 그 뒤 원하지 않은 큰 회전을 만들었다. 이 run에는 새 최종 좌우 간격 실측이 없다. 앞선 14/20 mm를 후속 run의 결과로 재사용하지 않는다.

**미해결:** 지도상의 후진 차단 지점, 가까운 재주차에서 중간 위치 이동을 선택한 운영 경로, 면을 잃은 자세에서 최종 접근 전용 재개의 처리. 제품 코드·캘리브레이션·지도는 바꾸지 않았고 해결됐다고 보고하지 않는다. 사용자가 요청한 RViz·뎁스·대시보드는 §21.5의 이유로 유지하며, 진행 중 이동 목표는 없다.

### 21.8 근접 재주차·최종 정렬 수정과 실행 결과

사용자가 로봇을 뒤로 옮기고 방향을 잡은 뒤, 앞선 문제와 정렬을 고쳐 재주차하라고 요청했다. 배치 완료를 받은 뒤 수정·관련 검증·Pi 반영을 마치고 실행했다. 사용자는 멀리 있는 배치에서 간격을 재기 어렵다고 알려 추가 출발 전 실측을 요구하지 않았다.

**소프트웨어 변경 (`b511aed`):**

- 책상 `face_alignment` 목표가 관측 당시 차체 뒤쪽이면 일반 위치 이동을 생략하고 기존 최종 면 정렬·접근 단계로 이어간다. 원거리에서 앞쪽에 있는 중간 목표는 유지한다.
- 마지막 fresh 면이 보이면 5 cm 이후 또는 처음부터 5 cm 안인 배치에서도 최종 각도 보정·재관측 후 전진한다. 기존에는 이 구간의 `allow_trim=False`가 잔여 오차를 유지했다.
- 과거 지연 이력 뒤 fresh frame quorum을 회복한 경우에는 `latency_limited=false`와 현재 거부 이유로 건강한 가림을 판정한다. 뎁스가 계속 오래되거나 소실된 경우는 허용하지 않는다.
- 중간 5 cm에서 건강한 경계 가림이 생기면 마지막 post-trim 면과 원래 `stage_to_odom`을 사용해 1 cm 구간을 이어간다. 이 가림 경로에서는 추가 회전을 하지 않고 정지·링크·센서·충돌 감지를 유지한다.

**카메라 후보와 좌표 일치:** 네 관측의 수평 책상 상판 법선을 평균해 roll `-1.7254356°`, pitch `33.4483001°` 후보를 산출했다. 이전 값은 각각 `+0.6837717°`, `23.6326010°`였다. 동일 관측을 새 후보로 변환한 상판 높이는 약 0.768–0.774 m로 모였지만, 실제 상판 높이·장착각을 외부에서 측정한 결과는 아니다. yaw는 수평 평면으로 식별할 수 없어 기존 값을 유지했다. 높이 1.26 m와 앞뒤 위치도 유지했으며 임의 거리 보정값을 더하지 않았다. 계산식·샘플·유효 범위는 `config/camera_mount_current.yaml`의 provenance에 기록하고 `new_base_real.urdf`의 RPY도 같은 값으로 맞췄다. 실물 yaw와 거리 정확도는 재주차 실측으로 확인해야 한다.

**검증·반영:** box service, table-edge geometry/detection, 실차 URDF 정합 검사 210건 통과. 독립 verifier가 정지·링크·센서 경계와 새 근접 분기·URDF 정합을 확인했다. Pi 소스·설치·운영 장착 파일 7개 경로를 백업·반영했고, URDF 설치 경로는 소스와 같은 파일을 가리킨다. 실행 중 `robot_description`을 같은 URDF로 갱신하고 상주 실행기를 재시작했다. 복구본은 `$HOME/.local/state/jdamr/table-alignment-fix/20261016_101805/`, PC 반영 근거는 `$HOME/.local/state/bimanual-robot/table_alignment_fix_deployment.json`이다. 지도·도크·펌웨어는 변경하지 않았다.

**현재 위치 초기화:** 수동 재배치 후 odometry는 이전 위치·방향을 유지했다. 현 LiDAR 스캔의 전역 정합 후보 `(0.615, -0.325, 3.176499 rad)`로 기존 비이동 초기화를 실행했다. 스캔의 5 cm 일치 비율은 약 21%였고 절대 위치 정확도로 취급하지 않는다. AMCL은 `(0.616714, -0.324818, -3.114931 rad)`로 활성화됐으며 내부 일치 확인이다. 등록된 도크를 현재 위치로 덮어쓰지 않았다.

**수정본 실행:** `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_fix_20261016_102002/parking_20261016_102137_07bfdaef/`. 목표 10 mm, 자동 도크 복귀 없음. 원시 `cycle_events.jsonl`, `command.json`, `result.json`, `final_dashboard_state.json`을 보존했다.

- 중간 면 정렬·5 cm 접근 후 fresh 면을 다시 관측하고, 최종 방향 보정 로그에서 보정 전 -1.567°·보정 후 +0.057°를 얻었다. 이는 내부 관측이며 외부 정렬 실측이 아니다.
- 마지막 전진 계획량은 약 55.32 mm였다. 최종 전진 후 새 면 관측은 가림으로 얻지 못했다. 마지막 유효 면과 동결 odom의 내부 추정 간격은 오른쪽 6.18 mm·왼쪽 5.91 mm, 차이는 약 0.27 mm다. `confirmation_basis=frozen_table_edge_odom_stop`, `final_depth_measurement=false`를 유지한다.
- 목표 10 mm에서 약 4 mm 더 가까운 내부 값이므로 `box_gap_not_confirmed`, 종료 코드 1로 끝났다. 1 cm 성공으로 보고하지 않는다.
- 종료 odometry의 선속도·각속도는 모두 0이었다. 추가 이동·자동 복귀는 수행하지 않았다. 처음 안내에서 이를 새 깊이 관측값으로 잘못 설명했으나 원시 이벤트의 동결 odom 기반 내부 값임을 확인해 정정했다. 근접 상태의 실제 좌우 간격을 사용자에게 요청했으며 외부 측정 응답은 대기 중이다. 이 관측 한 건만으로 거리 보정 상수를 추가하지 않는다.

### 21.9 로봇 좌우 실측 기준 보정과 1 cm 재주차

**후속 실측:** §21.8 실행 뒤 사용자가 로봇이 바라보는 기준 왼쪽 43 mm·오른쪽 27 mm로 확인했다. 이전 내부 추정 우 6.18 mm·좌 5.91 mm와 맞지 않으므로 이전 실행의 실물 1 cm 정확도는 성립하지 않는다. 카메라 하향 약 30°는 각도계 측정이 아닌 참고값으로 받았다. 반복 간격 측정을 매 출발 조건으로 요구하지 않도록 운영 규칙에 반영했다.

**관측 대조와 근거:** 같은 정지 위치에서 깊이 20장을 보존했다. 상판 면은 있지만 하단 후보 아래 깊이값이 0이어서 실제 깊이 불연속을 확인하지 못했다. 0값 경계를 강제 수용하면 실측과 반대 방향의 기울기가 나와 이를 책상 앞면으로 사용하지 않았다. 기존 웹 조종기로 짧은 후진 중 HTTP 응답 제한시간을 넘겼고, deadman 정지 뒤 실제 odom 이동량 53.40 mm와 정지를 확인했다. 후진 뒤 정상 검출은 20/20장이었다. 두 정지 LiDAR 관측의 최근접점 SVD 상대 정합은 후진 약 52.35 mm·yaw +0.0634°, 잔차 RMS 13.30 mm·대응점 378/520개였다. 바퀴 이동량과 약 1 mm 차이지만 외부 거리 측정이나 전체 주행 오차의 증명은 아니다.

**변경 (`fac0bad`):** 실측 좌우 간격·차체 폭으로 정의한 책상 면을 LiDAR 상대 변환으로 후진 위치에 운반하고 정상 깊이 관측 20장의 2D line과 비교했다. 관측 면 yaw 보정 -0.037999565 rad·signed plane offset +0.027985701 m를 적용했다. 물리 카메라 R/P/Y·URDF는 바꾸지 않았고 `n'=R(δ)n`, `d'=d+bias`, `center'=R(δ)center-bias*n'`를 task 관측에 한 번 적용한다. 기준 장착 6값이 바뀌면 재사용을 거부한다. 설정 provenance에 단일 외부 실측, 약 0.78 m 상판, 적합 평균 간격 35–87.35 mm, 도출 당시 10 mm 외삽의 외부 검증 부재와 후속 단일 배치 좌우 12 mm 확인(분해능·반복성 미확정)을 명시했다. 다른 높이·장착의 보편적 깊이 보정으로 취급하지 않는다. 마지막 precision 직진의 거리별 속도를 낮추고 계획량·정지 후 odom 이동량을 이벤트에 남긴다. 기존 충돌 감지·입력 유효성·timeout·정지 확인 경로는 유지한다.

**검증·반영:** ROS Jazzy와 PC overlay에서 table target·restaurant service·box service 표적 검사 491건 통과, 독립 verifier 재실행 491건 통과, diff 검사 통과. Pi 소스·설치의 두 Python 모듈과 소스·설치·운영 장착 설정 7개 경로를 이전 SHA 확인 후 백업·반영하고 새 SHA를 대조했다. 복구본은 `$HOME/.local/state/jdamr/table-line-calibration/20261016_105516/`, PC 반영 근거는 `$HOME/.local/state/bimanual-robot/table_line_calibration_deployment.json`이다. 지도·도크·펌웨어·물리 장착값과 현재 위치추정은 유지했다.

**재주차 결과:** `parking_20261016_105558_b898bae7`, 제출부터 종료 이벤트까지 약 57.9초. 현재 위치에서 기존 실행기의 `--resume-at-final-approach --zero-turn-final --final-gap-m 0.01`을 사용했고, 도크 복귀는 요청하지 않았다. 처음 방향 보정은 -2.155° 회전 후 fresh 면 잔차 -0.157°였다. 중간 전진 계획 36.72 mm·정지 후 odom 37.67 mm, 마지막 전진 계획 38.49 mm·정지 후 odom 38.81 mm였다. 마지막 속도는 0.01 m/s였다.

- 결과 JSON은 `code: 0`, 종료 선속도·각속도는 0이다. 내부 계산은 로봇 기준 왼쪽 10.46 mm·오른쪽 8.92 mm, 평균 9.69 mm, 방향 오차 약 0.196°로 기존 양쪽 모서리 허용 범위에 들었다.
- 실행기의 판정은 `confirmation_basis=frozen_table_edge_odom_stop`, `internal_estimate_only=true`, `final_depth_measurement=false`, `physical_accuracy=NOT_EXTERNALLY_MEASURED`다. 최종 근접면은 가려졌으므로 내부값을 새 깊이 실측으로 바꾸지 않는다. 이번 결과 뒤 추가 외부 간격 측정을 요구하지 않았다.
- **종료 뒤 사용자 실측:** "좌우측 모두 1.2"를 이전 보고와 같은 cm로 받아 로봇 기준 왼쪽·오른쪽 모두 12 mm로 기록했다. 목표 10 mm보다 각각 +2 mm이며 기존 거리 허용 범위 ±3 mm 안이다. 보고된 좌우 간격 차이는 0 mm지만 측정 분해능·실제 각도는 미확정이다. 이번 배치 한 건의 외부 간격 확인이며 반복 정밀도 검증은 아니다. `operator_measurement.json`을 추가했고 원시 실행 이벤트의 내부 판정 필드는 유지했다. 추가 2 mm 이동이나 실측에 맞춘 재보정은 수행하지 않았다.
- 정지 깊이·라이다·보정 계산 원본은 이전 실행 아래 `stationary_depth_20261016_103123/`에, 최신 command·원시 event·결과·종료 화면·정리 상태는 같은 데이터 루트의 이번 `parking_.../`에 있다. 데이터는 Git에 넣지 않았다.

**화면·프로세스 정리:** 열과 표시 부하를 줄이기 위해 기존 `display-stop`으로 RViz와 PC/Pi 표시 중계·표시 노드를 종료했다. 주차 후 추가 연산을 줄이도록 `jdamr-box-observer.service`를 중지하며, 다음 `--camera-always-on` 실행에는 이 관측기를 먼저 기동해야 한다. 사용자 요청 화면을 위해 PC `robot-dashboard.service`·`robot-dashboard-link.service`, Pi `jdamr-ammr-dashboard.service`·`jdamr-box-rgbd.service`는 유지한다. 정리는 PC `systemctl --user stop robot-dashboard robot-dashboard-link`, Pi `sudo systemctl stop jdamr-ammr-dashboard jdamr-box-rgbd`를 함께 사용한다. 기존 상주 `jdamr-box-executor.service`는 다음 명령의 재탐색 지연을 줄이도록 idle로 유지하며 종료 명령은 `sudo systemctl stop jdamr-box-executor`다. 새 임시 기록기·터널·속도 발행 프로세스는 남기지 않았다.


## 22. 전체 테이블 흐름의 경유 누락·잘못된 도크 방향과 점유 영역 수정

### 22.1 실행과 사용자 보고

실행은 `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_01_20261016_111358_03154e5e/`에 보존했다. 도크 등록·정지 스캔은 같은 runs 아래 `table_dock_cycle_20261016_111038/`다.

- 사용자는 도크 출발, 급수대·테이블 정밀 주차, 같은 도크 복귀의 전체 흐름을 요청했다. Codex가 급수대 없는 경로를 제출했고 도크를 전진 진입으로 등록했다. 사용자가 확인한 올바른 복귀 방향은 후진이다. 실행 선택·등록 오류이며 사용자의 잘못으로 귀속하지 않는다.
- 사용자는 1번 테이블로 가며 급수대의 상판과 거의 충돌했고, 1번 주차 간격은 약 3–5 cm였으며, 복귀 중 도크로 전진해 충돌했다고 보고했다. 기존 낮은 박스와 평행한 빈 공간이 아니라 그 위에 상판이 있는 배치다. 다리 사이도 점유 영역으로 채우라는 후속 지시를 받았다.
- 로그의 테이블 내부 최종 추정은 우 10.259 mm·좌 9.032 mm였지만 새 깊이 실측이 아닌 `frozen_table_edge_odom_stop` 결과다. 사용자 보고와 맞지 않아 실물 1 cm 주차 성공으로 판정하지 않는다.
- 기존 접근은 약 0.464 m 계획 후 중간 5 cm 부근에 정지하고 12초 동안 관측을 기다렸으며, 마지막 약 31 mm 전진 뒤에도 12초를 기다렸다. 이 중간 정지·재관측 대기가 끊기는 접근의 확인된 원인이다. 실물 거리 차이의 전체 원인은 아직 확정하지 않았다.
- 잘못 등록한 전진 복귀의 staging은 약 `(-0.568, 0.075) m`로 도크 뒤쪽이었다. 사용자는 충돌 후 로봇을 옮겼으며 잠깐 다시 이동한 뒤 중지했다고 보고했다. 정확한 충돌·들어올림 시각은 계측하지 못했으므로 정지 지연의 특정 코드 결함으로 단정하지 않는다.
- 취소 후 run은 종료 코드 1로 끝났고 fresh odometry의 선속도·각속도 0과 목표 취소를 확인했다. `operator_report.json`, `post_stop_executor_state.json`, `result.json`, 원시 이벤트를 보존했다. 현재 위치추정은 사용자의 재배치로 무효화했으며 도크 좌표로 덮어쓰지 않았다.

### 22.2 수정과 소프트웨어 검증

- 기본 출발은 급수대를 포함한다. `--skip-via`를 명시한 경우만 생략한다.
- 고정 도크 `(0.1289111014, 0.1389656609, 0.0912706384 rad)`는 유지하고 `parking_direction=reverse`를 PC·Pi 등록부에 반영했다. 후진 staging은 도크 앞쪽 약 `(0.826, 0.203) m`다. 이 변경 후 실차 복귀는 수행하지 않았다.
- 테이블 최종 접근은 기존 정지 각도 보정 뒤 단일 직진으로 바꿨다. 이동 중 새 유효 면을 해당 관측 시각의 odom TF로 옮겨 남은 거리를 갱신한다. 건강한 근접 가림 quorum 뒤에는 마지막 면을 동결하고 추가 관측으로 목표를 다시 늘리지 않는다. 이동 중 각도 탐색은 하지 않는다.
- 기존 속도 명령·충돌 감지·링크·스캔·odometry 유효성·정지 확인 경로를 사용한다. 가드·시간 종료는 목표 도달로 반환하지 않으며 정지 후 settle 구간에서 목표를 갱신하지 않는다. 중간 5 cm 정지와 종료 후 12초 재관측을 생략하고, 최종 동결 값은 내부 추정으로 기록한다.
- 대상 box service·restaurant service·keepout·출발 모의 검사 632건 통과, 별도 verifier도 632건 통과했다. callback 실패·가드 중단·정지 후 목표 갱신 금지·가림 뒤 가짜 fresh 면 무시 회귀를 확인했다. 연속 접근 수정본의 실차 주차는 아직 수행하지 않았다.
- `a31f00f`의 Python 3모듈을 Pi 소스·설치 6개 경로에 이전 SHA 확인 후 백업·반영했고 새 SHA를 모두 대조했다. 상주 실행기만 재시작해 active와 새 모듈 import를 확인했다. 복구본은 `$HOME/.local/state/jdamr/continuous-table/20261016_120517/`, PC 반영 근거는 `$HOME/.local/state/bimanual-robot/continuous_table_deployment.json`이다. 마스크·지도·물리 장착·펌웨어는 변경하지 않았고 이동 요청은 제출하지 않았다.

### 22.3 검은 지도 셀 기반 테이블 킵아웃 후보

현행 소스는 `config/table_keepout_current.yaml`과 `keepout_mask occupied`다. 기존 마스크를 보존한 채 추정 상판 영역을 OR 합성하며 추가 여유는 0 m다. 근거는 도크에서 받은 40개 스캔, 5 cm 격자의 24/40회 이상 지속 셀 179개와 기존 지도 검은 셀이다. LiDAR는 다리를 보므로 상판 외곽 실측으로 취급하지 않는다.

- 급수대와 1번 테이블의 앞·뒤 검은 셀 바깥선을 연결하고 다리 사이를 채웠다. 회색 지도 경계는 외곽 근거에서 제외했다. 2번 테이블은 고유 다리 쌍이 분리되지 않아 비활성으로 유지한다.
- 사용자가 왼쪽 테이블은 위쪽 도크 옆까지 이어진다고 확인해 급수대 영역을 지도 Y 약 0.75 m까지 연장했다. 급수대 기존 경계의 검은 원본 셀은 34개, 도크 확장부는 4개, 1번 영역은 41개다. 최종 설정은 기존 급수대를 활성으로 유지하고 확장부 `dock_table`은 `enabled: false`로 분리했다. 도크 좌표·추가 여유는 바꾸지 않았다.
- 비활성 확장 시각화 후보의 합성 마스크 SHA-256은 `b85ffbb62a140408b6e3eaaa53206e837a128ff2d46166e0e6e3dba1e74f4550`이다. `$HOME/.cache/jdamr/table_legs_20261016/keepout_tables_dock_extension.{pgm,yaml,json}`, `keepout_dock_extension_ko.png`, `dock_extension_geometry.json`에 로컬 산출물을 보존했다.
- 고정 도크의 현재 Nav2 footprint 안에 확장 마스크 차단 셀 중심 21개가 들어간다. 독립 검증의 뒤쪽 최대 겹침은 약 13.61 cm, 교차 면적은 0.05685 m²다. 사용자는 차체 전체가 상판 바깥에서 붙는다고 확인했으므로 동일 폭 연장 추정과 실제 배치는 불일치한다. 도크 위치를 이동하거나 가짜 빈 통로를 뚫어 검사를 통과시키지 않았다. 해당 마스크는 Pi에 적용하지 않았다.
- 확장 전 후보는 별도 verifier가 경로 연결·도크/관측/최종 위치와 실제 footprint sweep의 차단 셀 0을 확인했다. 이 승인을 확장 후 도크 겹침 후보에 재사용하지 않는다. 최종 적용 때는 PC·Pi 마스크와 등록부 및 모든 route의 SHA를 함께 대조해야 한다.

로봇 이동은 중지 상태를 유지한다. 도크 주변 기하 정합, 수정 코드의 실차 연속 접근·주차 간격, 급수대 포함 왕복과 후진 복귀는 남은 확인 항목이다. RViz·표시 중계는 꺼 두고, 사용자 화면용 PC `robot-dashboard`·`robot-dashboard-link`와 Pi `jdamr-ammr-dashboard` 및 다음 요청용 idle `jdamr-box-executor`를 유지한다. 종료 명령은 PC `systemctl --user stop robot-dashboard robot-dashboard-link`, Pi `sudo systemctl stop jdamr-ammr-dashboard jdamr-box-executor`다. 깊이 카메라·관측기는 앞선 사이클 정리로 중지돼 있으며 최신 영상이 실행 중이라고 주장하지 않는다.


### 22.4 사용자 3 cm 배치 근거로 도크 테이블 경계 수정·반영

사용자는 현재 차체가 테이블과 평행하고 약 3 cm 떨어져 있으며, 도크점 또는 킵아웃을 그 배치에 맞춰 조정해도 된다고 허용했다. 후진 도크 문맥의 뒤쪽 간격으로 해석했다. 이는 §22.3 당시 부족했던 물리 배치 근거이며, 차체만 비우는 좁은 통로를 임의로 그린 것이 아니다.

- 정지 스캔 40개의 현재 map TF는 `(-0.03559, 0.04841, 0.33810 rad)`였고, 정합 후보는 `(0.11441, 0.16841, 0.12867 rad)`, 5 cm inlier 35.29%, 전역 후보와 일치했다. 두 값 차이 19.2 cm·12°는 내부 대조이며 외부 측정 오차가 아니다. 기존 비이동 활성화 후 native `teach-home --parking-direction reverse`로 `(0.12002282933008113, 0.16495971131874898, 0.13404527824949672 rad)`를 등록했다. 이전 도크 좌표는 백업에 보존했다.
- 차체 rear는 실차 URDF의 뒤 레일 중심 -0.265 m와 두께 절반 0.010 m로 -0.275 m다. 사용자 평행 배치의 진행 방향 `n=(0.9910293759, 0.1336442148)`과 3 cm를 적용해 테이블 전면을 `n·p=-0.1640079393`으로 추정했다. 도크 테이블 전체 전면을 이 직선으로 바꿨고 급수대 영역과 이어지게 했다. 앞쪽 검은 셀 일부는 해당 테이블 소속이 확정되지 않았으므로 새 전면 근거에서 제외했지만 원본 정적 지도 점유는 그대로 유지한다.
- 도크 표식의 후면 간격은 등록부 `teaching.rear_gap_measurement`에 기록해 기존 앞쪽 간격 필드와 섞지 않았다. 사용자 대략값·단일 배치에 근거한 설정이며 정확한 테이블 각도·후진 복귀 3 cm 정밀도를 새로 검증한 것이 아니다. 기존 도크 위치 허용오차나 후면 거리 센서를 이번 작업에서 바꾸지 않았다.
- 새 마스크 SHA는 `8278dcef38b06d469b53f32b1c6ab2c5a14b27f23523f27bcb83aa11ab4a8526`이다. 기존 비자유 픽셀 삭제 0, 추가 여유 0 m, 원본 지도 격자와 동일하다. 실제 Nav2 footprint의 home↔0.7 m staging 1 cm 간격 71 pose, 급수대·테이블 관측/최종/탈출과 경유·직행 정적 sweep는 독립 `static_corridor_clear` 검증을 통과했다. 정적 검증을 실제 Nav2 계획이나 동적 장애물 통과로 확대하지 않는다.
- PC·Pi·저장소 번들의 keepout·도크 등록부·5 route를 맞추고 모든 `expected_mask_sha256`을 새 값으로 변경했다. 원래 unknown 마스크는 `keepout_base_unknown.{pgm,yaml}`로 보존해 다음 갱신이 이전 상판 영역을 누적해서 남기지 않게 했다. Pi 14경로 해시와 마스크 재로딩 후 transient-local 발행 마스크의 기대 occupancy 바이트 일치를 확인했다. 관련 운영 도구·정적 자산 검사 22건과 diff 검사가 통과했다.
- 원본·정지 관측·초기화·도크 등록 로그·파생 평면·동기화 receipt·새 프리뷰는 `$HOME/jdamr_data/map_20260930_manual/aligned/runs/dock_table_fit_20261016_121145/`에 있다. Pi 복원본은 `$HOME/.local/state/jdamr/dock-table-assets/dock_table_fit_20261016_121145/`다. 새 임시 상주 프로세스는 남기지 않았으며 기존 사용자 대시보드·베이스·Nav2·idle 실행기를 유지했다. 이번에는 이동 명령·실차 왕복·원격 push를 수행하지 않았다.

### 22.5 이동 중 안정화·깊이 지연 종료 수정과 사용자 정지

**첫 재시도:** `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_01_20261016_122520_a246070b/`. 급수대 접근의 마지막 상태는 면 검출 `true`, 안정화 `false`, 이유 `collecting_stability`였다. 새 승인 면이 1.5초 동안 없으면 이를 관측 소실로 처리하는 분기에 들어가 `table approach stable face unavailable`로 멈췄다. 요청 직진량 206.48 mm 중 odom 이동량은 105.93 mm였다. 영상 수신 소실이나 배터리 문제로 확정하지 않는다.

- 정렬 후 잔차가 +0.76994°→-0.58959°→+0.46178°로 바뀌었다. 마지막 -0.590° 명령의 실제 odom 회전은 -1.395°였다. 작은 보정에도 기존 0.08 rad/s를 적용하던 정지 후 회전량과 반복 재관측이 불필요한 좌우 왕복을 만든 근거다.
- 회전 뒤 `observe_target()`이 동일한 범위·장착·관측 모드를 다시 설정하면서 안정 이력을 초기화했다. 재관측 호출마다 같은 설정을 쓰는 분기에는 `reuse_configuration=True`를 적용했다. 입력 복구가 필요한 분기의 설정은 유지했다.
- `1de8d2a`에서 건강한 `collecting_stability`는 마지막 승인 면을 운반하고 불안정 후보를 새 목표로 채택하지 않도록 했다. 작은 회전 감속은 연속 테이블 접근에만 적용했다. 기존 깊이·TF·영역·법선·이동량·정지·링크 검사는 유지했다. box/restaurant 검사 473건과 별도 검증을 통과했다. Pi 복원본은 `$HOME/.local/state/jdamr/continuous-table/20261016_124001/`, PC receipt는 `$HOME/.local/state/bimanual-robot/moving_table_recovery_deployment.json`이다.

**두 번째 재시도:** `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_01_20261016_124036_3d364831/`. 완료한 급수대 관측 지점부터 기존 실행기의 재개 옵션으로 이어갔다.

- 급수대 회전 후 잔차는 +0.10382°, 1번 테이블은 -0.11639°로 각 지점에서 추가 좌우 왕복 없이 접근했다. 한 실행의 관찰이며 모든 배치에서 왕복이 사라졌다는 검증은 아니다.
- 급수대 접근은 12:41:19에 완료 판정, 대기·후진 이탈은 12:41:30에 완료됐다. 내부 최종 간격은 평균 9.740 mm, 양 모서리 10.335/9.144 mm였다. `frozen_table_edge_odom_stop`, `internal_estimate_only=true`, `final_depth_measurement=false`를 유지하며 외부 1 cm 실측으로 쓰지 않는다.
- 1번 테이블에서는 514.90 mm 계획에 odom 472.51 mm 전진 후 12:42:17에 `table approach depth stale or future-dated`가 발생했다. 깊이 영상 촬영 시각부터 제어 시점까지 0–0.5초 조건을 벗어난 분기다. 정확한 실패 프레임 나이와 연속 실패 개수는 원시 이벤트에 없으므로 “USB 문제”, “파이 과부하”, “한 프레임만 지연” 중 어느 것도 확정 원인으로 기록하지 않는다.
- 사용자 후속 보고는 1번 테이블과 약 6–7 cm다. 좌우별 측정이 아니며 목표 1 cm는 미완료다. `operator_measurement_and_pause.json`에 보존했다. 원시 `cycle_events.jsonl`과 실행기 결과를 PC에도 복사했다.

**입력 지연 복구 수정 (`dfb2c29`):** `_drive_zero_turn()`은 명시한 세 이유(`observer silent`, `depth stale or future-dated`, `observer latency`)에서 0속도 발행과 odom 정지를 확인한 뒤 `table_approach_waiting_for_input`을 기록한다. 같은 시작 odom·목표 갱신 closure·남은 거리 상태를 유지하며 기다리고, 새 유효 입력과 기존 가드가 정상일 때만 같은 직진을 재개한다. 대기 시간은 해당 직진의 이동 예산에서만 제외하고 상위 태스크 예산·실측 차체·신선도 기준은 바꾸지 않았다. TF·기하·정지 미확인·분류되지 않은 오류를 이 재개 분기에 넣지 않았다.

- 식사 중 사용자 정지는 복구보다 우선한다. 링크가 대기 중 끊기면 heartbeat가 돌아와도 같은 시도에서 자동 재개하지 않는다. stale 입력→가드 회복→같은 목표 완료, 사용자 정지, 링크 손실 뒤 회복의 세 plant 회귀와 기존 precision 회귀를 확인했다. 표적 6건, restaurant 전체 313건, 별도 verifier 검증과 diff 검사 통과다.
- Pi 소스·설치 총 6경로를 이전 SHA 확인 후 백업·대조했다. 실제 바뀐 모듈은 restaurant service이며 SHA는 `6e39e5c449163104ec1405cf68d80513964c47010ff017640ed3b31629417471`이다. 실행기 active·새 소스 import를 확인했다. Pi 복원본 `$HOME/.local/state/jdamr/continuous-table/20261016_125125/`, PC receipt `$HOME/.local/state/bimanual-robot/depth_pause_recovery_deployment.json`에 근거를 보존했다. 이 반영은 이동을 요청하지 않았다.

**정지 영상 진단:** 현 위치에서 카메라·관측기만 켜고 같은 table 모드·장착·거리 창으로 20초 측정했다. 93개 status의 수신 시 원본 영상 나이는 중앙값 9.37 ms·p95 257.01 ms·최대 312.93 ms였고 0.5초 초과는 0건이었다. 다만 93개 모두 `no_table_edge_candidate`여서 유효 면이 검출되는 주행 중 연산 부하를 재현한 검사는 아니다. 앞선 지연을 재현하지 못했으며 CPU·USB·통신 중 원인은 미확정이다. 새 실행에서 이 근접 위치의 경계를 곧바로 재검출할 수 있다고 보장하지 않는다. 필요 시 기존 후진 시야 확보 경로로 재관측해야 하며 식사 중에는 이를 실행하지 않았다. 원시값·요약·정리 결과는 이번 run의 `stationary_timing.json`, `stationary_timing_summary.json`, `stationary_timing_cleanup.txt`에 있다.

**중지 상태와 남은 일:** 기존 stop을 보낸 뒤 fresh odometry의 선속도·각속도 0, spool의 request/running 0개를 확인했다. 운영 상태에는 급수대 주차·대기·이탈과 1번 테이블 이동·정렬을 완료로, 남은 1번 최종 접근·이탈·후진 도크 복귀를 대기로 보존했다. 상위 작업은 `paused_by_operator`이며 다음 사용자 이동 요청 전에는 재개하지 않는다. 모든 실패에서 이동을 무조건 반복하는 공통 루프를 구현했다고 주장하지 않는다. 앞선 AGENTS 운영 규칙의 상위 작업 보존과 이번 세 종류의 입력 복구 구현은 구분한다.

카메라·관측기는 진단 후 inactive/PID 0으로 정리했다. RViz·표시 중계는 계속 꺼 두고 기존 사용자 대시보드·베이스·Nav2·idle 실행기를 유지한다. 대시보드 종료는 PC `systemctl --user stop robot-dashboard robot-dashboard-link`와 Pi `sudo systemctl stop jdamr-ammr-dashboard`를 함께 사용한다. 다음 요청 대기용 실행기 종료 명령은 Pi `sudo systemctl stop jdamr-box-executor`다. 새 임시 프로세스는 남기지 않았고 이동 명령·원격 push는 수행하지 않았다. 수정본의 실차 입력 지연 회복, 최종 1 cm 간격과 후진 도크 복귀는 미검증이다.

### 22.6 반복 정렬의 누락 경로 수정과 실행기 왕복 완료·후속 실물 실패

**재점검 요청과 범위:** 사용자는 좌우 반복 정렬이 이전에도 발생했던 문제라고 지적하고 로봇을 등록된 도크로 옮겼다고 알렸다. 우선 정지를 유지하고 이전 테이블 위치추정·중간 접근 재개를 무효화했다. 도크는 새로 등록하지 않았다.

- 과거 JD-AMR `3cbd779`는 중간 미세 정렬과 최종 허용각을 구분한 수정이고, `b511aed`는 가까운 테이블 뒤쪽의 45 cm 목표로 잘못 주행하던 경로를 건너뛴 수정이다. 작은 회전의 정지 후 회전량을 줄이는 변경과는 다르다. `1de8d2a`는 이 감속을 `continuous_table=True`에만 적용했다. 재귀에서도 그 값은 보존되므로 현행 1 cm 연속 접근에서 옵션이 유실된 증거는 없었다.
- 반면 도크 `_trim_dock_heading()`과 나머지 미세 접근 호출은 기존 고정 속도를 사용했다. 테이블 보정은 같은 면 관측에 대한 odom 보정을 최대 3회 하고, fresh 관측으로 최대 2회 재귀하므로 잔차 부호가 바뀌면 반복 반전할 수 있는 조건도 남았다. 재점검 당시 Pi box/restaurant 소스·설치 4경로는 저장소와 일치했다. 단순 미배포나 과거 코드 복원으로 귀속하지 않는다.
- Nav2의 거친 면 정렬은 별도 `Parking` 컨트롤러 경로다. 이번 변경에 그 회전 속도·goal checker를 포함하지 않았다. 모든 형태의 좌우 움직임이 같은 원인이라고 단정하지 않는다.

**수정·검증 (`5835cef`):** `_rotate_in_place()`의 `precision` opt-in을 없애 모든 기존 호출에 작은 회전 감속을 적용했다. 1.74° 이상에서는 최대 0.08 rad/s로 기존과 같으며, 정지 선행량·시간 예산은 선택한 실제 속도를 따른다. 테이블 재관측 callback이 있을 때 odom 잔차가 부호를 바꾸거나 개선되지 않으면 과거 면에 대한 추가 보정 대신 기존 fresh 재관측·정지 pose 취득·새 면 잔차 판정을 먼저 거친다. fresh 각도 허용값·재귀 상한·최종 양 모서리 판정·정지·센서·링크 검사는 유지했다. 새 실패 차단 조건을 추가하지 않았다.

- 양방향 0.32/0.59/0.87/1.2° 기본 회전과 양방향 1.2° 도크 plant 10건, 연속/기존 테이블 양방향의 `회전→새 면 관측→전진` 순서 4건은 수정 전 실패·수정 후 통과했다. box/restaurant 전체 489건 통과, 별도 verifier 표적 16건·기존 회전 3건 및 diff 검사 통과다. 모형 결과를 실차 회전 정확도로 간주하지 않는다.
- Pi 소스·설치 6경로를 대조·백업했고 box/restaurant 두 모듈이 바뀌었으며 keepout 모듈은 동일하다. 새 모듈 import와 실행기 active를 확인했다. 복원본은 `$HOME/.local/state/jdamr/continuous-table/20261016_132559/`, PC receipt는 `$HOME/.local/state/bimanual-robot/alignment_recurrence_deployment.json`이다.

**도크 재배치의 비이동 위치추정:** `$HOME/jdamr_data/map_20260930_manual/aligned/runs/alignment_recurrence_20261016_132540/`에 사용자 보고·fresh 40스캔·정합·초기 pose·native 활성화 원본을 남겼다. 등록 도크 주변 정합 후보 `(0.125023, 0.139960, 0.099139 rad)`와 전역 후보가 일치했고 5 cm inlier는 37.76%였다. 기존 비이동 활성화 후 AMCL은 `(0.126325, 0.141585, 0.094262 rad)`로 정합 후보와 2.08 mm·0.279° 차이였다. 이는 내부 일치 확인이다. 도크 등록값·테이블 지역·경로·마스크는 변경하지 않았고 이 준비 단계에서는 이동을 요청하지 않았다.

**후속 출발과 결과:** 사용자의 새 출발 요청에 기존 `go table_01`을 제출했다. 추가 코드·테스트·리뷰·문서 작업을 출발 앞에 넣지 않았다. 원본은 `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_01_20261016_132710_39ed71a7/`다.

- 기본 급수대 경유를 포함해 급수대 접근·대기·후진 이탈, 1번 테이블 접근·대기·후진 이탈, 도크 후진 복귀를 완료했다. 실행기 결과 `code: 0`, `home_arrived.parking_direction=reverse`, 첫 실행 이벤트부터 마지막까지 221.72초다. 제출 전 대기나 전체 요청 경과 시간을 이 값으로 바꾸지 않는다.
- 급수대는 보정 후 잔차 -0.10185°였다. 1번 테이블은 fresh 관측 잔차 -0.57047° 뒤 같은 음의 방향 보정 -0.408°를 하고 -0.19315°로 줄었다. 도크 마지막 보정은 1회였다. 이번 실행에서 좌우 반복 반전은 확인되지 않았으나 모든 배치의 제거를 보장하지 않는다. `table_trim_reobserve_requested` 분기는 이번 실제 주행에서 발생하지 않았고 plant 회귀로 확인했다.
- 입력 지연 대기/회복은 총 7쌍(급수대 5·1번 2)이었다. 모두 같은 목표를 유지한 채 복구했고 마지막 `target_failure`는 null이었다. 기록된 `paused_s`는 정지 확인을 마친 뒤의 대기 시간만 뜻하며 감속·정지 확인 시간을 합친 전체 중단 시간으로 해석하지 않는다. 지연 자체의 원인은 여전히 미확정이다.

| 지점 | 좌측 내부 간격 | 우측 내부 간격 | 내부 방향 오차 |
|---|---:|---:|---:|
| 급수대 | 10.588 mm | 9.141 mm | 0.184° |
| 1번 테이블 | 11.448 mm | 8.916 mm | 0.322° |

위 표는 원시 배열의 순서가 우측·좌측임을 반영했다. 두 지점 모두 `confirmation_basis=frozen_table_edge_odom_stop`, `internal_estimate_only=true`, `final_depth_measurement=false`이며 외부 실측 간격이 아니다. 도크 정지 확인 오차는 내부 위치 14.28 mm·각도 0.171°다. 이전 사용자 배치의 뒤쪽 3 cm를 이번 복귀의 새 실측값으로 쓰지 않는다.

**종료 정리:** fresh odometry 선속도·각속도 0을 확인하고 상위 작업을 완료로 기록했다. 카메라·관측기는 inactive/PID 0이며 추가 이동을 요청하지 않았다. 원시 이벤트·결과·종료 대시보드·요약은 같은 run의 `cycle_events.jsonl`, `result.json`, `final_dashboard_state.json`, `post_run_summary.json`에 있다. 사용자 화면용 PC `robot-dashboard`·`robot-dashboard-link`, Pi `jdamr-ammr-dashboard`와 다음 명령용 idle `jdamr-box-executor`는 유지한다. 실제 PID는 `retained_pc_services.txt`, `retained_pi_services.txt`에 보존했다. 종료 명령은 앞 절과 같고 새 임시 상주 프로세스는 남기지 않았다. 이 실행으로 입력 지연 회복과 후진 도크를 포함한 전체 순서의 실행기 완료는 확인했으나, 반복 성능·외부 1 cm 정확도·도크 뒤쪽 3 cm 정확도는 아직 미검증이다. 원격 push는 수행하지 않았다.

**후속 실물 보고로 판정 보완:** 사용자는 도크의 차체 뒤쪽 좌/우 모서리–테이블 간격이 약 5/11 cm이며 기울어졌고, 급수대 주차도 5–7 cm였다고 보고했다. 원시 `code: 0`은 보존하고 `operator_post_cycle_report.json`, `outcome_interpretation.json`에 실물 정밀주차 실패를 별도로 기록했다. 앞선 내부 1 cm 수치를 외부 성공으로 해석하지 않는다. 도크 종료 지도 오차는 약 112 mm·6.967°였고, 정지 후 `post_cycle_stationary_dock/scan.json`의 40스캔과 `refine.json`은 지역·전역 정합이 일치하면서 등록 도크 대비 약 149.16 mm·7° 차이를 보였다(inlier 33.11%). RViz 지도 축만의 기울기라고 처리하지 않는다.

### 22.7 도크 지도 오차 판정과 안정 면 재획득 수정

**확정한 소스 문제:** 도크 최종 판정은 staging 때 동결한 지도→odom 변환에서 만든 목표와 odom 정지만 확인했다. 당시 원래 지도 목표와의 오차를 최종 판정에 넣지 않았다. 또한 깊이 `collecting_stability`가 임시 상태인데도 면 소실을 영구 latch해 이후 안정 관측을 반영하지 않았다. 첫 왕복은 급수대 마지막 승인 관측 후 약 18.06초, 1번 약 16.95초를 동결 면·odom으로 진행했다. 이 두 결함은 확인됐으나 급수대 실물 5–7 cm 거리 오차의 전체 원인이 확정된 것은 아니다.

**수정 (`782a2c6`):**

- 도크 odom 정지 확인 뒤 원래 지도 목표·home 계약으로도 확인하고 두 좌표계의 근거를 `dock_pose_frame_confirmation`에 나눠 남긴다. 지도 위치·방향 허용오차 초과 때만 기존 동결 축 이탈 후 후진 진입을 한 번 다시 수행한다. 재접근에서는 `already_at_home` 단축을 사용하지 않는다. TF·입력·정지·링크 실패를 같은 후진 복구로 확대하지 않는다.
- 건강한 `collecting_stability`는 마지막 승인 면을 임시 유지하되 `occluded_frames=0`으로 두어 이후 안정 면 갱신을 허용한다. 불안정 후보를 새 제어 면으로 채택하지 않으며, 건강한 실제 `no_table_edge_candidate` quorum의 기존 동결은 유지한다.
- 관련 전체 스위트 629건을 통과한 뒤 재접근 단축 차단 회귀를 보강했고, 마지막 표적 7건·별도 verifier·diff 검사를 통과했다. 마지막 변경 뒤 전체 630건을 다시 실행했다고 주장하지 않는다.

**Pi 반영:** 소스·설치 6경로 SHA 대조, 변경 모듈 import, 실행기 active를 확인했다. box SHA는 `7cc169777ed59d76a0db9602adcf97aed880f0e77be98dc5609a4fef84ca8944`, restaurant SHA는 `3fa1eb3906c022dea070e129137946d73c7fd6525cd49651acbc6631b2b8a9d3`다. keepout 모듈·마스크는 바꾸지 않았다. 백업은 `$HOME/.local/state/jdamr/continuous-table/20261016_134701/`, receipt는 `$HOME/.local/state/bimanual-robot/dock_map_and_depth_reacquisition_deployment.json`이다.

### 22.8 급수대 이탈 후 1번 테이블 이동 중 충돌·사용자 정지

**실행과 현상:** 사용자가 주요 문제 개선 후 출발을 요청해 run `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_01_20261016_134726_cdd7abb7/`을 시작했다. 급수대 접근에서 안정화 임시 유지 이후 `table_approach_face_updated`가 다시 기록돼 면 재획득 수정의 해당 분기는 실차에서 관측됐다. 내부 정지 간격은 좌/우 8.874/10.512 mm, 방향 오차 0.209°였다. 최종은 `healthy_edge_occlusion_frozen_odom`이며 새 근접 깊이·외부 간격 실측이 아니다.

- 급수대 후진 이탈 목표가 완료되고 `box_escape_finished.face_distance_m=0.522139`였다. 이 값은 base 원점에서 면까지의 거리이며 차체 앞끝 간격이 아니다.
- 그 다음 기존 `table_01_pre=(0.438,-1.817)`와 `table_01_observation=(0.858,-1.817,yaw=0)`를 하나의 `NavigateThroughPoses`로 보냈다. Nav2는 관측점 도착 성공을 반환했으나 사용자는 이 전진·좌회전 구간에서 테이블에 부딪혔다고 보고했다. 정확한 접촉 시간·부위는 계측되지 않았다.
- 사용자가 정지를 요청했고 기존 stop을 적용했다. 이후 1번 면 관측은 실패했고 하위 실행은 `code: 1`로 끝났다. 소프트웨어의 관측점 도착을 충돌 없는 이동으로 판정하지 않는다. 도크 복귀까지 진행하지 않았으므로 §22.7 도크 재접근 수정은 실물 미검증이다.

**코드·좌표 대조로 확인한 사항:**

1. `scripts/map_20260930_manual/jdamr_depart.py`의 경로 생성은 과거 박스 앞면에서 `PRE_M=1.00`만큼 떨어진 점을 사용한다. 현행 1번 pre점은 같은 run의 깊이 기반 급수대 상판 직선 추정에서 법선 거리 약 6.1–6.3 cm다. 앞선 대화의 약 8 cm는 x좌표 차이였으며 기울어진 직선까지의 거리와 구분한다. 이 비교는 관측 직선 연장과 지도 좌표 추정이며 상판 전체 외곽을 실측한 값이 아니다.
2. `restaurant_service.py`의 `_execute_service_once()`·`_execute_through_once()`는 두 경유점을 연속 목표로 보낸다. `home_exit`도 경로를 만드는 점이며, 그곳까지 직진 이탈을 마쳐야 회전을 허용하는 별도 단계가 아니다. 따라서 사용자 요구인 `도크에서 어느 정도 전진한 뒤 회전`을 현재 경유점만으로 보장할 수 없다.
3. `config/table_keepout_current.yaml`의 급수대 전면 최대 x는 0.0 m이고, 검은 지도 셀·다리 후보를 연결한 경계다. 깊이 관측 상판 직선의 x는 약 0.354–0.355 m여서 표현이 일치하지 않는다. 상판 돌출·지도 위치추정·카메라 보정 각각의 기여는 아직 분리하지 못했다. 다리 위치만으로 상판 외곽이 확정된다고 취급하지 않는다.

**원인 판단과 남은 조치:** 기존 경유점이 깊이 관측 상판 가까이 놓이고, 직진 이탈과 방향 전환이 분리되지 않으며, 상판과 낮은 라이다 킵아웃 표현이 다르다는 점이 유력한 소프트웨어 원인이다. 전체 접촉 궤적·명령 속도 기록이 없어 충돌 순간의 세부 원인이나 모터 배선 결함을 확정하지 않는다. 먼저 실제 통로에 맞는 경유 경로와 차체 회전 공간, 상판 점유 표현을 맞춰야 한다. 단순히 후진 거리만 늘리면 기존 pre점으로 다시 접근할 수 있다. **충돌 경로·킵아웃 수정은 아직 적용하지 않았다.**

**증거·종료 상태:** `operator_collision_report.json`, `cycle_events.jsonl`, `result.json`, `after_collision_dashboard_state.json`, `collision_route_analysis.json`을 같은 run에 보존했다. 정지 후 선속도·각속도 0과 spool request/running 0개를 확인했다. 카메라 정리 실패 뒤 관측기만 남아 있었으며 정확한 `jdamr-box-observer`·`jdamr-box-rgbd` 유닛을 정리해 둘 다 inactive/PID 0을 확인했다(`post_stop_services.txt`, `post_stop_camera_cleanup.txt`). idle 실행기 PID는 44888이며 사용자 대시보드·베이스·Nav2는 유지한다. 종료 명령은 §22.5와 같다. 상위 작업은 `stopped_by_operator_after_collision`, 실물 성공 false, 자동 재개 미승인이다. 충돌 뒤 위치를 도크로 덮어쓰거나 추가 주행·원격 push를 수행하지 않았다.


### 22.9 도크 배치 해석·직진 이탈·사륜 외곽과 비이동 준비

**배치 계약:** 사용자가 도크에 뒀다고 알리면 후면은 테이블, 전면은 통로를 보는 배치다. 방향 질문을 반복하지 않는다. 이전 테이블 위치추정으로 제출했던 관측점 재개 `table_01_20261016_140332_2c793e23`은 배치 정정 후 취소했으며 이동 승인 이벤트는 0개였다. `operator_location_correction.json`에 보존했다. 등록 도크를 다시 가르치지 않았다.

**준비 수정 (`6d509b9`):**

- 경로의 `dock_departure_distance_m`은 기존 이탈 거리 0.7 m를 사용한다. 출발 정지 heading에서 직선을 만들고 odom에 동결해 `FollowPath(Parking)`로 이탈을 마친 뒤 나머지 경유점을 보낸다. 성공한 이동 뒤 입력 공백에서는 정지 확인만 재시도한다. 정지·입력 공백·후속 transit 재시도에 완료 구간 재전송이 없도록 했다.
- 1번 pre를 `(0.858,-1.270)`으로 옮겼으며 관측점 `(0.858,-1.817,yaw=0)`은 유지했다. 이전 pre `(0.438,-1.817)`가 급수대 관측 직선에 약 6 cm로 붙었던 경유 문제를 수정했다. 이 비교와 새 경로 회전 외곽 검사는 지도·관측 기하 근거이며 실측 상판 외곽이나 충돌 없는 반복 주행 증명이 아니다. 킵아웃·도크·1 cm 목표는 변경하지 않았다.
- 양팔 설계가 사륜이어도 실행 URDF에는 후방 caster와 이륜 외곽이 남아 있었다. 실행 후륜축 `x=-0.210,y=±0.255,z=0.0329 m` 및 전륜과 같은 바퀴를 반영했다. wheelbase 0.210 m는 사용자 대칭 배치와 설계값의 전륜축 원점 변환으로 계산한 값이며 새 줄자 실측이 아니다. 기존 패딩을 유지해 local/global footprint와 회전·후진 StopZone에 후륜을 포함하고 양쪽 costmap의 외곽 검증을 보강했다. 라이다·카메라 extrinsic은 바꾸지 않았다.
- 해당 단계 관련 7개 스위트 833건, standalone/runtime 23건, 변경 Python compile·diff 검사가 통과했다. 코드 검토가 찾은 완료 이동 재전송과 global-only 옛 외곽 허용을 수정한 뒤 구체 결함 0건을 확인했다. LSP 검증은 실행하지 못했으며 이를 검증 완료로 포함하지 않는다.

**Pi 배포와 초기화:** `$HOME/.local/state/bimanual-robot/dock_departure_4wd_deployment.json`에 14경로 SHA와 native readback을 보존했다. Pi 백업은 `$HOME/.local/state/jdamr/dock-departure-4wd/20261016_142845/pi/`, PC 백업은 같은 시각의 `pc/`다. inactive/dead·MainPID 0인 정확한 transient Nav2 unit fragment를 재확인·백업 후 정리해 재기동했다. 이 한 건의 복구이며 향후 자동 정리 기능을 구현한 것은 아니다.

- 첫 비이동 `init_20261016_140433`은 inlier 18.46%·전역 불일치로 실패했다. beam/laser TF 변경이나 180° 반전 근거는 없었고, 지도에 없는 후방의 일관된 scan 점 군집이 관측됐다. 물체의 정체나 초기화 실패 전체 원인은 확정하지 않았다.
- 베이스/라이다 재기동 뒤 `init_20261016_143746`의 fresh 40스캔은 inlier 45.45%·전역 정합 일치였다. 비이동 활성화 정지 확인과 `command_observed=false`가 남아 있다. 지도·등록 도크·마스크·임계값을 바꾸거나 pose를 강제하지 않았다. 정상화의 세부 원인을 재기동 하나로 확정하지 않는다.
- 최신 native readback은 양쪽 costmap 외곽 일치·후륜 포함, rear wheel joint 2개·caster 0개, controller/planner active로 PASS다. 초기 활성화 전 local costmap RPC 실패는 별도 필드에 보존했으며 현재 readback 실패로 해석하지 않는다. 준비 시점에는 주행 요청을 보내지 않았다.

### 22.10 도크 출발 단일 전역 검사 종료 수정과 후속 실차 결과

**출발 거부의 관측과 수정:** 사용자 출발 요청의 `table_01_20261016_144259_95104796`은 `reverse_path_checked(valid=false, invalid_pose_indices=[])` 뒤 종료됐다. 도크 직진 이동 승인 이벤트는 없었다. 같은 정지 TF `(0.130297,0.135227,0.096042 rad)`에서 재확인한 0.7 m 경로의 29개 지점은 모두 true였다. 거부 순간의 장애물·costmap 변화는 기록이 충분하지 않아 false positive로 단정하지 않는다. **한 번의 전역 snapshot 결과만으로 새 도크 이탈을 종료한 소스 분기**는 확인했다.

- `_depart_dock_straight()`의 중복 전역 `IsPathValid` 선검사만 제거했다. 정적 map·keepout의 전체 swept footprint, FollowPath의 기존 센서·링크·정지 확인, 실시간 Collision Monitor는 유지한다. 다른 후진·도크 진입의 검사는 이번 변경 범위가 아니다.
- 표적 회귀 11건·독립 verifier의 두 파일 diff 검토·diff 검사가 통과했다. 전체 스위트를 다시 실행했다고 주장하지 않는다. Pi 소스·설치 `restaurant_service.py` SHA는 `29a5fb7342d865ff8e23ddf68299c1a77764510669bef3f4c6ae1abd69689216`으로 일치했다. 백업은 `$HOME/.local/state/jdamr/dock-departure-snapshot/20261016_144628/`이며 베이스·Nav2·위치추정을 재시작하지 않고 idle 실행기만 재기동했다.

**사용자가 요청한 주행의 진행:**

| run (`$HOME/jdamr_data/map_20260930_manual/aligned/runs/` 아래) | 결과 |
|---|---|
| `table_01_20261016_144647_05015f5a` | 도크 직진 이탈·급수대 주차/이탈·1번 접근을 실행했으나 양쪽 간격 판정 미달, code 1 |
| `table_01_20261016_145049_a85b11e2` | 기존 resume-parked 경로로 1번 후진 이탈 완료, 도크 nominal path의 전역 검사 false로 종료 |
| `table_01_20261016_145234_3972a6e1` | 재확인한 도크 경로가 전부 true인 뒤 기존 dock-only 재개, 후진 진입·지도 오차 1회 재접근 후 지도 위치 허용오차 초과, code 1 |

- 첫 재출발에서 `dock_departure_motion_complete`·`dock_departure_finished`가 다음 transit 승인보다 먼저 기록됐다. 도크 출발의 단계 분리는 실차로 실행됐으나 전체 충돌 방지·정밀도 반복 성공을 보장하지 않는다.
- 급수대 내부 추정은 좌/우 9.014/8.856 mm이며 접근·정지·대기·이탈이 기록됐다. 1번은 좌/우 14.664/8.797 mm·면 방향 오차 0.747°로 양쪽 간격 기준에 미달했다. 원시 corner 배열은 우측·좌측 순서다. 대화에서 순서를 뒤집어 말한 것을 정정했다. `final_depth_measurement=false`인 동결 면·odom 추정이며 외부 실측 성공이 아니다. 가까운 모서리가 목표 안쪽이라 추가 전진으로 해결하지 않고 기존 이탈/복귀를 사용했다.
- 깊이 입력 시각 공백은 같은 목표를 유지한 정지·재개로 회복했다. 입력 지연 자체가 해결됐다는 뜻은 아니다.
- 복귀 전역 검사도 나중의 whole path와 모든 단일 지점은 true였다. 단일 거부의 세부 원인은 미확정이며 복귀 쪽 선검사 종료는 이번 소스 수정에 포함하지 않았다.
- 최종 도크 정지는 odom 오차 14.05 mm·0.149°로 확인됐지만 지도 오차는 66.83 mm·2.334°였다. 최초 진입의 지도 위치 오차 184.08 mm 뒤 기존 1회 재접근이 실행됐고, 그 뒤에도 성공 판정에 미달했다. 좌표를 덮어쓰거나 허용오차를 늘려 성공으로 만들지 않았다. 상위 결과는 **부분 실행·도크 근처 정지, 정확한 도크 도착 미확정**이다.

**종료 근거와 유지 서비스:** 마지막 run의 `cycle_events.jsonl`, `final_dashboard_state.json`, `post_run_summary.json`, `retained_pi_services.txt`, `retained_pc_services.txt`에 근거를 보존했다. fresh odometry age 0.02 s·선속도/각속도 0, spool request/running 0개, 카메라·관측기 inactive/PID 0이다. 자동 재개는 미승인으로 기록했다. 사용자가 보던 대시보드와 다음 명령에 필요한 기존 세션은 유지한다.

| 서비스 | PID | 유지 이유·종료 명령 |
|---|---:|---|
| PC `robot-dashboard`, `robot-dashboard-link` | 1212114, 2496089 | 사용자 조작 화면; `systemctl --user stop robot-dashboard robot-dashboard-link` |
| Pi `jdamr-ammr-dashboard`, `jdamr-box-executor` | 4251, 54265 | 화면 backend·idle 명령 실행기; `sudo systemctl stop jdamr-ammr-dashboard jdamr-box-executor` |
| Pi `jdamr-restaurant-navigation` | 51198 | 기존 Nav2 세션; PC에서 `python3 "$HOME/bimanual-robot/tools/jdamr/jdamr_depart.py" session-stop` |
| Pi `jdamr-base` | 50055 | 베이스 센서·라이다; Pi에서 `sudo systemctl stop jdamr-base` |

표시 중계·RViz·기록기는 띄우지 않았다. PC와 Pi 대시보드를 종료할 때는 위 짝을 함께 종료한다. 새 인계 문서·원격 push는 수행하지 않았다. 남은 문제는 1번의 양 모서리 1 cm 미달, 도크 map/odom 불일치, 복귀 전역 단일 검사 종료와 입력 지연이다.

**후면에 거리가 남았다는 사용자 보고 후 정지 진단:** 등록부의 `rear_gap_measurement.gap_m=0.03`은 `scope="taught placement; no closed-loop rear range measurement"`인 사용자 배치 기록이다. 현재 후면 도킹은 테이블 간격 측정 피드백이 아니라 등록 지도 좌표와 동결 odom 목표를 사용한다. 낮은 라이다의 차체 폭 후방 띠에서 3스캔의 가까운 반사체는 rear 기준 약 0.987–0.997 m였으며 상판 간격으로 해석하지 않는다. 지도 목표–현재 pose를 차체 축으로 투영하면 전방 약 19.56 mm·좌측 약 63.91 mm여서 지도 오차 66.83 mm를 추가 후진 거리로 사용할 수 없다. 진단은 이동 없이 수행했고 `post_return_rear_gap_diagnostic.json`에 보존했다. 실제 뒤쪽 간격이 남았다는 보고를 존중하며 도킹 미완료 상태를 유지한다.


### 22.11 주차·후진 뒤 지연과 도크 반복 동작의 분석·수정

**범위:** 사용자가 1번 테이블 주차 후와 후진 뒤 긴 지연, 도크 전 과도한 회전·전후진을 수정하도록 요청했다. 로봇은 §22.10의 도크 근처 정지 상태로 유지했다. 뒤이은 상용 서빙로봇 비교 질문은 같은 수정 작업의 설계 설명으로 처리했으며 새 실물 주행을 요청한 것으로 해석하지 않았다.

**실제 시간 분리:** §22.10의 3개 run `cycle_events.jsonl` wall time과 PC `command.txt` 생성 시각을 대조했다.

| 구간 | 시간 | 확인한 의미 |
|---|---:|---|
| 1번 최종 정지→카메라 정리 종료 | 2.733초 | 실패 판정·정리 |
| 그 마지막 이벤트→후진 재개 명령 생성 | 89.491초 | 에이전트/PC 로그 확인·재개 공백 |
| resume 시작→dwell 완료 | 3.059초 | 기존 2초 대기와 정지 확인 |
| 후진 결과→이탈 확인 | 2.268초 | 정지 자세·면 이격 확인 |
| 이탈 확인→복귀 실패 통보 | 0.422초 | 단일 전역 경로 거부; 긴 native 정지는 아님 |
| 그 마지막 이벤트→dock-only 명령 생성 | 69.833초 | 에이전트/PC 재개 공백 |
| 각 command 파일→첫 Pi 이벤트 | 7.259/9.849/2.945초 | 요청 전달·실행 준비 변동 |

기존 `visit_observed_box=False`는 `if ok and return_home` 분기로 복귀를 건너뛰었다. 다음 이탈과 복귀는 별도 PC 명령이 필요했다. 이 공백을 로봇이 90초 동안 센서를 안정화했다고 설명하지 않는다. 제출 전 35초/5초 차이는 invocation 시작·각 SSH 단계 시각이 없어 전체 원인을 확정하지 않았다. 제출 전 순차 SSH와 이후 별도 이벤트/결과 SSH·10초 polling은 통신/완료 인지 비용이며 이번에 무작정 polling을 늘려 Pi 부하를 추가하지 않았다.

**도크 움직임의 실제 전이:** staging에 도착한 뒤 약 -67.3° Spin, 약 9.9 cm 옆 오프셋을 흡수하는 곡선 후진, +10.23° 입구 보정, 직선 후진이 이어졌다. 첫 후진 말기 +5.36° 크기의 heading 잔차가 기존 5° 분기를 넘자 전진 이탈→입구 반대 보정→후진이 실행됐다. 그 뒤 최초 지도 오차 184.08 mm로 전체 지도 재접근이 발생했고 재접근에서도 추가 회전·곡선·직선·마지막 trim이 있었다. 약 111초의 복귀 중 대부분은 이 실제 동작과 확인이었다. 작은 단계가 누적되는 구조, 곡선/직선 RPP의 끝 heading, odom/지도 불일치가 구분되는 문제이며 무작위 주행으로 단정하지 않는다.

**적용한 수정:**

1. Fine table의 양수·유한 양쪽 간격 중 가까운 쪽은 목표 부근이지만 먼 쪽이 미달이면, 가까운 상태에서 더 전진하지 않고 기존 면 기준 후진 이탈→새 면 관측→정렬·접근을 같은 visit/deadline에서 1회 수행한다. 동결 면으로 정지했으면 그 면도 odom 좌표로 보존한다. 관측을 잃거나 이탈을 확인하지 못하면 새 접근을 보내지 않는다.
2. 재차 간격 판정에 미달하더라도 정지·양수 간격·유효 입력이 확인된 불완전 정지에 한해 기존 이탈/복귀를 같은 실행에서 수행한다. `incomplete_parking_return_*`에 분리 기록하고 정밀주차 실패와 종료 코드 1은 유지한다. 운영자 정지·링크 소실·센서 불량을 자동 복귀로 확대하지 않는다. 정상 복귀와 같이 RGB-D/관측기를 끈 뒤 지도 복귀를 수행한다.
3. 입구 heading은 기존 감쇠 `_rotate_in_place`로 맞추고 정지한 뒤 재측정한다. 기본 `ParkingReverse`의 마지막 직선은 기존 `_drive_zero_turn(precision=True, hold_heading=True)`를 재사용한다. RPP의 횡방향 추종 조향을 마지막 직선에서 제거하며, 원래 동결 odom 목표와 기존 2 cm position checker 범위·home 계약은 유지한다. 입력 공백 뒤 새 전체 거리를 더하지 않고 같은 목표의 남은 거리만 재개한다. 선택 가능한 `GracefulReverse`는 기존 FollowPath 동작을 유지한다.
4. 모든 해당 명령은 기존 `cmd_vel_nav→velocity_smoother→Collision Monitor→base`를 거친다. guard/link/stop/timeout/정지 미확인 원인을 latch해 정지 뒤 우연히 입력이 회복돼도 성공으로 오판하지 않는다. 회전 중 링크 소실과 stale 입력 뒤 재회전 문제는 독립 검토에서 찾아 보강했다. 각 입구/최종 trim은 입력·링크 상태를 다시 확인한다.
5. 전역 후진 경로 snapshot이 false일 때는 정지 상태에서 기존 1 Hz costmap의 업데이트 두 번에 걸쳐 최대 3회 재확인한다. 정적 map/keepout 실패는 RPC 전에 거부하고, 재확인 중 stop/link/guard 실패면 대기를 끝낸다. 지속 장애물을 무시하거나 기존 최종 목표·1 cm 계약을 완화하지 않았다. 기존 도크 지도 확인·재접근 상한도 유지한다.

**검증:** box/restaurant/input recovery 665건, reverse/parking contract/parking integration/standalone 139건, Python compile·diff 검사가 통과했다. 그 뒤 카메라 종료 1줄을 보강하고 fallback 8건을 다시 통과했다. 독립 verifier는 최종 두 소스 SHA와 8건을 확인해 승인했다. 가상 clock/odom/센서/명령 plant에서 양쪽 gap 재접근 1회·새 관측 소실·기존 deadline, 절대 후진 목표를 보존한 scan 복구, 회전/후진 중 운영자 stop·링크 소실, 지속/일시 경로 거부를 검증했다. plant는 이상적 이동 모델이므로 실제 4WD 마찰·미끄럼·외부 1 cm 정확도는 증명하지 않는다.

**배포·정리:** Pi source/install 4경로 SHA와 모듈 import, 실행기 ready를 확인했다. box SHA는 `e4bdad507f8dd76ec9887c2711fe2d02753656cc69fe1d92166db3f145bb981a`, restaurant SHA는 `51c5fa8e86a90584e13155f10b2038607c3ae72efb29c8fe7e9463e7bcd228d8`다. 첫 백업은 `$HOME/.local/state/jdamr/smooth-docking/20261016_152043/`, 최종 백업은 `20261016_152421/`다. PC receipt는 `$HOME/.local/state/bimanual-robot/smooth_docking_deployment.json`이다. 배포 helper의 import 명령 shell 인용 오류와 이미 정지해 unloaded된 unit의 stop 응답 처리를 바로잡았고, stdin import PASS 뒤 idle 실행기를 재시작했다. 베이스·Nav2·도크·지도·마스크·카메라 장착은 바꾸지 않았다.

- 배포 뒤 fresh odometry age 0.01초·선속도/각속도 0, spool request/running 0, RGB-D/관측기 inactive/PID 0이다. 이동 요청은 없었다.
- 유지 PID: Pi base 50055, Nav2 51198, idle executor 61193, dashboard 4251. PC dashboard/link는 §22.10과 같은 1212114/2496089다. 사용자 화면과 다음 명령용 세션이므로 유지했으며 종료 명령은 §22.10과 같다. 새 상주 진단·중계·기록기는 없다.
- 현재 위치는 도크 근처·정확한 도크 도착 미확정, 자동 재개 미승인이다. 수정 뒤 실차 재주행·성공 주차·원격 push를 수행하지 않았다.

**상용 로봇 비교와 남은 한계:** [Nav2 도킹 공식 절차](https://docs.nav2.org/rolling/tutorials/general_tutorials/using_docking/)는 staging 뒤 검출한 목표 pose를 갱신하는 접근 제어와 접촉/충전 확인을 설명한다. [Rotation Shim 공식 설명](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_rotation_shim_controller/)은 heading을 고려하지 않는 NavFn 계열과 운동학 경로를 만드는 planner의 차이를 설명한다. 현재 코드가 Rotation Shim 플러그인을 사용한다는 뜻은 아니다. [BellaBot 제조사 자료](https://www.pudurobotics.com/product/detail/bellabot)는 LiDAR·visual SLAM과 RGB-D 등 다중 센서를 제시한다. 공개 제품 자료만으로 테이블 앞 1 cm 정렬 성능을 보증한다고 해석하지 않는다.

현재 전면은 근접 상판 경계가 가려진 뒤 동결 면/odom에 의존하며 후면은 상대 간격 센서가 연결돼 있지 않다. 베이스 드라이버는 `ds=(dl+dr)/2`, `dth=(dr-dl)/wheel_separation`으로 odom을 적분하고 IMU는 별도 토픽으로 발행한다. 이번에 IMU 융합·회전 보정값·기구 제원은 바꾸지 않았다. 4WD의 실제 회전 미끄럼과 전륜축 runtime frame에 대한 운동학 기준점은 추가 확인할 항목이며, 이 하나가 현재 지도 오차 전체 원인이라고 확정하지 않는다. 눈에 띄는 단계 전환과 반복을 줄이는 것은 소프트웨어로 가능하지만, 끝까지 목표를 관측하는 센서 배치와 실제 운동 오차를 해결하지 않은 채 상용 수준의 연속 동작·1 cm 정확도를 달성했다고 기록하지 않는다.


### 22.12 도크 시작 뒤쪽 keepout 겹침의 직진 탈출과 연속 접근 후보 보류

**관찰:** `table_01_20261016_161625_82b528c7`은 이동 전 `dock_departure_path_checked=false`로 끝났다. 정지 map pose `(0.111854, 0.098619, 0.175672 rad)`와 외곽 `x=-0.28..0.07, y=-0.275..0.275 m`에서 최초 위치만 keepout black cell(column70, map_row110, world bounds x=-0.20..-0.15/y=-0.05..0.00 m)과 겹쳤다. 전방 0.025..0.700 m의 28개 pose는 free였다. 이는 뒤쪽 기존 겹침을 향후 진입 장애물과 구분하지 못한 출발 거부이며 전방 실물 장애물이 확인됐다는 뜻이 아니다. PC의 해당 run `departure_static_diagnostic.json`에 정지 진단을 보존했다.

**조치:** `static_corridor_clear(..., allow_rear_keepout_escape=True)`를 도크 직진 이탈 한 호출에서만 사용한다. 고정 yaw·측방 변위 0·단조 전진을 만족하고, 최초 외곽과 겹친 셀 중 keepout negate=0 black cell이며 셀 전체가 시작 base 뒤쪽이고 최종 외곽이 완전히 벗어나는 셀만 면제한다. source map의 occupied/unknown, unknown keepout, 앞쪽/새 keepout, 회전/후진, 최종 겹침은 거부한다. 기본 corridor 검사와 live Collision Monitor는 유지한다.

**검증·배포:** 기존 static/departure 17건·신규 8개 경계 사례 통과, 독립 reviewer의 고정 후보 2개 승인을 받았다. 배포 SHA는 restaurant `b8497fdd42a10f6ce8fca083600512666d0080a7331cebfe6019b338cac71904`, reverse `3e1cabbd1865dfe247c66d41f9b0790f7d3df9b331b2aaf72a89507d57a2942b`다. Pi source/install 4경로 일치·import·실행기 PID64151 active를 확인했다. 복원본 `$HOME/.local/state/jdamr/departure-rear-overlap/20261016_162145/`, PC receipt `$HOME/.local/state/bimanual-robot/departure_rear_overlap_fix/deployment.json`. Nav2·AMCL·지도·keepout·센서·속도는 바꾸지 않았다. 이어 제출한 `table_01_20261016_162157_2dce7549`은 도크 0.7 m 이탈과 급수대 관측 위치 도착을 완료했으며 이 항목 작성 시점에는 진행 중이다.

**배포하지 않은 후보:** 이동 중 책상 면 heading 추종·원형 필터·전진 감속과 0.24 m/s 통로 속도 후보는 관련 948건을 통과했지만, 독립 검토에서 계획 경로와 실제 곡선의 정적 keepout 검사 불일치를 발견해 배포하지 않았다. 실제 nominal 곡선/외곽 검사 및 5 cm 이전 heading 수렴 보완이 남았다. 0.24 설정의 live monitor 재구성도 미완료다. RGB는 현재 주차 서비스에서 비활성이며 RGB 영역 분할과 동기·좌표 정합을 거친 깊이 결합은 향후 항목으로 구분한다. 이 후보를 현재 실차에 반영한 연속 제어·RGB 주차 성공으로 기록하지 않는다.

**후속:** `162157_2dce7549`는 급수대 면 관측이 불안정한 뒤 탐색 기준 정지 pose 캡처에서 AMCL age24.302초·quiet exemption dyaw0.056rad로 종료됐다. 도크 이탈은 완료했으므로 반복하지 않았다. 기존 `/request_nomotion_update` 서비스 응답을 확인하고 `162425_234ffa1b`를 `--with-via --resume-at-observation`으로 제출했다. 카메라 준비와 급수대 관측 위치 재개까지 확인했으며 주차/복귀 결과는 아직 미확정이다. 새 pose를 만들어 발행하거나 도크로 위치를 덮어쓰지 않았다.


### 22.13 고정 급수대의 역방향 탐색·관측 대기와 도크 재접근 원인

**실행 결과:** `table_01_20261016_162425_234ffa1b`는 급수대 관측점 재개부터 급수대·1번 주차와 후진 도크까지 292.6369초에 실행기 완료했다. `cycle_events.jsonl`, `final_dashboard_state.json`, `post_run_summary.json`은 `$HOME/jdamr_data/map_20260930_manual/aligned/runs/` 아래 같은 run에 있다. 급수대/1번 우·좌 내부 간격은 각각 8.836/12.341 mm, 8.754/12.767 mm다. 최종 근접 깊이·외부 줄자 계측은 없어 실제 1 cm 성공으로 쓰지 않는다. 사용자는 긴 관측 대기, 급수대 반대 방향까지 탐색, 큰 이탈 회전과 도크 반복 동작을 보고했다.

**관측과 확정 결함:**

1. 급수대 도착 map yaw -179.20°와 등록 yaw -180°의 차이는 -0.798°였다. 그러나 영역 중심 bearing +2.377°가 8°보다 작으면 기존 탐색 예산이 기본 +30°를 골랐다. 실제 odom +33.202°/map +36.299° 회전 뒤 안정 면에 맞추려고 -32.629°를 되돌았다. 고정 지점에서 무방향 탐색을 재사용한 결함이다.
2. 정지 중 마지막 16개 관측에서 보이는 경계 구간의 중심은 64.902 mm 움직였지만, 무방향 3D 직선의 수직 offset 차이는 최대 2.378 mm, 방향은 0.16754°, 상판 높이는 0.493 mm였다. 중심 lateral 30 mm 검사 때문에 안정 프레임이 7개까지 쌓였다가 1개로 초기화됐다. 카메라 준비 11.507초와 관측 window 13.586초도 있었으며 모든 대기를 하나의 CPU 문제로 단정하지 않는다.
3. 도크 최초 odom 오차 10.138 mm/0.248°는 통과했지만 같은 목표의 map 오차 94.815 mm/4.374°가 미달해 전체 이탈·재접근했다. 최종 map 오차 31.305 mm/1.942°는 기존 계약을 통과했다. 도크 staging odom 회전은 첫 요청 -65.690°에 -68.786°, 다음 요청 -16.250°에 -19.345°로 각각 약 3.10° 더 돌았다. 상대 yaw 차이는 일정한 map↔odom 회전에서 불변이므로 이 호출 하나를 좌표계 혼합 원인으로 기록하지 않는다. 실제 바퀴 미끄럼도 이 자료만으로 확정하지 않는다.

**소프트웨어 조치:** table_edge 고정 목표는 등록 heading으로 최대 한 번·30° 이내 탐색하며 이미 방향이 맞으면 같은 heading으로 한 번 20 cm 후진해 재관측한다. 표적이 있는 반대 방향으로 무제한 원형 탐색하지 않는다. table_edge 안정성은 endpoint 중심의 일치·무방향 3D 직선 offset과 방향·상판 높이를 비교하고 기존 투영각·유효한 평면 법선 검사는 유지한다. 마지막 16프레임 회귀 자료는 `test/fixtures/table_edge_stability_20261016.json`이다.

전면 접근은 stamped odom 면과 원형 필터 heading을 사용해 전진·각도 보정을 함께 수행하고 기존 5 cm 정밀 구간 전에 0.3°로 수렴하도록 감속한다. 실제 command의 필터 상태·속도 법칙과 같은 nominal rollout을 검증한다. 너무 가까운데 기울었으면 한 번 이탈·재관측·재접근한다. 정지·링크·오래된 센서 실패를 무조건 후진 복구로 취급하지 않는다. 상용 주행 정확도나 실제 마찰 성능을 이상적 rollout 테스트로 증명하지 않는다.

도크에서는 정지·유효 odom·연결 상태에서 기존 `/request_nomotion_update`를 한 번 호출해 새 AMCL 메시지를 받은 뒤 같은 지도 목표·허용오차를 재판정한다. 계속 미달이면 기존 한 번의 전체 재접근을 유지한다. staging의 큰 Spin은 기존 감속 odom 회전으로 교체한다. 초기 pose·map/odom TF·도크 등록·keepout·외곽은 바꾸지 않는다. 속도는 현행 0.12 m/s다.

**미해결 범위:** 급수대 이탈 후 pre `(0.858,-1.270)`까지 명목 방향 차이는 약 145°이며 pre→관측점→최종 방향에도 꺾임이 있다. 이전 `(0.438,-1.817)`은 실제 충돌 뒤 폐기한 경유점이므로 회전 수를 줄이려고 되돌리지 않는다. 저층 라이다 keepout과 관측 상판 돌출 표현 차이, 실제 출구 궤적 자료의 한계 때문에 현재 변경으로 큰 반경의 통로 경로가 모두 해소됐다고 판정하지 않는다. 경로 접선·차체 외곽과 상판 영역을 함께 반영하는 후속 경로 개선이 남아 있다. 후면 상대 간격 센서도 없어 후면 0 cm 주차는 확인할 수 없다.


**검증·비이동 배포:** 관련 10개 테스트 파일 1,058건 통과(21.43초), 독립 6개 스위트 817건 통과(14.30초), production 5개 모듈 ament_flake8·AST·diff 검사가 통과했고 독립 리뷰는 PASS다. 초기 후보에서 새 깊이마다 정확 corridor 전체를 동기 검사해 PC 약 1초, 첫 캐시 후보도 Pi 30° 자세에서 p95 168 ms를 썼다. 최종본은 지도를 한 번 읽고 전체 swept footprint를 16방향 외접 support polygon과 최대 회전 sagitta로 포함시킨다. clearly-free 셀은 SAT를 생략하고 unknown/blocked/outside는 유지한다. 최종 Pi 실제 지도·외곽의 합성 각도별 30회 전체 callback p95는 -30/-12/0/+12/+30°에서 43.83/23.88/18.89/24.30/43.17 ms, 전체 최대 88.05 ms였다. 이는 계측한 조건의 처리 시간이며 실제 주행 성능이나 모든 시스템 부하에서의 지연 상한을 보장하지 않는다.

소스·설치 5모듈 10경로 SHA 일치와 import, idle 실행기 PID70616 active, spool request/running 0을 확인했다. 백업은 `$HOME/.local/state/jdamr/continuous-fixed-station/20261016_171004/`, PC receipt·성능 결과는 `$HOME/.local/state/bimanual-robot/continuous_fixed_station/`이다. Nav2 PID51198·베이스 PID50055는 재시작하지 않았고 속도 0.12 m/s를 보존했다. 카메라·관측기는 inactive/PID0이며 새 주행은 보내지 않았다. 배포 후 대시보드 fresh odom 선속도0·각속도0, 수신 나이0.02초를 확인했다. 유지 서비스와 종료 명령은 PROGRESS 최상단에 있다. 기존 깊이-gap 대기 중 operator link가 끊기면 정지를 유지하며 즉시 반환하지 않고 상위 timeout까지 기다리는 동작은 후속 MEDIUM으로 남겼다. 이번 새 제어의 실차 재주행과 정확도 검증, 큰 반경 transit 경로 개선은 아직 남아 있다.


### 22.14 후면 하단 박스 라이다 주차와 외곽 근사·각도 잡음 수정

**요청·관측:** 사용자가 도크 뒤 라이다 높이에 박스를 배치하고 1 cm 후면 주차를 요청했다. 노트북 Wi-Fi를 aicampus_287에서 기존 aicampus_286으로 복원한 뒤 Pi 192.168.0.159에 연결했다. 정지 스캔 12장 모두 최근접 후면 층에서 58–62개 점·약 20–21 cm의 면을 검출했다. 중앙 간격 11.58–11.91 mm와 면 각도 -2.21–-1.91°로부터 차체 양 뒤 모서리에 연장한 최소 간격은 3.20–4.12 mm였다. 라이다 기반 추정이며 외부 실측은 없다. 원본은 PC `$HOME/.local/state/bimanual-robot/rear_box_parking/scan_20261016_174326.json`이다.

**구현:** 저장소 루트에서 `python3 tools/jdamr/jdamr_depart.py go table_01 --rear-box-only --final-gap-m 0.01`을 사용한다. 기존 상주 실행기에 후면 박스 단독 모드를 더했고 테이블 경유·등록 HOME 복귀·깊이 카메라는 실행하지 않는다. stamped scan/odom 면을 추적하며 기존 precision 속도 경로로 후진한다. 가까운데 기울어진 초기 배치에서는 검증된 직진 이탈을 한 번 사용한다. 실제 차체 제원·정적 지도/keepout·Collision Monitor·링크/정지/입력 검사는 유지한다. 첫 승인 odom 평면에 대한 변화 검사로 먼 벽·다른 박스와의 대상 교체를 막는다.

**실행별 관찰·원인·조치:**

1. `table_01_20261016_180006_0311aada`: 약 20 cm 전진 이탈 뒤 후진 0 m에서 정적 검사 실패. 계단형 실제 외곽 대신 후진용 bounding rectangle을 사용한 과대 범위가 확인됐다. `_footprint_outline()`을 사용하도록 수정했다. runtime padding이 있으면 기존 보수적 fallback을 유지한다.
2. `table_01_20261016_180401_03e297fb`: 이탈 반복 없이 약 5.4 mm 후진한 뒤 정적 검사 실패. 실제 지도·자세·각도 조합 8개에서 cached=False/exact=True를 재현했다. 고정 world 16축 외접 polygon에 시작·끝 실제 convex footprint 변의 법선을 추가하고 전체 swept point의 최대 support를 사용했다. exact sweep 포함과 sagitta padding은 유지하며 8개 모두 cached/exact=True로 확인했다. Pi 80회 p95 13.65 ms·최대 16.47 ms다.
3. `table_01_20261016_181236_dff590ad`: 16.30 cm 후진해 약 2.7 cm까지 접근했다. raw 한 프레임 -0.309°가 기존 0.3° 정렬 기준을 넘어 종료됐다. 후면 stationary 관측은 마지막 프레임 대신 8프레임의 circular normal·signed offset 평균을 사용하고, 이동 중에도 같은 0.35초 법칙으로 heading/offset을 함께 필터링한다. 동일 filtered plane을 거리·명령·preview에 사용하며 raw 대상 변경 검사와 로그는 유지한다. 반복 stamp는 필터를 갱신하지 않고 timestamp 역행은 거부한다. 전면 주차의 기존 허용오차는 바꾸지 않았다.
4. `table_01_20261016_181732_6a4c955f`: 현재 위치에서 17.51 mm 후진한 뒤 `rear_box_parking_finished.confirmed=true`, CLI exit 0으로 완료했다. 양 뒤 모서리 내부 간격 11.398/12.419 mm·각도 0.12993°다. 목표 10 mm의 기존 ±3 mm 판정을 양쪽 모두 통과했으며, 실제 외부 정확도·반복성·충전 접촉 성공은 미검증이다.

**검증·배포:** 최종 관련 7개 스위트 766건을 통과했다. 최초 broad 검사에서는 ROS overlay를 빠뜨려 package index 조회 22건이 실패했고 overlay를 적용해 같은 스위트를 모두 통과했다. 독립 검토는 cached 검사 67건·무작위 swept point 포함성 2,000건, 최종 후면 40건과 lint를 확인해 PASS했다. Pi source/install의 최종 service SHA는 `2c9d1f25a2248f0f7ca46f48c997f52b411b6d2ed052fef14b389df6bd672fc8`, reverse SHA는 `cbd1a6a95a8a88ff266c51a14daa24f15f1571177ac3c2c6ede6dcf3b5d41093`다. 첫 전체 배포와 두 외곽 수정·최종 필터 배포의 receipt는 PC `$HOME/.local/state/bimanual-robot/rear_box_parking/`에 보존했다. 마지막 Pi 백업은 `$HOME/.local/state/jdamr/rear-box/20261016_181710/`다. 4개 run의 `cycle_events.jsonl`은 PC/Pi 양쪽 `$HOME/jdamr_data/map_20260930_manual/aligned/runs/`에 있다.

**최종 상태·잔여 범위:** fresh odom age 0.02초·선속도/각속도0, spool request/running0을 확인했다. 사용자 화면과 다음 요청을 위해 Pi base PID50055·Nav2 PID51198·idle executor PID75956·dashboard PID4251, PC dashboard/link PID1212114/2496089를 유지한다. 종료는 PC `systemctl --user stop robot-dashboard robot-dashboard-link`, Pi `sudo systemctl stop jdamr-ammr-dashboard jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. RGB-D/관측기는 inactive/PID0이며 새 상주 진단 프로세스는 없다. 이 박스 상대 접근은 단독 모드이고 기존 HOME 자동 복귀의 마지막 단계에 아직 연결하지 않았다. 0 cm 접촉·충전 전류 판정, 외부 정밀도·반복성, 급수대→테이블 transit 회전 개선은 별도 후속 범위다.


### 22.15 도크 복귀의 live 경로 거부와 후면 박스 상대 제어 연결

**관측:** `table_01_20261019_093818_f936fa34`는 급수대·1번 테이블·후진 이탈 뒤 nominal 도크 경로의 live IsPathValid false 3회로 끝났다. 정지 표본 검사에서는 70 cm staging만 false, 60 cm부터 HOME까지 true였다. HOME는 보존한 채 staging만 60 cm인 임시 등록부로 실행한 `table_01_20261019_094411_9445486d`는 첫 odom 정지 통과 뒤 지도 오차 164.15 mm/5.99°로 한 번 재접근했고, 이어 curve는 true·마지막 straight는 false 3회로 끝났다. indices는 빈 배열이어서 응답만으로 충돌 셀을 특정할 수 없었다.

**원인 구분:** 정지 상태의 global/local costmap clear와 새 scan 33개 뒤에도 HOME 직전 두 표본은 false였다. 갱신 뒤 HOME 외곽과 겹친 lethal 셀 중심은 x≈−0.125 m, y≈0.125/0.175/0.225/0.275 m이며 네 셀 모두 원본 지도·keepout은 free다. 후면 스캔은 차체 기준 x≈−0.855 m의 횡방향 면을 검출했다. 관측 박스 면과 과거 map HOME pose의 차체 영역이 겹치는 조건이므로 충돌 판정을 끄거나 오래된 지도 잔상으로 단정하지 않는다. 단독 후면 라이다 기능이 전체 HOME 복귀에 연결되지 않아, 박스 앞에서 멈춰야 하는 최종 목표를 과거 지도 좌표까지의 경로로 검사한 것이 운영 구성의 문제다.

**복귀 결과:** 기존 단독 모드 `table_01_20261019_094849_435c91f5`로 도크 박스 앞까지 이동한 뒤 정지했다. 이 당시 실행의 최종 양쪽 목표 ±3 mm 확인은 false였다. 이후 사용자가 확인한 1 cm 성공과 구분하는 과거 실패 기록이다. 도크 귀환 위치 도달과 1 cm 정밀 판정을 구분한다. 동일 전체 실행의 table_01은 내부 8.96/8.92 mm였으나 사용자는 실제 3–5 cm를 보고했다. 마지막 면 관측에서 정지까지 약 19초의 odometry 추적이 있어, 이 실행을 전면 1 cm 성공으로 기록하지 않는다. 카메라 보정·경계 위치·실제 이동 대 odometry 중 어느 원인인지는 이 자료만으로 확정하지 않는다.

**수정:** HOME의 명시적 `rear_parking_reference: lidar_box` 설정에서만 기존 staging→후면 LiDAR 추종→양쪽 gap 확인을 한 실행으로 연결한다. 지도 HOME 좌표/방향은 유지하며 staging offset은 0.6 m, 후면 목표는 전면 목표와 분리한 0.01 m다. 라이다가 관측하는 최근접 횡방향 면을 등록 도크 접근 위치에서 안정화하고 최초 평면을 유지한다. 개별 물체 ID를 인식하는 기능은 아니다. final 관측 미달은 최대 한 번만 같은 표적을 보정하고, 정지·통신·센서 실패는 재시도하지 않는다. 실제 외곽·정적 지도/keepout 검사·fresh scan·Collision Monitor는 유지한다. 이전 map 모드의 live 검사도 유지하고, 거부 단계/이유/indices/시도 횟수와 대기 중 중단 이유를 기록한다.

**검증과 남은 범위:** 작성 패스 관련 7스위트 합 884건 통과. 수정본의 새 실차 이동은 수행하지 않았으며 전체 왕복·후면 양쪽 1 cm·전면 사용자 관측과 내부 추정의 차이는 별도 실차 확인이 남는다. 원본은 PC/Pi `$HOME/jdamr_data/map_20260930_manual/aligned/runs/`의 위 3개 실행, 비용지도 갱신 전후·scan은 `table_01_20261019_094411_9445486d/costmap_recovery_snapshot.json`, PC 사본은 `$HOME/.local/state/bimanual-robot/dock_return_20261019/`다.

**독립 확인·비이동 배포:** 독립 verifier 관련 931건·compile·diff·HOME x/y/yaw 보존 확인을 통과했다. 테스트 import 정렬 신규 오류 1건은 수정했으며 패키지 전체의 기존 lint 31건은 별도다. Pi source/install 4모듈 SHA·import·registry 검증 뒤 idle executor PID217586으로 반영했다. 수집형 unit은 stop 뒤 사라져 일반 start 실패를 기존 생성기로 복구했다. base50055/Nav2 51198/dashboard4251 유지, RGB-D/observer inactive/PID0, pending/running0·fresh odom0이다. Pi 복원본은 `$HOME/.local/state/jdamr/dock-return/20261019_100047/`, PC receipt는 `$HOME/.local/state/bimanual-robot/dock_return_20261019/deployment_receipt.json`이다. 새 임시 상주 프로세스는 없다. 유지 서비스 종료는 위 §22.14와 같은 명령을 사용한다.

**전면 오차 조사 보완:** 마지막 면 이후 odom 약439.95 mm를 갱신 없이 이동했다. `no_table_edge_candidate` 8프레임은 거리와 무관하게 occlusion을 latch하고 후속 면을 무시하며, 나중 precision-band basis가 앞선 조기 동결 사실을 가린다. 물리 부족량을 모두 odom 축척으로 설명하면 약4.8–9.3% 과대계상에 해당하지만 실거리/odom 비율을 측정한 값은 아니다. 카메라/edge 편향도 배제되지 않았고 앞끝 기준34cm의 모델 불일치는 발견하지 않았다. 전면 정확도를 해결했다고 주장하거나 임의 bias를 적용하지 않는다.


### 22.16 근접 후면 박스에서 출발 실패와 도크 복귀의 표적·위치추정 불일치

첫 출발 `table_01_20261019_103426_4f6bd107`는 home_exit Parking FollowPath에서 105(진행 실패), FootprintApproach→StopZone, odom 변화 없음으로 종료됐다. 뒤 박스에서 멀어져야 하는 시작 이탈에 기존 감시형 forward escape를 연결했다. fresh 앞밴드·이동 중 scan 감시·finally CM 복구·기존 전체 static corridor와 같은 endpoint를 유지한다. 12건/production lint/diff/독립 검수 PASS 뒤 Pi source/install SHA bb10bfe340bf5bdd6b271236f69b6af9fdc126f2a995908ce0860b68c8cfc6ae, 복원본 `$HOME/.local/state/jdamr/dock-release/20261019_104008/`로 반영했다.

후속 `table_01_20261019_104016_588becba`는 직선 이탈과 급수대·1번 테이블·각 후진 이탈을 수행했다. 도크 staging 후 최근접 rear ROI 면은 gap1.363m/angle−18.2°여서0.6m 접근 한계로 미이동 종료했다. 도크 박스인 줄 알고 한계를 늘리지 않는다. AMCL 근처 stationary scan 최적 정합은 inlier21.8%, 전역 후보와 불일치해 실제 위치 확인이 필요하다. 새 위치를 임의로 가정하거나 HOME/AMCL를 바꾸지 않았다. 원본은 기존runs와 PC `$HOME/.local/state/bimanual-robot/dock_return_20261019/staging_recovery_1044/`에 있다. 사용자 후면 접촉 허용은 기록하되 전방 충돌·오래된 센서 운행 허가로 확대하지 않는다.

사용자는 후진하면 도크이고 박스가 후면 중앙에 있다고 확인했다. 최신 정지 scan8장의 후면 면 추정은1.445–1.452m, yaw약−13.3°에서−13.7°로 일관됐다. 지도 staging과0.6m 정밀접근 사이 중간 접근이 연결되지 않았으며 이 센서 면을 고유 식별한 상태는 아니다. 임시 map 중간 목표 `104912_cdf96bac`는 작은 전진 정렬 뒤 lateral0.0300249m로 거부됐고, `105104_fc5b070e`는 이미 staging에 있는 조건에서 새 최종 명령이 없어 후진 발행 전에 거부됐다. 중간·최종 확인 계약을 분리하는 로컬 실험은 회귀 실패로 원복했고 Pi에는 배포하지 않았다. 유효한 도크 이탈 패치만 유지하며 원복 뒤12건·production lint PASS다. 마지막 odom은v0/w0, executor active/요청 없음이며 도크 복귀 성공으로 기록하지 않는다. 최신 scan은 PC `$HOME/.local/state/bimanual-robot/dock_return_20261019/rear_scan_latest.json`에 있다.


### 22.17 원거리 도크 연결·일시 입력 실패·예측 감속과 경계 대기 수정

**현재 결과 정정:** 사용자가 1 cm 정밀주차 성공을 확인했다. 과거의 미달 간격을 현행 결과로 쓰지 않는다. 아래는 그 성공 이후 준비 단계에서 분석·수정한 실패이며, 새 수정본의 실차 성공이나 반복성을 증명하는 기록은 아니다.

**관측과 원인:** `104016_588becba`의 1.363 m 면을 최종 0.6 m 제어에 넘겨 이동 전에 종료했다. 사용자는 도크 박스가 후방 중앙이고 실제 간격이 1 m 이상이라고 확인했다. 따라서 당시 면을 다른 물체로 단정하지 않는다. 임시 연결본 `110756_5b302044`는 약 0.178 m 후진한 뒤 raw 표적 일치 실패로 종료했고, `111146_be520c46`는 5초 내 원거리 안정 8프레임을 모으지 못해 종료했다. 최종 정밀도 계약을 원거리 이동에 적용한 결합과 일시 입력 실패의 상위 실행 종료가 원인이다.

수동 후진은 odom 약 0.674 m 후 정지했다. Pi journal에서 11:15:42.522–11:16:06.283의 FootprintApproach 개입을 확인했으며 중간 StopZone/Slowdown 전환은 없었다. 당시 scan/bag가 없어 해당 점이 차체 반사인지 외부 장애물인지는 확정하지 않는다. 기존 면 제어의 remaining/4초 법칙은 마지막 20 cm부터 감속했고, 중간 stationary hold 1초도 경계마다 중복됐다.

**수정:** 원거리는 후방 통로의 유효 최근접점을 단일 점까지 보존해 관측하면서 같은 축으로 접근하고, 0.5 m 이내에서만 안정된 면을 확정한다. 0.5 m 구간·총 2 m 이동 범위, 짧은 구간을 포함한 정적 지도/keepout sweep는 유지한다. 안정 관측은 sliding window로 갱신하며 일시 scan/면 입력 실패는 같은 태스크·기존 deadline 안에서 정지 후 남은 구간을 재개한다. 후면 접근에만 StopZone과 속도·제동거리 기반 실시간 swept footprint 감시를 적용하고 일반 FootprintApproach를 복원한다. 복원 실패는 비상정지와 원래 오류로 전파한다. 면 제어 감속 시간은 0.2초, 중간 정지 hold는 기존 0.08초를 사용한다. 최종 확인과 서비스 체류는 별도다. CLI는 nonzero를 숨기지 않고 실제 동작 원인을 출력한다.

**검증·범위:** 최종 관련 8개 스위트 931건(13.93초), 생산 모듈 6개 lint·구문 검사와 diff 확인을 통과했다. 출발 helper lint는 기존 17건 그대로이며 신규 오류는 없다. 독립 검토는 추가 대상 157건과 scan 오류 복구 연결 10건을 통과해 PASS했다. 저장 정지 scan의 새 통로 거리는 접근 전 약 1.476–1.478 m, 후진 뒤 약 1.296–1.298 m로 관측 가능했다. 이번 준비 단계는 비이동이며, 전면 근접 가림의 odom 추적 오차·새 수정본 전체 왕복 시간과 반복 정밀도는 이 소프트웨어 검증으로 해결됐다고 주장하지 않는다. 원본과 배포 receipt는 PC `$HOME/.local/state/bimanual-robot/dock_return_20261019/`에 보존한다.

**비이동 배포:** Pi source/install 6쌍 SHA·compile·import가 일치했다. 복원본은 `$HOME/.local/state/jdamr/dock-continuity/20261019_115147/`, PC receipt는 `$HOME/.local/state/bimanual-robot/dock_return_20261019/dock_continuity_deployment.json`이다. idle executor PID231465·Nav2 PID51198·base PID50055와 사용자 대시보드는 다음 요청을 위해 유지하며 fresh scan/odom·속도 0·pending/running 0·StopZone/FootprintApproach 활성 상태를 확인했다. 임시 확인 노드는 종료했다. 유지 서비스 종료는 §22.14 명령과 같다. HOME/AMCL는 변경하지 않았다.


### 22.18 급수대 정지 오차에서 불필요한 자동 복귀·도크 중간점 최종 기준 적용

**관측:** `table_01_20261019_122616_83b101eb`는 도크 0.7 m 직선 이탈과 급수대 접근을 수행했다. 깊이 stale 두 번은 같은 목표로 회복했고 `precision_straight_finished.target_failure=null`이다. 최종 양쪽 내부 간격 3.407/3.019 mm로 10 mm 목표 ±3 mm 확인은 미달이었다. 이 작은 overshoot를 후진 보정하지 않고 `incomplete_parking_return_started`로 전환했다. 후진 이탈 뒤 `reverse_staging_planned`는 NavFn 성공/오류0이지만 endpoint 70.7107 mm > 50 mm로 거부됐고, `home_staging_not_planned`와 exit1로 종료됐다. 1번 테이블과 도크 복귀는 수행하지 못했다.

**수정:** lidar_box의 중간 staging에는 기존 transit endpoint 0.15 m 기준을 사용하고 실제 이동·방향 정렬 뒤 후면 제어로 잇는다. 최종 도크 목표/양쪽 ±3 mm 및 일반 map 도크의 계약은 유지한다. 전면 정렬이 맞고 양쪽 양수 간격이 목표보다 조금 좁으면 동일 면·동일 deadline·기존 보정 횟수 안에서 수 mm 후진으로 맞춘다. 정적 지도/keepout sweep와 기존 명령·센서·정지 감시는 유지한다. 기울어 양쪽을 동시에 허용대역으로 옮길 수 없으면 기존 1회 이탈·재관측을 사용한다.

**제동 근거와 한계:** 0.2초 taper가 마지막 10 mm에 집중된 점과 정지 후 약 7 mm 초과는 확인했으나 이 로그에는 실제 제동 계수 측정이 없다. 0.08 m/s² 합성 slow-coast 조건에서 기존 6.09 mm 초과를 재현했으며 이는 차체 캘리브레이션 수치가 아니다. 0.5초 taper로 마지막 25 mm에서만 감속해 같은 회귀를 ±3 mm 안으로 맞췄다. 나머지 이동 속도·정렬 상한은 유지한다.

**검증:** 관련 8개 스위트 950건(15.23초), 변경 생산 모듈 2개 lint·구문·diff 확인 통과다. 실제 계획 함수의 endpoint 경계, 같은 visit의 짧은 후진·frozen 면 유지, skew 재관측, 막힘/정지/링크/무진전, slow-coast를 포함한다. 원본 run 로그는 PC/Pi `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_01_20261019_122616_83b101eb/cycle_events.jsonl`이다. 사용자가 도크 재배치를 예고한 뒤 이동하지 않았으며 HOME/AMCL를 갱신하지 않았다. 이 수정의 실차 완주는 아직 수행하지 않았다.

**독립 검수·비이동 배포:** 주행 도중 partial travelled와 함께 stop/link/sensor/timeout이 반환되는 경로의 성공 오분류를 수정하고 4종 회귀·검수 PASS를 확인했다. Pi 소스/설치 2쌍 SHA·import 일치와 idle executor PID235200·기존 Nav2/base active, fresh scan/odom·속도0·pending0·CM 두 구역 활성 상태를 확인했다. 복원본은 `$HOME/.local/state/jdamr/parking-sequence-repair/20261019_124055/`, PC receipt는 `$HOME/.local/state/bimanual-robot/dock_return_20261019/parking_sequence_repair_deployment.json`이다. 기존 사용자용 서비스는 유지하며 종료는 §22.14 명령과 같다. 임시 확인 노드는 종료했다. 도크 재배치 완료를 받기 전 HOME/AMCL는 바꾸지 않았으며 새 주행은 제출하지 않았다.


### 22.19 2026-10-20 첫 구동 준비시간과 재부팅 후 도크 초기화 거부

**첫 구동의 관측:** `table_01_20261020_103756_40c6c2e1`은 관측점 재개 뒤 0.284532 m 전진을 요청했지만 이동은 0 m였다. 속도 0.05 m/s와 추가 5초에서 이동 예산은 10.6906초였고, 접근 시작부터 종료 기록까지 10.8407초였다. 이동 deadline은 `_direct_motion`의 첫 발행기 생성·초기 odom 조회보다 먼저 계산됐다. 기존 지연 없는 발행기 mock으로는 이 경로를 검증하지 못했다. 원래 로그에는 준비 하위 단계별 시간이 없어 11초 지연 전부를 DDS 생성 때문이라고 확정하지 않는다.

**조치와 증거:** `a25b1fd`는 준비시간을 짧은 이동 예산에서 분리하되 전체 태스크 deadline은 연장하지 않는다. 준비 중 사용자 정지·전체 deadline 만료·센서 오류 시 구동하지 않는 회귀를 포함했다. 실제 정밀 직선 제어 모형의 11초 첫 발행 지연은 이전 구현에서 실패하고 수정본에서 통과했다. 관련 642건·구문·치명 lint·diff 검사를 통과했고 Pi 소스/설치 SHA와 import를 확인했다. 현재 로그에는 `direct_motion_ready`의 준비시간과 `direct_motion_timeout` 원인을 남긴다. 이는 첫 준비 지연 자체나 전체 왕복 실차 연속성이 해결됐다는 뜻은 아니다.

**재부팅과 도크 확인:** 부팅 후 베이스는 자동 기동했으나 Nav2·상주 실행기는 내려갔고 Astra USB 복구 실패 기록이 남았다. 사용자 USB 재연결 뒤 새 320×240 깊이 영상·scan·odom·배터리 수신을 확인했다. 이후 USB 복구 결과는 success, 시스템 상태는 running이었다. 기존 준비 전용 세션과 idle 실행기를 복구했고 과거 위치초기화는 무효화했다. 사용자는 현재 도크이며 후면 박스가 있다고 확인했다. 이전 테이블 위치의 seed나 임의 전역 정합 후보를 현재 위치로 발행하지 않았다.

**중복 비율 조건:** `init_20261020_133500`에서 현재/기준 스캔 130/143개 중 93/98점이 5 cm 내 일치했다. 상대 자세는 평행이동 5 mm·회전 1°, 65% trimmed 평균 잔차 3.834 mm였다. 기존 양방향 70% 조건 중 기준 쪽 68.531%에서 거부됐다. `init_20261020_134025`에서는 같은 상대 자세·94/99개 일치점·잔차 5.248 mm였지만 현재 반환점이 143개로 늘어 현재 비율도 65.734%가 됐다. 이는 일치점의 소실과 전체 반환점 수 변화가 같은 상태를 반복 거부한 사례다.

도크 고정 기준에 한해 두 비율 거부를 양방향 최소 80개 일치점으로 정리했다. 서로 다른 3개 사분면의 지지, 65% trimmed 평균 잔차 2 cm 이내, 이동/회전 경계, 인접 다른 자세와의 점수 차, 지도·HOME·라이다 장착 일치는 유지한다. 지도 기반 초기화의 기존 inlier 기준은 변경하지 않았다. 비율은 진단값으로 남기고 정합 근거·일치점 수를 기록한다. 저장 기준 일부 가림·새 반환점 추가는 수락하지만 희소 일치점·무작위 장면·대칭 원·일직선·다른 지도/도크/큰 이동은 거부하는 회귀를 확인했다.

**좌표 저장 경계 오류:** 정합 후 `initial_pose.yaml`을 작성하면서 NumPy 실수형을 `yaml.safe_dump`가 거부했다. 기존 초기화 모의 결과는 일반 실수형이라 이 경계를 놓쳤다. 도크 정합의 반환 좌표와 상대 자세를 일반 실수형으로 변환하고 실제 정합 결과의 YAML 왕복 검사를 추가했다. helper 스위트 131건·구문·치명 lint·diff 검사를 통과했다. Pi 소스 scripts 경로와 운영 tools 사본에 같은 파일을 반영했다.

**실제 준비 결과와 남은 범위:** `init_20261020_134208`에서 새 도크 스캔 정합, AMCL과 약 2 mm·0.1° 일치, Nav2 비이동 활성화를 완료했다. 등록 HOME은 유지하고 도크 출발 대기를 기록했다. 사용자의 출발 전 준비 요청에 따라 카메라·인식기는 기동 대기를 줄이도록 함께 유지하며 별도 녹화·RViz·진단 상주 프로세스는 만들지 않았다. 해당 쌍의 종료는 Pi `sudo systemctl stop jdamr-box-rgbd.service jdamr-box-observer.service`다. 기존 base·Nav2·상주 실행기는 다음 명시적 출발을 기다린다. 실행 로그는 기존 runs, 배포·검사 receipt는 `$HOME/.local/state/bimanual-robot/motion_setup_fix_20261020/`와 `$HOME/.local/state/bimanual-robot/reboot_check_20261020/`에 보존한다. 이번 준비 중 실물 이동·전체 왕복 검증은 수행하지 않았다.


### 22.20 2026-10-20 정지 중 지도 자세 갱신과 깊이 면 혼용으로 성공 간격을 무진전으로 오판

**관측:** `table_01_20261020_141003_4d291fb0`은 도크 이탈·급수대 관측점 접근과 정밀 전진을 수행했다. 마지막 보정 요청은 19.898 mm, odom 이동은 20.339 mm였고 이동 종료의 표적 오류는 없었다. 새 깊이 관측은 stable 8프레임·처리 0.123초·입력 지연 0.003초로 유효했다. 그 면과 `observation_robot_pose_map`을 함께 계산하면 차체 양쪽 간격은 8.261/12.877 mm, yaw 오차는 0.588°로 기존 목표 10±3 mm·1° 안이다. 이는 저장된 깊이 관측의 내부 기하 재생이며 외부 실측이 아니다. 당시 로그의 후속 판정은 26.546/34.340 mm였고 이전 nearest 27.414 mm와 비교해 개선 0.869 mm로 분류됐다.

**이후 사용자 실물 관측:** 사용자가 방금 근접 간격은 거의 0 cm였다고 정정했다. 이는 내부 추정 8.261/12.877 mm와 약 1 cm 차이가 있으므로 이번 실차를 목표 1 cm 성공으로 기록하지 않는다. 정확한 양쪽 수치·측정 해상도는 없고 현재 로봇도 그 관측 위치를 벗어나 있다. 지도 좌표 혼용 수정은 실패 오판을 고치지만 물리 거리 보정까지 해결한 증거가 아니다. 현행 카메라 line 보정은 한 정지 자세의 7/5.1 cm 기준으로 유도됐고 공장 intrinsics는 유효값이 없어 기본 투영을 사용한다. 거리별 보정 오차·검출선과 실물 기준의 차이·실제 이동/정지 오차 중 어느 요인이 남았는지는 이 run만으로 확정하지 않는다. 현행 보정 상수를 약 0 cm 보고 하나에 맞춰 바꾸거나 새 이동 검증을 수행하지 않았다.

**확정 원인과 기능의 출처:** `observe_target`는 관측 전 정지 자세에 면 좌표를 결합한다. 최종 판정은 추가 `capture_stationary_pose`의 더 늦은 지도 자세를 사용하면서 면을 갱신하지 않았다. 따라서 정지 중 map→odom 갱신이 물리적 간격 변화처럼 들어갔다. 기존 `TABLE_FINE_MIN_PROGRESS_M=0.001` 조건이 `final_gap_no_progress`를 반환했고 `incomplete_parking_return_started`로 이어졌다. 이 조건은 원래 JD-AMR 저장소의 2026-10-13 `f36232fa51f63aa39d0aaa5fbf4474c9d541bce6`에서 들어와 양팔 저장소에 이관된 코드다. 이번 사용자가 새로 요청한 성공 기준이 아니다. 원래 로그에는 두 번째 정지 자세 자체가 없어 그 지도 갱신의 전체 XY를 재구성했다고 주장하지 않는다.

**수정:** 최종 새 깊이 관측 뒤 이동 revision을 보존하고 정지 확인 뒤 같은지 비교한다. 변하지 않았으면 목표 XY·yaw, 면 중심·법선·접선, 관측 자세를 하나의 SE(2) 변환으로 정지 확인의 지도 자세에 맞춘다. 양쪽 간격과 각도는 관측과 일관되고 정지 hold·후속 이탈·필요한 보정 목표도 같은 지도 좌표를 쓴다. 실제 이동 revision이 달라지면 해당 면으로 성공/추가 전진을 판단하지 않고 새 정지 관측으로 이어간다. 기존 전체 deadline을 연장하거나 목표 허용오차·실물 제원·센서/링크/정지·보정 상한을 바꾸지 않았다. `table_stop_target_rebased`에는 관측/정지 지도 자세를 함께 기록한다.

**검증:** 원본 run SHA와 마지막 target/event index를 `test/fixtures/table_final_gap_map_shift_20261020.json`에 보존했다. 기존 코드에서 지도 평행이동·회전의 3가지 재생이 실패했고 수정본에서는 같은 8.261/12.877 mm로 확인한다. 관측 뒤 이동 발생 시 새 관측을 기다리고 추가 전진을 발행하지 않는 회귀도 통과했다. 관련 7개 스위트 848건·구문·치명 lint·diff 확인을 통과했다. 별도 로컬 검증 패스를 수행했으며 native code-reviewer 생성은 agent thread limit으로 거부돼 독립 승인으로 취급하지 않는다.

**비이동 반영과 현재 상태:** Pi 소스/설치/import의 `box_service.py` SHA는 `a5d99c060d62d14e07180882da7412db6677fb324b4a1498d3022ef8420e91dc`로 일치하며 idle executor PID7399를 반영했다. 복원본은 Pi `$HOME/.local/state/jdamr/final-gap-frame/20261020_142206/`, receipt는 PC `$HOME/.local/state/bimanual-robot/final_gap_frame_20261020/deployment.json`이다. base PID1212·Nav2 PID2010은 재시작하지 않았고 카메라/인식기는 inactive 상태다. spool pending/running 0이며 이번 수정에서 이동을 요청하지 않았다. 기존 서비스 유지 이유는 다음 명시적 요청 대기이며 종료 명령은 Pi `sudo systemctl stop jdamr-box-executor.service jdamr-restaurant-navigation.service jdamr-base.service`다.

**별도 남은 복귀 문제:** 잘못 전환된 복귀는 급수대 후진 이탈과 `home_dock_approach` 이동까지 수행했지만 `rear box coarse static corridor blocked`로 후면 주차에 들어가지 못했다. 이는 이번 최종 간격 좌표 수정과 별개이며 아직 해결됐다고 주장하지 않는다. 1번 테이블은 방문하지 않았고 도크 도착도 미확정이다. 사용자 정지 뒤 자동 재개는 비허용이며 과거 `ready_not_started` 도크 준비 상태를 최신 run의 정지 상태로 정정했다. HOME·AMCL·물리 위치를 임의로 덮어쓰지 않았고 수정 후 전체 왕복 실차 검증도 수행하지 않았다.


### 22.21 2026-10-20 후면 TF 시작 외삽·도크 킵아웃 충돌·남은 조기 감속

**관측:** `table_01_20261020_152202_c340f169`은 직선 도크 이탈 0.7 m, 급수대와 1번 테이블 접근·각 후진 이탈, 도크 staging을 수행했다. 전면 양쪽 내부 추정은 각각 7.731/11.092 mm와 11.662/9.666 mm로, 외부 줄자 정밀도가 아니다. 복귀는 coarse 후진 전에 `_align_rear_reference`가 완전한 폭의 박스를 요구해 `rear box observation unavailable`로 끝났다. 후속 `152631_b6893a5a`는 requested TF 1792477595.756052 < earliest 1792477595.877148로 약 121 ms 과거 외삽, `152720_aa8626a9`도 약 42 ms 같은 시작 오류였다. 원인 변화 없이 추가 반복하지 않았다.

기존 staging 경유 복귀 `152813_ba33585f`는 159° 회전에 약 36초가 걸렸다. 후진 중 통로 거리는 1.401 m→0.484 m로 갱신됐고, 후면 면 gap 0.477646 m·각도 −0.218519 rad·span 0.351175 m·60점을 얻었다. 그 후 요청 −0.478678 m의 실제 이동은 0이며 `rear box approach static corridor blocked`로 끝났다. 현재 위치는 후면 복귀 접근 중 정지이며 도크 도착으로 기록하지 않는다. 원본 네 run은 PC/Pi `$HOME/jdamr_data/map_20260930_manual/aligned/runs/`에 있다.

**확정 원인과 수정:**

1. 새 노드의 map→base와 odom→base 최신 조회는 서로 다른 TF 이력 범위에 의존했다. 한 번의 odom←map edge 조회로 frozen 좌표계를 계산하도록 바꿨다. 경로·목표에 같은 변환을 사용하며 누락 TF는 오류로 남긴다. 실제 `tf2_ros.Buffer`에 42/121 ms 시작 차이를 넣으면 이전 조회는 실패하고 새 조회는 통과한다. 별도 대기·임의 pose 초기화는 추가하지 않았다.
2. 도크 전체 폭을 이용한 랜드마크 정합을 거리 관측 coarse 후진 뒤로 옮겼다. 충분히 가까워진 뒤 관측한 폭·끝점으로 기존 같은 run의 실측 위치 보정·중앙 접근을 수행하고, 먼 staging 재방문만 생략한다. 잘못된 면·다른 폭·ROI 가림·위치 불일치·센서/링크/정지 실패는 그대로 전파한다.
3. 마지막 면과 이후 정지 TF를 합친 비이동 재생에서 원본 지도는 통과하고 keepout만 막혔다. 첫 셀은 column70/map-row110, bounds [−0.20,−0.05,−0.15,0] m였다. 15 cm 후퇴는 정렬 preview까지만 통과하고 명목 1 cm 종점에서는 여전히 막혀, 5 cm 격자에서 20 cm 후퇴를 선택했다. 실제 마스크의 도크 영역 107셀만 0→254로 해제했다. 전체 폴리곤 재생성은 다른 수동 픽셀을 바꾸므로 사용하지 않았다. 원본 지도 SHA·HOME·다른 영역은 유지하며 새 마스크 SHA는 `334370d62965730df807a1a2a42a9145429551c95947b0b369bd877e01cb7283`다. 등록부·모든 route hash·설정 사본을 함께 갱신했다. 새 rear target-surface 통과 예외는 검수에서 제거했으며 일반 validator로 끝까지 통과한다.
4. 실제 파라미터에서 FollowPath min-approach 0.075 m/s·curve scaling true, Parking/ParkingReverse approach-band 0.15 m·curve scaling true를 확인했다. 이동의 min-approach를 desired 0.12 m/s와 맞추고 세 controller의 curve scaling을 false로 바꿨다. 주차 band는 desired×0.5초(현재 4 cm)다. 면 제어는 방향 보정 상한을 0.10 rad/s·Kp2로 맞춰 중간 각도에서 전진 전 구간의 creep를 줄였고, 큰 staging 회전은 0.3 rad/s로 진행한 뒤 기존 0.08 rad/s 보정으로 잇는다. 실제 이번 run에서 CM 개입점은 수집하지 않았으므로 감속 전부가 이 설정 때문이라고 단정하지 않는다. 실제 충돌·제동·센서 신선도·명령 timeout·통신·정지 감시는 유지한다.

**검증과 한계:** 실측 로그/정지 TF 출처와 SHA·이전 실제 마스크·HOME은 `test/fixtures/rear_dock_keepout_20261020.json`에 있다. 재생에 사용한 map 변환은 실패 시각의 변환이 아닌 이후 정지 표본이므로 정확한 과거 궤적 재현이 아니다. frozen TF·회전 plant·면 접근 rollout·실패 입력·실제 마스크 재생·후면 reference의 coarse/fine 연결·정지 조건을 검사했다. native code-reviewer는 신규 target-surface의 span 중심 결함을 발견했고 예외 제거 후 재검수 PASS했다. 수정본의 전체 왕복·외부 1 cm 반복 정밀도·시간 단축은 아직 실차에서 검증하지 않았다. 전면 실물 거의 0 cm와 내부 1 cm의 불일치는 §22.20의 미해결 항목으로 유지한다.

**비이동 반영:** 최종 관련 13스위트 1,221건·저장소 기본 1,036통과/3skip, 구문·치명 lint·diff·문서 링크/README 상태 검사를 통과했다. Pi source/install 및 운영 자산 24개 SHA·import·등록부, 실제 세 controller 값과 keepout signature 일치, fresh odom/scan·속도0·spool0을 확인했다. base PID1212·Nav2 PID2010은 유지하고 idle executor만 PID15019로 재기동했다. 복원본은 Pi `$HOME/.local/state/jdamr/navigation-repair/20261020_160033/`, PC receipt는 `$HOME/.local/state/bimanual-robot/navigation_repair_20261020/deployment.json`이다. 배포 도구의 최초 map reload는 잘못된 nav_msgs 서비스 타입 때문에 미적용됐고 nav2_msgs/LoadMap으로 수정한 뒤 result0·실시간 새 mask를 확인했다. 이 도구 오류를 주행 알고리즘 실패로 분류하지 않는다. 임시 probe는 종료했으며 기존 서비스 종료 명령은 §22.20과 같다. 수정 중 새 이동은 요청하지 않았다.

### 22.22 2026-10-20 급수대 접촉 뒤 보정 전진·근접 비틀림 분기 누락

**현상과 실행 근거:** `table_01_20261020_164220_05cd09c4`에서 사용자는 급수대에 부딪힌 뒤에도 더 전진했다고 보고했고 정지를 요청했다. 원본은 PC/Pi `$HOME/jdamr_data/map_20260930_manual/aligned/runs/table_01_20261020_164220_05cd09c4/cycle_events.jsonl`이다. 첫 접근은 요청 289.411 mm·odom 이동 304.619 mm였다. 정지 깊이의 양쪽 내부 간격 28.687/28.781 mm·yaw 0.012°를 보고 18.734 mm를 요청해 18.976 mm 이동했다. 이후 내부 간격은 30.461/14.800 mm·yaw 1.994°였고 4.803 mm를 요청해 12.937 mm 이동했다. 마지막은 `final_depth_measurement`의 `stable detected table_edge surface is required`로 끝났다. 1번 테이블과 도크 복귀는 수행하지 않았다. 실제 접촉 시각·접촉한 모서리·줄자 간격은 로그에 없어 모른다.

**확정된 제어 분기 오류:** 최종 재정렬은 가까운 모서리 ≤목표 10 mm+허용 3 mm에서만 시작됐다. 14.800 mm 상태는 양쪽 차 15.661 mm·yaw 1.994°가 허용을 벗어났는데도 이 분기를 건너뛰고 전진 보정으로 내려갔다. 기존 5 cm 정밀 구간 안의 각도/양쪽 차 미달을 같은 기존 1회 이탈·재관측·재접근으로 연결하고, 예산 소진 뒤에는 `final_table_alignment_unconfirmed`를 기록해 재전진을 막았다. 정렬된 미세 보정·작은 후진 보정과 상위 자동복귀 정책은 변경하지 않았다.

**요청 거리와 실제 이동의 구분:** live 면 제어의 최초 요청값은 이동 상한이 아니다. 기울어진 차체를 면과 평행하게 바꾸면 앞쪽 선행 모서리가 물러나 남은 접근량도 변한다. 두 번째 보정의 1.994°를 기존 앞끝 65 mm·반폭 225 mm에 대입하면 약 7.791 mm의 기하 변화가 생겨, 4.803+7.791=12.594 mm와 odom 12.937 mm가 가깝다. 이를 제동 초과의 증거로 쓰지 않는다. 이는 계산 일관성만 설명하며 실제 접촉 여부나 깊이 거리의 정답을 증명하지 않는다.

**회귀 격리와 미확정 원인:** 최근 `surface_approach.py`의 조향 상한/Kp 상향만 `a6f8eaf`의 0.02 rad/s·Kp1으로 복원했다. 동일 시작 면과 이상 odom의 이전/상향 재생은 둘 다 명목 5 cm 구간에 도달하므로 실제 접촉 원인을 이 변경으로 확정하지 않는다. 비교 결과는 PC `$HOME/.local/state/bimanual-robot/front_collision_20261020/ideal_steering_comparison.json`에 있다. 통로 Nav2 속도·후면 TF·킵아웃 수정은 유지했다.

**거리 보정의 남은 한계:** 카메라 장착각 원인설은 미확정이다. 현재 보정은 사용자 좌 7 cm·우 5.1 cm라는 한 실측 배치로 면 signed offset에 +49.605 mm를 더한다. 같은 배치 관측의 적합/holdout은 다른 거리에서의 검증이 아니다. 저장된 SDK 공장 intrinsics/extrinsics는 NaN이며 기본 투영을 사용한다. 원본은 `$HOME/jdamr_data/map_20260930_manual/aligned/camera_checks/camera_gap_mismatch_20261019_152144/`의 `factory_intrinsics.json`, `camera_info.json`, `depth.npz`다. 원본 깊이는 남아 있지만 독립 실측 바닥/상판 기준은 확인되지 않았다. 상판 높이 0.780125 m는 깊이 유도값이므로 외부 정답으로 재사용하지 않는다. 실제 간격과 내부 추정의 불일치는 §22.20처럼 미해결이며, 임의 거리 상수를 새로 맞추지 않았다.

**검증과 완료 범위:** 원본 run ID와 간격으로 만든 회귀는 이전 코드에서 호출 순서가 `approach, approach`라 실패했다. 수정 뒤 `approach, escape, approach`, 이탈 한 번, 그 뒤 추가 전진 없음과 구체적 종료 사유를 확인했다. 관련 4개 스위트 699건·치명 lint·구문·diff PASS, 별도 native code-reviewer PASS다. 테스트는 내부 기하·분기를 검증하며 물리 1 cm 정확도·접촉 방지·전체 왕복 성공을 입증하지 않는다. 사용자 정지를 유지하며 새 주행은 요청하지 않았다.

**비이동 반영:** Pi 소스/설치의 두 모듈 4개 SHA 및 import가 일치하고 executor PID19080이 수정본을 읽는다. 전후 fresh odom 각 3표본은 v/w 0이며 pending/running은 0이다. base PID1212·Nav2 PID2010을 유지하고 카메라/인식기는 inactive 상태다. 복원본은 Pi `$HOME/.local/state/jdamr/front-collision-repair/20261020_170148/`, receipt는 PC `$HOME/.local/state/bimanual-robot/front_collision_20261020/deployment.json`이다. 첫 배포 전 확인 도구는 0이 아닌 표본에서 소스 수정 전에 중단됐으나 해당 속도 수치는 수집하지 못했다. 기존 정지 요청을 다시 전송한 뒤 v/w 0을 확인하고 반영했으며 새 이동 명령은 보내지 않았다. 초기 확인 도구의 거부를 주행 재실패나 접촉 원인으로 쓰지 않는다. HOME·AMCL·카메라 보정은 변경하지 않았다. 과거 도크 준비 receipt를 무효화하고 현재 위치를 급수대 접근 중 정지로 일치시켰다. 임시 probe는 모두 종료했고 기존 서비스 유지 이유는 다음 명시적 요청 대기다. 종료 명령은 Pi `sudo systemctl stop jdamr-box-executor.service jdamr-restaurant-navigation.service jdamr-base.service`다.


### 22.23 2026-10-20 도크 킵아웃 재차단·후면 실측 각도 불일치·보정 후 기둥 필터 누락

**현상과 근거:** `table_01_20261020_171136_1385519b`는 급수대·1번 테이블 접근 및 이탈과 도크 앞 복귀를 수행했지만, 후면 0.453 m 접근의 정적 preview가 도크 keepout에 겹쳐 후진 이동 없이 끝났다. 당시 급수대 내부 양쪽 간격 8.897/12.633 mm, 1번 10.235/9.128 mm는 외부 정확도 검증이 아니다. 사용자는 전면 간격이 5 mm 미만이었다고 보고해 전면 1 cm 정확도 문제는 미해결로 유지한다.

**킵아웃과 대기:** 도크 경계 두 꼭짓점을 map −X로 추가 12 cm 후퇴시켰고 셀 62개만 해제했다. +11 cm에서는 같은 명목 1 cm 종점까지 막히고 +12 cm에서 통과하는 재생을 보존했다. 새 mask SHA는 `76bdb54527ab9b73aa658ff5decce5837d7af7e5c61393c270f47732a42673f2`다. 그 뒤 `table_01_20261020_172153_2f936673`는 odom 474.835 mm 후진했지만 약 17.5 mm에서 목표 박스 면 자체가 충돌 예측에 걸려 입력 복구 deadline으로 종료했다. 사용자 거리 도달 보고와 내부 실행 실패를 구분한다. 동일 scan·동일 면·관측 횡범위·허용 gap에서만 목표면을 구분하고, 로봇 쪽 signed residual은 기존 gap 허용오차 3 mm 이내로 제한했다. 횡범위 밖·더 큰 돌출·다른 기억점·회전만 수행하는 명령은 그대로 감시한다. 실제 정지 scan으로 예외 전 차단→검증된 목표면 문맥에서 통과→문맥 해제 뒤 차단을 재현했다.

**각도 실측과 후보:** 사용자가 로봇 기준 왼쪽 뒤 28 mm·오른쪽 뒤 거의 접촉을 보고했고, 이후 두 간격이 같은 뒤쪽 박스 면 기준임을 명시했다. 박스는 테이블보다 약 60 mm 앞에 있다. 차체 폭 450 mm에서 `atan2(28,450)=3.5605°`, 같은 정지 자세 8프레임의 기존 TF 법선 평균은 0.04327°였다. 차이 +3.5172°를 provisional laser yaw `3.202979549615386 rad`로 반영했다. 우측의 정확한 수치·줄자 해상도는 알려지지 않았고, 8프레임은 독립 자세 8번이 아니다. 영구 교정 완료나 외부 정밀도 증명으로 쓰지 않는다. 원본은 `$HOME/.local/state/bimanual-robot/rear_angle_20261020/stationary_scan.json`, 최소 재생과 실측 한계는 `test/fixtures/rear_laser_yaw_20261020.json`이다.

**좌표 보존과 복구:** YAML/URDF/Pi source/install을 한 세트로 반영하고 `H_new=H_old*L_old*inverse(L_new)`로 HOME과 저장 anchor를 이행했다. HOME은 `(0.120086151,0.164349214,0.072658382 rad)`로 표현되며 물리 도크와 원본 지도는 유지한다. 원 raw anchor를 보존하고 정적 laser TF만 갱신한 파생 anchor를 사용한다. 전역 map scan match는 inlier 0.14·seed 위치 차 0.54 m로 거부됐으며 이를 통과했다고 기록하지 않는다. 기존 같은 박스 landmark 변환으로 현재 위치 `(0.129923,0.139543,0.018049)`를 계산해 기존 무이동 activation을 수행했고 AMCL `(0.132078,0.133592,0.009717)`과 일치 및 DDS health를 확인했다. 전역 지도 정합의 낮은 지지는 별도 미해결 사항이다. Nav2 재기동 중 inactive/dead·MainPID0·빈 cgroup의 transient unit fragment가 남아 이름 재사용을 막았다. 정확한 unit fragment만 `/run/jdamr-transient-backup/`에 보존 이동한 뒤 순정 기동을 재개했다. 다른 유닛은 변경하지 않았다.

**보정 이행 누락과 수정:** `table_01_20261020_181251_4e495898`는 보정 후 기울기를 3.506°로 인식하고 전진 이탈을 odom 197.2 mm 수행했지만, 후진 시작은 새 자기반사점 때문에 차단됐다. live TF yaw는 −3.080205758 rad로 보정값과 일치한다. 기존 기둥 필터의 두 XY box가 경험적 scan 기반 yaw=π 좌표라는 provenance를 함께 이행하지 않은 누락이었다. 좌우 후면 기둥 반사 8점이 실제 footprint 안에 남았고 목표면이나 blind memory가 원인이 아니었다. 기존 box를 old base←laser 역변환으로 `laser_link` 좌표에 옮겨 raw-beam 소속과 크기·5 mm margin을 보존했다. launch가 YAML `base_frame`을 전달하도록 고쳤다. 양·음 increment와 5개 반경의 old/new 마스크 완전 동등, 실제 8점 제거, 뒤쪽 박스 반사 보존 회귀를 통과했다. 로봇은 전진 이탈 후 정지했으며, 이 run의 후면 주차는 성공하지 않았다.

**지연의 별도 수정:** 급수대 관측 전진 99.904 mm에 기본 4초 접근법이 적용돼 약 0.025 m/s로 주행한 로그가 있었다. 이 관측 전진만 기존 면 접근의 0.5초 법칙을 명시해 최대 0.05 m/s·제동·충돌 검사를 보존했다. 전체 감속 문제가 모두 해결됐다는 뜻은 아니며 큰 각도에서의 기존 조향 상한과 실제 전면 거리 오차는 남아 있다.

**검증과 산출물:** 주행 관련 777건과 self-filter 15건, 치명 lint·구문·diff 검사를 통과했다. 별도 native code-reviewer는 과대한 near-side 면 예외를 지적했고 3 mm 비대칭 상한으로 수정 후 차단 결함 없음을 확인했다. 후보 TF와 HOME 이행, 자기반사 필터의 exact raw membership도 독립 검토했다. Python LSP 미설치로 LSP 진단은 실행하지 않았으며 이를 완료로 주장하지 않는다. Pi 코드 6경로·geometry/URDF 및 운영 자산 11경로·self-filter 5경로의 배포/복원 receipt는 같은 `rear_angle_20261020` 폴더에 있다. 임시 관측기는 각 finally에서 종료했다. 이번 변경은 로컬 브랜치 커밋 범위이며 push/main 병합은 수행하지 않는다.

**1.5 cm 근접의 관측 한계:** 현재 G4 `range_min=0.280 m`, laser X=-0.010 m, 차체 뒤끝 X=-0.275 m이므로 라이다에서 뒤끝까지는 0.265 m다. 명목 gap 15 mm에서 중앙 반사점이 최소거리 경계에 들어가고, gap 10 mm의 중앙 range 0.275 m는 최소거리보다 작다. 양 끝점은 더 멀어 여전히 검출된다. 수정된 필터 뒤 `182157_dfa8b654`는 odom 191.43 mm 후진해 각도 약 0.1°·gap 약 13 mm까지 접근했지만 blind-range 감시에서 멈췄다. 다음 `182421_80d30c2f`의 미세 후진 3.457 mm는 완료됐고 내부 9.834/13.318 mm·0.4435°로 한쪽 허용을 0.318 mm 벗어나 기존 20 cm 이탈·재접근이 발생했다. 재접근도 근접 감시 deadline으로 끝났다. 종료 후 fresh live scan 4프레임은 gap 15.36–15.98 mm, angle -0.01–0.15°였으며 같은 목표면을 구분한 live sweep에는 충돌점이 0개였다. 실행 실패를 성공으로 기록하지 않으며, retry 실패 뒤 이전 final 측정값을 종료 이벤트에 재사용한 기록 문제와 과거 기억점의 취득 시점 면 소속 판정을 추가 수정한다.

**실물 각도 확인:** 최종 정지 후 사용자가 같은 박스 면을 보고 “거의 평행 훌륭하다”라고 확인했다. 이번 배치의 각도 개선에 대한 운영자 확인이며, 좌우 간격 숫자·각도계 측정·반복 주차 성공률은 받지 않았다. 현재 자세를 유지하고 추가 이동은 수행하지 않는다. 기억점 수정은 무이동 배포와 코드 검증으로 마무리하며 재주행 성공으로 확대하지 않는다.

**운영 도구 오류의 구분:** 비이동 검증 도구에서 LaserScan 메시지를 mapping API에 넘긴 오류와 NumPy 실수의 YAML 직렬화 오류가 있었고 수정했다. 첫 검증·go의 셸 명령을 조건 없이 연속 배치해 검증 도구 실패 뒤에도 승인된 rear-only가 제출된 절차 오류도 기록한다. 그 실행은 기존 센서·충돌 감시를 거쳐 20 cm 이탈했고 이후 TF·라이브 점 확인을 회수했으나, 앞으로 검증과 이동 명령을 별도 도구 호출로 순차 수행한다. 이 도구 오류들과 실제 `rear box input recovery deadline exceeded`를 같은 원인으로 섞지 않는다.

**기억점 수정 최종 반영:** 과거 scan의 원시 좌표에서 동일 목표면·폭·잔차·관측 구간을 검증한 점만 후면 예측에서 구분한다. 일반/coarse 입력과 다른 물체·돌출점은 유지한다. 나이를 제한한 기존 메모리 helper가 한 odom 자세로 비목표 점을 투영해, 서로 다른 자세로 재계산한 float tuple 교집합의 경쟁을 없앴다. scan 캐시는 원 객체 강참조와 취득 자세를 함께 검증해 객체 ID 재사용을 막는다. 실제 분류기에 합성 scan을 넣은 goal/다른 면/4 mm 돌출/횡범위 밖과 ID 충돌·취득 자세 변경 회귀를 추가했다. 재시도 실패 뒤 새 관측이 있으면 현재 값을 기록하되 실행 실패를 성공으로 바꾸지 않는다. 재관측까지 실패하면 마지막 유효 값이 남는 관찰성 한계는 있으며 새 성공 근거로 쓰지 않는다.

최종 관련 10개 스위트 872건(6.41초), 치명 lint·diff 검사를 통과했고 독립 code-reviewer는 캐시 식별 HIGH 수정 후 APPROVE했다. Pi Python 모듈 4개 source/install 8경로 SHA·import와 idle 실행기 재시작, 전후 정지 odom 3표본씩·pending/running 0을 확인했다. 최종 receipt는 `$HOME/.local/state/bimanual-robot/rear_angle_20261020/memory_source_deployment.json`이다. 사용자 실물 평행 확인 뒤 새 주행은 하지 않았으므로 이 마지막 기억점 수정의 실차 재검증은 남아 있다. `executor_exit_nonzero`였던 지난 run을 성공으로 고치지 않았다.


### 22.24 2026-10-20 출발 거부의 배터리 부족 사유가 위치·센서 오류로 표시됨

- **관찰:** `table_01_20261020_184412_2ee8a0b2`는 출발 직후 `localization or sensor data unavailable`로 실패했다. 실행기 journal에는 `navigation readiness timeout: battery low: voltage=10.192V limit=10.500V`가 남아 있다. `/map`·keepout·laser TF·scan의 별도 DDS health는 정상이었다. 이번 출발 거부의 확인된 원인은 배터리 부족이며 위치추정·USB·DDS 고장으로 단정할 근거는 없다.
- **표시 오류:** `BoxServiceRoute`는 `wait_until_ready()` 실패 사유를 일반적인 위치·센서 오류로 치환한다. 이때 `10.5 V`는 주행 하한이며, 현재 계약의 신규 출발 예비 기준은 별도로 `10.8 V`다. 준비 완료 보고에서 정지 odom·활성 서비스만 확인하고 배터리 조건을 놓친 점도 기록한다.
- **운영 조치:** 사용자의 정지 요청으로 출발 대기를 취소했고 fresh odom 3표본의 선속도·각속도 0을 확인했다. 배터리 교체·충전 완료는 확인되지 않았다. 새 주행·위치초기화·서비스 재시작은 수행하지 않았다.
- **재발 시 우선순위:** 같은 메시지 또는 출발 직후 실행기 실패가 나면 현재 run의 배터리 사유·새 전압부터 확인하고 측정값과 기준을 설명한다. 전압이 충분하면 실제 센서·위치·통신 사유를 확인한다. 기존 입력과 guard를 사용하며 출발에 새 장기 검수 단계를 추가하지 않는다. 이 우선순위를 `src/navigation/AGENTS.md`에 저장했다.
- **미완료:** 이번 요청은 기억·운영 규칙 반영이며 실제 사유를 구조화 실패 이벤트로 전달하는 실행기 코드 수정은 아직 반영하지 않았다. 메시지 표시가 고쳐졌다고 주장하지 않는다. 로그 근거는 해당 run의 `cycle_events.jsonl` 및 Pi `jdamr-box-executor.service`의 2026-10-20 18:44 journal이다.


### 22.25. 준비 실패의 실제 사유를 최종 오류까지 보존 (2026-10-20)

- **확정 원인과 수정:** §22.24의 `10.192 V < 10.5 V`는 하위 guard에만 남고 box caller가 고정 위치·센서 오류로 바꿨다. `CorridorRoute.wait_until_ready()`가 실패 사유를 저장하고 새 시도에서 초기화하도록 수정했다. `ServiceRoute`의 별도 출발 예비전압·operator link 검사도 사유를 보존한다. box 방문/관측 재개/복귀/정차 재개, restaurant 방문/복귀, 단독 후면 주차, `activate_navigation`, 수동 이탈 caller에 이를 전달했다. 정지 요청은 준비 실패 반환 시 우선한다.
- **검증:** 관련 5개 스위트 873건과 activation 58건, 치명 flake8·구문·diff 통과. 기존 일반 오류 문자열을 기대하던 covariance 테스트를 구체 사유 보존으로 갱신하고, 예전 배터리 실행 테스트에서 누락된 `config={}` fixture 상태를 보완했다. 초기 패키지 조회 실패는 통합 리포 install만 source한 검증 환경 문제였으며 기존 `$HOME/jdamr_rgbd_ws/install/setup.bash` overlay를 포함한 재검증에서 통과했다. 독립 리뷰의 activation 전달 누락 지적을 반영한 뒤 APPROVE를 받았다.
- **Pi 반영:** source/install 모듈 5쌍과 `$HOME/jdamr_data/map_20260930_manual/tools/box_escape_once.py`, 총 11경로의 SHA·import 확인. 실제 설치 모듈에 모의 입력만 넣어 low `10.192/10.5 V`, reserve `10.6/10.8 V`, 충분 `12.2 V` 결과를 확인했다. 이 모의 입력을 실물 저전압 재현 또는 주행 성공으로 해석하지 않는다. 배포 receipt는 PC `$HOME/.local/state/bimanual-robot/battery_reason_20261020/deployment_receipt.json`, 복원본은 Pi `$HOME/.local/state/jdamr/battery-reason/20261020_191954/`다.
- **현재 상태/한계:** 실물 새 전압 12.200–12.228 V, 배포 전후 fresh odom 각 3개 모두 선속도·각속도 0, spool request/running 0. idle 실행기 PID39453만 새 코드로 재시작했고 base PID31767·Nav2 PID32377은 유지했다. RGB-D/관측기는 비활성이며 새 상주 진단 프로세스는 없다. 다음 요청을 받기 위해 기존 서비스는 유지한다. 종료 명령은 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. 사용자 정지 유지, 새 이동 명령 없음, 수정본의 전체 왕복/실물 정밀주차 재검증은 남아 있다. 배터리 기준·guard·대기시간을 변경하거나 새 출발 절차를 추가하지 않았다.


### 22.26. 살아 있는 controller 프로세스의 통신·cleanup hang과 출발 전 복구 (2026-10-21)

- **현상/근거:** `table_01_20261021_102400_3bc45058`는 10:24 관측 경로 선택 뒤 controller parameter wait 5초 실패로 preflight에서 끝났고 이동 목표는 제출하지 않았다. Nav2 journal에는 10:22:39 controller graph 소실, 종료 때 controller/plugin cleanup 이후 local_costmap cleanup 정체와 SIGKILL(-9)이 남았다. 다른 Nav2 노드는 정상 종료했다. 신호 handler 전체가 미응답이었다고 확대하지 않는다. 별도 fresh parameter client도 서비스를 찾지 못했다.
- **확정/가설 분리:** controller의 실제 hang과 서비스 응답 부재, 상위 `failed(preflight)`가 원인을 숨긴 것은 확인했다. native 내부의 최초 원인은 backtrace 없이 확정할 수 없다. UDPv4 환경이 있었고 단일 SHM 오류만으로 해당 프로세스의 SHM 사용·인과를 확정하지 않는다. 배터리와 AMCL은 이번 실패 사유가 아니었다.
- **코드 조치:** 기존 read-only GetParameters·같은 5초 응답 예산 내 재전송을 공통화해 parking 조회에 사용한다. 불필요한 6개 서비스 전체 의존을 제거하되 실제 hang·설정 불일치를 통과시키지 않는다. 실패 사유를 저장해 preflight 이벤트·최종 CLI 오류로 전달하며 activation 준비 완료 전에 실제 controller 응답과 값을 검증한다. 정상 주행 호출에 검수·재기동을 추가하지 않았다.
- **남은 요청 처리:** anchored dock scan으로 초기화한 fresh 일반 출발만 대상으로, 정확한 controller preflight 실패·이동 전 허용 이벤트·executor code 1이 확인되면 Nav2/executor stop/start→기존 registered dock init→동일 목표 제출을 한 번 수행한다. 이동·알 수 없는 이벤트·카메라 start가 있거나 재개/단독 복귀/사용자 경로/local init이면 이 초기화 복구를 하지 않는다. 각 단계 전후 사용자 정지를 확인하며 두 번째 실패를 무한 반복하거나 숨기지 않는다. 이번 작업은 사용자 정지 상태라 이 실차 재출발을 실행하지 않았다.
- **복구 시 발견한 별도 문제:** 종료된 transient unit이 inactive/dead로 남아 새 systemd-run이 이름 충돌로 실패했다. launcher는 같은 identity와 정확한 transient 경로의 정지 unit만 재사용하고 다른 identity·persistent unit·다른 상태는 변경하지 않는다. 배포로 바뀐 script hash는 MainPID0·argv/User/경로 검증 아래 runtime identity drop-in으로 이행하고 시작 후 `/proc/PID/environ`의 유효 identity를 확인했다. 원 transient 파일·다른 서비스는 삭제하지 않았다.
- **검증:** 관련 Python 875건 + parking/session 98건 + PC 흐름 144건 = 1,117건 통과. get-only client와 기존 0/3/6초·전후진·설정 불일치·불완전 응답을 검증했다. 복구 1회 한도·미이동/등록 도크 제한·각 단계 stop 차단도 검증했다. 코드와 제한된 복구·runtime 이행 계획은 독립 리뷰 APPROVE, 치명 flake8·compile/bash/diff 통과. shellcheck은 설치되지 않아 실행하지 못했다.
- **실차 반영/한계:** Pi 10경로 SHA·import/구문 일치, 새 Nav2 PID47789·idle executor PID48133, base PID31767 유지. 원 도크 scan seed 재사용과 무이동 activation에서 controller 응답 검사를 통과했다. 최종 pose는 내부 map 추정 `(0.135473,0.095759,0.048701 rad)`이며 도크 등록은 유지했다. fresh odom 3개 모두 v/w0·spool 0, RGB-D/관측기는 inactive. 전체 왕복·정밀주차 반복성과 16시간 hang의 비재발은 미검증이다.
- **근거/복원/유지 서비스:** PC `$HOME/.local/state/bimanual-robot/controller_connection_20261021/`의 `deployment_receipt.json`, `reactivation.json`, `final_stopped_state.json`, 원 failure journal을 보존한다. Pi 복원본은 `$HOME/.local/state/jdamr/controller-connection/20261021_104341/`다. runtime drop-in은 `/run/systemd/system/jdamr-restaurant-navigation.service.d/80-prepared-session-identity.conf`이며 재부팅 때 사라진다. 복원 시 먼저 `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation`으로 정지하고 배포 파일을 복원한 뒤 해당 drop-in 한 파일만 제거·`sudo systemctl daemon-reload`한다. 다음 사용자 출발을 위해 Nav2와 idle 실행기를 유지하며 전체 종료는 `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. 신규 자동 이동은 허가하지 않았다.


### 22.27 2026-10-21 도크 킵아웃 전체 제거와 후면 fine의 지도 차단 해제

- **현상·확정 근거:** `table_01_20261021_105628_0deeaf19`는 급수대·1번 테이블 접근/이탈과 도크 staging·coarse를 마쳤다. 후면 라이다 면 gap 0.489142 m, support 64점·span 0.333383 m·RMS 1.116 mm를 관측했고 fine의 요청 후진 0.478517 m는 실제 이동 0 m로 끝났다. 이유는 keepout column64/image_row116의 셀 `[-0.50,-0.55,-0.45,-0.50]`과 전체 외곽 preview가 겹친 것이다. `rear_box_parking_not_confirmed`는 상위 결과이며 실제 차단은 `rear_box_static_corridor_blocked`다. 종료 뒤 배터리 11.528 V를 읽었으며 이번 오류를 배터리/제어기 불응답으로 분류하지 않는다.
- **사용자 배치·변경 범위:** 사용자가 도킹 지점의 킵아웃을 아예 제거하고 그쪽 앞에는 박스가 있다고 확인했다. `dock_table.enabled=false`, 해당 polygon의 현재 인위적 셀 119개만 base mask의 254로 복원했다. 새 mask SHA는 `834b0823b6796334527390f843f5a26d9b9d3336a5ed1417f4cd7c7d21cb2af8`이며 registry·5 route identity도 갱신했다. 원본 지도·HOME·다른 zone은 유지한다.
- **분리한 지도 문제:** 같은 재생은 keepout-only에서 통과하지만 원본 지도 셀 `[-0.60,-0.50,-0.55,-0.45]`와도 겹친다. 킵아웃 제거만으로 전체 static 검사가 통과했다고 기록하지 않는다. 사용자 배치를 기준으로 `_rear_box_update`의 fine 단계만 불필요한 map 변환·정적 preview 검사를 제거하고 기존 live LiDAR 면/odom 제어로 잇는다. 원본 지도는 지우지 않았고 coarse·centering·일반 주행 지도 검사는 유지했다. 동일 면·scan/clock·30도 범위·기존 0.6m 한도·실시간 외곽 감시·사용자 정지는 그대로다.
- **검증:** 관련 7스위트 630 PASS, 치명 lint·구문·diff 검사 PASS. 새 회귀의 변경 전 실패→변경 후 통과, 119셀 외 차이 없음, 과거 2개 재생 증거의 마스크 SHA 보존, 새 fine의 입력/heading/travel 실패를 확인했다. 독립 리뷰 APPROVE. full static map_not_free 결과도 별도로 고정해 물리 주차 성공을 꾸미지 않는다.
- **정지 반영:** Pi 10개 자산을 백업 후 갱신하고 기존 `/keepout_filter_mask_server/load_map`의 result0 및 실제 발행 OccupancyGrid signature 일치를 확인했다. fine source/install 2경로 SHA·import를 검증한 뒤 idle executor만 PID52030으로 시작했다. base PID31767·Nav2 PID47789·위치초기화·active unit identity는 유지했다. 배포 전후 odom은 기존 정지 허용범위 v≤0.002 m/s, w≤0.01 rad/s를 통과했으며 후 표본에는 ±0.001262 m/s·±0.004948 rad/s가 있어 완전한 0이라고 주장하지 않는다. 새 이동·물리 성공 검증은 수행하지 않았다. 현재는 도크 전면 통로에서 fine 재개 대기이며 HOME으로 위치를 덮어쓰지 않는다.
- **근거·복원:** PC `$HOME/.local/state/bimanual-robot/dock_keepout_removal_20261021/`의 `cycle_events.jsonl`, `deployment.json`, `fine_deployment.json`을 보존한다. Pi 자산 복원본은 `$HOME/.local/state/jdamr/dock-keepout-removal/20261021_111116/`, 코드 복원본은 `$HOME/.local/state/jdamr/dock-fine-control/20261021_111204/`다. 복원할 때 idle executor를 정지하고 해당 파일만 복구한 뒤 원 mask를 LoadMap으로 다시 적재한다. active runtime ID/drop-in은 바꾸지 않았다. registry 해시가 바뀌었으므로 다음 Nav2 세션 재기동에서는 exact unit을 inactive/dead로 정지한 다음 새 fingerprint를 이행하고 새 PID 환경을 대조해야 한다. active 상태에서 표시 ID만 바꿔 재사용 검사를 통과시키지 않는다.
- **유지 서비스:** 다음 요청 대기로 기존 base/Nav2와 idle executor를 유지한다. 전체 종료 명령은 `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. 임시 probe와 reload 노드는 종료됐고 카메라/observer는 비활성이다. 현재 브랜치 로컬 커밋 범위이며 push/main 병합은 하지 않는다.


### 22.28 2026-10-21 도킹 간격은 성공했으나 면 중앙에서 벗어남·전면 crawl 수정

- **사용자 확인과 실행기 결과 분리:** `table_01_20261021_112230_3b93c93f`는 급수대·1번 테이블 접근/이탈과 도크 앞 복귀·후면 접근을 수행했다. 사용자는 도킹의 정밀 간격은 성공했으나 박스 면의 좌우 중앙이 아닌 다른 위치로 갔다고 확인했다. 외부 수치 측정값은 이번 보고에 없다. 내부 실행기는 외곽/각도 확인 후 `rear_box_parking_not_confirmed`로 code 1 종료했으며 이를 전체 실행기 성공으로 바꾸지 않는다.
- **확인된 중앙 정렬 누락:** 출발 후 면 support 부족으로 `_rear_dock_reference`가 없었고 기존 reference 정렬은 생략됐다. 초기 후면 full face는 base midpoint `[-0.754305, -0.259417] m`, normal `[0.998960, 0.045585]`, gap 0.480377 m, span 0.340657 m, support 62점이었다. 접선 방향 중심차 약 22.5 cm는 이 내부 관측의 기하 계산이며 사용자의 줄자 측정이 아니다. 기존 fine은 면 거리와 각도만 조절하고, reference가 있을 때도 출발 origin의 접선 위치를 복원했으므로 현재 박스 중앙 도달을 보장하지 않았다.
- **중앙 제어 변경:** 현재 full/unclipped face의 midpoint·normal을 기준으로 center 목표를 만들고 기존 곡선 추종으로 20 cm staging에 접근한다. 중앙 오차 2 cm 이내를 새 관측으로 확인한 뒤 fine으로 넘긴다. coarse는 1.0 m에서 넘기며 면 미검출 range 복구도 중앙 정렬 전에는 0.9 m에서 멈춰 곡선 공간을 보존한다. 이미 근접한 off-center 재시도는 기존 guarded 전진 이탈로 공간을 확보한 뒤 같은 면을 재확인한다. 중앙이면 불필요한 추가 이탈을 하지 않는다. 출발 전 유효 관측 fallback·동일 목표 확인·전체 외곽 정적 검사를 유지한다. 중앙 곡선은 정상 CM profile에서 수행하고 fine만 기존 scoped rear profile로 실행한다.
- **전면 저속의 확인 범위:** 1번 테이블 live 접근 이벤트의 대표 speed는 0.05 m/s였고, 기존 공유 제어는 작은 각도 오차에도 남은 정렬 공간/시간으로 선속도를 더 줄였다. 기록에는 전체 순간 cmd trace가 없어 사용자가 본 매 감속의 기여도를 수치로 확정하지 않는다. 실제 사용 launch의 transit RPP는 desired/min approach 0.12 m/s·곡률/비용 감속 false이고 precision CM SlowdownZone도 false였다. 이번 변경은 전면 live 접근만 순항 0.1 m/s·원거리 각도 보정 상한 0.08 rad/s로 분리한다. 회전 정렬을 끝낼 공간이 부족한 경우에만 제자리 각도 수렴을 사용한다. 최종 제동·근접 조향·기본 및 후면 제어는 보존한다.
- **작성 중 발견한 제동 오류와 검증:** 두 배 순항에 기존 마지막 25 mm taper를 강제로 재사용한 첫 수정안은 기존 합성 slow-coast 조건(감속 0.08 m/s²)에서 정지 확인에 실패했다. 전면 taper 시간을 속도비로 늘려 1.0 s·진입 거리 0.1 m의 연속 제동으로 수정했다. 기본/후면은 0.5 s·25 mm 그대로다. 최종 관련 7스위트 829 PASS(6.21초), fatal flake8·compile·diff PASS, 독립 code-reviewer APPROVE다. 기본·전면 양쪽의 합성 정지 완료/최종 위치 ±3 mm, raw gap과 filtered heading 차이, 22.5 cm 중심차의 현재 지도/전체 외곽 곡선 재생을 확인했다. 이는 실물 반복 정밀도나 시간 개선 실측이 아니다.
- **Pi 반영:** Python 4모듈 source/install 8경로를 백업·원자 교체하고 SHA·compile·설치 import와 명령 출력을 검증했다. idle executor만 PID56534로 재시작했고 base PID31767·Nav2 PID47789·active unit identity는 유지했다. 배포 전후 fresh odom 각 3표본은 모두 선속도·각속도 0이며 spool pending/running 0을 확인했다. HOME·AMCL 초기화·카메라 장착은 변경하지 않았고 새 이동은 없다. 수정본의 실차 중앙 정렬·접근 시간 검증은 남아 있다.
- **근거·복원·대기:** PC `$HOME/.local/state/bimanual-robot/dock_center_and_table_speed_20261021/`에 원 실행 `cycle_events.jsonl`, `deployment.json`, `state_before.json`, `state_after.json`을 보존했다. Pi 복원본은 `$HOME/.local/state/jdamr/dock-center-and-table-speed/20261021_115913/`다. 복원은 idle executor 정지 후 이 8경로만 복구하고 재기동한다. active Nav2 identity/drop-in은 바꾸지 않았으며 다음 Nav2 세션 재기동의 inactive/dead 상태 fingerprint 이행 원칙은 §22.27과 같다. 기존 base/Nav2와 idle 실행기는 다음 사용자 요청 대기로 유지한다. 전체 종료는 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. 임시 probe는 종료됐으며 자동 재출발은 없다. 이번 변경은 목적 브랜치 로컬 커밋만 남기고 push/main 병합은 하지 않는다.


### 22.29 2026-10-21 도크 검출 대기 중 접근 차단·1번 테이블의 불필요한 전체 후진

- **실행 범위·정지:** `table_01_20261021_120603_ce541cf6`는 급수대 접근/이탈·1번 테이블 접근/이탈·도크 앞 이동을 수행했다. 후면 도킹은 완료하지 못했고 사용자의 정지로 `failed(reason=operator_stop)` 뒤 종료했다. CLI code 1을 새로운 배터리/통신 고장으로 분류하지 않는다. 사용자 재배치 및 수정 요청 뒤 새 이동은 수행하지 않았다.
- **확정된 도크 대기 원인:** `rear_box_fine_handoff`의 raw corridor gap은 0.654145929 m였다. 이후 `rear face line has no majority support`가 반복됐지만 앞선 수정의 pre-center range 복구 하한 0.9 m 때문에 `gap - handoff < 0`, `rear_box_range_reobserve`는 0회였다. handoff부터 정지까지 133.887초, 첫 waiting 이벤트부터 정지까지 128.857초, waiting 이벤트 26개를 확인했다. 같은 위치의 반복 검출은 관측 기하를 바꾸지 못하며, 이 소프트웨어 거리 조건이 필요한 추가 접근을 막은 것은 확정이다.
- **검출 원인의 한계:** 실패 순간 raw scan은 기본 실행 로그에 보존되지 않아 비다수 점이 박스/테이블/다른 반사 중 무엇이었는지 복원하지 못한다. 넓은 ROI의 혼합 가능성을 해당 실차의 확정 원인이라고 쓰지 않는다. 재배치 이후 별도 정지 scan 3개에서는 full face가 검출됐고 내부 gap은 6.591–7.134 mm였다. 이는 이후 배치의 관측으로 원래 실패 위치나 물리 간격의 외부 실측이 아니다.
- **도크 제어 수정:** full face가 없으면 1초 관측 후 기존 fresh raw LiDAR corridor·전체 외곽 지도 검사·정상 CM·stop/link 경로로 최대 0.2 m씩 추가 후진한다. 매 구간 면 검출을 다시 시도하고 기존 0.2 m 관측 하한 및 총 이동 한도를 유지한다. 이 하한을 최종 도킹 목표로 사용하지 않는다. 근거리에서 얻은 면으로 landmark 위치 보정을 먼저 끝내며, 중앙 오차가 커 곡선 공간을 확보한 경우 이미 확정한 odom 면을 곡선 목표로 유지한다. 먼 곳에서 다시 full face를 찾는 대기는 삭제했다. clearance 뒤에는 중앙 곡선을 생략하지 않고, 곡선 종점에서 fresh 동일면과 좌우 중앙을 확인한 뒤 1 cm fine으로 잇는다. stale base 관측으로 이탈 후 현재 위치를 초기화하지 않는다.
- **1번 테이블 원인·수정:** 첫 정지의 내부 양쪽 간격 30.941691/22.965001 mm, yaw 1.015677°는 허용 yaw 1°와 양쪽 차 6 mm를 넘었다. 그 작은 각도에 대한 근접 정렬 분기가 없어 전체 후진 재관측으로 내려갔다. fresh 면의 모든 모서리가 양수 간격이고 목표보다 남은 거리가 있으며 기존 30°/모서리 후퇴 기하 범위 안이면, 같은 면을 쓰는 근접 continuous 보정으로 제자리 정렬 후 접근한다. 근접 보정 미수렴 뒤 full-back으로 전환하지 않는다. 기존 근접 접촉/목표보다 가까운 모서리의 처리·실제 외곽/CM·사용자 정지를 유지한다. 이전 재접근 후 최종 내부 간격 10.159/10.984 mm는 이번 수정본의 성공 근거가 아니다.
- **검증·원인 보존:** 관련 8스위트 871 PASS(6.68초), fatal flake8·compile·diff PASS, 독립 code-reviewer APPROVE다. 실제 0.654 m handoff에서 0.2+0.2+0.054 m 추가 접근 선택, 하한/차단/동작 중 stop·link·scan·CM 사유, clearance 전 위치 보정→accepted odom 면 carry→곡선→fresh 확인 순서를 검증했다. 1번 실제 수치의 후진 0회·근접 보정 1회 성공/미수렴/실패와 dispatch 전 stop/link/nav 중단도 확인했다. 리뷰가 찾은 near 실패의 terminal 이벤트 누락과 coarse 실패 사유 치환도 수정했다. 실차 재주행 및 반복 정밀도는 미검증이다.
- **Pi 반영·복원:** Python 2모듈 source/install 4경로를 SHA·백업·원자 교체·compile·설치 import로 확인했다. idle executor만 PID61035로 재시작했고 base PID31767·Nav2 PID47789·active unit identity는 보존했다. 배포 전후 fresh odom 각 3표본 모두 선속도/각속도 0, pending/running 0이다. PC 근거는 `$HOME/.local/state/bimanual-robot/rear_reobserve_and_table_retry_20261021/`의 `cycle_events.jsonl`, `current_scan_diagnostic.json`, `deployment.json`, `state_before.json`, `state_after.json`이다. Pi 복원본은 `$HOME/.local/state/jdamr/rear-reobserve-and-table-retry/20261021_122428/`이며 idle executor 정지 후 이 4경로만 복구한다.
- **현재 대기:** 사용자의 수동 재배치 통보 때문에 이전 위치초기화의 현재 유효성은 해제하고 등록 HOME·기존 AMCL/초기화 기록은 보존했다. 재출발 때 수동 재배치 후의 기존 도크 초기화 경로를 사용하며 도크를 새로 가르치지 않는다. 새 자동 출발은 없다. base/Nav2와 idle 실행기는 다음 요청 대기로 유지하고 전체 종료는 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. 임시 관측/probe는 종료됐다. 브랜치 로컬 커밋만 남기고 push/main 병합은 하지 않는다.


### 22.30 2026-10-21 terminal AMCL 재초기화·기울어진 clearance·중간 최종 정밀도 요구

- **실행/미완료:** `table_01_20261021_133245_3e51a829`는 수동 재배치 후 기존 도크 init을 거쳐 급수대·1번 테이블과 도크 앞까지 이동했다. 1번 테이블은 `table_near_alignment_correction` 뒤 확인·이탈로 이어져 전체 후진 재접근이 없었다. 후면은 raw gap 0.819 m에서 추가 0.2 m 접근 2회 후 면을 얻었으나 landmark seed 적용 뒤 `rear dock landmark localization not retained`로 끝났다. 당시 약 4.3 ms future TF 경고는 확인했지만 seed 유지 실패의 단독 원인으로 확정하지 않는다. `133614_efdae838` 후면 재개는 curve에서 105(`FAILED_TO_MAKE_PROGRESS`)로 끝났다. 두 run 모두 후면 도킹 완료가 아니다.
- **전후/좌우 반전의 확인 원인:** 재개의 면 각도 0.606561 rad(약 35°) 상태에서 clearance 0.589133 m를 전진해 횡오차 +0.171420→-0.163480 m가 됐다. `sin(angle)*distance` 약 0.336 m가 관측 변화 0.334901 m와 부합한다. 같은 면을 찾은 뒤 기울어진 로봇 방향으로 긴 직선을 실행한 것이 좌우를 크게 가로지르는 동작을 만들었다. 최종 정지 LiDAR 3프레임은 gap 0.156942–0.157621 m, 횡차 0.041903–0.045212 m, 각도 약 -9°였고 fresh odom은 v/w 0이었다. 이 값은 내부 관측이며 외부 실측이 아니다.
- **중앙 기준·제어 수정:** 사용자는 정확한 중앙이 필요 없고 일정 오차 내면 된다고 정정했다. 운영값을 좌우 5 cm로 잡아 이내면 추가 중앙 이탈·곡선을 생략한다. 최초 coarse는 기존 0.5 m handoff로 돌아가며, 면 미검출의 기존 추가 접근은 유지한다. gap 5 cm 이상에서 큰 각도는 긴 clearance 전에 기존 회전 경로로 정렬하고 fresh 동일면을 확인한다. 곡선 뒤에도 같은 heading 처리를 사용한다. 기존 근접 접촉 처리와 CM·정적 전체 외곽·scan/link/stop 경로는 유지한다.
- **위치 확인과 인계 수정:** 후면 terminal은 live 면을 full-width/reference로 검증한 뒤 odom으로 제어하며 AMCL reset·map pose 정밀 확인·seed 유지 차단을 실행하지 않는다. 비terminal 보정·출발 초기화는 유지한다. RPP reversing은 목표에서 제자리 yaw 회전을 할 수 없는데 곡선 중간 목표에 최종 yaw/위치 checker를 적용하고 있었다. 기존 position-only `dock_position_checker`와 비최종 odom 종점 5 cm 인계를 사용하며 action 취소/종료 확인 뒤 fresh 면·중앙 검증과 1 cm fine으로 넘긴다. 인계는 최종 주차 성공 판정이 아니고 최종 간격·양쪽 모서리 기준을 완화하지 않는다.
- **복귀 경로 관찰:** 테이블 후퇴 이탈 1회, 도크 앞 방향 전환 174.399° 요청/174.318° odom 관측 1회였다. 다음 9.252°는 측정만 했고 추가 회전 이벤트는 없었다. 복귀는 급수대 waypoint 재실행 없이 도크 stage를 계획한다. 해당 회전은 후면 진입 방향 전환이며, 이번 기록에 모든 순간 조향/전후 명령 trace가 없으므로 복귀 이동 중 사용자 관찰 전체를 재현했다고 쓰지 않는다. 이번 수정은 위 terminal/clearance의 확정된 원인에 한정한다.
- **검증/배포:** 관련 8스위트 881 PASS(6.36초), fatal flake8·compile·diff PASS 및 독립 code-reviewer APPROVE다. terminal AMCL 미호출, 좌우 4.9 cm의 곡선 생략, 35° 이탈 전 정렬, 비최종 위치 인계의 yaw 독립성과 범위 밖·stop/nav/link·취소 실패 보존을 확인했다. Pi 두 모듈 source/install 4경로 SHA·compile·import를 대조했고 idle executor만 PID66113으로 재시작했다. base PID31767·Nav2 PID47789·active identity는 유지했고 배포 전후 fresh odom 각 3표본 v/w 0 및 pending/running 0을 확인했다. 새 이동은 없으며 수정본의 실차 재검증은 남아 있다.
- **근거/복원/현재:** PC `$HOME/.local/state/bimanual-robot/departure_20261021_133245/`에 두 raw run JSONL, `rear_failure_events.jsonl`, `rear_resume_status.json`, `deployment.json`, `state_before_repair.json`, `state_after_repair.json`을 보존한다. Pi 백업은 `$HOME/.local/state/jdamr/rear-relative-dock-tolerance/20261021_135302/`이며 idle executor 정지 후 4경로만 복원한다. 임시 probe는 종료됐고 다음 후면 도킹 요청 대기로 기존 base/Nav2/idle 실행기를 유지한다. 전체 종료는 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. 현재 위치를 등록 HOME으로 덮어쓰거나 자동 재출발하지 않았다. 브랜치 로컬 커밋까지만 수행하고 push/main 병합은 하지 않는다.

### 22.31 2026-10-21 복귀 clearance에 출발 계약 적용·중앙 잔차에서 상위 실행 종료

- **현상/근거:** 전체 실행 `table_01_20261021_161945_ca889276`가 급수대·1번 테이블을 마치고 도크에서 `dock departure requires a positive distance and first home_exit`로 끝났다. 후면 재개 `162345_6715a304`는 중앙 오차 -0.229945 m에서 -0.113492 m로 줄었지만 `rear_box_center_not_reached`로 끝났다. 다음 `162525_a5bcf9c2`는 -0.038205 m로 보정하고 접근했으며 사용자가 도킹 상태를 긍정 확인했다. 추가 보정은 사용자 확인 뒤 중지했고 자동 주차 confirmed=false, 정리의 camera_stop_failed도 보존한다.
- **확정 원인/조치:** 도착 보정 공간 확보가 실제 출발용 `_depart_dock_straight`를 호출했고, 그 래퍼는 `home_exit`와 두 개 이상 waypoint를 요구했다. 공통 `_advance_dock_straight`를 분리해 후면은 `_clear_rear_dock_straight`로 실행하며 출발 reference 기록·캐시를 재사용하지 않는다. 중앙 보정은 한 곡선 뒤 잔차가 남았다는 이유만으로 종료하지 않고 fresh 동일면·기존 task deadline·stop 범위에서 같은 실행으로 다시 계산한다. 최종 1 cm·중앙 5 cm 및 기존 CM/전체 외곽/취소 종료 검사는 유지한다.
- **검증/미확인:** 관련 501 PASS, fatal lint·compile·diff PASS, 독립 code-reviewer APPROVE. 파이 두 모듈 source/install SHA 대조, idle executor PID71361 active, base PID31767·Nav2 PID47789 유지, pending/running0이다. 첫 배포에서 임시 유닛 stop 후 일반 start 실패→원복했으며 기존 start_executor 경로로 최종 복구했다. 이 수정 뒤 새 이동은 없고 단일 실행 완주·자동 최종 판정은 실차 미확인이다. camera_stop_failed와 마지막 swept-footprint 대기의 원인을 이 수정으로 해결했다고 주장하지 않는다.
- **원본/복원:** Pi `$HOME/jdamr_data/map_20260930_manual/aligned/runs/`의 위 run별 cycle_events.jsonl, PC `$HOME/.local/state/bimanual-robot/dock_clearance_20261021/`, Pi 백업 `$HOME/.local/state/jdamr/rear-dock-clearance/20261021_163844/`. 새 상주 진단기는 없다. 기존 base/Nav2/idle executor는 다음 실행을 위해 유지하며 전체 종료는 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다.

### 22.32 2026-10-21 후면 도킹 도중 약 49 cm 전진 후 재후진

- **현상/근거:** `table_01_20261021_164729_07911177`의 rear coarse handoff gap 0.484132 m, 중앙 횡오차 -0.078844 m에서 clearance 0.489206 m를 요청했다. 같은 실행의 정렬 뒤 횡오차 -0.031471 m였고 사용자는 도킹 상태를 긍정 확인했지만 큰 앞뒤 이동을 지적했다. 마지막 fine의 swept-footprint 대기 후 추가 보정은 중지했으며 자동 최종 주차 확인은 통과하지 않았다.
- **원인/수정:** 기본 coarse 0.5 m와 중앙 오차 5 cm 초과 시 적용하던 1 m 공간 확보의 순서가 서로 충돌했다. 복귀의 reference 원점·normal 축을 유지해 stage를 최소 기존 center handoff 1 m로 잡고, coarse 관측/정렬도 1 m에서 시작한다. 이미 현재 pose에서 기존 최소반경 0.3 m 후진 곡선이 가능하면 forward clearance를 생략한다. invalid/nonfinite/2 m 초과 기하는 기존 오류를 유지하고, 실제 곡선이 불가능한 두 기하 사유만 기존 공간 확보로 보낸다. static 전체 외곽·CM·동일면·stop/deadline·중앙 5 cm·최종 1 cm는 유지한다.
- **검증/범위:** 관련 505 PASS와 독립 505 PASS/APPROVE, fatal lint·compile·diff PASS. 합성 gap 0.85 m/횡오차 ±0.23 m에서 전진 없이 후진 중앙 정렬→fine 인계를 확인했다. full-face를 못 얻는 near recovery, 실제 기하가 부족할 때의 공간 확보, fine 20 cm clearance는 남아 있어 항상 단일 후진이라고 쓰지 않는다. 수정 후 새 실차 주행은 없다.
- **배포/복원:** PC `$HOME/.local/state/bimanual-robot/rear_reverse_entry_20261021/deployment.json`에 원본 실행 선택 이벤트·2경로 SHA·compile/import exit0·PID·정지3표본·pending/running0을 보존했다. Pi source/install 모듈 일치, idle PID74615, base31767/Nav2 47789 유지다. 백업 `$HOME/.local/state/jdamr/rear-dock-reverse-entry/20261021_170158/`. 임시 관측기는 종료했고 기존 3서비스는 다음 실행을 위해 유지한다. 전체 종료는 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다.


### 22.33 2026-10-21 중앙 잔차 재전진 반복·최종 간격 도달 뒤 이동 예측 대기

- **현상/원본:** `table_01_20261021_175440_87661747`의 중앙 곡선 후 횡오차 +0.057698 m, gap 0.188589 m에서 중앙 재시도가 시작됐고, 각도 정렬 뒤 clearance 0.788528 m를 요청했다. 처음에도 0.646248 m clearance가 있었다. stage/coarse 1 m 변경만으로 full-face 확보 시점과 큰 재전진 문제가 해결되지 않았다. face 미검출 로그는 `tangent spread is too narrow`와 support 부족이며, 저장 스캔 없이 최초 오검출 물체의 종류를 단정하지 않는다.
- **구분한 원인:** 보정 후 잔차가 5 cm를 넘으면 동일 while 루프에서 다시 큰 clearance를 만드는 것은 소스·실행 로그로 확인했다. 종점 인계도 5 cm여서 같은 5 cm 중앙 허용 기준을 안정적으로 만족할 여유가 없었다. fine에는 별도로 근접 각도 잔차 시 20 cm 전진과 terminal 재접근 분기가 남아 있었다. 후면의 큰 앞뒤 움직임은 이 제어 순서와 반복 분기로 설명되며, 카메라 각도 오류로 분류하지 않는다.
- **정지 후 재개 근거:** `table_01_20261021_180116_f1b5f0ea`의 요청 후진 0.200463 m, odom 이동 0.214025 m, 마지막 raw gap 0.011634 m/angle -0.000785 rad를 확인했다. `rear docking swept footprint occupied` 후 입력 대기를 지속했고 자동 최종 확인까지 가지 못했다. PC `stopped_scan.json` 40프레임/정지 odom 91표본의 재생 gap 범위는 0.010985–0.011888 m, 마지막 양끝 0.011729/0.011780 m였다. 정지 표본 최대 |v|=0.001262 m/s, |w|=0.004948 rad/s로 기존 정지 허용 안이며 완전한 0은 아니다. 배터리는 11.684–11.692 V였고 이번 실패의 배터리 차단 근거는 없다. 라이다 추정과 외부 정확도 검증은 구분한다.
- **조치:** position-only checker를 유지하고 곡선 endpoint의 odom handoff만 2.5 cm로 좁힌다. 이후 중앙 잔차에서 큰 전진을 반복하지 않고 사유를 남긴다. fine의 전진 clearance·자동 재주차를 제거한다. fresh 동일면을 현재 odom에 투영한 양끝이 기존 최종 1 cm±3 mm 안이면 명시적 `target_confirmed`를 전달하고, 필터 잔차/heading 정렬 상태보다 우선해 속도 0과 정지 확인으로 끝낸다. rear recovery도 guard/link/유효 target 뒤 도착이면 추가 이동의 collision preview를 요구하지 않는다. 멈춘 뒤 fresh 면으로 최종 확인하며 실제 장애물 무시나 성공 판정 위조를 하지 않는다.
- **검증/범위:** 관련 512 PASS(5.74초), fatal lint·compile·diff PASS. 변경 전 중앙 반복·fine 전진·필터 잔차/도착 뒤 영구 watch 대기를 재현했고 수정 뒤 해당 회귀가 통과했다. 7 mm/0.7°의 기존 최종 양끝 허용 안에서 heading trim이 도착을 막는 검수 지적도 수정했다. 최초 곡선이 불가능할 때의 초기 clearance는 남아 있고, 수정본 실차 단일 후진 반복성은 미검증이다. swept-footprint에 걸린 원시 점의 정체 자체를 확정한 수정은 아니다.
- **근거 경로:** PC `$HOME/.local/state/bimanual-robot/rear_no_terminal_replay_20261021/`의 `stopped_state.json`, `stopped_scan.json`, `geometry.yaml`, `stopped_face_estimates.json`. 그 이전 선택 로그는 `$HOME/.local/state/bimanual-robot/rear_reverse_entry_20261021/failed_second_run.json`에 보존한다. 새 주행·위치초기화·HOME 변경은 하지 않았다.
- **독립 검수/정지 반영:** code-reviewer 최종 APPROVE, 별도 512 PASS(5.49초). Pi 두 모듈 source/install 4경로 SHA·compile/import 확인 후 idle executor PID80464만 재기동했다. base PID31767·Nav2 PID47789는 유지했다. pending/running0이며 배포 뒤 odom 3표본은 기존 정지 허용 안이었다(첫 |v|=0.001262 m/s/|w|=0.004948 rad/s, 이후 2개 0). 배터리는 새 표본 11.620–11.632 V다. 추가 주행·센서 재시작·위치초기화는 하지 않았다.
- **복원/프로세스 정리:** Pi `$HOME/.local/state/jdamr/rear-no-terminal-replay/20261021_182235/`의 `source_*.py`/`install_*.py`가 복원본이며 먼저 idle executor를 정지하고 원래 4경로에 복원한다. PC `deployment.json`, `after_deployment_state.json`, `after_deployment_scan.json`에 반영 근거를 남겼다. 임시 읽기 노드는 종료됐다. 다음 사용자 출발을 위해 기존 base/Nav2/idle executor를 유지하며 전체 종료는 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다. 브랜치 로컬 커밋만 남기며 push/main 병합은 수행하지 않는다.


### 22.34 2026-10-21 도크 맞은편 의자 다리 충돌 보고와 순수 회전 감시 공백

- **사용자 관찰:** `table_01_20261021_184007_d1467858` 복귀 후 사용자가 도크 맞은편 의자 다리 충돌을 보고했다. 물리 충돌이 있는 실패 주행으로 기록한다. 구조화 로그에 접촉 순간의 독립 측정이 없어 정확한 시점/접촉 차체 부위/의자 이동량은 모른다. 이동/회전/후진 중 동작 확인을 요청했지만 이 기록 시점에는 답이 없었다.
- **실행 로그:** 급수대 최종 양끝 추정 gap 10.632/10.499 mm, 1번 테이블 10.133/8.429 mm를 확인한 뒤 도크 앞 `home_dock_approach`가 18:42:34.255에 성공했다. 34.954에 -0.997919 rad 회전을 요청하고 38.893에 odom -0.997781 rad를 기록했다. 39.126 coarse가 0.480171 m 후진을 계획했으나 39.147 정적 지도 검사에서 종료돼 해당 후진 구동은 시작하지 않았다. 마지막 정지가 앞선 이동·회전의 무충돌을 증명하지 않는다. 사용자 보고를 센서/지도 결과로 부정하지 않는다.
- **확인한 코드 공백/가설 분리:** `_rotate_in_place`는 `_direct_motion`의 CM/guard/link 경로를 썼지만 `watch`를 전달하지 않았다. `_scan_in_base`/`_remembered_blind_points`로 보존하는 근거리 점은 escape와 rear fine에는 쓰였고 이 순수 회전에는 연결되지 않았다. 실제 `/scan.range_min`은 0.280000001 m다. 가까워져 blind range 안에 들어온 다리 감시가 빠질 수 있다는 공백은 확인했지만, 이번 의자 다리가 그 경로로 사라졌다는 원시 시계열 근거는 없다. 의자 충돌 시점/최초 원인을 확정하지 않는다.
- **현재 설정 조회 한계:** 충돌 뒤 알려진 두 파라미터의 조회에서 StopZone.enabled/FootprintApproach.enabled는 둘 다 true였다. 여러 이름을 한꺼번에 조회한 첫 응답은 values0이었고 이를 CM 비활성 증거로 사용하지 않는다. 현재 enable 확인도 충돌 순간 각 점/명령에 대한 CM 개입 여부를 재구성하지 못한다. max_decel은 [-0.5,0,-1.5], 실제 stepped footprint/padding0을 별도 저장했다.
- **수정:** 기존 rear collision 예측 엔진의 상태 구성만 공용화해 `rotation_collision_watch`를 추가했다. 실제 scan/TF/outline과 blind memory, commanded/odom 중 큰 속도, 실제 smoother 감속 및 기존 guard/stop 지연으로 회전 sweep을 확인한다. 첫 비영 명령 전과 기존 0.1초 guard 주기에 callback을 실행한다. 회전 watch는 target=None이고 CM 파라미터를 바꾸지 않으며 목표면 예외가 없다. 입력/footprint/감속 조회 실패는 원인을 남기고 비영 명령을 시작하지 않는다. 실제 충돌 예측이면 회전을 중지하고 자동 전후진/새 회피 기동으로 바꾸지 않는다.
- **검증/한계:** 관련 rear profile/restaurant/rear box/reference 4스위트 561 PASS(5.82초), range_min=.28 밖의 live 단일 점과 안의 memory 점을 양쪽 회전에서 확인하는 강화 회귀 포함 7 targeted PASS(0.54초), fatal lint·compile·diff PASS. 기존 CM 복원/목표면/입력/stop 계약도 통과했고 독립 code-reviewer APPROVE다. 이 검증은 합성 입력과 코드 경로 검증이며 실제 의자 충돌 재현이나 원인 확정이 아니다. Nav2 RPP 이동 구간의 별도 회피 감시를 변경한 작업도 아니다. 도크 coarse 정적 지도 차단은 남아 있다.
- **정지 반영/복원:** Pi rear_collision_profile.py/restaurant_service.py source/install 4경로 PC SHA·compile/import를 확인하고 idle executor PID83287만 재기동했다. base31767·Nav2 47789 유지, pending/running0, 배포 뒤 fresh odom3개 v/w0이다. 정지 후 새 실물 동작·위치초기화·충돌 검사 우회는 하지 않았다. 읽기 노드는 종료했고 base/Nav2/idle executor만 다음 요청을 위해 유지한다. 전체 종료는 Pi `sudo systemctl stop jdamr-box-executor jdamr-restaurant-navigation jdamr-base`다.
- **근거:** Pi 원본 run `cycle_events.jsonl`, PC `$HOME/.local/state/bimanual-robot/chair_collision_20261021/`의 `run_and_journals.json`, `current_parameters.txt`, `cm_enabled_only.txt`, `deployment.json`, `after_deployment_state.json`, `after_deployment_scan.json`. 충돌 후 정지 scan은 `$HOME/.local/state/bimanual-robot/rear_no_terminal_replay_20261021/blocked_184007_scan.json`이다. 복원은 idle executor 정지 뒤 Pi `$HOME/.local/state/jdamr/rotation-blind-leg-watch/20261021_185720/`의 source/install 복사본을 원래 4경로에 돌린다. 브랜치 로컬 커밋만 남기며 push/main 병합은 하지 않는다.


### 22.35 2026-10-27 수동 조종 ACK 공백·지도 기록 실패와 관측 셀 갱신

- **기록 ID/증상:** `AMR-20261027-webteleop-ack-gap`, `AMR-20261027-manual-map-recorder`. 사용자는 수동 조종 중 간헐적 끊김을 보고했다. 조종 서비스 재시작은 0회였고 당시 명령별 수락 시각·충돌 감시 기록은 없어 개별 실물 정지의 단독 원인은 미확정이다. 별도 SQLite 기록기는 `SQLite error (5): database is locked`로 종료됐고 scan 259장·odom 1,386개, 약 27.70초의 정지 데이터만 남았다. 사용자 한 바퀴 주행 데이터가 있다고 기록하지 않는다.
- **확인/수정:** 상태 GET RTT 15회는 129~620 ms·중앙값 약 246 ms였다. 기존 단일 ACK 대기가 100 ms keepalive를 지연시켜 350 ms deadman 정지를 만들 수 있는 코드를 분리된 편도/응답 지연으로 재현했다. 최대 3개 요청·역순 거절·긴급 정지 전체 취소·HTTP/1.1 지속 연결로 수정했다. 300 ms 요청 timeout과 350 ms deadman, 충돌 감시는 유지했다. 기록은 기존 표준 MCAP으로 전환해 약 40.86초·scan 396장·metadata 정상 마감을 확인했다. 실행 중 SQLite 조회와 잠금의 인과는 가설이며 기록기의 SQL 예외와 누락은 확정 사실이다.
- **남은 데이터 반영:** 등록 테이블1 고정 자세와 실제 finite 광선으로 명시 ROI의 관측 셀만 갱신했다. 230셀 변경(기존 점유→free 82, 새 점유 21, unknown→free 127), ROI 밖 변경 0셀이다. 삭제된 점유 셀은 별도 검토에서도 hit 0회·free 28~259회였다. 지도 격자·좌표계·세 등록점·원본 지도·pbstream을 유지했다. 새 지도·keepout을 Pi에서 LoadMap하고 반환 전체 격자를 파일과 대조했다. 이는 부분 점유지도 갱신이며 포즈 그래프 재SLAM이나 미관측 테이블 뒤쪽 갱신은 아니다.
- **검증/한계/복원:** 관련 27 PASS, fatal lint·문법·diff PASS, 별도 code-reviewer APPROVE. 수정 후 실차 연속 조종은 미확인이다. Pi 조종 소스 복원본은 `$HOME/jdamr_deploy/webteleop_ack_backup_20261027_101956/`, 지도/등록/런타임 복원본과 원시 캡처·처리/활성화 영수증은 PC·Pi `$HOME/jdamr_data/maps/20261026_manual_160135/local_updates/table_01_refresh_20261027_100353/`에 있다. 기록기 두 개와 읽기 노드는 종료했고 사용자 조종 화면·RViz 표시만 유지한다. 새 이동·도크 초기화는 하지 않았다.

### 2026-10-27 새 지도 급수대 접근 재현과 조치

- **도크 출발:** `table_01_20261027_115257_ab451911`은 0.7 m 직선 이탈의 정적 지도 검사에서 미이동 종료했다. 같은 날 `table_01_20261027_115429_48ed2937`은 동일한 `home_exit` 경유점을 Nav2로 통과해 급수대에 도착했다. 이 지도 세션에는 `nav2_waypoint` 이탈을 선택하고 다른 지도는 기존 직선 이탈 기본값을 유지한다. Nav2 경유점은 직선 이탈의 물리적 궤적과 동일하다고 확인된 것은 아니다.
- **급수대:** 위 두 번째 실행에서 0.293 m 전진 뒤 새 깊이 관측의 앞 모서리 간격 추정은 25.03/16.35 mm, 좌우 참조 오차 추정은 -9.93 mm였다. 근접 보정은 `table approach preview did not converge`로 구동 전 멈췄다. `table_01_20261027_115932_4fe64a27`도 약 0.28 m 접근 명령을 내기 전 같은 오류로 멈췄다. 이 값은 내부 추정이며 외부 실측이 아니다.
- **확인된 코드 원인:** 경로 미리보기가 근접 구간에서 좌우 목표를 향하는 조향과 면에 평행해지는 조향을 동시에 요구했다. 두 방향이 반대일 때 회전 명령이 0이 되어 같은 상태를 반복했다. 기록의 위치와 면을 재현한 계산에서도 원래 코드가 1,600회 후 같은 오류를 반환했다.
- **조치:** 실시간 제어와 미리보기가 같은 곡률 기반 좌우 접근을 사용하고, 회전 속도에 맞춰 이동 속도를 계산한다. 근접한 상태에서 목표 좌우 오차를 줄일 공간이 없으면 이유를 분리해 기존 후진·재관측 경로를 사용한다. 최종 앞끝 1 cm, 양 코너·좌우 판정과 센서·충돌 감시는 유지한다. 로그에 실제 실패 이유가 남도록 상위 메시지도 수정했다.
- **별도 미확정:** `table_01_20261027_115752_cb26a2f3`의 `odometry did not confirm stop`은 한 차례 확인됐다. 당시 로그에는 정지 확인 구간의 odom 표본 수와 속도가 없어 센서 누락과 실제 잔여 움직임 중 원인은 확정하지 못했다. 같은 경우 표본 수·나이·속도를 남기도록 진단만 추가했다.
- **검증 범위:** 기록값 기반 경로 재현과 모의 제어에서 위 비수렴을 재현·수정했다. 새 깊이 카메라의 설치 보정과 실제 앞끝 1 cm·좌우 5 mm 정확도, 수정 후 전체 왕복은 실차 미검증이다. 사용자가 도크로 이동한 뒤 주행을 정지했으며 새 주행은 수행하지 않았다.

### 2026-10-27 도크 정합 기준 덮어쓰기 수정

- **관찰:** `init_20261027_121739`은 10:48의 지도 정합 fallback에서 파생된 스캔을 도크 기준으로 사용했다. 같은 입력 스캔을 등록 원본 도크 스캔으로 다시 정합한 결과는 해당 초기화와 약 4.6 cm 떨어졌다. 두 수치는 스캔에 근거한 추정이며 실제 절대 위치 오차의 실측은 아니다. 원본 대조는 matched support 143/124개, 평균 잔차 약 12 mm로 기준을 통과했다.
- **확인된 원인:** taught-map 세션에서 지도 해시가 같으면 상태 파일에 남은 파생 기준을 계속 사용했고, 등록 도크의 지도 fallback이 성공하면 그 스캔과 추정 위치로 기준을 덮어썼다. 그 결과 등록된 원본 도크의 좌표 기준과 이후 기준이 달라질 수 있었다.
- **조치:** taught-map 세션은 포인터에 등록된 원본 도크 스캔을 고정 기준으로 복원하며, 기존 상태의 기준이 달랐다면 한 번만 위치추정을 무효화해 다시 초기화한다. 지도 fallback도 원본을 갱신하지 않는다. 종전 legacy-map 경로는 유지한다.
- **비이동 초기화:** `init_20261027_122859`에서 원본 도크 스캔을 사용했다. 현재/원본의 5 cm 안쪽 매칭 지지점은 134/116개, 현재 기준 비율은 69.8%였다. 정합 위치와 AMCL의 차이는 약 2 mm·0.5°다. HOME 좌표는 변경하지 않았고 새 주행은 하지 않았다. 수정된 기준의 전체 주행과 실측 도착 정확도는 미검증이다.
- **정지 중 AMCL:** 정지 시 AMCL 메시지가 갱신되지 않을 수 있다. 갱신 서비스 호출 전후 위치 차이는 약 2.4 mm·0.009°였고 odom 속도는 0이었다. 실행기의 기존 정지 상태 예외와 유효 TF·스캔 검사는 유지한다. 원본 기록은 `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/init_20261027_122859/`이다.
- **재계산한 경로:** 위 위치초기화 뒤 `ComputePathThroughPoses`의 `GridBased`로 도크 이탈점→급수대 관측점 경로만 비이동 계산했다. 결과 code 0, 94개 경로점, 마지막 경유점 오차 0 m다. 실제 추종이나 주차 성공을 뜻하지 않는다.

### 2026-10-27 전체 주행 재실패: 전면 목표 좌표 시점·카메라 자세 분리·후면 순간 검출

- **사용자 관찰:** 급수대의 최종 간격·좌우 정밀도가 부족했고, 1번 테이블은 근처에서 접근을 끝내지 못했으며 도크 복귀도 완료하지 못했다. 실행기 내부의 급수대 완료를 실측 정밀주차 성공으로 해석하지 않는다. 사용자의 정지 뒤 수정·비이동 검증만 수행한다.
- **근거:** `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/` 아래 `table_01_20261027_143504_89611adf`, `table_01_20261027_143829_e8c3a7d8`, `table_01_20261027_144912_946e66bb`의 `cycle_events.jsonl`. 마지막 run 폴더의 `stopped_scan.json`에는 이후 정지 스캔 40장·static TF·odom이 있다. 먼 거리에서 실패한 순간의 원시 스캔은 없어 정지 후 데이터로 당시 물체를 단정하지 않는다.
- **지도 확인:** 부분 갱신은 1번 테이블 주변 230셀뿐이며 지도 원점·해상도·격자 크기와 등록 좌표는 유지됐다. 도크는 변경 영역 밖이다. 전역 좌표계가 이동했다는 증거는 없다. 변경된 점유 셀이 AMCL 보정에 미친 영향량은 당시 `map→odom` 시계열이 없어 미확정이다. 원본 근거는 `local_updates/table_01_refresh_20261027_100353/map_update_receipt.json`이다.
- **뎁스 사용 사실:** 급수대·1번 테이블에서 상판 경계 관측과 이동 중 `table_approach_face_updated(reference_frame=odom)`를 확인했다. 깊이와 CameraInfo는 320×240·같은 광학 프레임이며 검출 상판 높이는 약 0.754–0.763 m, 경계 RMS는 약 1–2 mm였다. 이 적합 오차는 실제 차체 앞끝 거리의 절대 정확도가 아니다.
- **전면의 확인된 코드 결함:** 면과 시작점은 접근 시작 시점의 `map→odom`으로 고정하지만, `map_pose_only` 목표는 제어 중 최신 변환으로 재계산했다. 같은 면에 대한 목표만 움직일 수 있어 5 mm 좌우 판정과 충돌했다. 기록 기하를 같은 좌표계로 재생하면 두 접근 모두 가능하지만, 목표를 접선 방향 15 mm 이동시키면 같은 clearance 오류가 재현된다. 실제 실행 당시의 목표·변환 이동량은 로그에 없어 이 수치가 실차에서 발생했다고 단정하지 않는다. 목표·면·경로 검증·최종 좌우 판정이 같은 고정 목표를 사용하도록 수정하고, 실패 시 실제 odom 입력·목표·차체 제원을 기록한다. 12° 각도 자체를 실패의 확정 원인으로 삼거나 허용오차를 완화하지 않는다.
- **카메라 자세의 확인된 불일치:** 주차 런타임은 새 Astra 참조 상판에서 유도한 pitch 48.2128°·roll 1.0530°를 사용했지만, URDF는 이전 Astra S의 pitch 52.55°·roll -2.24°였다. 새 참조면 후보를 명시적으로 선택해 canonical·URDF·VSLAM을 일치시키고, 출발할 때 교시 데이터에서 장착 자세를 재생성하는 동작을 제거했다. 이전 Astra S의 yaw·거리 보정은 적용하지 않는다. 높이 1.26 m·렌즈 앞뒤 위치는 유지했으며 외부 장착 보정·밀리미터 정확도는 여전히 미검증이다. 자세값 두 개의 일치만으로 급수대의 실측 오차가 모두 해결됐다고 주장하지 않는다.
- **등록 위치의 한계:** 저장된 탐색용 면은 등록 차체 좌표와 제원에서 유도한 값이며 실측 면이 아니다. 최초 깊이 면과의 법선 방향 차이는 급수대 약 11.4 cm·1번 테이블 약 22.0 cm였다. 이 차이를 없애려고 사용자 등록점을 다시 쓰지 않는다. 최종 거리는 관측 면, 좌우는 등록 기준을 유지하며, 표식 없는 1번 테이블의 외부 ±5 mm 반복 정확도는 미검증이다.
- **후면의 확인된 코드 결함:** 약 0.98 m 구간에서 0.2 m 후진 요청 9회가 발생했지만 raw 간격 감소는 약 2.7 cm였다. 한 프레임 면 검출만으로 후진을 멈추고 정지 후 안정화는 실패해 짧은 후진·대기를 반복했다. 이동 중 같은 물리면을 odom에서 서로 다른 8개 스캔으로 추적한 뒤에만 면 제어로 넘긴다. 기존 raw corridor·정적 외곽·충돌 감시·이동 한도는 유지한다. 이후 정지 데이터는 40/40 검출, 내부 간격 평균 13.75 mm·변동폭 1.12 mm였지만 실제 복귀 완료나 먼 거리 검출 성공의 근거는 아니다.
- **별도 정지 확인:** 1번 테이블 재시도에서는 좌우 clearance 오류 뒤 `odometry did not confirm stop`이 최종 표시 사유가 됐다. 원래 제어 오류는 `precision_straight_finished.target_failure`에 남아 있다. 잔여 움직임과 속도 추정 잡음 중 어느 것이 정지 확인을 깨뜨렸는지는 미확정이며 임계값을 풀지 않았다.
- **소프트웨어 검증:** 전면·교시 기준·곡률 접근·후면·카메라·URDF·출발 흐름·VSLAM 계약 관련 10개 파일 675 PASS. 좌우 목표에 새 AMCL 보정이 들어와도 접근 목표가 움직이지 않는 회귀와, 순간 면 검출이 후진을 끊지 않는 회귀를 포함한다. 실차 재주행과 실제 1 cm·좌우 정확도는 별도 확인이 필요하다.
- **정지 반영:** Pi의 두 주차 모듈·카메라 설정·URDF·출발 도구와 활성 세션 장착값을 반영했다. `robot_state_publisher.robot_description`의 기존 카메라 관절 RPY만 갱신해 실제 `base_link→camera_link`의 높이 1.26 m와 선정 자세 일치를 확인했다. 베이스·Nav2는 재시작하지 않았고 odom 5표본은 선속도·각속도 모두 0이었다. 상주 실행기는 transient 유닛이라 일반 `systemctl start` 복원이 실패했으며 기존 `start_executor()` 경로로 재생성했다. 복원본·TF 변경 전 URDF·배포 영수증은 Pi `$HOME/.local/state/jdamr/parking-coordinate-and-mount/20261027_151558/`, PC 영수증은 `$HOME/.local/state/bimanual-robot/parking-coordinate-and-mount-20261027/deployment_result.json`에 남긴다. 실물 이동 명령·도크 좌표 변경·위치초기화는 수행하지 않았다.
- **운영 선호 저장:** 이후 사용자 요청에 따라 `src/navigation/AGENTS.md`에 AMR 작업의 별도 리뷰·검수·검토 절차를 자동 실행하지 않고 기능 수정·실제 반영을 우선하도록 기록했다. 추가 리뷰나 반복 검사를 출발 조건으로 넣지 않는다.

### 2026-10-27 도크 준비 중 전역 후보 불일치 거부 수정

- **증상:** 사용자 도크 배치 확인 뒤 초기화에서 도크 주변 지도 정합은 등록점과 약 3 cm·2.8° 차이, 5 cm 내 지지율 0.49였지만 전역 탐색의 다른 후보 때문에 거부됐다. 수정 전 실행은 `runtime/runs/init_20261027_152523/`이다.
- **수정:** 등록 도크로 시작하는 taught-map 초기화는 도크 주변 정합과 기존 원본 스캔을 사용한다. 원본 스캔을 못 쓰는 경우에도 정상 지도 지지율 0.45 이상과 기존 도크 범위 20 cm·15°를 만족하면 전역 후보 불일치만으로 거부하지 않는다. 이 경우 불필요한 전역 탐색도 수행하지 않으며 `registered_dock_local_map`으로 근거를 구분한다. 일반 임의 위치 초기화의 전역 탐색과 실제 정합 불량 거부는 유지한다. 기존 미이동 controller 복구도 이 도크 초기화 근거를 인정하도록 연결했다.
- **실제 비이동 적용 결과:** `init_20261027_152820`에서 로컬 지지율 약 0.48, 등록점 차이 약 4 cm·2.3°로 초기화했다. AMCL과 정합 위치의 차이는 약 8 mm·0.3°였고 기존 activation 경로가 controller 응답·설정을 확인해 정상 종료했다. 등록 HOME 좌표는 변경하지 않았으며 두 테이블 경로를 갱신한 뒤 `localized=true`를 저장했다. 이동 명령은 보내지 않았다. 이는 주행 준비 완료이며 실제 주차 정확도 검증은 아니다.
- **운영 규칙:** 오류 수정 후 도크 배치가 확인된 경우 지도 정합·위치초기화·controller 연결까지 수정 단계에서 완료하고, 이후 출발 요청에서는 준비된 실행 경로를 사용하도록 `src/navigation/AGENTS.md`에 기록했다.


### 2026-10-27 급수대 단독 재접근과 새 Astra 실측 거리 보정

- **주차 실패 후 행동:** 불완전한 주차를 자동 도크 복귀로 바꾸던 분기를 제거했다. 실패한 지점에서 정지하고 남은 주차를 이어간다. `go --park-only`는 해당 정류장에서 정지하며, 제출 표시도 복귀 없는 작업임을 표시한다.
- **확인된 제어 문제:** 이동 중 검출기의 `collecting_stability`가 새 면 갱신을 막던 부분을 수정했다. 최초 정지 관측은 안정된 면을 요구하고, 이동 중에는 기존 품질·시간·동일면 조건을 만족한 새 관측을 사용한다. 근접 보정을 한 번 시도했다는 사실만으로 종료하지 않고 실제 각도·거리 개선 여부를 사용한다. 같은 방문의 재접근에서도 좌우 목표는 고정한다. 후면 동일면 비교는 odom 전역 원점 대신 관측 지지점 주변에서 비교하도록 수정했다. 이 후면 수정의 실차 도킹 결과는 아직 없다.
- **실측 불일치:** `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/water_station_20261027_160027_9a8c634a/cycle_events.jsonl`의 최종 내부 간격은 9.210 mm, 내부 좌우 오차는 8.105 mm였다. 사용자는 양쪽 실제 간격 약 70 mm·목표보다 우측 40 mm·면과 평행하다고 보고했다. 이 실행은 실제 1 cm 성공이 아니다. RGB 보조 좌표가 무효인 동안에도 등록점이 주목표이며 RGB 결과로 등록점을 이동하지 않는다.
- **새 보정:** 같은 정지 관측의 면과 실측 차이 60.790 mm를 현재 Astra 장착에 한정한 `derived` 면 거리 보정으로 canonical·VSLAM 카메라 설정에 기록했다. 장착 TF는 바꾸지 않았다. 단일 배치에서 산출했으므로 카메라 각도·렌즈 위치·깊이 거리 오차·검출 경계 중 어느 것이 주원인인지는 미확정이다. 반복성·밀리미터 정확도는 검증되지 않았다. 이전 Astra S의 보정값은 재사용하지 않았다.
- **승인된 첫 재접근:** 사용자가 정밀도를 맞추기 위한 반복을 승인했다. 현재 급수대에서 `go water_station --park-only --reapproach-m 0.2 --station-left-correction-m 0.04`를 실행했다. 등록 급수대 좌표는 유지하고 해당 실행의 목표만 현재 위치 대비 왼쪽 40 mm로 고정했다. 도크 위치초기화·전체 경로 재실행은 하지 않았다. 기존 후진 경로·충돌 감시·센서 유효성 확인을 사용했다.
- **실행 결과와 남은 부분:** `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/water_station_20261027_161840_dc83f76c/cycle_events.jsonl`에서 최초 후진 후 좌우 보정 공간이 부족해 기존 경로로 추가 후진했다. 첫 후진 뒤 면-축 거리 315.758 mm, 추가 후진 뒤 481.675 mm였다. 접근 중 한 차례 깊이 입력 지연으로 정지했다가 새 관측으로 재개했고 자동 복귀 없이 급수대 앞에 정지했다. 최종 내부 양끝 간격 8.777/8.819 mm, 좌우 오차 3.210 mm, 방향 오차 0.0053°였다. 마지막 깊이 경계는 가려져 frozen odom 기반 확인이며 사용자 실측은 좌 9 mm·우 11 mm, 우측 치우침 20 mm였다. 앞 간격 평균 10 mm는 확인됐지만 좌우 ±5 mm는 미달이다.
- **최소 실행 확인:** 수정 모듈이 파이 상주 실행기에 로드되어 위 단독 재접근을 수행했다. 사용자 지시에 따라 별도 pytest·리뷰·검수는 실행하지 않았다. 파이 복원본은 `$HOME/.local/state/jdamr/water-station-measured-reapproach/20261027_161746/`이다. 상대 보정 목표의 출처와 고정 odom 목표를 로그에 남기도록 실행기에도 반영했다.

- **두 번째 재접근의 가림 문제:** `water_station_20261027_162233_edbde0dc`는 1 cm 근접 상태에서 경계 재검출이 안 돼 이동 전에 종료됐다. `--reapproach-from-log`로 직전 성공 실행의 고정 odom 면을 후진에 한해 사용할 수 있게 수정했다. 같은 베이스 odom 세션·해당 정류장·현재 근접 자세가 맞는 경우에만 사용하고, 후진 이후에는 새 깊이 면을 관측한다. 과거 면을 새 관측으로 표기하지 않는다.
- **두 번째 실제 재접근:** `water_station_20261027_162641_f2f2799a`는 현재 위치에서 30 cm 후진·왼쪽 20 mm 목표 보정을 수행해 내부 양끝 9.161/9.155 mm·좌우 잔차 4.196 mm로 종료했다. 외부 실측은 다시 좌 9 mm·우 11 mm·우측 치우침 15 mm였다. 내부 좌우 허용오차 통과만으로 외부 ±5 mm를 달성했다고 볼 수 없다. 요청한 횡이동량과 외부 실측 변화량이 다른 원인은 미확정이다.
- **방향 보정과 세 번째 실제 재접근:** 두 번 반복된 좌우 간격 차이 2 mm와 차체 모서리 폭 450 mm로 약 0.254°의 면 방향 보정을 유도했다. 거리 보정값은 유지했다. `water_station_20261027_163032_388b62f9`에서 30 cm 후진·남은 왼쪽 15 mm 목표 보정·새 면 방향을 적용한 재접근이 내부 주차 조건을 통과해 정지했다. 외부 실측은 다시 좌 9 mm·우 11 mm·우측 15 mm로, 방향 보정·좌우 목표 보정 후에도 외부 개선이 없었다. 파이 복원본은 `$HOME/.local/state/jdamr/close-range-reapproach/20261027_162619/`, `$HOME/.local/state/jdamr/reapproach-angle-calibration/20261027_163004/`이다.
- **반복 목표 보존:** `--reapproach-from-log`에 새 좌우 보정이 없으면 직전 로그의 고정 odom 목표를 재사용한다. 매번 멈춘 위치를 새 목표로 가르치지 않는다. 이 반복 모드는 같은 정류장 단독 주차용이며 전체 왕복이나 다른 정류장 정확도를 증명하지 않는다.

- **영상 제어 시도 철회:** 세 번째 주차 후 정지 RGB 8장은 기준 영상 대비 왼쪽 잔차 평균 6.512 mm·yaw 0.029755 rad로 유효했지만 사용자 실측 우측 15 mm와 일치하지 않았다. operator-derived 영상 목표로 시도한 `water_station_20261027_164710_0493b18f`는 후진 관측 거리에서 매칭이 끊겨 이동 입력을 기다렸다. 직전 목표를 유지하도록 이어간 `water_station_20261027_165142_6b284e4b`에서는 접근 중 영상 목표가 바뀌어 두 번 모두 근접 좌우 보정 공간이 부족하다고 종료됐다. 0.218 s 사이 영상 목표 XY 변화 26.697 mm·동시 odom XY 변화 11.883 mm였다. 이 차이를 PnP 오차, 영상 타임스탬프 지연, 4륜 odom/미끄러짐 중 하나로 확정하지 않는다. 실행 뒤 실측은 우측 25 mm·앞 좌 59 mm/우 44 mm로 악화됐다. 이 영상 폐루프 추가 코드는 제거했으며 현행 기본 제어로 적용하지 않는다. 파생 영상 보정과 진단 자료는 해당 run 폴더의 `withdrawn_visual_goal_calibration.yaml`과 `static_rgb_reference_diagnostic.json`에 보존한다.
- **회차별 횡이동 보정 부족:** 30 cm 후진 재접근 두 번에서 요청 20 mm 대비 실측 변화 5 mm, 요청 15 mm 대비 실측 변화 0 mm였다. 명령 대비 실측 부족량 약 15 mm를 실험적 feedforward 후보로 기록했다. 내부 정지 odom을 로그 면·간격·횡오차로 재구성하면 두 실행 종점 사이 좌향 변화 11.657 mm지만 사용자는 같은 위치라고 보고했다. 미끄러짐이나 odom 원인을 확정하지 않으며, 이 값은 전역 바퀴 제원이나 등록 좌표에 적용하지 않는다.
- **깊이 방식 복귀 중 TF 종료:** 우측 잔차 25 mm에 위 후보 15 mm를 더해 해당 실행의 좌향 목표만 40 mm로 잡은 `water_station_20261027_165956_23234423`은 접근 중 `table approach stamped odom TF unavailable`로 종료됐다. 해당 예외는 상세 변환 오류를 남기고 기존 입력 회복 경로로 정지 대기한다. 처리 전 입력 stamp를 복원하여 같은 이미지의 TF가 도착한 뒤 재시도하며, 최신 TF를 과거 이미지에 대신 쓰지 않는다. 새 면·위치·성공 값을 만들어 넣지 않는다.
- **중간 지점 재개:** 성공 완료 로그뿐 아니라 같은 베이스 odom 세션의 관측 면과 해당 실행의 고정 목표를 이용해 후진 시야 확보 후 재접근할 수 있게 했다. 원본 목표는 유지하고 후진 이후 새 면을 관측한다. 진행 중 도크 위치초기화나 전체 태스크 반복은 하지 않는다. `$HOME/.local/state/jdamr/depth-stamped-tf-resume/20261027_170254/`에 반영 전 파일이 있다. 최종 외부 1 cm·좌우 ±5 mm 성공은 아직 확인되지 않았다.

- **TF 수정 후 같은 목표 재개 결과:** `water_station_20261027_170331_1577619b`는 도크 초기화 없이 이전 실행의 고정 목표를 유지해 20 cm 후진 후 재접근했다. 좌우 보정 공간 부족으로 기존 추가 후진을 한 번 수행한 뒤 주차를 내부 기준으로 완료했다. 최종 내부 양끝 간격 9.405/8.840 mm·횡오차 2.579 mm였다. 이 실행에서는 stamped TF 대기나 오류가 재현되지 않았으므로 실제 해당 예외의 자동 회복 성공까지 증명한 것은 아니다. 외부 실측은 앞 좌 9 mm/우 10 mm·차체 우측 17 mm였다. 앞 간격은 목표 1 cm 부근으로 돌아왔지만 외부 좌우 ±5 mm는 미달이다.

- **좌우 제어 재확인:** 같은 마지막 실행의 내부 횡오차 2.579 mm와 외부 우측 17 mm는 일치하지 않는다. `rgbd_auxiliary` 모드는 등록 지도 목표를 odom에 고정하고, 이동 제어·최종 좌우 판정 모두 `(goal_odom - actual_odom)`의 면 접선 성분으로 계산한다. 관측 면은 목표의 접선 위치로 재앵커링하며 RGB는 현재 보조 정보다. 따라서 실제 면까지 거리·각도가 맞아도 실제 횡위치가 옆으로 밀린 것을 이 깊이 면만으로 독립 확인하지 못한다. 실제 odom은 드라이버에서 좌우 바퀴 엔코더를 적분한다. 4륜 미끄러짐·유효 회전 기구학·이미지 시각 오차의 기여는 미확정이다. 15 mm 추가 횡보정 후보도 외부 목표 달성을 보증하지 못했으므로 기본 보정 상수로 채택하지 않는다. 거리/각도 개선과 실제 횡위치 미달을 구분하며, 현재 로봇을 움직이지 않는다.

### 2026-10-27 IMU 주행 odom 연결과 좌우 정밀도의 한계

- **확인과 수정:** `/imu/data_raw`는 수신되지만 기존 실차 bringup에는 준비된 EKF가 연결되지 않았다. `real_bringup_imu.launch.py`에서 드라이버는 `/wheel/odom`만 내고 odom TF는 끈다. 정지 바이어스를 보정한 `/imu/data`와 바퀴 속도를 EKF로 합쳐 `/odom`과 `odom→base_footprint`를 발행한다. 기존 바퀴 yaw rate도 유지하며 가속도는 융합하지 않는다. bias relay는 바퀴 입력이 오래됐거나 IMU 시각에 공백이 생기면 바이어스 학습을 보류한다.
- **실제 반영:** Pi 서비스와 저장소 배포 템플릿을 새 bringup으로 연결했다. 급수대 앞 정지 상태에서 이전 odom pose를 `/set_pose`로 인계했고 XY 차이는 0 m였다. AMCL은 전환 전 현재 급수대 map pose로 초기화했다. 기존 activation은 `navigation_activated_without_motion`을 남겼으며 전후 map 위치 차이는 내부 추정 3.290 mm·yaw 차이는 약 -0.082°였다. 도크·정류장 좌표는 바꾸지 않았고 이동 명령은 제출하지 않았다.
- **실행 근거:** Pi와 PC의 `$HOME/.local/state/jdamr/imu-odometry/20261027_172305/` 아래 `pre_switch.json`, `seed_result.json`, `actual_parameters.json`, `post_switch.json`, `activation.jsonl`, `deployment.json`에 보존한다. `/odom` 발행자는 EKF 한 개, `/wheel/odom` 발행자는 베이스 드라이버 한 개이며 실제 드라이버 `publish_tf=false`와 EKF의 입력·TF 설정을 조회했다. TF edge는 수신했으나 이 Jazzy rclpy 조회는 publisher GID를 제공하지 않아 GID별 TF authority 측정을 했다고 주장하지 않는다. 짧은 수신 표본 수는 발행 주기 벤치마크가 아니다.
- **남은 문제:** IMU 연결은 바퀴 회전 추정 의존도를 보완하는 변경이다. 앞 간격 9/10 mm·실제 우측 17 mm였던 좌우 문제의 해결이나 ±5 mm 달성은 아직 확인되지 않았다. 평면의 거리·법선만으로는 평면 접선 방향 이동을 독립 관측할 수 없다. 고정 컵·물병 등의 영상 기준을 등록한 물리 목표와 연결하고 시각·TF 및 이동 중 추정 흔들림을 해결하는 보완이 필요하다. 기존 참조 파일에 촬영 시각의 로봇 pose 연결은 없지만, 등록과 촬영 사이 로봇이 움직였다고 단정할 근거도 없다. 가속도 적분으로 횡위치를 대신 구하거나 등록 좌표에 17 mm를 더해 미달을 숨기지 않는다. 실차 이동·별도 pytest·리뷰는 수행하지 않았다.
- **재기동 복구:** 기존 Nav2 transient unit은 중지 뒤에도 예전 설정 식별자로 남아 현행 session-start에 거부됐다. inactive/dead·MainPID 0·정확한 transient 경로를 확인하고 원본을 백업한 뒤 해당 런타임 unit 하나를 제거하여 기존 세션 도구로 다시 만들었다. 이후 동일한 현재 급수대 pose로 activation을 마쳤다. 새 출발 절차나 로봇 이동을 추가하지 않았다.

### 2026-10-27 후진 재접근 실측과 정답 RGB-D 참조 재등록

- **이전 IMU 전환 기록 정정:** 위 activation은 현재 지도 정합을 입증하지 못했다. 에이전트가 `_load_helper()`가 반환한 모듈의 세션 함수를 호출하면서 `configure_map_session`을 거치지 않아, Nav2가 보존 지도 registry로 시작됐다. 실제 unit의 `--registry`는 `$HOME/jdamr_data/map_20260930_manual/aligned/service_destinations.yaml`이었다. 다음 실제 요청은 현재 지도·keepout 불일치로 이동 전에 종료됐다. live map/keepout을 현재 파일로 다시 읽혔지만 startup 설정까지 고친 것은 아니므로, 이후 현재 포인터를 처리하는 정식 CLI로 세션을 다시 만들었다. 앞서 기록한 odom XY 인계 0 m는 해당 서비스 응답의 사실이고, map 차이 3.290 mm는 현재 지도 정합·정확도 근거에서 제외한다.
- **실차 결과:** 기존 보호 직선 제어로 10.9 cm 후진한 뒤 재접근했다. `water_station_20261027_180441_7a8a58a2`는 추가 후진 후 전진 14.219 cm에서 정적 corridor 판정으로 멈췄다. 사용자가 더 후진해도 된다고 하여 같은 목표를 유지한 `water_station_20261027_180715_ba10ea94`에서 30 cm 후진·재접근을 수행했다. 최종 앞 간격은 내부 평균 11.034 mm·양끝 12.162/9.906 mm, 내부 횡오차 2.642 mm였으나 사용자 실측은 기준점 대비 우측 26 mm였다. 직전 우측 17 mm보다 악화돼 좌우 주차는 실패다. 앞 간격의 새 외부 실측은 없으며 IMU 추가가 횡위치 문제를 해결했다고 보지 않는다. 각 run의 `cycle_events.jsonl`과 마지막 run의 `operator_measurement.json`에 근거를 남겼다.
- **정답 배치와 참조:** 사용자가 차체를 목표 좌우 위치·책상과 평행·앞끝 1 cm 자세에 놓고 `배치 완료`를 회신했다. 이 상태의 RGB 16장·깊이 23장을 `$HOME/jdamr_data/maps/20261026_manual_160135/stations/water_station_goal_20261027_1815/`에 저장했다. 선택 RGB-D 쌍의 시각 차이는 11.377 ms다. `reference_capture.json`의 `goal_binding`은 이 영상 자세와 기존 등록 목표가 같다는 사용자 배치를 기록한다. 수동 재배치 뒤 과거 odom/map TF를 새 목표의 실측값으로 저장하지 않으며 도크·정류장 좌표도 바꾸지 않는다. 이전 참조와 원본 station YAML은 보존했다.
- **제어 연결:** 명시적으로 연결된 정답 참조에 대해 water station의 `reference_mode: rgbd`를 준비 자산 생성기가 유지하도록 수정했다. 영상으로 추정한 고정 목표를 실제 좌우 제어에 사용하고, 최근 유효 관측 중앙값으로 개별 PnP 추정의 튐을 줄인다. 최초 유효 관측부터 사용해 새 프레임 대기 조건을 만들지 않는다. 깊이 면의 거리·법선과 IMU/바퀴 odom 역할은 유지한다. 목표 연결이 없는 기존 `rgbd_auxiliary`와 표식 없는 `map_pose_only`의 동작은 유지한다.
- **근접 후진:** 사용자 배치로 정답에 연결된 RGB-D 참조가 있는 경우, `--reapproach-m`는 기존 `_drive_straight`로 후진한 뒤 새 면을 관측한다. 명시적으로 요청한 후진에는 첫 실시간 영상 매칭이나 근접 책상 경계를 선행 조건으로 요구하지 않는다. 현재 영상의 유효 여부와 사유는 후진 이벤트에 따로 남긴다. Collision Monitor·배터리·입력 신선도·명령 timeout·사용자 정지는 유지한다. 새 속도 발행기나 가짜 면 관측을 만들지 않는다.
- **최소 입력 확인과 한계:** 새 참조로 정지 RGB 16장 모두 유효했고, 영상 상대 좌우 잔차의 표준편차는 0.095 mm·최대 절댓값은 0.262 mm였다. 이는 같은 정지 자세에서 입력·기준 연결이 작동한다는 결과이며 이동 정확도·외부 ±5 mm 재현성 근거가 아니다. 사용자 배치 완료 뒤 추가 주행은 하지 않았다. 이동 중 영상 자세·카메라 yaw 가정·4륜 기구/미끄러짐의 영향은 아직 분리하지 못했고, 새 제어의 실차 결과는 다음 실행에서 확인해야 한다. 별도 pytest·리뷰는 실행하지 않았다.
- **현재 준비 상태:** 새 참조·제어 모듈을 PC와 Pi에 반영했다. 사용자가 배치한 기존 급수대 등록 pose로 초기화한 뒤 현재 지도에 대한 activation은 `navigation_activated_without_motion`을 남겼다. 실제 AMCL 값은 `(0.028342, -0.079721, -0.019245 rad)`이며 내부 위치추정 결과다. source registry·도크 좌표는 유지했다. 재기동 중에는 keepout mask 노드의 configure 응답 timeout과 inactive 상태가 확인돼, 촬영을 마친 카메라를 끄고 기존 Nav2 세션을 재기동하여 복구했다. 카메라 부하만을 원인으로 확정하지 않는다. 원본 실패 activation 로그도 같은 참조 폴더에 보존한다. 배포 원본은 `$HOME/.local/state/jdamr/rgbd-goal-reference/20261027_1815/`에 있고, 다음 명령을 위해 베이스·Nav2·idle 실행기를 유지한다. 전체 종료 명령은 `sudo systemctl stop jdamr-box-executor.service jdamr-restaurant-navigation.service jdamr-base.service`이며 임시 이동·촬영 프로세스는 남기지 않는다.

### 2026-10-27 깊이 지연·정지 확인 종료와 급수대 접근 재개

- **사용자 요청:** 오류는 기록하고 현재 급수대 위치에서 주차를 이어간다. 도크 초기화·완료 구간 재실행·별도 테스트·리뷰를 출발 조건으로 추가하지 않는다.
- **실행 근거:** Pi의 `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/` 아래 `water_station_20261027_184822_51ec851d`, `water_station_20261027_185230_80e87400`, `water_station_20261027_185639_84d6079f`, `water_station_20261027_185757_2d6972fe`, `water_station_20261027_190224_4efd5291`, `water_station_20261027_190524_61b40723`의 `cycle_events.jsonl`이다.
- **깊이·정지 확인 사실:** 첫 실행은 30.9 cm 후진 뒤 깊이 입력 지연이 반복됐고 정지 확인 실패로 끝났다. 당시 odom은 12개 신선한 표본을 수신했으며 마지막 수신 나이 15.9 ms·선속도 14.17 mm/s였다. 따라서 odom 미수신으로 기록하지 않는다. 이후 재개에서 관측 도중 `parking_motion_revision` 변화로 종료된 경우도 있었다. 실제 잔류 이동·필터 지연·잡음 중 기여도와 깊이 지연의 최초 원인은 아직 분리되지 않았다.
- **좌우 제어 사실과 수정:** 재접근 중 동일한 RGB-D 목표의 odom 추정 좌표가 갱신됐고, 가까운 거리에서 `table lateral alignment requires more clearance`가 반복됐다. `185757` 실행의 두 거부 이벤트 사이 목표 XY 차이는 약 38 mm다. 이 값은 실제 물체 이동이나 외부 좌우 오차가 아니다. 접근 중 처음 수용한 유효 RGB-D 목표를 odom에 고정하고, 면의 깊이·법선은 기존의 시각이 맞는 최신 관측으로 계속 갱신하도록 수정했다. 재등록 좌표·허용오차를 바꾸지 않았다.
- **입력 복구 연결 수정:** 깊이·후면 입력의 회복 가능한 지연에서 짧은 정지 확인이 실패하면 기존 `capture_stationary_pose(intermediate=True)`로 정지를 확인한 뒤 같은 목표의 입력 복구로 이어간다. 정지가 확인되지 않으면 재출발하지 않는다. 정지 속도 기준·센서 신선도·충돌 감시·사용자 정지는 유지한다. 이 추가 분기는 마지막 성공 실행에서는 발생하지 않아 재발 시 복구 효과가 확인된 것으로 주장하지 않는다.
- **최종 실행 결과:** `190524_61b40723`은 남은 접근만 재개해 전진 27.685 cm, `target_failure=null`, 실행기 exit 0으로 끝났다. 내부 앞 양끝 간격 8.102/9.071 mm·평균 8.586 mm, 면 각도 오차 0.123°·좌우 오차 2.692 mm다. 최종 책상 경계는 가려져 신선한 깊이 8장으로 가림을 확인한 후 마지막 관측 면과 odom으로 정지 위치를 계산했다. `final_depth_measurement=false`, `internal_estimate_only=true`이며 외부 1 cm·좌우 ±5 mm 반복 정확도를 입증한 결과가 아니다.
- **반영·남은 확인:** 두 Python 모듈을 Pi에 반영하고 구문 로드를 확인했다. 마지막 실행에서 `table_station_goal_frozen`과 정상 주차 종료를 확인했다. 배포 SHA·원본 백업은 Pi `$HOME/.local/state/jdamr/rgbd-goal-reference/20261027_1815/goal_freeze_deployment.json`, `stop_recovery_deployment.json`에 남겼다. 깊이 지연과 IMU/필터 정지 추정의 원인 분리는 주행 이후 과제다. 카메라는 실행 종료 시 꺼졌고 임시 이동 프로세스는 남기지 않았다. 베이스·Nav2·idle 실행기는 다음 명령을 위해 유지한다.

### 2026-10-27 현재 교시 목표로 제어·도착 판정 통일

- **확인된 결함:** 앞 절의 목표 고정 수정 이후에도 `_station_lateral_status`는 RGB-D 모드에서 제어용 고정 목표 대신 최신 개별 영상 목표로 완료 오차를 계산했다. 같은 로그에 고정 목표의 관측 시각과 다른 목표의 오차를 함께 기록할 수 있었다. 따라서 앞 절의 좌우 2.692 mm는 제어 목표에 대한 도착 오차나 실제 외부 좌우 오차를 입증하지 않는다.
- **수정:** 접근 제어·근접 직선 보정·최종 도착 판정에서 동일한 `station_reference_goal`을 사용한다. 유효 영상 목표의 고정 처리를 한 경로로 모았으며, 현재 RGB-D 목표가 없을 때 이전 지도 좌표를 정밀주차 좌우 목표로 대체하던 두 경로를 제거했다. 목표 미확보 상태에서 깊이 면의 법선·간격은 사용하지만 좌우 정밀주차 성공으로 판정하지 않는다. 목적지 근처까지의 지도 경로와 표식 없는 1번 테이블의 지도 기준은 유지한다.
- **기록 개선:** 완료 로그에 선택 목표·현재 odom 자세·면 접선 방향·목표 고정 여부·참조 파일 경로와 SHA를 남긴다. 최신 영상 기준 잔차와 고정 목표와의 차이는 별도 필드로 기록하고, 이를 기존 목표 오차 대신 사용하지 않는다. 목표는 `$HOME/jdamr_data/maps/20261026_manual_160135/stations/water_station_goal_20261027_1815/reference_capture.json`에 연결된 사용자 정답 자세다. 등록 좌표와 허용오차는 변경하지 않았다.
- **범위와 한계:** 사용자 요청으로 로봇은 이동시키지 않고 코드 반영·기존 실행기 준비만 수행한다. 센서·충돌·정지 보호와 최종 깊이 가림 기록은 유지한다. 기준 일치는 계산 경로의 수정이며, 새 주행의 외부 1 cm·좌우 ±5 mm 정확도 달성을 의미하지 않는다. 별도 pytest·리뷰를 실행하지 않는다.
- **비이동 반영 결과:** Pi에서 실제 import되는 `box_service.py`와 로컬 수정본의 SHA-256 `08a2206a99dc22916f6d27b13f40e8f2ee2bb84e31b1b3ac88e9b18a4cedb213` 일치를 확인했다. 현재 `reference_mode=rgbd`와 정답 참조의 `goal_binding`을 실제 파일에서 로드했다. 대기 중 실행기만 갱신했고 기존 Nav2 세션은 재사용했다. 이동 요청은 제출하지 않았으며 실행기는 active 상태다. 배포 원본·SHA 영수증은 Pi `$HOME/.local/state/jdamr/station-goal-consistency/deployment.json`에 있다.

### 2026-10-27 현재 지도 포인터로 운영·배포 경로 통일

- **확인된 상태:** PC와 Pi의 선택 지도는 `$HOME/jdamr_data/maps/20261026_manual_160135/map_table01_observed.yaml`이며 10-27 테이블 주변 관측이 반영된 지도다. Pi의 실제 `/map`과 `/keepout_filter_mask` 격자 서명은 현재 runtime 파일에서 계산한 값과 각각 일치했다. 저장소 `config/navigation/maps/classroom/`의 지도 이미지와 도크·급수대·1번 테이블 좌표도 운영본과 같아 새 지도를 다시 생성하거나 좌표를 변경하지 않았다.
- **확인된 낡은 경로:** 출발 helper를 함수로 불렀을 때 CLI의 지도 선택을 거치지 않아 초기 보존 지도 경로가 남을 수 있었다. 자산 초기화·Pi 배포 대상도 9월 지도 번들로 고정돼 있었다. 현재 지도 모드의 면 탐색 위치는 최신 등록값보다 `state.json`의 캐시를 먼저 사용했다. Pi 선택 포인터에는 PC의 `dock_departure_mode=nav2_waypoint`가 빠져 있었다.
- **수정:** 출발 도구의 함수·CLI, 자산 생성, 배포 경로가 현재 포인터를 사용하도록 통일했다. 보존 번들은 명시한 `--legacy-map`에서만 선택한다. 현재 모드의 탐색 영역은 현재 등록 좌표에서 계산한다. 사용되지 않는 과거 DDS 환경 선언을 제거했다. 원본 지도·등록 파일·도크 스캔·RGB-D 참조를 동기화한 뒤 포인터를 마지막에 반영한다. 코드 빌드에서는 지도 자산 생성을 분리했다. 날짜가 붙은 helper 폴더는 관리 코드의 위치이며 선택 지도를 고정하지 않는다.
- **비이동 반영:** 수정 helper를 Pi 실제 실행 소스에 반영하고 기존 `prepare`로 지도·참조·포인터를 동기화했다. PC/Pi 포인터 SHA는 `8fe00aadce71dae4071bd0d6fc92307327b977e9ba62367d2108a7b0de65dbe9`로 같고, map·keepout·등록부·station 프로파일·급수대/테이블 경유점 6개 파일의 SHA도 일치했다. 도크 이탈 방식은 양쪽 모두 `nav2_waypoint`다. 기존 위치초기화 상태는 유지했고 Nav2·idle 실행기는 active 상태다. 이동·위치초기화·추가 빌드·별도 pytest·리뷰는 실행하지 않았다.
- **근거·한계:** 현재 격자 서명은 map `bbd1a560e8918ccad1248dbe2f6baaed2629d46a0bede75de937e3b09f64c8c8`, keepout `c26711f2d36de9486b80ac79ec2e091898a3b766a6a13046d1e4f9cbc5d0dc60`다. 배포 원본과 helper SHA는 Pi `$HOME/.local/state/jdamr/current-map-runtime/deployment.json`에 보존했다. 이는 현재 지도·등록·운영 경로의 일치를 확인한 결과이며 실차 주차 반복 정확도 달성을 뜻하지 않는다. 다음 주행을 위해 기존 베이스·Nav2·idle 실행기는 유지하며 종료 명령은 앞 절과 같다.

### 2026-10-27 좌우 영상 피드백과 정지 관측 복구 개선 — 비이동 작업

- **확인된 구조:** 첫 유효 RGB-D 목표를 고정한 뒤 새 영상이 있어도 제어 목표에는 반영하지 않았다. 관측 이력도 영상 작업에서 수집하지 않고 제어 함수가 호출될 때만 모아 첫 접근에서 누적 영상을 활용하지 못했다. 좌우 오차가 허용오차 이내면 조향 계산의 좌우 항을 0으로 바꾸는 불연속이 있었고, 영상 목표 미확보 상태도 근접 면 동결 조건을 통과할 수 있었다. 정지 면 관측 중 이동 revision 변경은 새 정지 확인 없이 관측 전체를 종료했다. 이것들이 외부 치우침 전체의 단독 원인인지는 미확정이다.
- **영상 제어 수정:** 기존 영상 작업에서 유효 관측을 접근 전부터 저장하고 최근 0.5초·최대 5개 중앙값을 접근 중에도 갱신한다. 최초 목표는 비교용 anchor로 보존하고 실제 제어·도착 판정은 같은 갱신 목표를 사용한다. 각 제어 주기의 경로 확인에는 그 주기에서 선택한 목표를 전달해 영상 작업과의 갱신 시점 차이도 없앴다. 원본 odom·깊이 면의 거리·물리 footprint는 영상 목표에 맞춰 이동시키지 않는다. 무효·오래된 영상은 성공 근거가 아니며 기존 목표 보존과 신선한 센서 근거를 구분한다.
- **연속 조향·복구 수정:** 좌우 허용오차에서 조향 항을 갑자기 끄지 않고 같은 곡률식을 유지한다. RGB-D 목표 미확보 상태에서는 근접 면을 정렬 완료로 동결하지 않는다. 정지 관측 중 움직임은 기존 제한시간 안에서 한 번 정지를 다시 확인하고 이전 영상을 폐기한 뒤 새 면으로 이어간다. 반복 움직임·사용자 정지·센서 유효성·충돌·명령 timeout 보호는 유지하며 별도 출발 검사나 새 프레임 대기 조건을 추가하지 않았다.
- **요청 범위의 비이동 확인:** 변경 동작에 해당하는 확인 15개를 실행해 통과했다. 첫 목표 오차 이후 새 관측 반영, 누적 영상 사용, 무효 영상의 성공 판정 배제, 허용오차 경계에서 조향 연속성, 주기별 제어·경로 목표 일치, 기존 지도 목표 동결, 실제 기록 자세의 이상적 접근 계산, 한 번의 정지 재확인과 이전 영상 폐기·반복 움직임 중단을 포함한다. 전체 테스트·별도 리뷰 에이전트·실물 이동은 수행하지 않았다.
- **저장 영상 재생:** `$HOME/jdamr_data/maps/20261026_manual_160135/stations/water_station_goal_20261027_1815/stationary_rgb_samples.npz`의 정지 RGB 16장이 모두 유효했다. 원시 좌우 추정 범위는 약 -0.203~0.262 mm, 중앙값 피드백 범위는 -0.018~0.053 mm였다. 같은 정지 장면·같은 참조의 내부 변동이며 중첩 이력으로 계산한 값이다. 이동 정확도·거리별 획득 성능·외부 ±5 mm 정확도를 입증하지 않는다. 먼 거리나 가림에서 특징점이 확보되는지와 새 접근의 실제 반복 정확도는 아직 확인하지 않았다.
- **Pi 반영:** 실제 실행 소스의 세 모듈을 백업 후 교체하고 기존 Nav2 세션을 재사용해 대기 실행기를 갱신했다. 복원본·파일 SHA·적용 기록은 Pi `$HOME/.local/state/jdamr/live-station-feedback/20261027_234830/deployment.json`에 있다. 새 지도·등록 좌표·정답 참조·현재 위치는 변경하지 않았다. 이동 요청은 제출하지 않았다.

### 2026-10-29 반복되는 우측 10 mm — 영상 참조 바인딩 불일치와 고정 odom 보정 — 비이동 작업

- **현상:** `water_station_20261029_100650_16007840`(B), `102012_2780e855`(C), `104205_7cd636aa`(D) 모두 사용자 실측으로 차체가 목표보다 오른쪽 약 10 mm였다. 내부 좌우 오차는 -0.79/-0.27/+0.32 mm, D의 단일 프레임 RGB-D 진단은 +0.78 mm였다.
- **확정 1 — 고정 odom 보정은 실물에 실현되지 않았다:** B·C·D는 `operator_measured_relative_lateral_correction` 고정 odom 목표를 사용해 RGB-D가 조향에 쓰이지 않았다. D 면 접선(1.0304 rad)으로 투영하면 odom상 B→C +6.5 mm, C→D +9.7 mm 왼쪽으로 이동했지만 사용자 실측은 세 번 모두 같았다. odom 안의 책상 면 방향도 B 1.0519 → D 1.0304 rad로 1.23° 바뀌었다. 0.3 m 후진·재접근의 odom 좌우 누적 오차가 보정량과 같은 크기다.
- **확정 2 — 10-27 영상 참조 위치 자체가 사용자 목표와 약 10 mm 어긋났다:** D 정지 위치에서 읽기 전용 30프레임을 다시 받아 기존 관측기로 재생했다. 30/30 유효, 좌우 평균 +1.24 mm(표준편차 2.16 mm/프레임), yaw 0.35°다. 새 참조로 10-27 캡처 위치 영상을 거꾸로 관측해도 -0.74 mm로 같은 결론이다. 즉 현재 정지 위치는 10-27 캡처 시점의 카메라 시점과 1~2 mm 안에서 같고, 사용자 표식 기준으로는 우측 10 mm다. "영상도 0"은 단일 프레임 잡음이 아니라 목표 정의의 문제다.
- **관측된 장면·장착 변화:** 10-27 이후 물병 내용물 외관이 달라졌다. 같은 세션 기준 inlier 약 457·RMSE 0.58 px가 오늘 약 60·1.36 px로 떨어졌다. 컵(dx -2.5 px, 0.81 m)과 물병(dx -6.9 px, 0.94 m)의 영상 이동이 강체 카메라 이동으로 설명되지 않아 물체 상대 이동 가능성이 있다. 차체 상판 영역도 dy +5.8 px 이동했고 테이블 평면 기준 카메라 pitch가 약 0.65° 달라졌다. 캡처 당시 위치 자체가 어긋났는지, 물체·카메라 변화 때문인지는 이 자료로 분리하지 못했다.
- **확정 3 — RGB-D 좌우는 yaw와 강하게 결합된다:** 정지 30프레임에서 corr(left, yaw) = -0.91, 기울기 -1.10 m/rad(랜드마크 깊이 약 0.87 m + 카메라 lever 0.275 m)다. yaw 추정 0.5° 오차가 좌우 약 5.5 mm가 된다. A(`095831_3219a210`)에서 접근 중 연속 유효 샘플의 목표 좌우가 약 48 mm 달라졌고 당시 면 대비 yaw는 약 2.1°였다. 10-27 `185757`의 38 mm 목표 이동과 같은 계열로 본다.
- **기각·한계:** 전륜축 기준 EKF(vy=0) 구조의 skid-steer ICR 오프셋은 0.105 m × 순 heading 변화로 B→D 약 2 mm에 그쳐 단독 원인이 아니다. odom 좌우 드리프트의 기구적 원인(횡미끄럼·바닥 경사·자이로 배율)은 미확정이다.
- **조치 1 — 영상 참조 재바인딩:** D 정지 위치(odom 변화 0.05 mm 이내) 읽기 전용 캡처로 `$HOME/jdamr_data/maps/20261026_manual_160135/stations/water_station_goal_20261029_1058_offset/`를 만들었다. 깊이는 정지 30프레임의 픽셀별 중앙값이다. `goal_binding.kind=operator_measured_offset_from_capture`, `goal_offset_in_capture_base_m={forward_m: 0, left_m: 0.010}`로 사용자 실측 "계속해서 우측으로 1cm"를 캡처 좌표계에 묶었다. 앞 간격은 바인딩하지 않고 깊이가 맡는다. 물병 ROI는 오른팔 센서 보드를 빼려고 x≤512로 줄였고, 같은 보드를 테이블 평면 추정에서 제외했다. 이전 캡처와 원본 기록은 보존했다.
- **조치 2 — 수동 좌우 보정의 영상 바인딩:** RGB-D 모드에서 `--station-left-correction-m`을 주면 정지 상태 유효 관측 3~5개의 중앙값으로 현재 위치를 영상 좌표계에서 읽고, 목표를 그 위치에서 보정량만큼 왼쪽에 두는 run 오프셋으로 바꾼다. 이후 접근은 기존 라이브 영상 필터(0.5초·최대 5개 중앙값)로 같은 물리 목표를 추적한다. 오프셋을 바꾸기 전 계산된 결과는 작업자에서 버린다. 정지 영상이 3개 미만이면 기존 고정 odom 경로를 그대로 쓰고 `station_correction_rgbd_binding_unavailable`을 남긴다. 캡처·run 오프셋은 각각 ±0.05 m로 제한하고 앞뒤 오프셋은 거부한다.
- **최소 실행 확인:** 새 참조로 현재 29프레임(캡처 프레임 제외) 재생 시 목표가 로봇보다 왼쪽 9.98 ± 0.10 mm, inlier 중앙값 364, RMSE 0.54 px였다. run 오프셋 -4 mm 적용 시 5.91 mm. 잘못된 바인딩 3종·범위 초과 run 오프셋은 거부했다. 바인딩 함수는 스텁 노드에서 정상·영상 부재·범위 초과 세 경우를 확인했다. 새 runtime을 임시 폴더에 생성해 기존 runtime과 비교했으며 바뀐 것은 급수대 RGB-D 항목뿐이었다. 전체 pytest·별도 리뷰는 실행하지 않았다.
- **Pi 반영:** 세 모듈·새 캡처 폴더·`water_station.yaml`·`runtime/station_profiles.json`·`runtime/service_destinations.yaml`을 반영하고 PC/Pi SHA 전부 일치, Pi 컴파일과 runtime 프로필의 참조 초기화(`goal_left_offset=0.01`)를 확인했다. 대기 실행기만 재시작했다. 원본은 Pi `$HOME/jdamr_deploy/claude_lateral_binding_backup_20261029/`에 있다. 이동·위치초기화는 하지 않았다.
- **남은 실차 확인:** 새 바인딩으로 실제 좌우 ±5 mm에 서는지는 주행 전이다. 접근 중 RGB-D가 접근 후반에만 유효해지는 문제와 yaw–좌우 결합은 남아 있어, 곡선 여유 부족(`table lateral alignment requires more clearance`)이 재발할 수 있다. 사용자 실측 해상도는 1 cm 단위다. 물병·컵을 다시 옮기거나 내용물이 바뀌면 참조가 다시 어긋난다.
- **새 바인딩 첫 실차 재접근 `water_station_20261029_112346_7c4567d6`(사용자 "해봐", D와 같은 임시 mount, 좌우 보정 인자 없음):** 0.309 m 직선 후진 뒤 접근했다. 영상 목표는 첫 구간 남은 약 5 cm, 자동 공간 확보 후 두 번째 구간 남은 약 3 cm에서야 유효해졌다. 두 구간 모두 `table lateral alignment requires more clearance`로 정지했고 실행기 exit 1이다. 공간 확보 후진(Nav2 box_escape)은 code 105로 목표에 못 미쳤고 면 대비 약 7.9° 틀어진 채 끝났다. 후진 지점 영상은 `ambiguous_coplanar_inlier_geometry`(약 0.75 m), 더 먼 지점은 `insufficient_correspondences`(약 0.84 m)였다. 최종 위치 한 장의 기준 사진은 그 시점과 비슷한 마지막 수 cm에서만 맞춰진다. 그래서 좌우 보정에 필요한 접근 거리에서는 좌우 관측이 없다. 같은 조건의 재시도는 같은 실패를 낸다. 마지막 유효 영상은 목표가 로봇보다 왼쪽 약 5.8 mm, 남은 거리 약 2.6 cm, 면 대비 yaw 약 -0.65°였다(연속 샘플 간 좌우 차 약 9 mm, 내부 추정). 로봇은 그 자리에 정지해 있고 카메라는 꺼졌다.
- **거리별 기준 촬영과 원인 재확정(사용자 "1번 니가 알아서", 같은 날):** 실행기에 `--station-capture-retreat-m`(재접근 후진과 같은 보호 직선 제어로 한 단계만, 최대 0.05 m, 카메라 미조작)을 추가했다. 정지 촬영 17곳·보호 후진 16회를 수행했고 모두 `done=True`였다. 요청은 단계당 25 mm였지만 실제 이동은 32~34 mm였고 총 0.537 m 후진했다. 원본은 Pi `runtime/runs/station_keyframes_20261029_113250/`에 있다. 로봇은 책상에서 약 0.57 m 떨어진 곳에 정지했고 카메라는 꺼졌다.
- **확정 4 — 컵·병 깊이가 물체 표면이 아니다:** 기준 특징점의 3D 점 중 컵 55%·물병 62%가 책상 평면 높이(5 mm 미만)에 있었다. 컵 하나의 점이 시선 방향으로 약 35 cm 퍼져 있다. 스테인리스·투명 표면에서 깊이가 뒤쪽 책상에서 읽힌 것이다. 이 3D 점으로 PnP를 풀면 기준 위치에서는 정확히 0이지만, 앞뒤로 움직이면 yaw↔좌우 방향으로 계통 오차가 난다. 같은 직선 후진에서 odom은 0.197 m 동안 좌우 0.37 mm·yaw -0.12°였고, 영상 essential matrix(kf00→kf04)도 회전 0.0°의 직선을 보였다. 반면 PnP 연결은 단계당 yaw +0.36°·좌우 -6.6 mm씩 누적됐다. 높이 10 mm 이상 점만 남겨도 yaw +2.8°·좌우 -20 mm, 앞뒤 거리 40% 축소로 해결되지 않았다.
- **확정 5 — 관측 가능 거리와 보정에 필요한 거리가 겹치지 않는다:** 카메라가 약 49° 아래를 보므로 컵·병은 목표에서 약 0.29 m에서 화면 상단에 작게, 0.42 m에서 아랫부분만, 0.54 m에서 사라진다. 제어기 조건(5 cm band 안에서 좌우 ±5 mm, yaw 속도 0.08 rad/s, Hermite 곡률)상 10 mm 보정은 약 0.35 m, 5 mm는 약 0.27 m 앞에서 좌우를 알아야 한다.
- **기각 — 영상 사슬·삼각측량:** 3.3 cm 이웃 essential matrix는 단계당 좌우 ±(0.3~10) mm·yaw ±1°로 흔들렸다. 긴 기선에서도 회전 1.3~4.3°로 서로 맞지 않았다(책상 평면 위주의 좁은 시야). 이 자세로 만든 삼각측량 keyframe의 재현 오차는 좌우 1~10 mm였고, kf08 이후는 컵 특징 0개, kf11 이후는 점이 없었다. 운영 경로에는 반영하지 않았다.
- **남은 판단(사용자 결정):** 현재 카메라 각도와 컵·병 조합으로는 접근 중 ±5 mm 좌우 보정을 보장할 근거가 없다. 직선 후진 odom은 0.2 m에 좌우 0.4 mm 수준으로 정확하고, 영상은 기준 위치 바로 그 자리에서만 정확하다. 이 두 사실을 쓰는 방법과 장착·랜드마크 변경 중에서 고를 일이다.


### 2026-10-29 접촉선 참조·연속 좌우 제어 수정 후 사용자 근접 성공 확인

- **이번 실제 결과:** `water_station_20261029_142312_dcb8108b`는 `precision_straight_finished`와 `table_continuous_stop_confirmed`까지 진행했다. 사용자가 “지금 성공이다 거의”라고 확인하고 현재 주차 결과의 반영을 요청했다. 사용자 확인상 근접 성공으로 기록한다. 새 좌우·간격 실측은 없으며 앞끝 1 cm·좌우 ±5 mm 정확도를 입증한 결과로 승격하지 않는다. 현재는 급수대 앞이며 전체 왕복이나 후면 도킹 결과가 아니다.
- **관측 방법 변경:** 사용자 정답 배치에서 캡처한 컵·병의 책상 접촉선에 Sobel 1차 미분을 적용하고 상판 평면에 투영해 좌우·방향을 추정한다. 반사·투명 표면에서 뒤쪽 책상 깊이를 물체 표면 깊이로 취급하던 PnP 경로 대신 `table_contact_edges`를 사용한다. CNN·세그먼테이션은 추가하지 않았다. 장착 변경 없이 기존 17개 후진 정지점 영상과 현재 참조를 계산에 활용했다. 목표 근처 한 장의 자기 일치가 접근 중 외부 정확도를 증명한다는 가정은 사용하지 않는다.
- **지연과 오인식 조치:** RGB 구독을 최신 1장으로 제한하고 OpenCV·BLAS 스레드 수와 추적의 이동·회전 검색 범위를 줄였다. 이미지 시각의 odom 변환과 완료 시점의 map 표시 변환을 분리해 map 대기가 영상 유효 시간을 소모하지 않게 했다. 기존 0.5초 freshness는 유지했다. 실제 동시 입력 기록은 초기 전체 탐색 뒤 24/24개가 fresh였고 추적 처리 시간은 약 0.08–0.09초였다. 원본은 `water_station_20261029_133813_1628690f/recovery/stationary_inputs.json`에 있다. 횡방향 약 4.7 cm·yaw 약 6.1°의 일시적인 정적 목표 점프는 최근 유효 목표와 비교해 거절하고 tracker 상태를 복원한다. 전후 odom 미끄럼을 같은 점프 판정에 넣지는 않았다.
- **경로와 제어 조치:** 근접 시 검출 폭이 약 0.873 m에서 0.264 m로 줄어, 이전에 본 같은 면의 좌우 범위까지 잊고 unknown 205셀에서 멈췄다. odom의 관측 endpoint를 보존하고 실제 이동 외곽이 관측 면 바깥·관측 폭 안에 있는 경우에만 기존 두 지도 셀 가장자리 처리를 사용한다. 실제 occupied 셀은 계속 막는다. 좌우 제어의 고정 5 cm 완료 band·추가 3 cm lead를 제거하고 남은 거리와 도착 허용폭을 사용해 근접 S곡선 비수렴을 줄였다. 일반 면 접근의 기존 band·양 모서리 충돌 검사·freshness·CM·명령 timeout은 유지한다. 실패 좌표 두 건의 계산은 각각 좌우 2.185/3.154 mm로 수렴했으나 실측 정확도는 아니다. 근거는 `water_station_20261029_135945_2678b099/clearance_control_calculation.json`이다.
- **후속 실패는 별개:** 접근 정지 후 `final_depth_measurement` 단계에서 `robot moved during stationary box observation`이 발생해 실행기 exit 상태는 실패다. 원본 이벤트를 성공으로 덮어쓰지 않는다. 실제 잔여 움직임과 odom 순간 잡음 중 무엇이 정지 재관측을 깨뜨렸는지는 미확정이며 전역 정지 보호를 완화하지 않았다. 사용자 확인 뒤 추가 물리 이동은 하지 않았다.
- **보존한 실행 상태:** 활성 참조는 `stations/water_station_goal_20261029_120509_exact_contact_tracked_responsive/reference_capture.json`, SHA-256 `588a3865e8730224b2748eb6f6fa8ed672b9cd07365f1e2e86627b396c74e566`이다. 최초 정답 배치의 바인딩을 유지하며 이번 도착 위치를 새 목표로 다시 캡처하지 않았다. 원본 이벤트의 해시·선택 이벤트·사용자 원문·실측 미기입·배포 소스 해시는 `runtime/runs/water_station_20261029_142312_dcb8108b/operator_parking_result.json`에 보존했다. 모든 경로는 `$HOME/jdamr_data/maps/20261026_manual_160135/` 아래다. 마지막 기록 시 대기·실행 중 요청은 없었고 주차 제어 모듈 3개는 PC/Pi SHA가 일치했다. 별도 pytest·리뷰 없이 변경 파일 4개의 문법만 확인하고 작업 브랜치 로컬 커밋으로 남긴다.


### 2026-10-29 도크 출발 후 급수대 성공·후면 도킹 성공과 중간 정지 수정

- **사용자 수용:** `water_station_20261029_143236_5bfe51f0`에서 “우측 약 5mm 느낌이지만 충분하다”는 확인을 받았다. 눈대중 추정이며 실측 ±5 mm 판정은 아니다. 내부 6.195 mm 때문에 자동 완료는 거절됐던 사실과 사용자 수용을 함께 보존하고 허용오차를 변경하지 않았다.
- **급수대 완료:** 도크 초기화 뒤 첫 출발 `water_station_20261029_143752_6afb552e`는 관측점 도착 후 추가 접근 전에 정지 확인이 실패했다. 기록은 fresh odom 15개·최종 나이 0.023초·선속도 0.000399 m/s·각속도 -0.012708 rad/s다. `_direct_motion`은 각속도 0.01을 하드코딩했고 주차 계약은 0.02여서 기준이 달랐다. 계약 값을 사용하도록 통일했으며 비주차 경로는 종전 기준을 유지한다. 이후 현재 관측점에서 재개한 `water_station_20261029_143909_bbf68345`는 앞 양끝 내부 추정 12.175/10.144 mm·좌우 2.877 mm·면 각도 0.259°로 완료됐고 사용자도 정밀주차가 잘됐다고 확인했다. 근접 최종 면은 가려져 기존 면과 odom에 근거하며 새 실측은 없다.
- **TF 순서로 생긴 중단:** 위 접근에서 stamped TF 부재와 깊이 입력 지연으로 정지·회복이 발생했다. 새 영상의 TF가 먼저 준비되지 않았더라도 직전 승인 면 자체의 시각이 기존 freshness 0.5초 안이면 이번 제어 주기는 그 면으로 계산하고 새 영상의 처리를 다음 주기로 미룬다. 이전 면의 시각을 새 시각으로 덮지 않는다. 실제로 오래된 영상·센서 소실·시계 역행은 계속 정지한다. 이 수정이 모든 입력 지연을 없앴다는 실차 근거는 아직 없다.
- **복귀 첫 실패:** `water_station_20261029_144115_40b3a4de`는 급수대 후진 이탈·도크 앞 이동·면 확보 후 약 29 cm 후진하다 `rear docking swept footprint occupied`로 대기했다. 최종 면 간격은 약 0.159 m였다. 이는 정적 지도보다 현재 스캔과 근거리 기억점의 실제 이동 외곽 계산에서 나온 사유다. 당시 차단점 원본은 기록하지 않아 현재점·기억점 중 원인은 미확정이다. 정지 후 스캔 5장에서는 차체 주변 같은 점이 없었으나, 이것만으로 과거 오탐을 확정하지 않는다. `rear_collision_witness`에 첫 차단점·source·시각을 기존 계산 한 번으로 남기도록 수정했고 점을 지우거나 차단을 우회하지 않았다.
- **후면 마지막 잔차:** 기존 후면 실행 `water_station_20261029_144649_ca216ece`를 재개해 약 15 cm 후진했지만 최종 내부 간격 14.522–16.089 mm로 자동 완료가 아니었다. 다음 작은 후진 `water_station_20261029_144753_4fbf904f`에서 11.334–11.800 mm·0.059°·confirmed=true가 됐다. 이후에는 같은 요청 안에서 전진 없이 1 cm 이하의 작은 후진 잔차 보정만 한 번 수행하도록 연결했다. 기존 시간 한도·동일면·스캔·충돌 감시는 유지한다.
- **보존과 반영:** 사용자는 전체 주행을 성공으로 확인했다. 중간 재개가 있었으므로 연속 무중단 성공으로 기록하지 않는다. 사용자 확인·선택 이벤트·원본 이벤트 해시는 마지막 도킹 run의 `mission_parking_result.json`에 있다. 모든 run은 `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/` 아래다. 변경 5모듈 문법·보존 스캔 외곽 계산의 기존/수정 일치 10건·Pi SHA 대조만 수행했으며 별도 pytest·리뷰·새 실물 주행은 하지 않았다. 배포·복원 경로와 다음 Windows 추론 연결의 미완료 범위는 저장소 `PROGRESS.md`의 같은 날짜 절에 남겼다.

### 2026-10-29 주행 검토: 주행 중 정지·20 cm 직진·후진 후 재정렬·앞뒤 미세 왕복 — 비이동 작업

- **현상/근거:** 사용자 보고와 10-29 기록 기준이다. 이 PC에는 run 로그가 없어 원본 이벤트는 다시 읽지 않았고,
  코드·기존 기록·단위 시험으로 판단했다. (1) 급수대 이동 중 Nav2 오류 102로 run 종료(`191747`), 깊이 신선도 정지 11건·
  정지 odom 미확인 5건·stamped TF 미확인 1건(`192609`). (2) 주차 위치와 약 20 cm 어긋난 곳에서 직진 접근.
  (3) 좌우 곡선 공간 부족 뒤 자동 후진 재접근이 같은 사유로 실패. (4) 마지막 1 cm 부근의 작은 전후 보정 반복
  (`173345`: 10.195/6.435 mm, 작은 전후 보정 뒤 `final_gap_no_progress`).
- **원인(코드 확인):**
  - (1) 새 깊이 결과가 처리 지연으로 0.5 s를 넘기면 `table approach depth stale`로 정지·정착·재출발했다. 관측기가
    조용할 때는 같은 면을 1.5 s까지 유지했으므로 처리 지연만 더 엄격했다. Nav2 Jazzy는 FollowPath TF_ERROR(102)를
    즉시 중단하고 BT 재시도 조건에서도 제외하며, 실행기도 102를 재시도하지 않았다. 실행기·관측기의 TF 수신이 제어
    루프와 같은 단일 스레드라 계산 중에는 /tf가 쌓였고 `can_transform(timeout)` 대기 중에는 콜백이 돌 수 없었다.
    정지 중 자이로 표본 하나(> 0.02 rad/s)가 `parking_motion_revision`을 올려 정지 관측을 다시 시작시켰다.
  - (2) 테이블1 `surface_only`에는 좌우 목표가 없어 관측 위치 도착 오차(Nav2 위치 허용 0.15 m + AMCL)가 그대로 남는다.
    면 목표 좌우 위치도 보이는 테이블 모서리의 중점이라 시야에 따라 달라졌다. 정렬 생략 판정은 거리·각도만 봐서
    옆으로 움직일 수 있는 유일한 경로인 face-alignment Nav2 구간을 0.5 m 관측 위치에서 항상 생략했다. 같은 미리보기로
    계산하면 연속 S곡선은 0.45-0.565 m에서 약 10-12 cm까지만 흡수하며 20 cm는 어느 거리에서도 불가능하다.
  - (3) `_table_fine_approach`의 재접근은 같은 방향 직선 후진이라 좌우 오차를 바꾸지 못하고, `preview heading out of
    range`는 재접근 대상에서도 빠져 있었다. 후진 시 마지막 1 cm 목표 고정(`_station_terminal_goal`)과 이전 접근의
    map→odom 목표 스냅샷이 풀리지 않았고, 방문당 재접근 횟수 제한이 없었다.
  - (4) 두 앞모서리 모두 ±3 mm여야 해 0.5° 안팎의 기울기에서는 중심 허용 폭이 1 mm 남짓인데 0.01 m/s의 정지 지연만
    0.7-1.7 mm다. 후퇴 뒤 다시 전진할 수 있었고(주석과 반대) 방향별 진전 기록이라 F/R/F 교대를 막지 못했다.
- **조치:**
  - (1) 늦은 새 깊이 결과는 소비하지 않고 odom에 고정한 직전 면을 유지한다(직전 면 ≤ `OBSERVER_SILENT_S` 1.5 s,
    직전 면도 1.5 s 이내(건강한 가림 고정 제외), CameraInfo 생존 조건. 영상 TF 대기 중 늦어진 프레임도 같다). 102는 기존 입력 공백 경로로 1회 재전송하되 map→odom이 다시 발행된 뒤에만(8 s 안) 보낸다.
    실행기·관측기 TF를 전용 노드·데몬 스레드로 받는다(생성자 실패 시 프로세스가 남지 않음). 자이로 단독 회전은 0.08 s
    연속일 때만 이동으로 본다(도크 초기화와 같은 규칙; 병진·NaN·명령·0.04 rad/s 초과는 즉시). 직접 명령 정지 이벤트에 `max_publish_gap_s`를 남긴다.
  - (2) `surface_only`의 map 면 목표를 등록 좌우 위치에 맞추고(1 cm 접근의 좌우 미세보정·판정은 여전히 없음), RGB-D
    미일치 급수대의 map 목표도 등록 자세로 둔다(`lateral_goal_pending` 유지, 도착 증거 아님). 원래 정렬 구간을
    생략하는 경우에만 실제 제어와 같은 미리보기(`surface_only`는 정렬 허용 5 cm)로 확인하고, 불가능하며 후진 전제(면을
    15° 안으로 향함)가 맞으면 회전 공간(0.565 m)으로 후진한 뒤
    기존 face-alignment Nav2 구간으로 옆으로 재배치한다.
  - (3) 재정렬 후진 때 마지막 1 cm 고정과 이전 접근의 등록 자세 스냅샷을 푼다(운영자 보정·로그 고정 목표는 유지).
    `preview heading out of range`도 재접근에 포함하고, 재관측한 형상이 여전히 곡선으로 불가능하면 방문당 1회만
    옆 재배치한다. 직선 재접근도 방문당 1회다.
  - (4) 후퇴 뒤 같은 정지에서 다시 전진하지 않는다(`final_gap_reversal_blocked`). 1 mm(`TABLE_FINE_MIN_PROGRESS_M`)
    미만 보정은 명령하지 않고 측정 간격과 함께 끝낸다(`final_gap_correction_below_resolution`).
  - 기타: 공개 저장소 문서의 Pi 로그인 비밀번호를 지웠다(이력에는 남아 있어 비밀번호 교체가 필요하다).
    브랜치에서 코드만 바뀌고 갱신되지 않은 시험 37건을 현재 동작에 맞췄다.
- **검증/범위:** Windows PC의 RoboStack ROS 2 Jazzy 환경에서 단위 시험만 실행했다. 주행 관련 18개 시험 파일은
  HEAD 대비 새 실패 0건, 기존 실패 37건 해소, 남은 실패 2건은 Windows 경로 구분자(`'\w' != '/w'`)다. 새 시험 36개는 모두
  통과한다. HEAD 코드에서는 상자 서비스 17개 중 16개가 실패하고(1개는 작은 좌우 오차의 기존 연속 접근 유지 확인), 정지
  복구 19개는 새 TF 모듈이 없어 수집되지 않는다(모듈 도입 전 14개 기준 8개 실패, 6개는 유지한 보호 확인). 전체 패키지 시험은 2852 통과·180 실패(평가·시뮬
  전용 모듈 누락과 Windows 경로·심볼릭 링크)로 기준보다 실패가 줄었다. 실차 주행·Pi 배포는 하지 않았다.
- **남은 실차 확인:** 늦은 깊이 결과 유지 중 실제 정지 감소(`table_approach_late_frame_skipped` 빈도), 102 재시도 후
  이어가기, 20 cm 어긋난 도착에서 face-alignment 재배치 후 테이블1 좌우 결과(정렬 허용 5 cm + AMCL), 회전 공간 후진이
  충돌 감시와 겹치지 않는지, 1 cm 미세 보정의 전후 교대 소멸. 후면 대기 65 s와 이동 중 RGB 작업자 부하는
  `20261029_DRIVE_TIMING_REVIEW.md` 4절의 측정 후 결정 항목이다.

## 2026-10-31 도크 전진 여유 시작 전 스캔 공백

- 근거: `table_01_20261031_182702_46ed27ea/cycle_events.jsonl`, 18:27:53 rear_dock_center_clearance(0.802729m, lateral -0.066279m) → 18:27:54 rear_dock_clearance_started → 0.011초 후 release_failed(no fresh scan) → rear_box_parking_not_confirmed. 실제 도크 도착은 팀원 보고이고 내부 최종 주차 확인은 없다.
- 확인한 결함: 경로를 준비한 뒤 snapshot을 단 한 번 조회하고 None이면 즉시 종료한다. None은 scan 없음/수신나이0.5초초과/TF·outline·memory 실패를 모두 포함해 당시 원인은 분리되지 않았다. 해당 구간 직전 rear box face stable 기록이 있어 전체 라이다 고장으로 단정하지 않는다.
- 수정: 정지 상태에서 최대 기존 freshness window0.5초 동안 콜백을 처리해 유효 snapshot 재취득. 기존 입력 신선도·충돌·중단·deadline 유지. 재취득 불가 시 scan_present와 scan_callback_age_s 기록. 후진 완료를 임의 성공으로 바꾸지 않는다.
- 확인: 단발 공백 뒤 회복과 지속 무입력을 포함한 관련6개 확인 통과. Pi 백업 후 모듈 교체 및 해시 일치. 실제 주행·도킹은 수행하지 않았으며 좌우 보정 전진 거리의 적절성과 실차 재발 여부는 남은 확인이다.
