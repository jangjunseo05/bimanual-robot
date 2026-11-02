#!/usr/bin/env bash
# 식당 서비스용 Nav2 세션만 관리한다. 목적지 action과 초기 pose는 보내지 않는다.
set -o pipefail

UNIT="jdamr-restaurant-navigation.service"
WORKSPACE="${JDAMR_WORKSPACE:-$HOME/jdamr_ws}"
REGISTRY="${JDAMR_RESTAURANT_REGISTRY:-}"
PARAMS_FILE="${JDAMR_RESTAURANT_PARAMS:-}"
PRECISION_PARKING=false
PREPARE_ONLY=false
USE_COMPOSITION=true
DISCOVERY_RANGE=LOCALHOST

usage() {
  cat <<'EOF'
사용법:
  restaurant_session.sh status
  restaurant_session.sh start [--workspace PATH] [--registry PATH] [--params-file PATH]
                              [--precision-parking]
                              [--prepare-only]
                              [--use-composition true|false]
                              [--discovery-range LOCALHOST|SUBNET]
  restaurant_session.sh stop

start는 센서가 이미 실행 중인 Pi에서 식당 서비스용 Nav2 서버만 시작한다.
초기 pose, NavigateToPose, FollowPath 등 이동 명령은 보내지 않는다.
--precision-parking은 명시적으로 승인된 5 cm 박스 주차 시험에서만 사용한다.
--prepare-only는 지도·AMCL만 활성화하고 Nav2 이동 실행부는 미활성 상태로 준비한다.
--use-composition은 Nav2 서버 실행 방식을 선택한다. 기본값은 완주 검증에 사용한 true이며,
false는 독립 프로세스 비교 진단용 폴백이다.
--discovery-range는 베이스·카메라·관측기·실행기와 동일하게 설정해야 한다.
기본값은 온보드 제어용 LOCALHOST이며, PC 관제는 표시 중계를 사용한다.
LOCALHOST에서는 다른 PC의 ROS 구독이 연결되지 않는다.
배치 후 위치추정 확인을 마친 뒤 navigation lifecycle startup이 필요하다.

기본값:
  workspace   $HOME/jdamr_ws
  registry    $HOME/jdamr_data/maps/20260922_manual_final_run2/service_destinations.yaml
  params-file <workspace>/install/jdamr_cube_navigation/share/
              jdamr_cube_navigation/config/new_base_nav2_params.yaml

status의 active는 프로세스 실행 상태일 뿐 위치추정·주행 준비 완료를 뜻하지 않는다.
EOF
}

die() {
  printf '실패: %s\n' "$*" >&2
  exit 4
}

absolute_file() {
  local path="$1" label="$2"
  [ -n "$path" ] || die "$label 경로가 비어 있다"
  [ "${path#/}" != "$path" ] || die "$label 경로는 절대경로여야 한다: $path"
  [ -f "$path" ] && [ -r "$path" ] || die "$label 파일을 읽을 수 없다: $path"
}

absolute_directory() {
  local path="$1" label="$2"
  [ "${path#/}" != "$path" ] || die "$label 경로는 절대경로여야 한다: $path"
  [ -d "$path" ] && [ -x "$path" ] || die "$label 디렉터리를 사용할 수 없다: $path"
}

load_ros_environment() {
  local ros_setup="${JDAMR_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
  local overlay="$WORKSPACE/install/setup.bash"
  [ -r "$ros_setup" ] || die "ROS setup을 읽을 수 없다: $ros_setup"
  [ -r "$overlay" ] || die "workspace overlay를 읽을 수 없다: $overlay"

  # ROS setup 파일은 정의되지 않은 변수를 조회할 수 있어 nounset보다 먼저 읽는다.
  set +u
  # shellcheck disable=SC1090
  source "$ros_setup" || die "ROS setup 적용 실패: $ros_setup"
  # shellcheck disable=SC1090
  source "$overlay" || die "workspace overlay 적용 실패: $overlay"
  set -u

  export ROS_DOMAIN_ID=12
  export ROS_LOCALHOST_ONLY=0
  export ROS_AUTOMATIC_DISCOVERY_RANGE="$DISCOVERY_RANGE"
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
}

system_service_state() {
  local service="$1" rc
  systemctl is-active --quiet "$service"
  rc=$?
  case "$rc" in
    0) return 0 ;;
    3|4) return 1 ;;
    *) die "systemd에서 $service 상태를 확인할 수 없다 (exit=$rc)" ;;
  esac
}

reject_conflicting_services() {
  local service
  for service in \
      jdamr-cartographer-session.service \
      jdamr-operator-mapping.service \
      jdamr-localization.service \
      jdamr-slam-toolbox.service \
      jdamr-navigation.service; do
    if system_service_state "$service"; then
      die "$service 실행 중: 중지시키지 않았으며 localization/Nav2 중복 기동을 거부한다"
    fi
  done
}

check_ros_graph_and_sensors() {
  local nodes topics conflict missing="" topic
  if ! nodes=$(timeout 15 ros2 node list --no-daemon --spin-time 2 2>/dev/null); then
    die "ROS 노드 목록 조회 실패: domain 12/$DISCOVERY_RANGE 센서 연결을 확인할 것"
  fi
  conflict=$(grep -E \
    '(^|/)(amcl|map_server|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|collision_monitor|slam_toolbox|lifecycle_manager_localization|lifecycle_manager_navigation)$|cartographer' \
    <<<"$nodes" || true)
  if [ -n "$conflict" ]; then
    die "기존 localization/Cartographer/Nav2 노드가 있어 중복 기동하지 않는다: $(tr '\n' ' ' <<<"$conflict")"
  fi

  if ! topics=$(timeout 15 ros2 topic list --no-daemon --spin-time 2 2>/dev/null); then
    die "ROS 토픽 목록 조회 실패: 센서 세션을 별도로 진단할 것"
  fi
  for topic in /scan /odom /tf; do
    grep -qx "$topic" <<<"$topics" || missing="$missing $topic"
  done
  if [ -n "$missing" ]; then
    # A short first discovery sample can miss a running sensor after teardown.
    topics=$(timeout 15 ros2 topic list --no-daemon --spin-time 5 2>/dev/null) || \
      die "ROS 토픽 재조회 실패: 센서 연결을 확인할 것"
    missing=""
    for topic in /scan /odom /tf; do
      grep -qx "$topic" <<<"$topics" || missing="$missing $topic"
    done
  fi
  [ -z "$missing" ] || die "센서 진단 실패: 다음 토픽이 없다:$missing (jdamr-base.service와 센서 상태 확인)"
  printf '센서 진단: /scan /odom /tf 발견 (데이터 품질이나 주행 준비 완료 판정은 아님)\n'
}

validate_registry() {
  absolute_file "$REGISTRY" "registry"
  absolute_file "$PARAMS_FILE" "Nav2 params"
  if ! python3 - "$REGISTRY" <<'PY'
import sys
from jdamr_cube_navigation.service_destinations import load_registry

load_registry(sys.argv[1])
PY
  then
    die "registry 스키마·지도·keepout 경로 또는 해시 검증 실패: $REGISTRY"
  fi
}

parking_contract_path() {
  local name=parking_contract.yaml
  if [ "$PRECISION_PARKING" = true ]; then
    name=box_parking_contract.yaml
  fi
  printf '%s/config/%s' \
    "$WORKSPACE/install/jdamr_cube_navigation/share/jdamr_cube_navigation" "$name"
}

session_identity() {
  local asset_hashes digest parking_contract transport_hash=""
  parking_contract=$(parking_contract_path)
  absolute_file "$parking_contract" "주차 contract"
  asset_hashes=$(sha256sum -- "$REGISTRY" "$PARAMS_FILE" "$parking_contract" \
    "${BASH_SOURCE[0]}") || \
    die "세션 설정 해시 계산 실패"
  if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    absolute_file "$FASTRTPS_DEFAULT_PROFILES_FILE" "DDS profile"
    transport_hash=$(sha256sum -- "$FASTRTPS_DEFAULT_PROFILES_FILE") || \
      die "DDS profile 해시 계산 실패"
  fi
  digest=$(printf '%s\0' "$WORKSPACE" "$REGISTRY" "$PARAMS_FILE" \
    "$PRECISION_PARKING" "$PREPARE_ONLY" "$USE_COMPOSITION" "$DISCOVERY_RANGE" \
    "$asset_hashes" "$transport_hash" | sha256sum) || \
    die "세션 식별자 계산 실패"
  printf '%s' "${digest%% *}"
}

reuse_active_navigation() {
  local identity="$1" snapshot line state="" environment="" expected
  snapshot=$(systemctl show "$UNIT" --property=ActiveState --property=Environment) || \
    die "$UNIT 실행 설정 조회 실패"
  while IFS= read -r line; do
    case "$line" in
      ActiveState=*) state="${line#ActiveState=}" ;;
      Environment=*) environment="${line#Environment=}" ;;
    esac
  done <<<"$snapshot"
  [ "$state" = active ] || die "$UNIT가 실행 중이 아니다: ${state:-상태 미수신}"
  expected="JDAMR_RESTAURANT_SESSION_ID=$identity"
  if [[ " $environment " != *" $expected "* ]]; then
    die "$UNIT 실행 설정을 확인할 수 없거나 요청과 다르다; 기존 세션을 변경하지 않았다"
  fi
  printf '%s\n' "$UNIT가 같은 설정으로 실행 중이다. 재시작하지 않았다."
  printf '%s\n' "이는 주행 준비 완료 판정이 아니다. 초기 위치나 이동 명령을 보내지 않았다."
}

reuse_inactive_navigation() {
  local identity="$1" snapshot line load_state="" active_state="" sub_state=""
  local fragment_path="" environment="" expected
  snapshot=$(systemctl show "$UNIT" --property=LoadState --property=ActiveState \
    --property=SubState --property=FragmentPath --property=Environment) || \
    die "$UNIT 중지 설정 조회 실패"
  while IFS= read -r line; do
    case "$line" in
      LoadState=*) load_state="${line#LoadState=}" ;;
      ActiveState=*) active_state="${line#ActiveState=}" ;;
      SubState=*) sub_state="${line#SubState=}" ;;
      FragmentPath=*) fragment_path="${line#FragmentPath=}" ;;
      Environment=*) environment="${line#Environment=}" ;;
    esac
  done <<<"$snapshot"
  [ "$load_state" != not-found ] || return 1
  [ "$load_state" = loaded ] || \
    die "$UNIT load 상태를 확인할 수 없다: ${load_state:-상태 미수신}"
  [ "$active_state" = inactive ] && [ "$sub_state" = dead ] || \
    die "$UNIT가 재사용 가능한 inactive/dead 상태가 아니다: ${active_state:-?}/${sub_state:-?}"
  [ "$fragment_path" = "/run/systemd/transient/$UNIT" ] || \
    die "$UNIT가 이 스크립트가 만든 transient unit인지 확인할 수 없다; 기존 unit을 변경하지 않았다"
  expected="JDAMR_RESTAURANT_SESSION_ID=$identity"
  if [[ " $environment " != *" $expected "* ]]; then
    # A completed transient session can outlive its configuration. Release only
    # our inactive unit, then let start_navigation create the requested version.
    [[ " $environment " == *" JDAMR_RESTAURANT_INTERNAL=1 "* ]] || \
      die "$UNIT 중지 설정의 소유 표식을 확인할 수 없다; 기존 unit을 변경하지 않았다"
    # Preserve the owned fragment before --collect can remove it on StopUnit.
    local backup_dir
    [ "$(systemctl show "$UNIT" --property=ActiveState --value)" = inactive ] || \
      die "$UNIT 해제 중 상태가 변경됐다; 기존 unit을 변경하지 않았다"
    backup_dir=$(mktemp -d "${TMPDIR:-/tmp}/jdamr-retired-session.XXXXXX") || \
      die "종료된 세션 백업 경로 생성 실패"
    sudo -n cp -- "$fragment_path" "$backup_dir/$UNIT" || \
      die "종료된 세션 설정 보존 실패"
    printf '%s\n' "이전 세션 설정 보존: $backup_dir/$UNIT"
    sudo -n systemctl stop "$UNIT" || die "$UNIT 중지된 세션 해제 실패"
    sudo -n systemctl reset-failed "$UNIT" 2>/dev/null || true
    [ "$(systemctl show "$UNIT" --property=ActiveState --value)" = inactive ] || \
      die "$UNIT 해제 중 상태가 변경됐다; 기존 unit을 변경하지 않았다"
    # A referenced inactive unit may retain its fragment; an already collected
    # fragment is equally valid here. Never remove persistent or active units.
    sudo -n rm -f -- "$fragment_path" || die "종료된 세션 설정 해제 실패"
    sudo -n systemctl daemon-reload || die "systemd 세션 설정 갱신 실패"
    local attempt released
    for attempt in {1..10}; do
      released=$(systemctl show "$UNIT" --property=LoadState --value) || \
        die "$UNIT 해제 결과 조회 실패"
      if [ "$released" = not-found ]; then
        printf '%s\n' "$UNIT의 종료된 이전 설정을 해제했다. 새 설정으로 생성한다."
        return 1
      fi
      sleep 0.2
    done
    die "$UNIT 종료된 이전 설정이 남아 있다; 새 세션을 중복 생성하지 않았다"
  fi

  check_ros_graph_and_sensors
  sudo -n systemctl start "$UNIT" || die "$UNIT 재기동 요청 실패"
  snapshot=$(systemctl show "$UNIT" --property=ActiveState --property=Environment) || \
    die "$UNIT 재기동 결과 조회 실패"
  active_state=""
  environment=""
  while IFS= read -r line; do
    case "$line" in
      ActiveState=*) active_state="${line#ActiveState=}" ;;
      Environment=*) environment="${line#Environment=}" ;;
    esac
  done <<<"$snapshot"
  [ "$active_state" = active ] || \
    die "$UNIT 재기동 뒤 active가 아니다: ${active_state:-상태 미수신}"
  if [[ " $environment " != *" $expected "* ]]; then
    die "$UNIT 재기동 뒤 실행 설정을 확인할 수 없거나 요청과 다르다"
  fi
  printf '%s\n' "$UNIT의 같은 설정의 중지된 transient unit을 다시 시작했다."
  printf '%s\n' "이는 주행 준비 완료 판정이 아니다. 초기 위치나 이동 명령을 보내지 않았다."
  return 0
}

run_navigation() {
  local parking_contract
  local coordinated=true navigation_autostart=true
  if [ "$PREPARE_ONLY" = true ]; then
    coordinated=false
    navigation_autostart=false
  fi
  load_ros_environment
  validate_registry
  parking_contract=$(parking_contract_path)
  cd "$WORKSPACE" || die "workspace로 이동할 수 없다: $WORKSPACE"
  exec ros2 launch jdamr_cube_navigation restaurant_service.launch.py \
    "registry:=$REGISTRY" \
    "params_file:=$PARAMS_FILE" \
    navigation_profile:=new_base_candidate \
    "use_composition:=$USE_COMPOSITION" \
    "coordinated_startup:=$coordinated" \
    "navigation_autostart:=$navigation_autostart" \
    "precision_parking:=$PRECISION_PARKING" \
    "parking_contract:=$parking_contract" \
    use_box_observer:=false \
    "discovery_range:=$DISCOVERY_RANGE"
}

start_navigation() {
  absolute_directory "$WORKSPACE" "workspace"
  REGISTRY="${REGISTRY:-$HOME/jdamr_data/maps/20260922_manual_final_run2/service_destinations.yaml}"
  PARAMS_FILE="${PARAMS_FILE:-$WORKSPACE/install/jdamr_cube_navigation/share/jdamr_cube_navigation/config/new_base_nav2_params.yaml}"
  load_ros_environment
  validate_registry

  local identity
  identity=$(session_identity) || die "세션 식별자 생성 실패"
  system_service_state jdamr-base.service || die "jdamr-base.service가 active가 아니다; 이 스크립트는 베이스를 시작하지 않는다"
  reject_conflicting_services
  if system_service_state "$UNIT"; then
    reuse_active_navigation "$identity"
    return
  fi
  if reuse_inactive_navigation "$identity"; then
    return
  fi
  check_ros_graph_and_sensors

  local script_path run_user precision_argument=() prepare_argument=() transport_environment=()
  script_path=$(readlink -f "${BASH_SOURCE[0]}") || die "스크립트 경로 확인 실패"
  run_user=$(id -un) || die "실행 사용자 확인 실패"
  if [ "$PRECISION_PARKING" = true ]; then
    precision_argument=(--precision-parking)
  fi
  if [ "$PREPARE_ONLY" = true ]; then
    prepare_argument=(--prepare-only)
  fi
  if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    transport_environment=(--setenv="FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE")
  fi
  sudo -n systemd-run \
    --unit="$UNIT" --collect \
    --property="User=$run_user" \
    --property=KillMode=mixed \
    --property=KillSignal=SIGINT \
    --property=TimeoutStopSec=20 \
    --setenv="HOME=$HOME" \
    --setenv="ROS_DISTRO=${ROS_DISTRO:-jazzy}" \
    --setenv=JDAMR_RESTAURANT_INTERNAL=1 \
    --setenv="JDAMR_RESTAURANT_SESSION_ID=$identity" \
    "${transport_environment[@]}" \
    --working-directory="$WORKSPACE" \
    /bin/bash "$script_path" __run \
    --workspace "$WORKSPACE" --registry "$REGISTRY" --params-file "$PARAMS_FILE" \
    --use-composition "$USE_COMPOSITION" \
    --discovery-range "$DISCOVERY_RANGE" \
    "${precision_argument[@]}" "${prepare_argument[@]}" \
    || die "$UNIT 기동 요청 실패"

  printf '%s\n' "$UNIT 기동 요청을 수락했다."
  printf '%s\n' "이는 Nav2 프로세스 시작 요청이며 위치추정·주행 준비 완료가 아니다."
  printf '%s\n' "초기 pose를 추측해 넣거나 주행 action을 보내지 않았다. status와 journal로 별도 확인할 것."
}

status_navigation() {
  systemctl status --no-pager "$UNIT"
  local rc=$?
  printf '%s\n' "주의: active는 프로세스 상태이며 위치추정·주행 준비 완료 판정이 아니다."
  return "$rc"
}

stop_navigation() {
  sudo -n systemctl stop "$UNIT" || die "$UNIT 중지 요청 실패"
  printf '%s\n' "$UNIT 중지 요청을 보냈다. 베이스·센서 서비스는 변경하지 않았다."
}

COMMAND="${1:-}"
[ $# -gt 0 ] && shift
case "$COMMAND" in
  -h|--help|help)
    [ $# -eq 0 ] || die "help에는 추가 인자를 사용할 수 없다"
    usage
    ;;
  status|stop)
    [ $# -eq 0 ] || die "$COMMAND에는 추가 인자를 사용할 수 없다"
    "${COMMAND}_navigation"
    ;;
  start|__run)
    while [ $# -gt 0 ]; do
      case "$1" in
        --workspace|--registry|--params-file)
          [ $# -ge 2 ] || die "$1 값이 필요하다"
          case "$1" in
            --workspace) WORKSPACE="$2" ;;
            --registry) REGISTRY="$2" ;;
            --params-file) PARAMS_FILE="$2" ;;
          esac
          shift 2
          ;;
        --precision-parking)
          PRECISION_PARKING=true
          shift
          ;;
        --prepare-only)
          PREPARE_ONLY=true
          shift
          ;;
        --use-composition)
          [ $# -ge 2 ] || die "$1 값이 필요하다"
          case "$2" in
            true|false) USE_COMPOSITION="$2" ;;
            *) die "--use-composition 값은 true 또는 false여야 한다: $2" ;;
          esac
          shift 2
          ;;
        --discovery-range)
          [ $# -ge 2 ] || die "$1 값이 필요하다"
          case "$2" in
            LOCALHOST|SUBNET) DISCOVERY_RANGE="$2" ;;
            *) die "--discovery-range 값은 LOCALHOST 또는 SUBNET이어야 한다: $2" ;;
          esac
          shift 2
          ;;
        *) die "알 수 없는 인자: $1 (도움말: --help)" ;;
      esac
    done
    if [ "$COMMAND" = start ]; then
      start_navigation
    else
      [ "${JDAMR_RESTAURANT_INTERNAL:-}" = 1 ] || \
        die "내부 실행 경로는 start가 만든 systemd 유닛에서만 사용할 수 있다"
      run_navigation
    fi
    ;;
  '')
    usage >&2
    exit 2
    ;;
  *)
    die "알 수 없는 명령: $COMMAND (status/start/stop 또는 --help)"
    ;;
esac
