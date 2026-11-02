"use strict";

// 로봇 운영 콘솔. /api/state를 1초마다 읽어 주문 수락·미션 호출, 긴급 정지, 대기열, 직원 서빙과
// 배터리, 미션 단계, 조작 실행기, 지도 위 로봇 위치, ROS 2 토픽, 백엔드 명령, 이벤트를 스크롤 없이 한 화면에 보여 준다.
// 위치와 파이 전압은 서버를 --robot-pose-url 로 띄워 파이의 주행 대시보드(ammr_dashboard)를 읽을 때만 실제 값이다.
const SERVICE_PHASES = [
  ["NAVIGATE_KITCHEN", "주방 이동", "nav"], ["ALIGN_KITCHEN", "주방 정렬", "nav"], ["GRASP_CUP", "컵 파지", "manip"],
  ["GRASP_BOTTLE", "물통 파지", "manip"], ["POUR", "따르기", "manip"], ["RETURN_BOTTLE", "물통 반환", "manip"],
  ["PLACE_DECK", "상판 적재", "manip"], ["NAVIGATE_TABLE", "테이블 이동", "nav"], ["ALIGN_TABLE", "테이블 정렬", "nav"],
  ["REGRASP_CUP", "컵 재파지", "manip"], ["SERVE", "테이블 배치", "manip"],
];
const PHASE_LABEL = Object.fromEntries([
  ...SERVICE_PHASES.map(([k, l]) => [k, l]),
  ["IDLE_AT_DOCK", "충전소 대기"], ["NAVIGATE_DOCK", "충전소 복귀"],
  ["CHARGING", "충전 중"], ["TERMINAL_HOLD", "정지 대기"],
]);
const STATE_LABEL = { PENDING: "수락 대기", ACCEPTED: "수락됨", QUEUED: "출발 대기", RUNNING: "진행 중", SUCCEEDED: "완료", FAILED: "실패", CANCELED: "취소", REJECTED: "거절" };
const START_HINT = {
  emergency_stop: "긴급 정지 상태입니다",
  mission_running: "진행 중인 미션이 있습니다",
  no_accepted_order: "수락한 주문이 없습니다",
  manipulation_not_ready: "조작 실행기가 준비되지 않았습니다",
  drive_init_running: "주행 초기화 중입니다",
  drive_not_connected: "주행 PC가 연결되지 않았습니다",
};
const DRINKS = { COLD_WATER: "냉수", HOT_WATER: "온수", LEMONADE: "레몬에이드", COLA: "콜라" };
const MISSION_LABEL = { RUNNING: "진행 중", TERMINAL: "정지 대기" };
const DRIVE_LABEL = { running: "주행 중", stopped: "정지", succeeded: "완료", complete: "완료", failed: "실패", idle: "대기" };
const MANIP_LABEL = { ready: "준비됨", running: "실행 중", loading: "로딩 중", unreachable: "연결 안 됨" };
const TASK_LABEL = { pour: "물따르기", place: "컵배치", stow: "팔 접기", stow_right: "오른팔 접기" };
// 퍼센트 환산 기준 [0 %, 100 %] 전압. 저장소에 근거가 없어 비워 두며, 비어 있으면 볼트만 표시한다.
const BATTERY_RANGE_V = { arm: null, pi: null };
// 경고 전압: 팔은 tools/rollout/arm_health.py, 파이는 restaurant_service_contract.yaml(출발 10.8 V, 주행 10.5 V)
const BATTERY_ALERT_V = { arm: { warn: 9.0, bad: 9.0 }, pi: { warn: 10.8, bad: 10.5 } };

// 파이 주행 대시보드(ammr_dashboard)가 구독하는 토픽과 스냅샷 안의 위치
const vel = (v) => v.linear_mps != null ? `v ${v.linear_mps.toFixed(2)} · ω ${v.angular_rps.toFixed(2)}` : "";
const TOPICS = [
  ["/scan", (n) => n.lidar, (v) => v.fps != null ? `${v.fps} Hz` : ""],
  ["/odom", (n) => n.motion?.odom, vel],
  ["/cmd_vel", (n) => n.motion?.cmd, vel],
  ["/cmd_vel_smoothed", (n) => n.motion?.requested_cmd, vel],
  ["/tf map→base_link", (n) => n.navigation?.localization, (v) => v.x_m != null ? `(${v.x_m.toFixed(2)}, ${v.y_m.toFixed(2)}) ${Math.round(v.yaw_deg)}°` : ""],
  ["/map", (n) => n.navigation?.map, (v) => v.width ? `${v.width}×${v.height} @${v.resolution_m} m` : ""],
  ["/plan", (n) => n.navigation?.path, (v) => v.points_xy_m ? `${v.points_xy_m.count ?? v.points_xy_m.length} pts` : ""],
  ["/navigate_to_pose", (n) => n.navigation?.goal, (v) => v.status || ""],
  ["/collision_monitor_state", (n) => n.navigation?.collision, (v) => v.action || v.state || ""],
  ["/box_parking/execution_status", (n) => n.navigation?.execution, (v) => v.state || ""],
  ["/battery_state", (n) => n.battery, (v) => v.voltage_v != null ? `${v.voltage_v.toFixed(2)} V` : ""],
  ["/imu/data_raw", (n) => n.imu, (v) => v.angular_z_rps != null ? `ωz ${v.angular_z_rps.toFixed(2)}` : ""],
  ["/depth_navigation/status", (n) => n.depth_navigation, (v) => v.state || v.mode || ""],
];

const $ = (id) => document.getElementById(id);
let state = null;
let apiLatencyMs = null;

const place = (id) => !id ? "-" : id === "dock" ? "충전소" : id === "kitchen" ? "주방" : /^table_\d+$/.test(id) ? `${id.slice(6)}번 테이블` : id;
const time = (iso) => iso ? new Date(iso).toLocaleTimeString("en-GB") : "-";
const secs = (s) => s == null ? "-" : s < 1 ? `${Math.round(s * 1000)} ms` : s < 120 ? `${s.toFixed(1)} s` : `${Math.floor(s / 60)}m ${Math.round(s % 60)}s`;

function el(tag, text, className) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
}
function row(cells) {
  const tr = document.createElement("tr");
  cells.forEach((c) => { const td = document.createElement("td"); c instanceof Node ? td.append(c) : (td.textContent = c); tr.append(td); });
  return tr;
}
function emptyRow(text, span) {
  const tr = document.createElement("tr");
  const td = el("td", text, "empty");
  td.colSpan = span;
  tr.append(td);
  return tr;
}
function failureText(failure) {
  if (!failure) return "-";
  const [phase, code] = failure.split(":");
  return code ? `${PHASE_LABEL[phase] || phase} · ${code}` : failure;
}
function badge(orderState) { return el("span", STATE_LABEL[orderState] || orderState, `badge ${orderState}`); }
function setDot(id, level) { $(id).className = `dot ${level}`; }
function statusCell(level, text) { const s = el("span", text, `st ${level}`); s.prepend(el("i", undefined, `dot ${level === "off" ? "" : level}`)); return s; }
function logLine(at, key, text, className) {
  const li = el("li", undefined, className);
  li.append(el("span", time(at), "t"), el("span", key, "k"), document.createTextNode(text));
  li.title = `${time(at)} ${key} ${text}`;
  return li;
}

// 주문 번호 표시: 9자리(YYMMDDNNN)가 아닌 예전 번호(WEB-…, STAFF-…, 시험용)도 같은 형식으로 보여 준다.
// 접수 날짜별로 시간순으로, 그날 이미 쓰인 9자리 번호를 건너뛰며 남는 번호를 붙인다(화면 표시만, 서버 기록은 그대로).
let displayNo = new Map();
const ORDER_NO = /^\d{9}$/;
function buildDisplayNumbers(orders, tickets) {
  const all = new Map();
  for (const o of [...orders, ...tickets]) if (!all.has(o.order_id)) all.set(o.order_id, o.created_at);
  const used = new Map(); // YYMMDD -> 쓰인 일련번호
  const legacy = [];
  for (const [id, created] of all) {
    if (ORDER_NO.test(id)) {
      if (!used.has(id.slice(0, 6))) used.set(id.slice(0, 6), new Set());
      used.get(id.slice(0, 6)).add(Number(id.slice(6)));
    } else legacy.push([id, created ? new Date(created) : new Date()]);
  }
  legacy.sort((a, b) => a[1] - b[1]);
  const map = new Map();
  const two = (n) => String(n).padStart(2, "0");
  for (const [id, when] of legacy) {
    const day = `${String(when.getFullYear()).slice(2)}${two(when.getMonth() + 1)}${two(when.getDate())}`;
    if (!used.has(day)) used.set(day, new Set());
    const taken = used.get(day);
    let n = 1;
    while (taken.has(n)) n += 1;
    taken.add(n);
    map.set(id, `${day}${String(n).padStart(3, "0")}`);
  }
  displayNo = map;
}
const no = (id) => (id ? displayNo.get(id) || id : id);
const itemsText = (ticket) => ticket ? ticket.items.filter((i) => !i.robot).map((i) => `${i.name}×${i.qty}`).join(", ") : "";
const payload = (o) => DRINKS[o.drink] || o.drink;

async function api(path, body) {
  const options = {};
  if (body !== undefined) {
    options.method = "POST";
    options.headers = { "Content-Type": "application/json" };
    options.body = JSON.stringify(body);
  }
  return window.orderApi(path, options);
}

async function refresh() {
  try {
    const t0 = performance.now();
    state = await api("/api/state");
    apiLatencyMs = performance.now() - t0;
    $("conn").textContent = `서버 연결 · ${apiLatencyMs.toFixed(0)} ms`;
    $("conn").className = "conn";
    render();
  } catch {
    $("conn").textContent = "서버 연결 끊김 — 재연결 중";
    $("conn").className = "conn bad";
  }
}

const isSim = () => ["browser_demo", "simulation_dry_run"].includes(state.execution_backend?.mode);

// ---------- 데모·dry-run 용 시뮬레이션 값 (실제 로봇 값이 아님) ----------
function simManip() {
  const phase = state.phase;
  const running = ["GRASP_CUP", "GRASP_BOTTLE", "POUR", "RETURN_BOTTLE", "PLACE_DECK"].includes(phase) ? "pour"
    : ["REGRASP_CUP", "SERVE"].includes(phase) ? "place" : null;
  return {
    state: running ? "running" : "ready", current: running,
    last_result: null,
  };
}

// ---------- 그리기 ----------
function render() {
  const tickets = new Map((state.tickets || []).map((t) => [t.order_id, t]));
  const active = state.orders.find((o) => o.order_id === state.active_order_id) || null;
  const eb = state.execution_backend || {};
  const mode = eb.mode || "-";
  buildDisplayNumbers(state.orders, state.tickets || []);
  const sim = isSim();

  $("demo-note").hidden = window.orderApiMode !== "demo";

  // 미션
  $("s-mission").textContent = PHASE_LABEL[state.phase] || state.phase;
  $("s-mission-sub").textContent = active ? `주문 ${no(active.order_id)} · ${state.phase_attempt}회째 시도` : `대기열 ${state.queue.length}건`;
  setDot("k-mission-dot", state.phase === "TERMINAL_HOLD" ? "bad" : active ? "run" : "ok");
  $("fsm-tag").textContent = MISSION_LABEL[state.mission_state] || "대기";
  $("fsm-tag").className = `tag ${active ? "run" : "ok"}`;

  // 조작 실행기
  const manip = eb.manipulation_service_state ? {
    state: eb.manipulation_service_state, current: eb.manipulation_service_current || eb.running_policy,
    last_result: eb.manipulation_last_result, arms: eb.arms,
  } : sim ? simManip() : null;
  renderManip(manip, sim && !eb.manipulation_service_state);

  renderBatteries(eb);

  // 정렬 완료 신호 대기 (rollout-service backend). 주행 연동 전에는 운영자가 확인 후 누른다.
  const waiting = eb.waiting_alignment;
  $("align-banner").hidden = !waiting;
  if (waiting) {
    const next = waiting.station === "kitchen" ? "물따르기" : "컵배치";
    const where = waiting.station === "kitchen" ? "주방(급수대)" : "테이블";
    const armsOk = eb.drive ? eb.drive.arms_ready : true;
    const arms = eb.arms ? ` · 팔 ${ARMS_KO[eb.arms] || eb.arms}` : "";
    $("align-text").textContent = `${armsOk ? `차를 ${where}로 이동·정렬하세요` : "⚠ 팔이 이동 자세가 아닙니다. 차를 움직이지 마세요"}${arms}. 차가 멈추고 정렬됐는지 확인한 뒤 누르면 ${next}을 실행합니다.`;
    $("align-confirm").textContent = `정렬 완료 → ${next} 실행`;
    $("align-confirm").dataset.station = waiting.station;
  }
  $("paused-banner").hidden = !state.auto_paused;
  renderRunControls();
  $("test-tools").hidden = !sim;
  $("t-reset").hidden = mode !== "browser_demo";

  renderFsm(active, tickets);
  renderJobs(tickets);
  renderStaff();
  renderTopics();
  renderCommands(eb);
  renderEvents();
  drawMap(active);
  renderMapInfo();
}

function renderRunControls() {
  const start = state.start || {};
  $("start").disabled = !start.ready;
  const link = state.drive_link;
  $("drive-link").textContent = !link ? "" : !link.connected ? "주행 PC 미연결"
    : `주행 PC 연결 · ${link.dry_run ? "감지만 · " : ""}${DRIVE_LABEL[link.phase || link.state] || link.phase || link.state || "-"}${link.error ? ` · ${link.error.slice(0, 80)}` : ""}`;
  $("drive-link").parentElement.title = $("drive-link").textContent;
  $("drive-link").className = link && !link.connected ? "conn bad" : "conn";
  const noDrive = link && !link.connected ? " · 주행 PC 미연결: 차는 출발하지 않습니다" : "";
  $("start-hint").textContent = start.ready ? `START: 주문 ${no(start.order_id)}${noDrive}` : START_HINT[start.reason] || start.reason || "";
  const init = state.drive_init || {};
  const initActive = ["requested", "running"].includes(init.state);
  $("drive-init").disabled = !init.ready;
  $("drive-init").textContent = initActive ? "초기화 중…" : "주행 초기화";
  $("drive-init").title = START_HINT[init.reason] || "";
  $("init-hint").textContent = init.state === "succeeded" ? "주행 초기화 완료"
    : init.state === "failed" ? `주행 초기화 실패: ${(init.message || "").slice(0, 120)}` : "";
  $("init-hint").title = $("init-hint").textContent;
  const stop = state.emergency_stop || {};
  $("estop-banner").hidden = !stop.active;
  if (stop.active) {
    const order = stop.order_id ? ` · 주문 ${no(stop.order_id)} 중단` : "";
    $("estop-text").textContent = `긴급 정지 ${time(stop.at)}${order}. 다음 단계와 START를 막고 있습니다. 로봇과 팔 상태를 직접 확인한 뒤 해제하세요.`;
  }
}

function phaseDurations(active) {
  // 현재 미션의 단계별 걸린 시간: MISSION_STARTED -> 각 PHASE_SUCCEEDED 사이 간격
  const out = {};
  if (!active) return out;
  let prev = null;
  for (const e of state.events) {
    if (e.order_id !== active.order_id) continue;
    const t = new Date(e.timestamp).getTime();
    if (e.kind === "MISSION_STARTED") prev = t;
    else if (e.kind === "PHASE_SUCCEEDED" && prev != null) { out[e.completed_phase] = (t - prev) / 1000; prev = t; }
  }
  if (prev != null) out.__now = (Date.now() - prev) / 1000;
  return out;
}

function renderFsm(active, tickets) {
  const head = $("current-head");
  const phaseName = PHASE_LABEL[state.phase] || state.phase;
  if (active) {
    const food = itemsText(tickets.get(active.order_id));
    head.replaceChildren(
      el("b", `주문 ${no(active.order_id)}`),
      el("span", place(active.table_id)),
      el("span", `${payload(active)}${food ? ` + ${food}` : ""}`),
      el("span", `${phaseName} · ${state.phase_attempt}회째 시도`),
    );
  } else {
    head.replaceChildren(el("span", `진행 중인 미션 없음 · ${phaseName} · 위치 ${place(state.current_location)}`, "muted"));
  }
  const durations = phaseDurations(active);
  const nowIndex = active ? SERVICE_PHASES.findIndex(([key]) => key === state.phase) : -1;
  $("phases").replaceChildren(...SERVICE_PHASES.map(([key, label, layer], i) => {
    const cls = nowIndex < 0 ? "ph" : i < nowIndex ? "ph done" : i === nowIndex ? "ph now" : "ph";
    const box = el("div", undefined, cls);
    box.title = label;
    box.append(el("em", layer === "nav" ? "NAV" : "ARM", `layer ${layer}`), el("b", label),
      el("i", i === nowIndex ? secs(durations.__now) : durations[key] != null ? secs(durations[key]) : "–"));
    return box;
  }));
}

// 배터리: 값이 없으면 가짜 값 대신 연결 안 됨으로 표시한다. battery_percent(미션 모델 추정값)는 실측이 아니라 쓰지 않는다.
function showBattery(id, kind, voltage, source) {
  const range = BATTERY_RANGE_V[kind];
  const alert = BATTERY_ALERT_V[kind];
  if (!Number.isFinite(voltage)) {
    $(id).textContent = "연결 안 됨";
    $(`${id}-sub`).textContent = source;
    setDot(`${id}-dot`, "");
    return;
  }
  const volts = `${voltage.toFixed(kind === "pi" ? 2 : 1)} V`;
  const percent = range ? Math.round(Math.min(1, Math.max(0, (voltage - range[0]) / (range[1] - range[0]))) * 100) : null;
  $(id).textContent = percent == null ? volts : `${percent} % · ${volts}`;
  $(`${id}-sub`).textContent = percent == null ? "퍼센트 기준 미정" : "전압 환산";
  setDot(`${id}-dot`, voltage < alert.bad ? "bad" : voltage < alert.warn ? "warn" : "ok");
}
function renderBatteries(eb) {
  const nav = state.robot_nav;
  const arm = (side) => eb.arm_batteries?.[side];
  const armSource = eb.manipulation_service_state ? "실행기가 전압을 보내지 않음" : "조작 실행기 없음";
  showBattery("b-left", "arm", arm("left"), armSource);
  showBattery("b-right", "arm", arm("right"), armSource);
  const pi = nav?.connected ? nav.snapshot?.battery : null;
  showBattery("b-pi", "pi", pi?.live ? pi.voltage_v : NaN, !nav ? "주행 대시보드 미연결" : nav.connected ? "전압 수신 없음" : "주행 대시보드 연결 실패");
}

// rollout_service /status 의 arms 값
const ARMS_KO = { stowed: "이동 자세", carrying_cup: "컵 유지·오른팔 접힘", busy: "동작 중", hold: "그 자리 유지(확인 필요)", unknown: "미확인" };

function renderManip(m, simulated) {
  if (!m) {
    $("s-manip").textContent = "사용 안 함";
    $("s-manip-sub").textContent = "모의 실행";
    setDot("k-manip-dot", "");
    return;
  }
  const level = m.state === "running" ? "run" : m.state === "ready" ? "ok" : m.state === "unreachable" ? "bad" : "warn";
  $("s-manip").textContent = `${MANIP_LABEL[m.state] || m.state}${m.current ? ` · ${TASK_LABEL[m.current] || m.current}` : ""}`;
  const r = m.last_result;
  const failed = r && !r.ok ? ` · 마지막 실패 ${TASK_LABEL[r.task] || r.task} ${r.reason || ""}` : "";
  $("s-manip-sub").textContent = `${simulated ? "데모 · " : ""}팔 ${m.arms ? (ARMS_KO[m.arms] || m.arms) : "미확인"}${failed}`;
  $("s-manip-sub").title = $("s-manip-sub").textContent;
  setDot("k-manip-dot", level);
}

function renderTopics() {
  const feed = state.robot_nav;
  const nav = feed?.connected ? feed.snapshot : null;
  let liveCount = 0;
  $("topics").replaceChildren(...TOPICS.map(([name, pick, detail]) => {
    const v = nav ? pick(nav) : null;
    let cell = statusCell("off", "없음");
    if (v && v.age_s != null) {
      cell = v.live ? statusCell("ok", "수신") : statusCell("warn", "지연");
      if (v.live) liveCount += 1;
    }
    const text = v ? detail(v) : "";
    const age = v && v.age_s != null ? secs(v.age_s) : "-";
    const tr = row([cell, el("span", name, "mono"), text || age]);
    tr.title = `${name} · ${text || "-"} · 경과 ${age}`;
    return tr;
  }));
  $("topic-tag").textContent = nav ? `${liveCount}/${TOPICS.length} 수신` : "미연결";
  $("topic-tag").className = `tag ${nav ? "ok" : ""}`;
  const parts = [];
  if (nav) {
    const g = nav.graph || {};
    parts.push(`대시보드 ${feed.latency_ms} ms`, `노드 ${g.nodes ?? "-"}`, `토픽 ${g.topics ?? "-"}`);
    for (const [k, val] of Object.entries(nav.system || {})) if (typeof val === "number" && k !== "age_s") parts.push(`${k} ${val.toFixed(1)}`);
  }
  $("topic-note").textContent = nav ? parts.join(" · ") : feed?.error ? `수신 실패: ${feed.error}` : "주행 대시보드 미연결(--robot-pose-url)";
  $("topic-note").title = $("topic-note").textContent;
}

function renderCommands(eb) {
  let cmds = state.recent_commands || [];
  if (!cmds.length && isSim()) {
    // 데모 백엔드는 명령 기록이 없어서 이벤트의 단계 완료로 대신 보여 준다.
    cmds = state.events.filter((e) => e.kind === "PHASE_SUCCEEDED" || e.kind === "MISSION_FAILED").map((e) => ({
      at: e.timestamp, phase: e.completed_phase || e.failed_phase, success: e.kind === "PHASE_SUCCEEDED", failure_code: e.failure || null,
      adapter: eb.mode,
    }));
  }
  $("cmd-count").textContent = `${cmds.length}`;
  $("commands").replaceChildren(...(cmds.length ? cmds.slice(-20).reverse().map((c) => logLine(c.at, PHASE_LABEL[c.phase] || c.phase || "-",
    [c.success == null ? "-" : c.success ? "성공" : `실패 ${c.failure_code || ""}`, c.kind ? `${c.kind}:${c.name || "-"}` : "", c.duration_s != null ? secs(c.duration_s) : "", c.adapter || ""].filter(Boolean).join(" · "),
    c.success === false ? "fail" : c.success ? "ok" : "")) : [el("li", "아직 실행된 명령이 없습니다.")]));
  const t = eb.manipulation_service_timings || {};
  const ms = (s) => s != null ? `${Math.round(s * 1000)} ms` : "-";
  const r = eb.manipulation_last_result;
  const last = r ? ` · 마지막 ${TASK_LABEL[r.task] || r.task} ${r.ok ? "성공" : "실패"}${r.reason ? ` ${r.reason}` : ""}` : "";
  $("manip-note").textContent = eb.manipulation_service_state ? `추론 물따르기 ${ms(t.pour_infer_s)} · 컵배치 ${ms(t.place_infer_s)}${last}` : "";
  $("manip-note").title = $("manip-note").textContent;
}

function renderEvents() {
  $("events").replaceChildren(...state.events.slice(-40).reverse().map((e) => {
    const phase = e.completed_phase || e.failed_phase;
    return logLine(e.timestamp, e.kind, [no(e.order_id), PHASE_LABEL[phase] || phase, failureText(e.failure)].filter((x) => x && x !== "-").join(" · "),
      /FAIL/.test(e.kind) ? "fail" : /SUCCEEDED|ARRIVED|COMPLETED/.test(e.kind) ? "ok" : "");
  }));
}

function orderButton(label, className, handler) {
  const button = el("button", label, `btn small ${className}`);
  button.type = "button";
  button.addEventListener("click", handler);
  return button;
}

function orderActions(o) {
  const box = el("div", undefined, "row-actions");
  const id = o.order_id;
  if (o.state === "PENDING") {
    box.append(
      orderButton("수락", "primary", () => act(() => api(`/api/orders/${encodeURIComponent(id)}/accept`, {}))),
      orderButton("거절", "danger", () => {
        if (confirm(`주문 ${no(id)}을(를) 거절할까요?`)) act(() => api(`/api/orders/${encodeURIComponent(id)}/reject`, {}));
      }),
    );
  } else if (o.state === "ACCEPTED") {
    box.append(orderButton("취소", "", () => cancelOrder(id)));
  }
  return box;
}

function renderJobs(tickets) {
  const open = state.orders.filter((o) => ["PENDING", "ACCEPTED", "QUEUED", "RUNNING"].includes(o.state));
  const pending = open.filter((o) => o.state === "PENDING").length;
  $("queue-count").textContent = pending ? `· 수락 대기 ${pending}건` : "";
  const queueRows = open.map((o) => {
    const food = itemsText(tickets.get(o.order_id));
    return row([el("span", no(o.order_id), "mono"), place(o.table_id), `${payload(o)}${food ? ` + ${food}` : ""}`,
      badge(o.state), orderActions(o)]);
  });
  $("queue").replaceChildren(...(queueRows.length ? queueRows : [emptyRow("들어온 주문이 없습니다.", 5)]));
}

function renderStaff() {
  // 직원이 가져다줄 음식(로봇은 음료만 맡는다)
  const staff = (state.tickets || []).filter((t) => t.staff_state === "PREPARING");
  $("staff-count").textContent = staff.length ? `${staff.length}건` : "";
  $("staff-card").hidden = !staff.length;
  $("staff").replaceChildren(...staff.map((t) => {
    const served = el("button", "서빙 완료", "btn small primary");
    served.type = "button";
    served.addEventListener("click", () => act(() => api(`/api/tickets/${encodeURIComponent(t.order_id)}/served`, {})));
    return row([el("span", no(t.order_id), "mono"), place(t.table_id), itemsText(t), served]);
  }));
}

async function act(fn) {
  try { await fn(); } catch (error) { alert(`요청 실패: ${error.message}`); }
  refresh();
}
function cancelOrder(id) {
  if (confirm(`주문 ${no(id)}을(를) 취소할까요?`)) act(() => api(`/api/orders/${encodeURIComponent(id)}/cancel`, {}));
}

$("align-confirm").addEventListener("click", () => {
  const station = $("align-confirm").dataset.station;
  if (confirm("차가 멈추고 정렬된 것을 확인했나요? 누르면 팔이 움직입니다.")) {
    act(() => api("/api/align/confirm", { station }));
  }
});
$("drive-init").addEventListener("click", () => {
  if (confirm("차가 도크에 있나요?\n차는 움직이지 않고 Nav2를 켜서 위치만 다시 잡습니다.")) {
    act(() => api("/api/drive/init", {}));
  }
});
$("start").addEventListener("click", () => {
  const id = state?.start?.order_id;
  if (id && confirm(`주문 ${no(id)}을(를) 시작할까요?\n팔이 접히고 전체 시나리오가 시작됩니다.`)) {
    act(() => api("/api/mission/start", {}));
  }
});
// 긴급 정지는 확인 창 없이 바로 보낸다.
$("stop").addEventListener("click", () => act(() => api("/api/emergency-stop", {})));
$("estop-reset").addEventListener("click", () => {
  if (confirm("로봇과 팔 상태를 직접 확인했나요?\n해제해도 로봇은 스스로 움직이지 않습니다. 다음 주문은 START를 눌러야 시작됩니다.")) {
    act(() => api("/api/emergency-stop/reset", {}));
  }
});
// ---------- 지도: 교실 지도(classroom) 위에 충전소·급수대·테이블과 로봇 위치 ----------
// 실시간 위치는 서버의 --robot-pose-url(파이 주행 대시보드)에서 온다. 없으면 진행 단계와 경과 시간으로 추정한다.
const MAP = { meta: null, img: null };
fetch("/map/classroom.json").then((r) => r.json()).then((meta) => {
  const img = new Image();
  img.onload = () => { MAP.meta = meta; MAP.img = img; if (state) render(); };
  img.src = "/map/classroom.png";
}).catch(() => { $("map-note").textContent = "지도를 불러오지 못했습니다."; });

const STATION_COLOR = { dock: "#2b8a5a", kitchen: "#e8590c" };
const STATION_FRAME = { dock: "home_dock", kitchen: "water_station", table_1: "table_01" };
function stationOf(id) {
  if (!MAP.meta || !id) return null;
  const stations = MAP.meta.stations;
  // 실제 지도에는 아직 1번 테이블만 등록돼 있다. 다른 테이블은 1번 자리로 대신 표시한다.
  return stations.find((s) => s.id === id) || (/^table_\d+$/.test(id) ? stations.find((s) => s.id === "table_1") : null);
}
// 추정 위치: 이동 단계가 시작된 뒤 지난 시간으로 출발지 -> 목적지 사이를 채운다(실위치 연동 전 임시).
const NAV_TARGET = { NAVIGATE_KITCHEN: "kitchen", NAVIGATE_DOCK: "dock" };
const ASSUMED_SPEED_MPS = 0.3; // 실차 평균 속도 가정(측정값 아님)
const motion = { key: null, startedAt: 0, from: null };
let lastActive = null;
let lastPose = null;
let lastPoseLabel = "";
function estimatedPose(active) {
  const here = stationOf(state.current_location) || stationOf("dock");
  const target = state.phase === "NAVIGATE_TABLE" ? active?.table_id : NAV_TARGET[state.phase];
  const there = target ? stationOf(target) : null;
  const key = `${state.active_order_id}|${state.phase}|${state.phase_attempt}`;
  if (motion.key !== key) { motion.key = key; motion.startedAt = performance.now(); motion.from = here; }
  if (!there || there === motion.from) return { x_m: here.x_m, y_m: here.y_m, yaw_deg: here.yaw_rad * 180 / Math.PI, goal: null };
  const from = motion.from;
  const dist = Math.hypot(there.x_m - from.x_m, there.y_m - from.y_m);
  const mode = state.execution_backend?.mode;
  // 데모·dry-run은 단계가 정해진 시간에 끝나므로 그 시간에 맞추고, 실차는 거리/가정 속도로 잡는다.
  const expectedMs = mode === "browser_demo" ? 2500 : mode === "simulation_dry_run" ? 750 : (dist / ASSUMED_SPEED_MPS) * 1000;
  const t = Math.min(0.97, (performance.now() - motion.startedAt) / expectedMs);
  return {
    x_m: from.x_m + (there.x_m - from.x_m) * t, y_m: from.y_m + (there.y_m - from.y_m) * t,
    yaw_deg: Math.atan2(there.y_m - from.y_m, there.x_m - from.x_m) * 180 / Math.PI, from, to: there,
    goal: there, remaining_m: dist * (1 - t),
  };
}
function fitMapCanvas(canvas, aspect) {
  // 전체 지도를 칸 크기에 맞춰 비율을 유지한 채 가장 크게 그린다. 크기가 바뀔 때만 다시 잡는다.
  const box = $("map-wrap");
  const cssWidth = Math.max(1, Math.floor(Math.min(box.clientWidth, box.clientHeight * aspect)));
  const cssHeight = Math.max(1, Math.floor(cssWidth / aspect));
  const ratio = window.devicePixelRatio || 1;
  const width = Math.round(cssWidth * ratio);
  const height = Math.round(cssHeight * ratio);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width; canvas.height = height;
    canvas.style.width = `${cssWidth}px`; canvas.style.height = `${cssHeight}px`;
  }
  return { cssWidth, cssHeight, ratio };
}
function drawMap(active) {
  lastActive = active;
  const canvas = $("map");
  if (!MAP.meta || !state) return;
  const { meta, img } = MAP;
  const { cssWidth, cssHeight, ratio } = fitMapCanvas(canvas, meta.width / meta.height);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(img, 0, 0, cssWidth, cssHeight);
  const scale = cssWidth / meta.width;
  const px = (x, y) => [((x - meta.origin.x_m) / meta.resolution_m) * scale, (meta.height - (y - meta.origin.y_m) / meta.resolution_m) * scale];
  // 1 m 격자
  ctx.save(); ctx.strokeStyle = "rgba(22,119,200,.10)"; ctx.lineWidth = 1;
  for (let x = Math.ceil(meta.origin.x_m); x < meta.origin.x_m + meta.width * meta.resolution_m; x += 1) { const [gx] = px(x, 0); ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, cssHeight); ctx.stroke(); }
  for (let y = Math.ceil(meta.origin.y_m); y < meta.origin.y_m + meta.height * meta.resolution_m; y += 1) { const [, gy] = px(0, y); ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(cssWidth, gy); ctx.stroke(); }
  ctx.restore();
  ctx.font = "bold 12px Consolas, monospace";
  for (const s of meta.stations) {
    const [x, y] = px(s.x_m, s.y_m);
    ctx.fillStyle = STATION_COLOR[s.id] || "#7a4fd1";
    ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#1f3553";
    const right = x > cssWidth * 0.6; // 오른쪽 끝 역은 이름을 왼쪽에 써서 잘리지 않게 한다
    ctx.textAlign = right ? "right" : "left";
    // 전체 지도에서는 충전소와 급수대 이름이 겹치므로 급수대 이름은 점 아래에 쓴다
    ctx.fillText(STATION_FRAME[s.id] || s.id, right ? x - 9 : x + 9, s.id === "kitchen" ? y + 16 : y - 8);
  }
  const rp = state.robot_pose;
  let pose = null;
  let label = "";
  if (rp && rp.live) { pose = rp; label = "LIVE"; }
  else if (rp && rp.last) { pose = rp.last; label = "LAST"; }
  else { pose = estimatedPose(active); label = "EST"; }
  // 실시간 경로(/plan)가 있으면 그린다
  const plan = state.robot_nav?.snapshot?.navigation?.path?.points_xy_m;
  if (Array.isArray(plan) && plan.length > 1) {
    ctx.save(); ctx.strokeStyle = "rgba(60,207,142,.9)"; ctx.lineWidth = 2; ctx.beginPath();
    plan.forEach(([x, y], i) => { const [qx, qy] = px(x, y); i ? ctx.lineTo(qx, qy) : ctx.moveTo(qx, qy); });
    ctx.stroke(); ctx.restore();
  } else if (pose.from) {
    const [ax, ay] = px(pose.from.x_m, pose.from.y_m);
    const [bx, by] = px(pose.to.x_m, pose.to.y_m);
    ctx.save(); ctx.setLineDash([6, 5]); ctx.strokeStyle = "rgba(22,119,200,.7)"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.restore();
  }
  // 로봇: 퍼지는 원 + 파란 점 + 진행 방향 짧은 선
  const [rx, ry] = px(pose.x_m, pose.y_m);
  const color = label === "LAST" ? "120,134,150" : "22,119,200";
  const pulse = (performance.now() % 1600) / 1600;
  ctx.fillStyle = `rgba(${color},${0.35 * (1 - pulse)})`;
  ctx.beginPath(); ctx.arc(rx, ry, 8 + 14 * pulse, 0, Math.PI * 2); ctx.fill();
  const yaw = -(pose.yaw_deg - (meta.origin.yaw_deg || 0)) * Math.PI / 180;
  ctx.strokeStyle = `rgb(${color})`; ctx.lineWidth = 3; ctx.lineCap = "round";
  ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(rx + Math.cos(yaw) * 15, ry + Math.sin(yaw) * 15); ctx.stroke();
  ctx.fillStyle = `rgb(${color})`; ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.arc(rx, ry, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  lastPose = pose;
  lastPoseLabel = label;
}
function renderMapInfo() {
  const pose = lastPose;
  const label = lastPoseLabel;
  const src = $("map-src");
  src.textContent = { LIVE: "실시간 위치", LAST: "마지막 위치", EST: "추정 위치" }[label] || "";
  src.className = label === "LIVE" ? "pose-live" : "pose-est";
  if (!pose) return;
  const where = `(${pose.x_m.toFixed(2)}, ${pose.y_m.toFixed(2)}) m · ${pose.yaw_deg.toFixed(0)}°`;
  const reason = label === "LIVE" ? "" : state.robot_pose?.error ? ` · 수신 실패: ${state.robot_pose.error}` : " · 실시간 위치 미연결, 진행 단계로 추정";
  $("map-note").textContent = `${where}${reason}`;
  $("map-note").title = $("map-note").textContent;
}
// 1초마다 오는 상태 사이에도 점이 부드럽게 움직이도록 지도만 다시 그린다.
(function animateMap() {
  if (MAP.meta && state && !document.hidden) drawMap(lastActive);
  requestAnimationFrame(animateMap);
})();
$("demo-note").hidden = window.orderApiMode !== "demo";
$("t-reset").addEventListener("click", () => {
  if (confirm("데모 주문과 기록을 모두 지울까요?")) window.resetDemo().then(() => location.reload());
});
$("t-step").addEventListener("click", () => act(() => api("/api/step", { success: true })));
$("t-fail").addEventListener("click", () => act(() => api("/api/step", { success: false, failure: "OPERATOR_INJECTED" })));

refresh();
setInterval(refresh, 1000);
