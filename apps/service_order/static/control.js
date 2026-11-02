"use strict";

// 로봇 운영 콘솔(개발자용 관제). /api/state를 1초마다 읽어 미션 상태기계, ROS 2 토픽 상태, backend 명령 기록,
// 조작 실행기(LeRobot rollout service), 지도 위 로봇 위치를 보여 준다.
// 주행 쪽 값(토픽·위치)은 서버를 --robot-pose-url 로 띄워 파이의 주행 대시보드(ammr_dashboard)를 읽을 때만 실제 값이다.
// 브라우저 데모·dry-run 에서는 같은 화면을 시뮬레이션 값으로 채우고 DEMO/SIM 으로 표시한다.
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
};
const DRINKS = { COLD_WATER: "냉수", HOT_WATER: "온수", LEMONADE: "레몬에이드", COLA: "콜라" };
// 파이 주행 대시보드(ammr_dashboard)가 구독하는 토픽과 스냅샷 안의 위치
const TOPICS = [
  ["/scan", "sensor_msgs/LaserScan", (n) => n.lidar, (v) => v.fps != null ? `${v.fps} Hz` : ""],
  ["/odom", "nav_msgs/Odometry", (n) => n.motion?.odom, (v) => vel(v)],
  ["/cmd_vel", "geometry_msgs/Twist", (n) => n.motion?.cmd, (v) => vel(v)],
  ["/cmd_vel_smoothed", "geometry_msgs/Twist", (n) => n.motion?.requested_cmd, (v) => vel(v)],
  ["/tf  map→base_link", "tf2_msgs/TFMessage", (n) => n.navigation?.localization, (v) => v.x_m != null ? `(${v.x_m.toFixed(2)}, ${v.y_m.toFixed(2)}) ${Math.round(v.yaw_deg)}°` : ""],
  ["/map", "nav_msgs/OccupancyGrid", (n) => n.navigation?.map, (v) => v.width ? `${v.width}×${v.height} @${v.resolution_m} m` : ""],
  ["/plan", "nav_msgs/Path", (n) => n.navigation?.path, (v) => v.points_xy_m ? `${v.points_xy_m.count ?? v.points_xy_m.length} pts` : ""],
  ["/navigate_to_pose/_action/status", "action_msgs/GoalStatusArray", (n) => n.navigation?.goal, (v) => v.status || ""],
  ["/collision_monitor_state", "nav2_msgs/CollisionMonitorState", (n) => n.navigation?.collision, (v) => v.action || v.state || ""],
  ["/box_parking/execution_status", "std_msgs/String (JSON)", (n) => n.navigation?.execution, (v) => v.state || ""],
  ["/battery_state", "sensor_msgs/BatteryState", (n) => n.battery, (v) => v.percentage_pct != null ? `${v.percentage_pct.toFixed(0)}% · ${v.voltage_v?.toFixed(2)} V` : ""],
  ["/imu/data_raw", "sensor_msgs/Imu", (n) => n.imu, (v) => v.angular_z_rps != null ? `ωz ${v.angular_z_rps.toFixed(2)} rad/s` : ""],
  ["/depth_navigation/status", "std_msgs/String (JSON)", (n) => n.depth_navigation, (v) => v.state || v.mode || ""],
];
const vel = (v) => v.linear_mps != null ? `v ${v.linear_mps.toFixed(2)} m/s · ω ${v.angular_rps.toFixed(2)} rad/s` : "";

const $ = (id) => document.getElementById(id);
let state = null;
let apiLatencyMs = null;

const place = (id) => !id ? "-" : id === "dock" ? "충전소" : id === "kitchen" ? "주방" : /^table_\d+$/.test(id) ? `${id.slice(6)}번 테이블` : id;
const time = (iso) => iso ? new Date(iso).toLocaleTimeString("ko-KR", { hour12: false }) : "-";
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
  return code ? `${phase} · ${code}` : failure;
}
function badge(orderState) { return el("span", STATE_LABEL[orderState] || orderState, `badge ${orderState}`); }
function setDot(id, level) { $(id).className = `dot ${level}`; }
function statusCell(level, text) { const s = el("span", text, `st ${level}`); s.prepend(el("i", undefined, `dot ${level === "off" ? "" : level}`)); return s; }

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
const payload = (o) => `${o.drink}${DRINKS[o.drink] ? ` (${DRINKS[o.drink]})` : ""}`;

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
    $("conn").textContent = `GET /api/state ${apiLatencyMs.toFixed(0)} ms · ${time(state.timestamp)}`;
    $("conn").className = "conn";
    render();
  } catch {
    $("conn").textContent = "order API 연결 끊김 — 재연결 중";
    $("conn").className = "conn bad";
    setDot("k-api-dot", "bad");
  }
}

const isSim = () => ["browser_demo", "simulation_dry_run"].includes(state.execution_backend?.mode);

// ---------- 데모·dry-run 용 시뮬레이션 값 (실제 로봇 값이 아님) ----------
function simNav() {
  const moving = /^NAVIGATE_/.test(state.phase);
  const t = performance.now() / 1000;
  const j = (base, amp) => +(base + Math.sin(t * 1.7 + base) * amp).toFixed(2);
  const fresh = (extra, age = 0.08) => ({ live: true, age_s: j(age, age / 2), ...extra });
  const v = moving ? j(0.28, 0.03) : 0;
  const w = moving ? j(0, 0.08) : 0;
  const pose = lastPose || {};
  return {
    lidar: fresh({ fps: j(10, 0.15) }),
    motion: { odom: fresh({ linear_mps: v, angular_rps: w }, 0.04), cmd: fresh({ linear_mps: v, angular_rps: w }, 0.1), requested_cmd: fresh({ linear_mps: v, angular_rps: w }, 0.1) },
    navigation: {
      localization: fresh({ x_m: pose.x_m ?? 0, y_m: pose.y_m ?? 0, yaw_deg: pose.yaw_deg ?? 0 }, 0.5),
      map: { live: true, age_s: 3.2, width: 186, height: 254, resolution_m: 0.05 },
      path: moving ? fresh({ points_xy_m: { count: 58 } }, 0.6) : { live: false, age_s: null },
      goal: { live: true, age_s: 0.4, status: moving ? "EXECUTING" : "SUCCEEDED" },
      collision: fresh({ action: "DO_NOTHING" }, 0.2),
      execution: fresh({ state: moving ? "DRIVING" : /^ALIGN_/.test(state.phase) ? "ALIGNING" : "IDLE" }, 0.3),
    },
    battery: fresh({ percentage_pct: state.battery_percent, voltage_v: 10.8 + state.battery_percent / 100 * 1.8 }, 0.7),
    imu: fresh({ angular_z_rps: w }, 0.02),
    depth_navigation: fresh({ state: "OK" }, 0.15),
    graph: { live: true, nodes: 24, topics: 63 },
    system: { live: true, cpu_percent: j(41, 6), mem_percent: j(58, 1), cpu_temp_c: j(61, 1.5) },
  };
}
function simManip() {
  const phase = state.phase;
  const running = ["GRASP_CUP", "GRASP_BOTTLE", "POUR", "RETURN_BOTTLE", "PLACE_DECK"].includes(phase) ? "pour"
    : ["REGRASP_CUP", "SERVE"].includes(phase) ? "place" : null;
  return {
    state: running ? "running" : "ready", current: running,
    // 2026-10-26 가짜 로봇 + 실제 체크포인트 시험값(상주형 조작 실행기 문서): 로드 72.6 s, 호출→첫 동작 0.4 s
    timings: { ready_total_s: 72.6, pour_infer_s: 0.41, place_infer_s: 0.39 },
    last_result: null,
  };
}

// ---------- 그리기 ----------
function render() {
  const orders = new Map(state.orders.map((o) => [o.order_id, o]));
  const tickets = new Map((state.tickets || []).map((t) => [t.order_id, t]));
  const active = state.active_order_id ? orders.get(state.active_order_id) : null;
  const eb = state.execution_backend || {};
  const mode = eb.mode || "-";
  buildDisplayNumbers(state.orders, state.tickets || []);
  const sim = isSim();

  $("demo-note").hidden = window.orderApiMode !== "demo";
  $("srv").textContent = state.server ? `pid ${state.server.pid} · uptime ${secs(state.server.uptime_s)} · backend ${mode}` : `backend ${mode}`;

  // KPI: 미션
  $("s-mission").textContent = `${state.mission_state} · ${state.phase}`;
  $("s-mission-sub").textContent = active ? `job ${no(active.order_id)} · attempt ${state.phase_attempt}` : `queue ${state.queue.length}`;
  setDot("k-mission-dot", state.phase === "TERMINAL_HOLD" ? "bad" : active ? "run" : "ok");
  $("fsm-tag").textContent = state.mission_state;
  $("fsm-tag").className = `tag ${active ? "run" : "ok"}`;

  // KPI: 조작
  const manip = eb.manipulation_service_state ? {
    state: eb.manipulation_service_state, current: eb.manipulation_service_current || eb.running_policy,
    timings: eb.manipulation_service_timings || {}, last_result: eb.manipulation_last_result,
    arms: eb.arms, drive: eb.drive,
  } : sim ? simManip() : null;
  renderManip(manip, sim && !eb.manipulation_service_state);

  // KPI: 주행
  const nav = state.robot_nav?.connected ? state.robot_nav.snapshot : sim ? simNav() : null;
  renderNav(nav, sim && !state.robot_nav?.connected);

  $("s-battery").textContent = `${state.battery_percent.toFixed(1)} %`;
  $("s-bat-sub").textContent = nav?.battery?.voltage_v ? `${nav.battery.voltage_v.toFixed(2)} V · /battery_state` : "mission model";
  setDot("k-bat-dot", state.battery_percent < 20 ? "bad" : state.battery_percent < 40 ? "warn" : "ok");
  $("s-mode").textContent = mode;
  $("s-api-sub").textContent = apiLatencyMs != null ? `RTT ${apiLatencyMs.toFixed(0)} ms · ${window.orderApiMode}` : "-";
  setDot("k-api-dot", "ok");

  // 정렬 완료 신호 대기 (rollout-service backend). 주행 연동 전에는 운영자가 확인 후 누른다.
  const waiting = eb.waiting_alignment;
  $("align-banner").hidden = !waiting;
  if (waiting) {
    const next = waiting.station === "kitchen" ? "pour 정책" : "place 정책";
    const where = waiting.station === "kitchen" ? "주방(급수대)" : "테이블";
    const armsOk = eb.drive ? eb.drive.arms_ready : true;
    const arms = eb.arms ? ` · 팔 ${ARMS_KO[eb.arms] || eb.arms}` : "";
    $("align-text").textContent = `ALIGN GATE · ${armsOk ? `차를 ${where}로 이동·정렬하세요` : "⚠ 팔이 이동 자세가 아닙니다. 차를 움직이지 마세요"}${arms}. 차가 멈추고 정렬됐는지 확인한 뒤 누르면 ${next}을 실행합니다.`;
    $("align-confirm").textContent = `정렬 완료 → ${next} 실행`;
    $("align-confirm").dataset.station = waiting.station;
  }
  $("pause").textContent = state.auto_paused ? "자동 진행 재개" : "자동 진행 일시정지";
  $("pause").className = state.auto_paused ? "btn primary" : "btn";
  $("paused-banner").hidden = !state.auto_paused;
  renderRunControls();
  $("test-tools").hidden = !sim;
  $("t-reset").hidden = mode !== "browser_demo";

  renderFsm(active, tickets);
  renderCommands();
  renderJobs(orders, tickets);
  renderEvents();
  renderStaff();
  renderPoseKv();
  drawMap(active);
}

function renderRunControls() {
  const start = state.start || {};
  $("start").disabled = !start.ready;
  const link = state.drive_link;
  $("drive-link").textContent = !link ? "" : !link.connected ? "주행 PC 미연결"
    : `주행 PC 연결 · ${link.dry_run ? "감지만 · " : ""}${link.phase || link.state || "-"}${link.error ? ` · ${link.error.slice(0, 80)}` : ""}`;
  $("drive-link").className = link && !link.connected ? "conn bad" : "conn";
  const noDrive = link && !link.connected ? " · 주행 PC 미연결: 차는 출발하지 않습니다" : "";
  $("start-hint").textContent = start.ready ? `START: 주문 ${no(start.order_id)}${noDrive}` : START_HINT[start.reason] || start.reason || "";
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
  if (active) {
    const food = itemsText(tickets.get(active.order_id));
    head.replaceChildren(
      el("b", `job ${no(active.order_id)}`),
      el("span", `target ${active.table_id} (${place(active.table_id)})`),
      el("span", `payload ${payload(active)}${food ? ` + staff: ${food}` : ""}`),
      el("span", `phase ${state.phase} (${PHASE_LABEL[state.phase] || ""}) · attempt ${state.phase_attempt}`),
    );
  } else {
    head.replaceChildren(el("span", `no active mission · ${state.phase} (${PHASE_LABEL[state.phase] || ""}) · location ${state.current_location}`, "muted"));
  }
  const durations = phaseDurations(active);
  const nowIndex = active ? SERVICE_PHASES.findIndex(([key]) => key === state.phase) : -1;
  $("phases").replaceChildren(...SERVICE_PHASES.map(([key, label, layer], i) => {
    const cls = nowIndex < 0 ? "ph" : i < nowIndex ? "ph done" : i === nowIndex ? "ph now" : "ph";
    const box = el("div", undefined, cls);
    box.title = key;
    box.append(el("em", layer === "nav" ? "NAV" : "ARM", `layer ${layer}`), el("b", key), el("span", label),
      el("i", i === nowIndex ? secs(durations.__now) : durations[key] != null ? secs(durations[key]) : "–"));
    return box;
  }));
  const last = state.last_backend_result;
  const cmd = active || state.phase === "NAVIGATE_DOCK" ? {
    phase: state.phase, attempt: state.phase_attempt, mission_id: active?.mission_id || null,
    target: active?.table_id || "dock", payload: active?.drink || null,
    last_backend_result: last || null,
  } : { phase: state.phase, last_backend_result: last || null };
  $("cmd").textContent = JSON.stringify(cmd, null, 1);
}

function renderNav(nav, simulated) {
  const tag = $("topic-tag");
  if (!nav) {
    $("s-nav").textContent = "not connected";
    $("s-nav-sub").textContent = state.robot_nav?.error || "--robot-pose-url 미설정";
    setDot("k-nav-dot", state.robot_nav ? "bad" : "");
    tag.textContent = "NO SOURCE"; tag.className = "tag bad";
    $("topics").replaceChildren(...TOPICS.map(([name, type]) => row([el("span", name, "mono"), el("span", type, "mono muted"), statusCell("off", "NO DATA"), "-", ""])));
    $("topic-note").textContent = "주행 대시보드(파이 ammr_dashboard :8090, SSH 터널)에 연결되지 않았습니다. 서버를 --robot-pose-url http://127.0.0.1:8090/api/state --robot-ssh lim@<pi> 로 띄우면 실제 토픽 상태가 표시됩니다.";
    $("graph-kv").replaceChildren();
    $("graph-tag").textContent = "-";
    return;
  }
  let liveCount = 0;
  const rows = TOPICS.map(([name, type, pick, detail]) => {
    const v = pick(nav);
    let cell;
    if (!v || v.age_s == null) cell = statusCell("off", "NO DATA");
    else if (v.live) { cell = statusCell("ok", "LIVE"); liveCount += 1; }
    else cell = statusCell("warn", "STALE");
    return row([el("span", name, "mono"), el("span", type, "mono muted"), cell, v && v.age_s != null ? secs(v.age_s) : "-", el("span", v ? detail(v) : "", "mono")]);
  });
  $("topics").replaceChildren(...rows);
  tag.textContent = `${simulated ? "DEMO · " : ""}${liveCount}/${TOPICS.length} LIVE`;
  tag.className = `tag ${liveCount >= TOPICS.length - 2 ? "ok" : liveCount ? "warn" : "bad"}`;
  $("topic-note").textContent = simulated
    ? "DEMO 값입니다(실제 로봇 미연결). 실차에서는 파이 주행 대시보드가 구독하는 같은 토픽의 수신 시각으로 LIVE/STALE을 판정합니다(2 s 기준)."
    : `source ${state.robot_nav.url} · dashboard RTT ${state.robot_nav.latency_ms} ms · ROS_DOMAIN_ID ${nav.server?.ros_domain_id ?? "-"} · ${nav.server?.rmw_implementation || "rmw default"}`;
  const odom = nav.motion?.odom || {};
  $("s-nav").textContent = `${liveCount}/${TOPICS.length} topics live`;
  $("s-nav-sub").textContent = odom.linear_mps != null ? `v ${odom.linear_mps.toFixed(2)} m/s · goal ${nav.navigation?.goal?.status || "-"}` : "-";
  setDot("k-nav-dot", liveCount >= TOPICS.length - 2 ? "ok" : "warn");
  const g = nav.graph || {};
  const sys = nav.system || {};
  const kv = [["nodes", g.nodes ?? "-"], ["topics", g.topics ?? "-"], ["dup nodes", g.duplicates ? g.duplicates.length : 0]];
  for (const [k, val] of Object.entries(sys)) if (typeof val === "number" && !["age_s"].includes(k)) kv.push([k, val.toFixed ? val.toFixed(1) : val]);
  $("graph-kv").replaceChildren(...kv.slice(0, 8).map(([k, val]) => { const d = el("div"); d.append(el("span", k), el("b", String(val))); return d; }));
  $("graph-tag").textContent = simulated ? "DEMO" : `uptime ${secs(nav.server?.uptime_s)}`;
  $("graph-tag").className = `tag ${simulated ? "warn" : "ok"}`;
}

// rollout_service /status 의 arms 값
const ARMS_KO = { stowed: "이동 자세", carrying_cup: "컵 유지·오른팔 접힘", busy: "동작 중", hold: "그 자리 유지(확인 필요)", unknown: "미확인" };

function renderManip(m, simulated) {
  const tag = $("manip-tag");
  if (!m) {
    $("s-manip").textContent = "not used";
    $("s-manip-sub").textContent = "immediate mock backend";
    setDot("k-manip-dot", "");
    tag.textContent = "MOCK"; tag.className = "tag";
    $("manip-kv").replaceChildren();
    $("manip-note").textContent = "이 backend는 조작 단계를 즉시 성공으로 처리합니다. 실제 팔은 --backend rollout-service 로 띄웁니다.";
    return;
  }
  const level = m.state === "running" ? "run" : m.state === "ready" ? "ok" : m.state === "unreachable" ? "bad" : "warn";
  $("s-manip").textContent = `${m.state}${m.current ? ` · ${m.current}` : ""}`;
  $("s-manip-sub").textContent = `${simulated ? "DEMO · " : ""}rollout_service :8770`;
  setDot("k-manip-dot", level);
  tag.textContent = `${simulated ? "DEMO · " : ""}${m.state.toUpperCase()}`;
  tag.className = `tag ${level}`;
  const t = m.timings || {};
  const kv = [
    ["policy", "SmolVLA (chassis 80)"], ["pour ckpt", "016000"], ["place ckpt", "007000"],
    ["arms", m.arms ? (ARMS_KO[m.arms] || m.arms) : "-"],
    ["drive next", m.drive ? `${m.drive.next || "-"}${m.drive.arms_ready ? "" : " (팔 대기)"}` : "-"],
    ["ready total", t.ready_total_s != null ? `${t.ready_total_s} s` : "-"],
    ["pour infer", t.pour_infer_s != null ? `${Math.round(t.pour_infer_s * 1000)} ms` : "-"],
    ["place infer", t.place_infer_s != null ? `${Math.round(t.place_infer_s * 1000)} ms` : "-"],
  ];
  $("manip-kv").replaceChildren(...kv.map(([k, v]) => { const d = el("div"); d.append(el("span", k), el("b", v)); return d; }));
  const r = m.last_result;
  $("manip-note").textContent = r
    ? `last: ${r.task} ok=${r.ok} reason=${r.reason || "-"}${r.duration_s != null ? ` ${r.duration_s}s` : ""} @ ${r.started_at || "-"}`
    : simulated ? "DEMO 값(2026-10-26 가짜 로봇 + 실제 체크포인트 시험값). 실제 실행 결과는 rollout-service backend에서 표시됩니다." : "아직 실행 결과 없음";
}

function renderCommands() {
  let cmds = state.recent_commands || [];
  if (!cmds.length && isSim()) {
    // 데모 백엔드는 명령 기록이 없어서 이벤트의 단계 완료로 대신 보여 준다.
    cmds = state.events.filter((e) => e.kind === "PHASE_SUCCEEDED" || e.kind === "MISSION_FAILED").map((e) => ({
      at: e.timestamp, phase: e.completed_phase || e.failed_phase, kind: /^NAVIGATE|^ALIGN/.test(e.completed_phase || e.failed_phase || "") ? "navigate" : "manipulate",
      name: (e.completed_phase || e.failed_phase || "").toLowerCase(), success: e.kind === "PHASE_SUCCEEDED", failure_code: e.failure || null,
      duration_s: null, adapter: state.execution_backend?.mode,
    }));
  }
  $("cmd-count").textContent = `${cmds.length}`;
  const rows = cmds.slice(-12).reverse().map((c) => row([
    el("span", time(c.at), "mono"), el("span", c.phase || "-", "mono"), el("span", `${c.kind || "-"}:${c.name || "-"}`, "mono"),
    c.success == null ? "-" : statusCell(c.success ? "ok" : "bad", c.success ? "OK" : (c.failure_code || "FAIL")),
    el("span", c.duration_s != null ? secs(c.duration_s) : "-", "mono"), el("span", c.adapter || "-", "mono muted"),
  ]));
  $("commands").replaceChildren(...(rows.length ? rows : [emptyRow("아직 실행된 명령이 없습니다.", 6)]));
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

function renderJobs(orders, tickets) {
  const open = state.orders.filter((o) => ["PENDING", "ACCEPTED", "QUEUED", "RUNNING"].includes(o.state));
  const pending = open.filter((o) => o.state === "PENDING").length;
  $("queue-count").textContent = pending ? `· 수락 대기 ${pending}건` : "";
  const queueRows = open.map((o) => {
    const food = itemsText(tickets.get(o.order_id));
    return row([el("span", no(o.order_id), "mono"), place(o.table_id), `${payload(o)}${food ? ` + ${food}` : ""}`,
      time(o.created_at), badge(o.state), orderActions(o)]);
  });
  $("queue").replaceChildren(...(queueRows.length ? queueRows : [emptyRow("들어온 주문이 없습니다.", 6)]));
  const done = state.orders.filter((o) => ["SUCCEEDED", "FAILED", "CANCELED", "REJECTED"].includes(o.state)).reverse().slice(0, 8);
  $("history").replaceChildren(...(done.length
    ? done.map((o) => row([el("span", no(o.order_id), "mono"), `${o.table_id} (${place(o.table_id)})`, payload(o), badge(o.state), time(o.completed_at), failureText(o.failure)]))
    : [emptyRow("아직 끝난 미션이 없습니다.", 6)]));
}

function renderEvents() {
  $("events").replaceChildren(...state.events.slice(-40).reverse().map((e) => {
    const phase = e.completed_phase || e.failed_phase;
    const li = el("li", undefined, /FAIL/.test(e.kind) ? "fail" : /SUCCEEDED|ARRIVED|COMPLETED/.test(e.kind) ? "ok" : "");
    li.append(el("span", time(e.timestamp), "t"), el("span", e.kind, "k"),
      document.createTextNode([no(e.order_id), phase, e.failure].filter(Boolean).join(" · ")));
    return li;
  }));
}

function renderStaff() {
  // 직원이 가져다줄 음식(로봇은 음료만 맡는다)
  const staff = (state.tickets || []).filter((t) => t.staff_state === "PREPARING");
  $("staff-count").textContent = staff.length ? `· 음식 ${staff.length}건` : "";
  $("staff").replaceChildren(...(staff.length ? staff.map((t) => {
    const served = el("button", "서빙 완료", "btn small primary");
    served.type = "button";
    served.addEventListener("click", () => act(() => api(`/api/tickets/${encodeURIComponent(t.order_id)}/served`, {})));
    return row([el("span", no(t.order_id), "mono"), place(t.table_id), itemsText(t), time(t.created_at), served]);
  }) : [emptyRow("직원이 가져다줄 음식이 없습니다.", 5)]));
  if (!$("f-table").options.length) {
    const tables = state.restaurant.tables.map((t) => t.id);
    $("f-table").replaceChildren(...tables.map((id) => { const o = el("option", place(id)); o.value = id; return o; }));
    $("links").replaceChildren(...tables.map((id) => {
      const li = document.createElement("li");
      const a = el("a", `${place(id)} 손님 화면`);
      a.href = `/order?table=${id.slice(6)}`;
      a.target = "_blank";
      li.append(a);
      return li;
    }));
  }
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
$("pause").addEventListener("click", () => act(() => api("/api/control", { paused: !state?.auto_paused })));
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
$("f-submit").addEventListener("click", () => act(async () => {
  // 주문 번호(YYMMDDNNN)는 서버가 붙인다.
  const order = { table_id: $("f-table").value, drink: $("f-drink").value, accepted: true };
  const result = await api("/api/orders", order);
  $("f-msg").textContent = `job ${result.order.order_id} · ${order.table_id} · ${order.drink} 등록`;
}));

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
function drawMap(active) {
  lastActive = active;
  const canvas = $("map");
  if (!MAP.meta || !state) return;
  const { meta, img } = MAP;
  const scale = canvas.width / meta.width;
  if (canvas.height !== Math.round(meta.height * scale)) canvas.height = Math.round(meta.height * scale);
  const ctx = canvas.getContext("2d");
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
  const px = (x, y) => [((x - meta.origin.x_m) / meta.resolution_m) * scale, (meta.height - (y - meta.origin.y_m) / meta.resolution_m) * scale];
  // 1 m 격자
  ctx.save(); ctx.strokeStyle = "rgba(22,119,200,.10)"; ctx.lineWidth = 1;
  for (let x = Math.ceil(meta.origin.x_m); x < meta.origin.x_m + meta.width * meta.resolution_m; x += 1) { const [gx] = px(x, 0); ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, canvas.height); ctx.stroke(); }
  for (let y = Math.ceil(meta.origin.y_m); y < meta.origin.y_m + meta.height * meta.resolution_m; y += 1) { const [, gy] = px(0, y); ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(canvas.width, gy); ctx.stroke(); }
  ctx.restore();
  ctx.font = "bold 11px Consolas, monospace";
  for (const s of meta.stations) {
    const [x, y] = px(s.x_m, s.y_m);
    ctx.fillStyle = STATION_COLOR[s.id] || "#7a4fd1";
    ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#1f3553";
    const right = x > canvas.width * 0.6; // 오른쪽 끝 역은 이름을 왼쪽에 써서 잘리지 않게 한다
    ctx.textAlign = right ? "right" : "left";
    ctx.fillText(STATION_FRAME[s.id] || s.id, right ? x - 9 : x + 9, y - 8);
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
function renderPoseKv() {
  const pose = lastPose;
  const label = lastPoseLabel;
  const src = $("map-src");
  src.textContent = label ? `· ${label === "LIVE" ? "LIVE" : label === "LAST" ? "LAST KNOWN" : "ESTIMATED"}` : "";
  src.className = label === "LIVE" ? "pose-live" : "pose-est";
  if (!pose) return;
  const rp = state.robot_pose;
  $("s-location").textContent = `${label === "LIVE" ? "LIVE" : label === "LAST" ? "LAST KNOWN" : "ESTIMATED"} · ${place(state.current_location)}`;
  $("s-loc-sub").textContent = `(${pose.x_m.toFixed(2)}, ${pose.y_m.toFixed(2)}) m · ${pose.yaw_deg.toFixed(0)}°`;
  setDot("k-loc-dot", label === "LIVE" ? "ok" : label === "LAST" ? "bad" : "warn");
  const kv = [
    ["x (m)", pose.x_m.toFixed(3)], ["y (m)", pose.y_m.toFixed(3)], ["yaw (°)", pose.yaw_deg.toFixed(1)],
    ["source", label === "LIVE" ? "tf map→base_link" : label === "LAST" ? "last tf" : "phase+time"],
  ];
  if (pose.goal) kv.push(["goal", STATION_FRAME[pose.goal.id] || pose.goal.id], ["remain (m)", pose.remaining_m.toFixed(2)]);
  if (label === "LIVE" && rp.age_s != null) kv.push(["tf age", secs(rp.age_s)]);
  $("pose-kv").replaceChildren(...kv.map(([k, v]) => { const d = el("div"); d.append(el("span", k), el("b", v)); return d; }));
  $("map-note").textContent = label === "LIVE" ? "" : rp ? `실시간 위치 수신 실패: ${rp.error || "-"}`
    : "실시간 위치 미연결 — 진행 단계와 경과 시간으로 추정(실차 0.3 m/s 가정). --robot-pose-url 로 띄우면 tf(map→base_link) 실위치를 그립니다.";
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
