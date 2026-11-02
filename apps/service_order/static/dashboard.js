(() => {
  "use strict";

  const POLL_MS = 750;
  const PHASES = [
    ["NAVIGATE_KITCHEN", "주방 이동", "충전소 또는 현재 테이블에서 주방으로 이동"],
    ["ALIGN_KITCHEN", "주방 정렬", "컵과 물병 작업 위치에 정렬"],
    ["GRASP_CUP", "컵 파지", "왼팔로 컵 중간을 파지"],
    ["GRASP_BOTTLE", "물병 파지", "오른팔로 주문 음료의 물병을 파지"],
    ["POUR", "물붓기", "컵 안으로 목표 동작 수행"],
    ["RETURN_BOTTLE", "물병 복귀", "오른팔 물병을 원래 위치에 배치"],
    ["PLACE_DECK", "컵 상판 적재", "양팔 사이 상판에 컵 배치"],
    ["NAVIGATE_TABLE", "손님 테이블 이동", "주문 테이블로 주행"],
    ["ALIGN_TABLE", "테이블 정렬", "컵을 놓을 위치에 차체 정렬"],
    ["REGRASP_CUP", "컵 재파지", "상판의 컵을 왼팔로 다시 파지"],
    ["SERVE", "서빙 완료", "손님 테이블 위에 컵 배치"],
  ];
  const PHASE_LABELS = Object.fromEntries([
    ...PHASES.map(([key, label]) => [key, label]),
    ["IDLE_AT_DOCK", "충전소 대기"], ["NAVIGATE_DOCK", "충전소 복귀"], ["CHARGING", "충전 중"],
    ["TERMINAL_HOLD", "실행 종료 · 결과 확인"],
  ]);
  const EVENT_LABELS = {
    CONTROLLER_STARTED: "관제 시작", CONTROLLER_RESTORED: "저장 상태 복구",
    ORDER_RECEIVED: "주문 접수", ORDER_ACCEPTED: "주문 수락", ORDER_REJECTED: "주문 거절", ORDER_RELEASED: "START",
    EMERGENCY_STOP: "긴급 정지", EMERGENCY_STOP_RESET: "긴급 정지 해제",
    ORDER_QUEUED: "주문 접수", ORDER_DUPLICATE_IGNORED: "중복 주문 무시", ORDER_CANCELED: "주문 취소",
    MISSION_STARTED: "미션 시작", PHASE_SUCCEEDED: "단계 완료", PHASE_RETRY: "단계 재시도",
    MISSION_SUCCEEDED: "서빙 완료", MISSION_FAILED: "미션 실패", CHARGE_REQUIRED: "충전 선행 결정",
    RETURN_REQUIRED_FOR_CHARGE: "충전 복귀 결정", RETURN_TO_DOCK_STARTED: "충전소 복귀 시작",
    DOCK_ARRIVED: "충전소 도착", CHARGE_COMPLETED: "충전 완료", SYSTEM_PHASE_FAILED: "시스템 단계 실패",
    ROUNDTRIP_TERMINAL_HOLD: "왕복 실행 종료",
  };
  const LOCATION_LABELS = {dock: "충전소", kitchen: "주방", table_1: "1번 테이블", table_2: "2번 테이블", table_3: "3번 테이블", table_4: "4번 테이블"};
  const STATE_LABELS = {PENDING: "수락 대기", ACCEPTED: "수락됨", REJECTED: "거절", QUEUED: "대기", RUNNING: "실행 중", SUCCEEDED: "완료", FAILED: "실패", CANCELED: "취소"};
  const $ = id => document.getElementById(id);
  const state = {selectedDrink: "COLD_WATER", data: null};
  const text = (id, value) => { const node = $(id); if (node) node.textContent = value ?? "—"; };
  const element = (tag, className, value) => { const node = document.createElement(tag); if (className) node.className = className; if (value != null) node.textContent = value; return node; };
  const tableNumber = id => String(id || "").replace("table_", "");
  const drinkLabel = drink => drink === "HOT_WATER" ? "온수" : "냉수";
  const timeLabel = value => value ? new Date(value).toLocaleTimeString("ko-KR", {hour12: false, hour: "2-digit", minute: "2-digit", second: "2-digit"}) : "—";

  function tag(label, tone = "") { return element("span", `tag${tone ? ` ${tone}` : ""}`, label); }

  function renderMetrics(data) {
    const stats = data.stats || {};
    text("battery", `${Number(data.battery_percent || 0).toFixed(1)}%`);
    text("location", LOCATION_LABELS[data.current_location] || data.current_location);
    text("queued-count", `${stats.queued || 0}건`);
    text("success-count", `${stats.succeeded || 0}건`);
    text("failure-count", `${stats.failed || 0}건`);
    text("event-count", `${stats.events || 0}건`);
  }

  function renderMission(data) {
    const active = data.orders.find(order => order.order_id === data.active_order_id);
    text("phase", PHASE_LABELS[data.phase] || data.phase);
    text("phase-attempt", data.phase_attempt ? `시도 ${data.phase_attempt}` : "시도 —");
    text("mission-ribbon", active ? `${active.order_id} · ${tableNumber(active.table_id)}번 · ${drinkLabel(active.drink)}` : PHASE_LABELS[data.phase] || data.phase);
    text("mission-order", active ? `${active.order_id} / ${tableNumber(active.table_id)}번 테이블 / ${drinkLabel(active.drink)}` : "활성 주문이 없습니다.");
    const phases = data.execution_backend?.mode === "ros2_planned_roundtrip"
      ? [...PHASES, ["NAVIGATE_DOCK", "충전소 복귀", "주문 성공 전 충전소 도착과 정지 확인"]]
      : PHASES;
    const activeIndex = phases.findIndex(([key]) => key === data.phase);
    const list = $("phase-list"); list.replaceChildren();
    phases.forEach(([key, label, detail], index) => {
      const item = element("li", "phase-item");
      if (activeIndex >= 0 && index < activeIndex) item.classList.add("done");
      if (key === data.phase) item.classList.add("active");
      item.append(element("strong", "", label), document.createTextNode(detail));
      list.append(item);
    });
  }

  function mapPoint(layout, point) {
    const [xmin, xmax, ymin, ymax] = layout.room_bounds_m;
    return [45 + (point[0] - xmin) / (xmax - xmin) * 760, 650 - (point[1] - ymin) / (ymax - ymin) * 600];
  }

  function svgNode(name, attrs = {}) {
    const node = document.createElementNS("http://www.w3.org/2000/svg", name);
    Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, String(value)));
    return node;
  }

  function mapLabel(group, x, y, label, sub) {
    const main = svgNode("text", {x, y, class: "map-label"}); main.textContent = label; group.append(main);
    if (sub) { const detail = svgNode("text", {x, y: y + 25, class: "map-sub"}); detail.textContent = sub; group.append(detail); }
  }

  function renderMap(data) {
    const layout = data.restaurant;
    if (!layout) return;
    const group = $("map-static"); group.replaceChildren();
    const active = data.orders.find(order => order.order_id === data.active_order_id);
    const [kx, ky] = mapPoint(layout, layout.kitchen.center_m);
    group.append(svgNode("rect", {x: kx - 40, y: ky - 100, width: 80, height: 200, rx: 7, class: "map-counter"}));
    mapLabel(group, kx, ky - 8, "주방", "냉수 · 온수");
    layout.tables.forEach(table => {
      const [x, y] = mapPoint(layout, table.center_m);
      group.append(svgNode("rect", {x: x - 58, y: y - 34, width: 116, height: 68, rx: 7, class: `map-table${active?.table_id === table.id ? " is-target" : ""}`}));
      mapLabel(group, x, y - 3, `${tableNumber(table.id)}번`, "테이블");
    });
    const [dx, dy] = mapPoint(layout, layout.waypoints.dock);
    group.append(svgNode("rect", {x: dx - 32, y: dy - 38, width: 64, height: 76, rx: 7, class: "map-station"}));
    mapLabel(group, dx, dy - 3, "충전", "도크");
    const command = data.command || {};
    const points = Array.isArray(command.path_xy_m) ? command.path_xy_m.map(point => mapPoint(layout, point).join(",")).join(" ") : "";
    $("map-path").setAttribute("points", points);
    const waypoint = layout.waypoints[data.current_location] || layout.waypoints.dock;
    const [rx, ry] = mapPoint(layout, waypoint);
    const robot = $("map-robot"); robot.replaceChildren();
    robot.append(svgNode("circle", {cx: rx, cy: ry, r: 15, class: "map-robot"}));
    const robotLabel = svgNode("text", {x: rx, y: ry - 24, class: "map-sub"}); robotLabel.textContent = "로봇"; robot.append(robotLabel);
    text("map-meta", `4개 테이블 · ${LOCATION_LABELS[data.current_location] || data.current_location}`);
    text("route-meta", command.kind === "navigate" ? `${LOCATION_LABELS[command.start] || command.start} → ${LOCATION_LABELS[command.destination] || command.destination} · ${Number(command.route_length_m || 0).toFixed(2)} m` : "주행 경로 없음");
  }

  function appendCell(row, value, className = "") {
    const cell = element("td", className); if (value instanceof Node) cell.append(value); else cell.textContent = value; row.append(cell); return cell;
  }

  function renderQueue(data) {
    const body = $("queue-rows"); body.replaceChildren(); text("queue-meta", `${data.queue.length}건`);
    if (!data.queue.length) { const row = element("tr"); const cell = appendCell(row, "대기 주문이 없습니다.", "empty-row"); cell.colSpan = 5; body.append(row); return; }
    data.queue.forEach((id, index) => {
      const order = data.orders.find(item => item.order_id === id); const row = element("tr");
      appendCell(row, String(index + 1), "num"); appendCell(row, drinkLabel(order.drink)); appendCell(row, `${tableNumber(order.table_id)}번`); appendCell(row, String(order.priority), "num");
      const button = element("button", "cancel-button", "취소"); button.type = "button"; button.dataset.cancel = order.order_id; appendCell(row, button); body.append(row);
    });
  }

  function renderHistory(data) {
    const body = $("history-rows"); body.replaceChildren();
    const orders = [...data.orders].sort((a, b) => b.sequence - a.sequence).slice(0, 100);
    if (!orders.length) { const row = element("tr"); const cell = appendCell(row, "저장된 주문이 없습니다.", "empty-row"); cell.colSpan = 5; body.append(row); return; }
    orders.forEach(order => {
      const row = element("tr"); appendCell(row, order.order_id);
      appendCell(row, tag(drinkLabel(order.drink), order.drink === "HOT_WATER" ? "hot" : "cold")); appendCell(row, `${tableNumber(order.table_id)}번`);
      const tone = order.state === "SUCCEEDED" ? "ok" : order.state === "FAILED" ? "danger" : order.state === "RUNNING" ? "warn" : "";
      appendCell(row, tag(STATE_LABELS[order.state] || order.state, tone)); appendCell(row, order.failure || timeLabel(order.completed_at)); body.append(row);
    });
  }

  function renderEvents(data) {
    const list = $("event-list"); list.replaceChildren(); const events = [...data.events].reverse().slice(0, 80);
    if (!events.length) { list.append(element("div", "empty-row", "저장된 이벤트가 없습니다.")); return; }
    events.forEach(event => {
      const row = element("div", "event-row");
      row.append(element("time", "", timeLabel(event.timestamp)), element("span", "", EVENT_LABELS[event.kind] || event.kind), element("small", "", PHASE_LABELS[event.phase] || event.phase)); list.append(row);
    });
  }

  function renderDiagnostics(data) {
    const persistence = data.persistence || {};
    const backend = data.execution_backend || {};
    const result = data.last_backend_result;
    text("storage-ribbon", persistence.enabled ? `SQLite 저장 중 · ${persistence.updated_at ? timeLabel(persistence.updated_at) : "초기화"}` : "영속 저장 꺼짐");
    text("storage-backend", persistence.enabled ? `SQLite schema v${persistence.schema_version}` : "비활성"); text("storage-path", persistence.database_path || "—");
    text("command-name", data.command?.name || "—");
    const modeLabels = {ros2_manipulation_mock: "ROS 2 조작 mock", ros2_planned_artifact_replay: "PLANNED 산출물 재생", ros2_planned_session: "PLANNED 세션", ros2_planned_roundtrip: "Isaac 물리 주행 · 충전 모델 전용", simulation_dry_run: "CPU dry-run"};
    if (backend.executor_kind === "isaac_physics") {
      modeLabels.ros2_planned_session = "Isaac 물리 실행 · 충전소/충전 mock";
    }
    text("mode-badge", modeLabels[backend.mode] || backend.mode || "모드 미확인");
    text("execution-backend", backend.active_goal ? `${backend.mode} · Action 실행 중` : backend.mode || "—");
    text("backend-result", result ? (result.success ? `성공 · ${result.adapter}` : `실패 · ${result.failure_code || result.adapter}`) : "실행 기록 없음");
    text("hardware-state", backend.hardware_accessed || data.hardware_accessed ? "접속됨" : "미접속 · dry-run");
  }

  function render(data) {
    state.data = data; $("status-ribbon").classList.add("is-live"); $("connection").className = "ribbon-value";
    text("connection", `연결됨 · ${timeLabel(data.timestamp)} 갱신`); text("orders-updated", `최근 갱신 ${timeLabel(data.timestamp)}`);
    renderMetrics(data); renderMission(data); renderMap(data); renderQueue(data); renderHistory(data); renderEvents(data); renderDiagnostics(data);
  }

  async function request(path, options = {}) {
    const response = await fetch(path, {cache: "no-store", ...options}); const payload = await response.json();
    if (!response.ok) throw new Error(payload.detail || payload.error || `HTTP ${response.status}`); return payload;
  }

  async function submitOrder(tableId) {
    const payload = {order_id: `WEB-${Date.now()}-${Math.floor(Math.random() * 1000)}`, table_id: tableId, drink: state.selectedDrink, priority: Number($("priority").value)};
    try {
      await request("/api/orders", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(payload)});
      $("notice").className = "notice success"; text("notice", `${tableNumber(tableId)}번 테이블 ${drinkLabel(state.selectedDrink)} 주문을 저장했습니다.`); await refresh(true);
    } catch (error) { $("notice").className = "notice error"; text("notice", `주문 등록 실패: ${error.message}`); }
  }

  async function cancelOrder(orderId) {
    try {
      await request(`/api/orders/${encodeURIComponent(orderId)}/cancel`, {method: "POST", headers: {"Content-Type": "application/json"}, body: "{}"});
      $("notice").className = "notice success"; text("notice", `${orderId} 주문을 취소했습니다.`); await refresh(true);
    } catch (error) { $("notice").className = "notice error"; text("notice", `주문 취소 실패: ${error.message}`); }
  }

  async function refresh(immediate = false) {
    try { render(await request("/api/state")); }
    catch (error) {
      $("status-ribbon").classList.remove("is-live"); $("connection").className = "ribbon-value danger";
      text("connection", "관제 서버 연결 끊김"); text("mission-ribbon", "상태 미수신"); text("storage-ribbon", error.message);
    } finally { if (!immediate) window.setTimeout(refresh, document.hidden ? 3000 : POLL_MS); }
  }

  document.querySelectorAll("[data-drink]").forEach(button => button.addEventListener("click", () => {
    state.selectedDrink = button.dataset.drink; document.querySelectorAll("[data-drink]").forEach(item => item.setAttribute("aria-pressed", String(item === button)));
  }));
  document.querySelectorAll("[data-table]").forEach(button => button.addEventListener("click", () => submitOrder(button.dataset.table)));
  $("queue-rows").addEventListener("click", event => { const button = event.target.closest("[data-cancel]"); if (button) cancelOrder(button.dataset.cancel); });
  window.setInterval(() => text("clock", new Date().toLocaleTimeString("ko-KR", {hour12: false})), 1000);
  text("clock", new Date().toLocaleTimeString("ko-KR", {hour12: false})); refresh();
})();
