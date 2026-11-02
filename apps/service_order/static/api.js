"use strict";

// 주문·관제 화면 공통 API 연결.
// - 기본: 화면을 내려준 서버(같은 주소)의 /api 를 쓴다.
// - ?api=https://서버주소 : 다른 주소의 주문 서버를 쓴다(서버는 --cors-origin 필요). 브라우저에 기억된다.
// - ?api=demo 또는 config.js 의 ORDER_API_BASE="demo" : 서버 없이 브라우저 안에서 흉내 내는 데모.
//   데모 상태는 이 브라우저의 IndexedDB 에만 있어 같은 브라우저의 탭끼리만 공유된다.
(() => {
  const API_KEY = "bimanual-robot.api-base";
  const query = new URLSearchParams(location.search).get("api");
  if (query !== null) {
    try { query ? localStorage.setItem(API_KEY, query) : localStorage.removeItem(API_KEY); } catch { /* 기억 못 해도 이번 접속은 동작 */ }
  }
  let stored = null;
  try { stored = localStorage.getItem(API_KEY); } catch { /* 무시 */ }
  const base = (query || stored || window.ORDER_API_BASE || "").replace(/\/$/, "");
  const demo = base === "demo";

  class ApiError extends Error {
    constructor(message, status) { super(message); this.status = status; }
  }

  async function serverApi(path, options = {}) {
    const response = await fetch(base + path, { cache: "no-store", ...options });
    const body = await response.json().catch(() => ({}));
    if (!response.ok) throw new ApiError(body.detail || body.error || response.statusText, response.status);
    return body;
  }

  // ---------- 브라우저 데모 백엔드 ----------
  const DEMO_DB = "bimanual-robot-demo";
  const STEP_MS = 2500;
  const PHASES = ["NAVIGATE_KITCHEN", "ALIGN_KITCHEN", "GRASP_CUP", "GRASP_BOTTLE", "POUR", "RETURN_BOTTLE",
    "PLACE_DECK", "NAVIGATE_TABLE", "ALIGN_TABLE", "REGRASP_CUP", "SERVE"];
  const TABLES = ["table_1", "table_2", "table_3", "table_4"];
  const DRINKS = ["COLD_WATER", "HOT_WATER", "LEMONADE", "COLA"];
  const now = () => new Date().toISOString();

  function fresh() {
    return { phase: "IDLE_AT_DOCK", phase_attempt: 0, active_order_id: null, current_location: "dock",
      battery_percent: 100, orders: [], tickets: [], events: [], auto_paused: false, sequence: 0, event_sequence: 0, last_tick: Date.now(),
      emergency: { active: false, at: null, order_id: null } };
  }
  // 손님·관제 탭이 같은 상태를 동시에 고치므로, 읽기-수정-쓰기를 IndexedDB 트랜잭션 하나로 묶어
  // 탭 사이 쓰기가 서로 덮어쓰지 않게 한다. IndexedDB 를 못 쓰면 이 탭 메모리에만 둔다.
  let dbPromise = null;
  let memoryState = null;
  function openDb() {
    if (!dbPromise) {
      dbPromise = new Promise((resolve) => {
        try {
          const request = indexedDB.open(DEMO_DB, 1);
          request.onupgradeneeded = () => request.result.createObjectStore("state");
          request.onsuccess = () => resolve(request.result);
          request.onerror = () => resolve(null);
        } catch { resolve(null); }
      });
    }
    return dbPromise;
  }
  async function withState(mutate) {
    const db = await openDb();
    if (!db) {
      memoryState = memoryState || fresh();
      return mutate(memoryState);
    }
    return new Promise((resolve, reject) => {
      const tx = db.transaction("state", "readwrite");
      const objects = tx.objectStore("state");
      let result;
      let failure = null;
      objects.get("current").onsuccess = (event) => {
        const s = event.target.result || fresh();
        try {
          result = mutate(s);
          objects.put(s, "current");
        } catch (error) {
          failure = error;
          tx.abort();
        }
      };
      tx.oncomplete = () => resolve(result);
      tx.onabort = () => reject(failure || tx.error);
    });
  }
  function record(s, kind, details = {}) {
    s.event_sequence += 1;
    s.events.push({ sequence_number: s.event_sequence, timestamp: now(), kind, phase: s.phase,
      active_order_id: s.active_order_id, battery_percent: s.battery_percent, ...details });
    s.events = s.events.slice(-200);
  }
  const queued = (s) => s.orders.filter((o) => o.state === "QUEUED")
    .sort((a, b) => b.priority - a.priority || a.sequence - b.sequence);
  const active = (s) => s.orders.find((o) => o.order_id === s.active_order_id) || null;
  const accepted = (s) => s.orders.filter((o) => o.state === "ACCEPTED")
    .sort((a, b) => b.priority - a.priority || a.sequence - b.sequence);
  function startStatus(s) {
    const next = accepted(s)[0] || null;
    const reason = s.emergency.active ? "emergency_stop" : s.active_order_id ? "mission_running"
      : !next ? "no_accepted_order" : null;
    return { ready: reason === null, reason, order_id: next ? next.order_id : null };
  }

  function startNext(s) {
    const next = queued(s)[0];
    if (next) {
      next.state = "RUNNING";
      next.mission_id = `MISSION-${next.order_id}`;
      s.active_order_id = next.order_id;
      s.phase = "NAVIGATE_KITCHEN";
      s.phase_attempt = 1;
      record(s, "MISSION_STARTED", { order_id: next.order_id });
    } else {
      s.active_order_id = null;
      s.phase = s.current_location === "dock" ? "IDLE_AT_DOCK" : "NAVIGATE_DOCK";
      s.phase_attempt = 0;
      if (s.phase === "NAVIGATE_DOCK") record(s, "RETURN_TO_DOCK_STARTED");
    }
  }
  function finish(s, order, stateName, failure) {
    order.state = stateName;
    order.completed_at = now();
    if (failure) order.failure = failure;
    startNext(s);
  }
  function advance(s, success = true, failure = null) {
    const order = active(s);
    if (s.phase === "NAVIGATE_DOCK") {
      s.current_location = "dock";
      record(s, "DOCK_ARRIVED");
      startNext(s);
      return;
    }
    if (!order) { startNext(s); return; }
    const phase = s.phase;
    if (!success) {
      record(s, "MISSION_FAILED", { order_id: order.order_id, failed_phase: phase, failure });
      finish(s, order, "FAILED", `${phase}:${failure}`);
      return;
    }
    if (phase === "NAVIGATE_KITCHEN") s.current_location = "kitchen";
    if (phase === "NAVIGATE_TABLE") s.current_location = order.table_id;
    s.battery_percent = Math.max(0, +(s.battery_percent - 0.4).toFixed(1));
    record(s, "PHASE_SUCCEEDED", { completed_phase: phase, order_id: order.order_id });
    const index = PHASES.indexOf(phase);
    if (index + 1 < PHASES.length) {
      s.phase = PHASES[index + 1];
      s.phase_attempt = 1;
    } else {
      record(s, "MISSION_SUCCEEDED", { order_id: order.order_id });
      finish(s, order, "SUCCEEDED");
    }
  }
  function tick(s) {
    const steps = Math.min(20, Math.floor((Date.now() - s.last_tick) / STEP_MS));
    if (steps > 0) s.last_tick += steps * STEP_MS;
    if (s.auto_paused || s.emergency.active) { s.last_tick = Date.now(); return; }
    for (let i = 0; i < steps && s.phase !== "IDLE_AT_DOCK"; i += 1) advance(s);
  }
  function snapshot(s) {
    return {
      timestamp: now(),
      mission_state: s.active_order_id ? "RUNNING" : s.phase === "IDLE_AT_DOCK" ? "IDLE" : "RUNNING",
      phase: s.phase, phase_attempt: s.phase_attempt, active_order_id: s.active_order_id,
      current_location: s.current_location, battery_percent: s.battery_percent,
      queue: queued(s).map((o) => o.order_id), orders: s.orders, events: s.events,
      auto_paused: s.auto_paused,
      emergency_stop: { ...s.emergency },
      start: startStatus(s),
      tickets: s.tickets,
      restaurant: { tables: TABLES.map((id) => ({ id })) },
      execution_backend: { mode: "browser_demo", hardware_accessed: false },
    };
  }

  async function demoApi(path, options = {}) {
    const payload = options.body ? JSON.parse(options.body) : {};
    return withState((s) => handle(s, path, payload));
  }

  // 서버와 같은 주문 번호: YYMMDD + 일련번호 3자리.
  function nextOrderNo(s) {
    const d = new Date();
    const p = (n) => String(n).padStart(2, "0");
    const prefix = `${String(d.getFullYear()).slice(2)}${p(d.getMonth() + 1)}${p(d.getDate())}`;
    // 예전 형식 번호(WEB-…)는 관제 화면에서 그날 비어 있는 가장 작은 번호로 보이므로 새 번호는 그 자리도 건너뛴다.
    const records = new Map([...s.orders, ...s.tickets].map((o) => [o.order_id, o.created_at]));
    const used = new Set();
    let legacy = 0;
    for (const [id, created] of records) {
      if (/^\d{9}$/.test(id)) { if (id.startsWith(prefix)) used.add(Number(id.slice(6))); }
      else if (created) {
        const c = new Date(created);
        if (`${String(c.getFullYear()).slice(2)}${p(c.getMonth() + 1)}${p(c.getDate())}` === prefix) legacy += 1;
      }
    }
    let slot = 0;
    while (legacy) { slot += 1; if (!used.has(slot)) legacy -= 1; }
    return `${prefix}${String(Math.max(slot, ...used) + 1).padStart(3, "0")}`;
  }

  function submit(s, payload) {
    const items = Array.isArray(payload.items) ? payload.items : [];
    if (payload.client_ref) {
      const ticket = s.tickets.find((t) => t.client_ref === payload.client_ref);
      if (ticket) return { order: s.orders.find((o) => o.order_id === ticket.order_id) || null, ticket };
    }
    if (!TABLES.includes(payload.table_id) || (payload.drink == null && !items.length)
      || (payload.drink != null && !DRINKS.includes(payload.drink))) throw new ApiError("invalid order", 400);
    const orderId = payload.order_id || nextOrderNo(s);
    let order = s.orders.find((o) => o.order_id === orderId) || null;
    if (payload.drink != null && !order) {
      s.sequence += 1;
      order = { order_id: orderId, table_id: payload.table_id, drink: payload.drink, created_at: now(),
        priority: 0, sequence: s.sequence, state: payload.accepted === true ? "ACCEPTED" : "PENDING", mission_id: null, failure: null, completed_at: null };
      s.orders.push(order);
      record(s, "ORDER_RECEIVED", { order_id: order.order_id, table_id: order.table_id, drink: order.drink });
    }
    let ticket = s.tickets.find((t) => t.order_id === orderId) || null;
    if (!ticket && (items.length || payload.client_ref || payload.drink == null)) {
      ticket = { order_id: orderId, table_id: payload.table_id, drink: payload.drink ?? null, items,
        total: items.reduce((sum, i) => sum + i.qty * i.price, 0),
        staff_state: items.some((i) => !i.robot) ? "PREPARING" : null,
        created_at: now(), served_at: null, client_ref: payload.client_ref || null };
      s.tickets.push(ticket);
      s.tickets = s.tickets.slice(-100);
    }
    return { order, ticket };
  }

  function handle(s, path, payload) {
    s.tickets = s.tickets || [];
    s.emergency = s.emergency || { active: false, at: null, order_id: null };
    tick(s);
    let result;
    const cancel = path.match(/^\/api\/orders\/([^/]+)\/cancel$/);
    const decide = path.match(/^\/api\/orders\/([^/]+)\/(accept|reject)$/);
    const served = path.match(/^\/api\/tickets\/([^/]+)\/served$/);
    if (path === "/api/state") {
      result = snapshot(s);
    } else if (path === "/api/orders") {
      result = submit(s, payload);
    } else if (served) {
      const ticket = s.tickets.find((t) => t.order_id === decodeURIComponent(served[1]));
      if (!ticket) throw new ApiError("unknown order", 404);
      if (!ticket.staff_state) throw new ApiError("no staff items", 400);
      ticket.staff_state = "SERVED";
      ticket.served_at = now();
      result = { ticket };
    } else if (cancel) {
      const order = s.orders.find((o) => o.order_id === decodeURIComponent(cancel[1]));
      if (!order) throw new ApiError("unknown order", 404);
      if (["QUEUED", "RUNNING"].includes(order.state)) {
        record(s, "ORDER_CANCELED", { order_id: order.order_id });
        if (order.state === "RUNNING") finish(s, order, "CANCELED");
        else { order.state = "CANCELED"; order.completed_at = now(); }
      }
      result = { order };
    } else if (decide) {
      const order = s.orders.find((o) => o.order_id === decodeURIComponent(decide[1]));
      if (!order) throw new ApiError("unknown order", 404);
      if (order.state !== "PENDING") throw new ApiError(`order is ${order.state}`, 409);
      order.state = decide[2] === "accept" ? "ACCEPTED" : "REJECTED";
      if (order.state === "REJECTED") order.completed_at = now();
      record(s, decide[2] === "accept" ? "ORDER_ACCEPTED" : "ORDER_REJECTED", { order_id: order.order_id });
      result = { order };
    } else if (path === "/api/mission/start") {
      const status = startStatus(s);
      if (!status.ready) throw new ApiError(status.reason, 409);
      const order = accepted(s)[0];
      order.state = "QUEUED";
      record(s, "ORDER_RELEASED", { order_id: order.order_id });
      startNext(s);
      s.last_tick = Date.now();
      result = { order };
    } else if (path === "/api/emergency-stop") {
      if (!s.emergency.active) s.emergency = { active: true, at: now(), order_id: s.active_order_id };
      record(s, "EMERGENCY_STOP");
      const order = active(s);
      if (order) {
        record(s, "MISSION_FAILED", { order_id: order.order_id, failed_phase: s.phase, failure: "EMERGENCY_STOP" });
        finish(s, order, "FAILED", `${s.phase}:EMERGENCY_STOP`);
      }
      result = { emergency_stop: { ...s.emergency }, order };
    } else if (path === "/api/emergency-stop/reset") {
      if (s.emergency.active) record(s, "EMERGENCY_STOP_RESET");
      s.emergency = { active: false, at: null, order_id: null };
      result = { emergency_stop: { ...s.emergency } };
    } else if (path === "/api/control") {
      s.auto_paused = payload.paused === true;
      result = { auto_paused: s.auto_paused };
    } else if (path === "/api/step") {
      advance(s, payload.success !== false, payload.failure || null);
      result = { state: snapshot(s) };
    } else {
      throw new ApiError("not found", 404);
    }
    return result;
  }

  window.orderApi = demo ? demoApi : serverApi;
  window.orderApiMode = demo ? "demo" : base ? "remote" : "local";
  window.resetDemo = () => withState((s) => { Object.assign(s, fresh()); });
})();
