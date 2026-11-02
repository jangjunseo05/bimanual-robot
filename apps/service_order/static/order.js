"use strict";

// 손님용 주문 화면(식당 키오스크). 주문 API(/api/orders)와 상태 API(/api/state)만 사용한다.
// 로봇 서빙 칸의 음료는 로봇이 컵에 따라 가져다주고(주문 1건에 1잔), 식사·사이드는 직원이 가져다준다.
// 주문 번호(YYMMDDNNN)는 서버가 정한다. client_ref 로 같은 주문을 다시 보내도 한 번만 접수된다.

// 메뉴 사진은 /menu/<photo>.jpg (AI 생성 이미지).
const MENU = [
  { id: "robot", tab: "로봇 서빙", live: true, title: "로봇 서빙 · 음료",
    sub: "양팔 로봇이 컵에 따라 테이블까지 가져다드려요. 로봇은 한 번에 음료 한 잔을 서빙해요.",
    items: [
      { id: "COLD_WATER", drink: "COLD_WATER", name: "냉수", desc: "시원한 물 한 잔", price: 0, photo: "cold_water" },
      { id: "HOT_WATER", drink: "HOT_WATER", name: "따뜻한 온수", desc: "따뜻한 물 한 잔", price: 0, photo: "hot_water" },
      { id: "LEMONADE", drink: "LEMONADE", name: "수제 레몬에이드", desc: "레몬청, 탄산수, 민트", price: 6000, photo: "lemonade" },
      { id: "COLA", drink: "COLA", name: "콜라", desc: "얼음 가득", price: 3000, photo: "cola" },
    ] },
  { id: "main", tab: "식사", title: "식사", sub: "주방에서 만들어 직원이 테이블로 가져다드려요.",
    items: [
      { id: "beef_bowl", name: "소고기 덮밥", desc: "간장 불고기, 반숙 계란", price: 11000, photo: "beef_bowl" },
      { id: "cream_pasta", name: "크림 파스타", desc: "베이컨, 버섯, 파르메산", price: 13000, photo: "cream_pasta" },
      { id: "steak", name: "안심 스테이크", desc: "구운 채소, 매시드 포테이토", price: 24000, photo: "steak" },
      { id: "risotto", name: "해물 리조또", desc: "새우, 홍합, 토마토 소스", price: 14000, photo: "risotto" },
    ] },
  { id: "side", tab: "사이드", title: "사이드", sub: "식사와 함께 직원이 가져다드려요.",
    items: [
      { id: "caesar_salad", name: "시저 샐러드", desc: "로메인, 크루통, 시저 드레싱", price: 8000, photo: "caesar_salad" },
      { id: "fries", name: "감자튀김", desc: "트러플 소금", price: 5000, photo: "fries" },
      { id: "ricotta_salad", name: "리코타 샐러드", desc: "제철 과일, 발사믹", price: 9000, photo: "ricotta_salad" },
    ] },
];
const ITEM_BY_ID = new Map(MENU.flatMap((s) => s.items.map((item) => [item.id, item])));
const NAME_BY_DRINK = Object.fromEntries(MENU[0].items.map((item) => [item.drink, item.name]));

const STEPS = [
  { label: "주문 접수", title: "주문이 접수됐어요", sub: "로봇이 곧 출발합니다.", art: "receipt" },
  { label: "주방 이동", title: "로봇이 주방으로 가고 있어요", sub: "잠시만 기다려 주세요.", art: "robot" },
  { label: "따르기", title: "음료를 따르고 있어요", sub: "양팔 로봇이 컵에 음료를 담는 중입니다.", art: "pour" },
  { label: "테이블 이동", title: "테이블로 가고 있어요", sub: "음료를 싣고 이동 중입니다.", art: "robot" },
  { label: "서빙", title: "곧 도착해요", sub: "로봇이 컵을 테이블 위에 놓습니다. 컵에 손대지 말아 주세요.", art: "pour" },
];
const STEP_BY_PHASE = {
  NAVIGATE_KITCHEN: 1, ALIGN_KITCHEN: 1,
  GRASP_CUP: 2, GRASP_BOTTLE: 2, POUR: 2, RETURN_BOTTLE: 2, PLACE_DECK: 2,
  NAVIGATE_TABLE: 3, ALIGN_TABLE: 3,
  REGRASP_CUP: 4, SERVE: 4,
};
const STORAGE_KEY = "bimanual-robot.order";

const $ = (id) => document.getElementById(id);
const artOf = (name) => document.getElementById(`art-${name}`).content.cloneNode(true);
const won = (n) => (n === 0 ? "무료" : `${n.toLocaleString("ko-KR")}원`);
const params = new URLSearchParams(location.search);
const fixedTable = normalizeTable(params.get("table"));
let selectedTable = fixedTable;
const cart = new Map(); // 메뉴 id -> 수량. 로봇 음료는 최대 1개(1잔).
const linkedOrder = params.get("order");
let currentOrder = linkedOrder ? { order_id: linkedOrder } : loadSaved();
let pollTimer = null;

function normalizeTable(value) {
  if (!value) return null;
  const id = /^\d+$/.test(value) ? `table_${value}` : value;
  return /^table_\d+$/.test(id) ? id : null;
}
function tableName(id) { return `${id.replace("table_", "")}번 테이블`; }

function loadSaved() {
  try { return JSON.parse(localStorage.getItem(STORAGE_KEY) || "null"); } catch { return null; }
}
function save(order) {
  try {
    if (order) localStorage.setItem(STORAGE_KEY, JSON.stringify(order));
    else localStorage.removeItem(STORAGE_KEY);
  } catch { /* 저장 불가 환경에서는 새로고침 복구만 빠진다 */ }
}

// 주소에 주문 번호를 남겨 새로고침·링크 공유 때도 같은 주문을 계속 추적한다.
function setOrderParam(orderId) {
  const url = new URL(location.href);
  if (orderId) url.searchParams.set("order", orderId);
  else url.searchParams.delete("order");
  history.replaceState(null, "", url);
}

const api = (path, options) => window.orderApi(path, options);

// ---------- 장바구니 ----------
function cartLines() {
  return [...cart].map(([id, qty]) => {
    const item = ITEM_BY_ID.get(id);
    return { id, item, qty, amount: item.price * qty };
  });
}
const cartTotal = () => cartLines().reduce((sum, line) => sum + line.amount, 0);
const robotDrink = () => cartLines().find((line) => line.item.drink)?.item.drink || null;

function addItem(id) {
  const item = ITEM_BY_ID.get(id);
  if (item.drink) {
    // 로봇은 한 번에 한 잔: 다른 음료를 고르면 바꾸고, 같은 음료를 다시 누르면 뺀다.
    const had = cart.has(id);
    for (const key of [...cart.keys()]) if (ITEM_BY_ID.get(key).drink) cart.delete(key);
    if (!had) cart.set(id, 1);
  } else {
    cart.set(id, Math.min((cart.get(id) || 0) + 1, 20));
  }
  refreshForm();
}
function changeQty(id, delta) {
  const next = (cart.get(id) || 0) + delta;
  if (next <= 0) cart.delete(id);
  else cart.set(id, Math.min(next, ITEM_BY_ID.get(id).drink ? 1 : 20));
  refreshForm();
}

function lineEl(line, editable) {
  const li = document.createElement("li");
  li.className = "line";
  const name = document.createElement("div");
  name.className = "ln";
  name.textContent = line.item.name;
  const sub = document.createElement("small");
  sub.textContent = line.item.drink ? "로봇 서빙" : "직원 서빙";
  name.append(sub);
  const price = document.createElement("div");
  price.className = "lp";
  price.textContent = won(line.amount);
  li.append(name, price);
  const qty = document.createElement("div");
  if (editable) {
    qty.className = "qty";
    const minus = document.createElement("button");
    minus.type = "button";
    minus.textContent = "−";
    minus.setAttribute("aria-label", `${line.item.name} 빼기`);
    minus.addEventListener("click", () => changeQty(line.id, -1));
    const count = document.createElement("span");
    count.textContent = String(line.qty);
    const plus = document.createElement("button");
    plus.type = "button";
    plus.textContent = "+";
    plus.setAttribute("aria-label", `${line.item.name} 더하기`);
    plus.disabled = Boolean(line.item.drink);
    plus.addEventListener("click", () => changeQty(line.id, 1));
    qty.append(minus, count, plus);
    li.append(qty);
  } else {
    name.firstChild.textContent = `${line.item.name} × ${line.qty}`;
  }
  return li;
}

// ---------- 메뉴 화면 ----------
function renderTabs() {
  $("tabs").replaceChildren(...MENU.map((section, i) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "tab";
    b.dataset.tab = section.id;
    b.setAttribute("role", "tab");
    b.setAttribute("aria-selected", String(i === 0));
    if (section.live) {
      const dot = document.createElement("span");
      dot.className = "dot";
      b.append(dot);
    }
    b.append(section.tab);
    b.addEventListener("click", () => {
      setActiveTab(section.id);
      $(`sec-${section.id}`).scrollIntoView({ behavior: "smooth", block: "start" });
    });
    return b;
  }));
}

function setActiveTab(id) {
  document.querySelectorAll("[data-tab]").forEach((b) => b.setAttribute("aria-selected", String(b.dataset.tab === id)));
}

// 스크롤 위치에 따라 탭을 맞춘다: 탭 줄 바로 아래를 지난 마지막 칸이 현재 칸이다.
function syncTabToScroll() {
  if ($("order-view").hidden) return;
  const line = $("tabs").getBoundingClientRect().bottom + 12;
  let current = MENU[0].id;
  for (const section of MENU) if ($(`sec-${section.id}`).getBoundingClientRect().top <= line) current = section.id;
  if (window.scrollY > 0 && window.innerHeight + window.scrollY >= document.documentElement.scrollHeight - 4) current = MENU[MENU.length - 1].id;
  setActiveTab(current);
}

function renderSections() {
  $("sections").replaceChildren(...MENU.map((section) => {
    const sec = document.createElement("section");
    sec.className = "sec";
    sec.id = `sec-${section.id}`;
    const title = document.createElement("h2");
    title.className = "section-title";
    title.textContent = section.title;
    const sub = document.createElement("p");
    sub.className = "section-sub";
    sub.textContent = section.sub;
    const grid = document.createElement("div");
    grid.className = "items";
    grid.append(...section.items.map(itemCard));
    sec.append(title, sub, grid);
    return sec;
  }));
}

function itemCard(item) {
  const b = document.createElement("button");
  b.type = "button";
  b.className = "item";
  b.dataset.item = item.id;
  if (item.drink) b.dataset.drink = item.drink;
  const photo = document.createElement("div");
  photo.className = "photo";
  const img = document.createElement("img");
  img.src = `/menu/${item.photo}.jpg`;
  img.alt = item.name;
  img.loading = "lazy";
  img.addEventListener("error", () => img.remove());
  photo.append(img);
  const count = document.createElement("span");
  count.className = "count";
  const name = document.createElement("div");
  name.className = "name";
  name.textContent = item.name;
  const desc = document.createElement("div");
  desc.className = "desc";
  desc.textContent = item.desc;
  const foot = document.createElement("div");
  foot.className = "foot";
  const price = document.createElement("span");
  price.className = "price";
  price.textContent = won(item.price);
  const badge = document.createElement("span");
  badge.className = `badge ${item.drink ? "robot" : "staff"}`;
  badge.textContent = item.drink ? "로봇 서빙" : "직원 서빙";
  foot.append(price, badge);
  b.append(photo, count, name, desc, foot);
  b.addEventListener("click", () => addItem(item.id));
  return b;
}

function renderTables(tables) {
  $("tables").replaceChildren(...tables.map((t) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "table-btn";
    b.dataset.table = t.id;
    b.textContent = t.id.replace("table_", "");
    b.setAttribute("aria-label", tableName(t.id));
    b.addEventListener("click", () => { selectedTable = t.id; refreshForm(); });
    return b;
  }));
  refreshForm();
}

function refreshForm() {
  document.querySelectorAll("[data-table]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.table === selectedTable)));
  document.querySelectorAll("[data-item]").forEach((b) => {
    const qty = cart.get(b.dataset.item) || 0;
    b.setAttribute("aria-pressed", String(qty > 0));
    b.querySelector(".count").textContent = qty ? (ITEM_BY_ID.get(b.dataset.item).drink ? "✓" : String(qty)) : "";
  });
  $("table-label").textContent = selectedTable ? tableName(selectedTable) : "";
  const lines = cartLines();
  $("cart-lines").replaceChildren(...lines.map((line) => lineEl(line, true)));
  const count = lines.reduce((n, line) => n + line.qty, 0);
  $("cart-what").textContent = count ? `${count}개 · ${won(cartTotal())}` : "메뉴를 골라 주세요";
  $("cart-hint").textContent = !selectedTable ? "테이블을 먼저 골라 주세요"
    : robotDrink() ? `${NAME_BY_DRINK[robotDrink()]}는 로봇이 가져다드려요` : "로봇은 한 번에 음료 한 잔을 서빙해요";
  $("submit").disabled = !(selectedTable && count);
}

function openConfirm() {
  $("confirm-sub").textContent = `${tableName(selectedTable)}로 가져다드려요.`;
  $("confirm-lines").replaceChildren(...cartLines().map((line) => lineEl(line, false)));
  $("confirm-total").textContent = won(cartTotal());
  $("confirm").hidden = false;
  $("confirm-ok").focus();
}

async function submitOrder() {
  const button = $("confirm-ok");
  button.disabled = true;
  button.textContent = "주문 보내는 중…";
  $("order-error").textContent = "";
  const items = cartLines().map((line) => ({ name: line.item.name, qty: line.qty, price: line.item.price, robot: Boolean(line.item.drink) }));
  const payload = { client_ref: `web-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`, table_id: selectedTable, items };
  if (robotDrink()) payload.drink = robotDrink();
  try {
    const result = await api("/api/orders", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload) });
    const orderId = result.ticket ? result.ticket.order_id : result.order.order_id;
    currentOrder = { order_id: orderId, table_id: selectedTable, drink: payload.drink || null, items, total: cartTotal() };
    save(currentOrder);
    setOrderParam(currentOrder.order_id);
    $("confirm").hidden = true;
    cart.clear();
    showTracking();
  } catch (error) {
    $("confirm").hidden = true;
    $("order-error").textContent = error.status === 400 ? "주문할 수 없는 테이블 또는 메뉴예요." : "주문을 보내지 못했어요. 다시 눌러 주세요.";
  } finally {
    button.disabled = false;
    button.textContent = "주문 확정";
    refreshForm();
  }
}

// ---------- 진행 화면 ----------
function showOrderForm() {
  currentOrder = null;
  save(null);
  setOrderParam(null);
  clearInterval(pollTimer);
  cart.clear();
  $("track-view").hidden = true;
  $("order-view").hidden = false;
  window.scrollTo(0, 0);
  setActiveTab(MENU[0].id);
  refreshForm();
}

function showTracking() {
  $("order-view").hidden = true;
  $("track-view").hidden = false;
  window.scrollTo(0, 0);
  renderSummary();
  poll();
  clearInterval(pollTimer);
  pollTimer = setInterval(poll, 1000);
}

async function poll() {
  try {
    const state = await api("/api/state");
    $("offline").hidden = true;
    renderStatus(state);
  } catch {
    $("offline").hidden = false;
  }
}

function summaryText(order) {
  const items = order.items || [];
  if (!items.length) return order.drink ? `${NAME_BY_DRINK[order.drink] || order.drink} 1잔` : "";
  const rest = items.reduce((n, item) => n + item.qty, 0) - 1;
  return rest > 0 ? `${items[0].name} 외 ${rest}개` : items[0].name;
}

function renderSummary() {
  $("table-label").textContent = currentOrder.table_id ? tableName(currentOrder.table_id) : "";
  $("track-no").textContent = currentOrder.order_id;
  $("track-drink").textContent = summaryText(currentOrder);
  const items = currentOrder.items || [];
  $("receipt").hidden = !items.length;
  $("track-lines").replaceChildren(...items.map((item) => lineEl(
    { item: { name: item.name, drink: item.robot }, qty: item.qty, amount: item.qty * item.price }, false)));
  $("track-total").textContent = won(currentOrder.total ?? items.reduce((s, i) => s + i.qty * i.price, 0));
}

function renderStatus(state) {
  const order = state.orders.find((o) => o.order_id === currentOrder.order_id);
  const ticket = (state.tickets || []).find((t) => t.order_id === currentOrder.order_id);
  if ((order || ticket) && !currentOrder.table_id) {
    currentOrder = {
      order_id: currentOrder.order_id, table_id: (ticket || order).table_id, drink: order ? order.drink : null,
      items: ticket ? ticket.items : [], total: ticket ? ticket.total : 0,
    };
    save(currentOrder);
    renderSummary();
  }
  const card = $("status-card");
  card.className = "card";
  $("cancel").hidden = true;
  $("again").hidden = true;
  $("steps").hidden = false;

  // 직원이 가져다주는 음식 상태
  const staffPending = Boolean(ticket && ticket.staff_state === "PREPARING");
  $("staff-note").hidden = !(ticket && ticket.staff_state);
  $("staff-note").className = `staff-note${staffPending ? "" : " served"}`;
  $("staff-note").textContent = staffPending ? "음식은 주방에서 준비해 직원이 가져다드려요." : "음식이 테이블에 나왔어요.";

  if (!order && !ticket) {
    setStatus("주문을 찾을 수 없어요", "서버가 초기화되었을 수 있어요. 다시 주문해 주세요.", -1, "fail");
    card.classList.add("fail");
    $("again").hidden = false;
    return;
  }
  let robotDone = true;
  if (!order) {
    // 로봇 음료가 없는 주문: 직원 서빙만 있다.
    $("steps").hidden = true;
    if (staffPending) setStatus("주문이 접수됐어요", "주방에서 준비해 직원이 테이블로 가져다드려요.", 0, "receipt");
    else { setStatus("음식이 나왔어요", "맛있게 드세요!", STEPS.length, "done"); card.classList.add("ok"); }
  } else if (order.state === "PENDING") {
    robotDone = false;
    setStatus("주문을 확인하고 있어요", "직원이 확인하면 곧 준비를 시작해요.", 0, STEPS[0].art);
    $("cancel").hidden = false;
  } else if (order.state === "ACCEPTED") {
    robotDone = false;
    setStatus(STEPS[0].title, STEPS[0].sub, 0, STEPS[0].art);
    $("cancel").hidden = false;
  } else if (order.state === "QUEUED") {
    robotDone = false;
    const ahead = state.queue.indexOf(order.order_id) + (state.active_order_id ? 1 : 0);
    setStatus(STEPS[0].title, ahead > 0 ? `앞에 ${ahead}건의 주문이 있어요. 순서대로 준비합니다.` : STEPS[0].sub, 0, STEPS[0].art);
    $("cancel").hidden = false;
  } else if (order.state === "RUNNING") {
    robotDone = false;
    const step = STEP_BY_PHASE[state.phase] ?? 1;
    setStatus(STEPS[step].title, STEPS[step].sub, step, STEPS[step].art);
  } else if (order.state === "SUCCEEDED") {
    setStatus(order.drink.endsWith("_WATER") ? "물이 도착했어요" : "음료가 도착했어요", "맛있게 드세요!", STEPS.length, "done");
    card.classList.add("ok");
  } else if (order.state === "CANCELED") {
    setStatus("주문이 취소됐어요", "", -1, "fail");
  } else if (order.state === "REJECTED") {
    setStatus("주문을 받을 수 없어요", "죄송해요. 직원에게 문의해 주세요.", -1, "fail");
    card.classList.add("fail");
  } else {
    setStatus("준비 중 문제가 생겼어요", "직원이 확인 후 직접 가져다드릴게요.", -1, "fail");
    card.classList.add("fail");
  }
  if (robotDone) {
    $("again").hidden = false;
    if (!staffPending) clearInterval(pollTimer);
  }
}

let heroArt = null;
function setStatus(title, sub, step, art) {
  $("status-title").textContent = title;
  $("status-sub").textContent = sub;
  if (art !== heroArt) { $("hero").replaceChildren(artOf(art)); heroArt = art; }
  $("hero").classList.toggle("live", step >= 0 && step < STEPS.length);
  $("steps").replaceChildren(...STEPS.map((s, i) => {
    const li = document.createElement("li");
    li.textContent = s.label;
    if (step >= 0 && i < step) li.className = "done";
    else if (i === step) li.className = "now";
    return li;
  }));
}

async function cancelOrder() {
  if (!confirm("주문을 취소할까요?")) return;
  try {
    await api(`/api/orders/${encodeURIComponent(currentOrder.order_id)}/cancel`, { method: "POST" });
    poll();
  } catch {
    alert("취소하지 못했어요. 이미 준비가 시작됐을 수 있어요.");
  }
}

// ---------- 시작 ----------
$("submit").addEventListener("click", openConfirm);
$("confirm-ok").addEventListener("click", submitOrder);
$("confirm-cancel").addEventListener("click", () => { $("confirm").hidden = true; });
$("confirm").addEventListener("click", (e) => { if (e.target === $("confirm")) $("confirm").hidden = true; });
document.addEventListener("keydown", (e) => { if (e.key === "Escape") $("confirm").hidden = true; });
$("cancel").addEventListener("click", cancelOrder);
$("again").addEventListener("click", showOrderForm);
window.addEventListener("scroll", syncTabToScroll, { passive: true });
if (fixedTable) $("table-section").hidden = true;
$("demo-note").hidden = window.orderApiMode !== "demo";
document.body.classList.toggle("demo", window.orderApiMode === "demo");
renderTabs();
renderSections();
refreshForm();

api("/api/state")
  .then((state) => {
    const tables = state.restaurant.tables;
    if (fixedTable && !tables.some((t) => t.id === fixedTable)) {
      selectedTable = null;
      $("table-section").hidden = false;
    }
    renderTables(tables);
  })
  .catch(() => { $("order-error").textContent = "서버에 연결할 수 없어요."; });

if (currentOrder) showTracking();
