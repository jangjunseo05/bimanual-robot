// 배포된 데모 화면 E2E 확인: 손님 메뉴(스크롤 탭) -> 주문 확인 창 -> 관제 표시 -> 일시정지(API)/한 단계 진행 -> 수락 주문(API)·취소 -> 완료 -> 손님 취소 -> 음식 주문·직원 서빙.
// 준비: 빈 폴더에서 `npm i puppeteer-core@23.11.1` 후 이 파일을 복사해 실행한다(Edge 사용, Windows 경로).
// 실행: node e2e_demo.js [사이트 주소] [스크린샷 폴더]
const os = require("os");
const puppeteer = require("puppeteer-core");
const BASE = process.argv[2] || "https://ms-water-serving.netlify.app";
const OUT = process.argv[3] || os.tmpdir();
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const text = (page, sel) => page.$eval(sel, (e) => e.textContent.trim());
const ok = (cond, msg) => { console.log(`${cond ? "PASS" : "FAIL"} ${msg}`); if (!cond) process.exitCode = 1; };

(async () => {
  const browser = await puppeteer.launch({
    executablePath: "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
    headless: true, args: ["--no-first-run"],
  });
  const guest = await browser.newPage();
  await guest.setViewport({ width: 375, height: 760, isMobile: true, hasTouch: true });
  guest.on("dialog", (d) => d.accept());
  await guest.goto(`${BASE}/control`, { waitUntil: "networkidle0" });
  await guest.evaluate(() => window.resetDemo());

  await guest.goto(`${BASE}/order?table=3`, { waitUntil: "networkidle0" });
  ok(await guest.$eval("#submit", (b) => b.disabled), "주문 버튼은 음료 선택 전 비활성");
  ok((await text(guest, "#table-label")) === "3번 테이블", "QR 테이블 번호 표시");
  ok(await guest.evaluate(() => document.documentElement.scrollWidth <= innerWidth), "375px 가로 넘침 없음");
  ok((await guest.$$("[data-item]")).length === 11 && (await guest.$$("[data-drink]")).length === 4, "메뉴 11개 모두 주문 가능, 로봇 서빙 음료 4개");
  await guest.screenshot({ path: `${OUT}/0_guest_menu.png` });
  await guest.evaluate(() => document.getElementById("sec-side").scrollIntoView({ block: "start" }));
  await sleep(1500); // 부드러운 스크롤이 끝날 때까지
  ok((await guest.$eval('[data-tab="side"]', (b) => b.getAttribute("aria-selected"))) === "true", "스크롤하면 탭이 사이드로 바뀜");
  await guest.click('[data-tab="robot"]');
  await sleep(1500);
  await guest.click('[data-drink="COLD_WATER"]');
  await guest.click("#submit");
  await guest.waitForSelector("#confirm:not([hidden])");
  await guest.click("#confirm-ok");
  await guest.waitForSelector("#track-view:not([hidden])");
  ok(/order=\d{9}(&|$)/.test(guest.url()), `주소에 9자리 주문 번호 기록 (${guest.url().split("?")[1]})`);
  await guest.screenshot({ path: `${OUT}/1_guest_tracking.png` });

  const control = await browser.newPage();
  await control.setViewport({ width: 1280, height: 900 });
  control.on("dialog", (d) => d.accept());
  await control.goto(`${BASE}/control`, { waitUntil: "networkidle0" });
  await sleep(1200);
  // 주문은 수락 → START 뒤에 시작한다(#97)
  await (await control.$$("#queue tr button"))[0].click();
  await sleep(1200);
  await control.click("#start");
  await sleep(1200);
  ok((await text(control, "#current-head")).includes("3번 테이블"), "관제 화면에 현재 주문 표시");
  // 화면 버튼은 없앴고 API는 남아 있다. 시험 도구를 안정적으로 보려고 API로 멈춘다.
  const setPaused = (paused) => control.evaluate((value) => window.orderApi("/api/control", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ paused: value }),
  }), paused);
  await setPaused(true);
  await sleep(1200);
  ok(!(await control.$eval("#paused-banner", (e) => e.hidden)), "일시정지 배너 표시");
  const phaseA = await text(control, "#current-head");
  await sleep(6000);
  ok(phaseA === (await text(control, "#current-head")), "일시정지 중 단계 멈춤");
  await control.click("#t-step");
  await sleep(1200);
  ok(phaseA !== (await text(control, "#current-head")), "시험 도구: 한 단계 진행");
  await control.screenshot({ path: `${OUT}/2_control_paused.png` });

  // 관제에 대리 주문 칸은 없다. 수락된 주문을 API로 넣어 대기열과 취소를 확인한다.
  const addOrder = (table) => control.evaluate((id) => window.orderApi("/api/orders", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ table_id: id, drink: "COLD_WATER", accepted: true }),
  }), table);
  await addOrder("table_1");
  await sleep(300);
  await addOrder("table_2");
  await sleep(1200);
  ok((await control.$$("#queue tr button")).length === 2, "수락된 주문 2건 대기열");
  await (await control.$$("#queue tr button"))[1].click();
  await sleep(1200);
  ok((await control.$$("#queue tr button")).length === 1, "관제에서 대기 주문 취소");

  await setPaused(false);
  await guest.bringToFront();
  const deadline = Date.now() + 60000;
  while (Date.now() < deadline && (await text(guest, "#status-title")) !== "물이 도착했어요") await sleep(1000);
  const done = (await text(guest, "#status-title")) === "물이 도착했어요";
  ok(done, "손님 화면 완료 표시");
  if (!done) {
    console.log("guest:", await text(guest, "#status-title"), "|", await text(guest, "#status-sub"));
    console.log("demo:", await guest.evaluate(async () => { const s = await window.orderApi("/api/state"); return JSON.stringify({ phase: s.phase, paused: s.auto_paused, active: s.active_order_id, orders: s.orders.map((o) => o.state) }); }));
    await browser.close(); return;
  }
  await guest.screenshot({ path: `${OUT}/3_guest_done.png` });

  // 새 주문 -> 수락 대기 중 손님 취소
  await guest.click("#again");
  await guest.click('[data-drink="HOT_WATER"]');
  await guest.click("#submit");
  await guest.waitForSelector("#confirm:not([hidden])");
  await guest.click("#confirm-ok");
  await guest.waitForSelector("#track-view:not([hidden])");
  await sleep(1200);
  const queuedSub = await text(guest, "#status-sub");
  ok(queuedSub.includes("직원이 확인"), `수락 대기 안내 (${queuedSub})`);
  await guest.click("#cancel");
  await sleep(1500);
  ok((await text(guest, "#status-title")) === "주문이 취소됐어요", "손님 대기 주문 취소");

  await control.bringToFront();
  await sleep(1500);
  const eventLog = await text(control, "#events");
  ok(eventLog.includes("MISSION_SUCCEEDED") && eventLog.includes("ORDER_CANCELED"), "관제 이벤트 기록에 완료·취소 표시");
  await control.screenshot({ path: `${OUT}/4_control_history.png`, fullPage: true });

  // 음식만 주문 -> 직원 서빙 목록 -> 서빙 완료 -> 손님 화면 완료
  await guest.bringToFront();
  await guest.click("#again");
  await guest.click('[data-item="steak"]');
  await guest.click('[data-item="steak"]');
  ok((await text(guest, "#cart-what")) === "2개 · 48,000원", "음식 수량·합계 표시");
  await guest.click("#submit");
  await guest.waitForSelector("#confirm:not([hidden])");
  await guest.click("#confirm-ok");
  await guest.waitForSelector("#track-view:not([hidden])");
  await sleep(1200);
  ok((await text(guest, "#status-title")) === "주문이 접수됐어요", "음식 주문 접수 표시");
  await control.bringToFront();
  await sleep(1500);
  const staffButtons = await control.$$("#staff tr button");
  ok(staffButtons.length === 1, "관제 직원 서빙 목록에 음식 주문");
  if (staffButtons.length) await staffButtons[0].click();
  await guest.bringToFront();
  await sleep(2000);
  ok((await text(guest, "#status-title")) === "음식이 나왔어요", "직원 서빙 완료가 손님 화면에 반영");
  await browser.close();
})().catch((e) => { console.error(e); process.exit(1); });
