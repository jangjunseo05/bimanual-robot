# 화면과 서버

| 경로 | 용도 | 실행 |
|---|---|---|
| service_order/ | 주문 API·주문/미션 화면 | `python3 -m apps.service_order.service_order_server --backend immediate --port 8765` |
| service_order/ `/order` | 손님용 주문 화면(테이블·음료 선택, 진행 상황 확인, 대기 중 취소) | 위 서버 실행 뒤 `http://<주소>:8765/order?table=3` |
| service_order/ `/control` | 운영자용 관제 화면(주문 수락·거절, START·STOP, 로봇 상태, 진행 단계, 일시정지, 대리 주문) | 위 서버 실행 뒤 `http://<주소>:8765/control` |
| camera_dashboard/ | RGB 카메라 관제·요청에 따른 정지 | `python3 -m apps.camera_dashboard.camera_dashboard --help` |
| project_site/ | 프로젝트 소개 소스 | 해당 폴더에서 `npm ci`, `npm run dev` |
| ../src/navigation/dashboard/ | Pi 주행 telemetry 대시보드 | [배포 안내](../src/navigation/deployment/README.md) |

Python 명령은 저장소 루트에서 실행한다.

```bash
cd "$HOME/bimanual-robot"
python3 -m apps.service_order.service_order_server --help
python3 -m apps.camera_dashboard.camera_dashboard --help
```

카메라·서보 옵션은 각 도움말과 [실물 실행 문서](../docs/guides/20260929_IK_픽앤플레이스_실행전_준비.md)를 따른다.
CPU immediate 주문 모드는 하드웨어를 움직이지 않는다. 주문 상태·저장·backend는 src/mission/에 두며 복제하지 않는다.
프로젝트 소개 사이트는 로봇 주행 대시보드와 별개의 화면이다.

## 주문 웹 두 화면

같은 서버 하나가 두 화면을 함께 제공한다. 상세 관제 화면(`/`)은 지도·진단까지 보여 주는 개발용이다.

- 손님용 `/order?table=N`: 테이블 QR에 이 주소를 넣으면 테이블 선택을 건너뛴다. 주문 뒤 주소에
  `order=<주문 번호>`가 붙어 새로고침해도 같은 주문을 계속 보여 준다. 취소는 대기 중일 때만 가능하다.
- 운영자용 `/control`: 들어온 주문을 수락·거절하고, 수락한 주문을 START로 시작하며, STOP으로 긴급 정지한다.
  직원 대리 주문은 수락된 상태로 들어간다. 자동 진행 일시정지·재개(`POST /api/control {"paused": bool}`),
  최근 주문·실패 원인, 이벤트 기록도 본다. dry-run 모드에서는 한 단계 진행·실패 주입 시험 도구가 보인다.
- 주문 상태: `PENDING`(수락 대기) → `ACCEPTED`(수락됨) → START → `QUEUED`·`RUNNING`, 거절은 `REJECTED`.
  주문이 들어와도 START 전에는 팔이 움직이지 않는다. 예전처럼 주문 즉시 시작하려면 서버를 `--auto-dispatch`로 띄운다.
- START는 수락한 주문 중 가장 오래된 것을 시작한다(`POST /api/mission/start`). 진행 중 미션, 긴급 정지,
  조작 실행기 미준비 상태에서는 거절하며 이유는 `/api/state`의 `start`에 있다.
- STOP(`POST /api/emergency-stop`)은 잠금을 먼저 걸고 조작 실행기에 `/stop`을 보낸 뒤 진행 주문을 `FAILED`로 끝낸다.
  잠금은 [긴급 정지 해제](`POST /api/emergency-stop/reset`) 전까지 다음 단계와 START를 막는다.
  상태는 `/api/state`의 `emergency_stop`이다. 차량 정지는 주행 PC 연동에 달려 있다. 물리 비상 정지 장치를 대신하지 않는다.
- 주행 PC가 `--follow-web`으로 붙으면 START 뒤 자동 출발하고, 연결 상태가 제목 아래에 표시된다([사용법](../tools/jdamr/README.md)).
- 일시정지는 다음 단계를 시작하지 않을 뿐이며, 진행 중인 단계는 끝까지 수행한다. 비상 정지 장치가 아니다.
- 다른 기기(휴대폰)에서 접속하려면 `--host 0.0.0.0`으로 실행한다. 인증이 없으므로 같은 내부망에서만 쓴다.
- 실물 팔 연동: `--backend rollout-service`는 조작 단계를 상주형 조작 실행기(`tools/rollout/rollout_service.py`)로 보낸다. 주방·테이블 정렬 단계에서 관제 화면의 [정렬 완료] 버튼(`POST /api/align/confirm`)을 기다린다. 한 번에 켜는 방법은 `tools/rollout/start_scenario.bat`, 상세는 [상주형 조작 실행기 문서](../docs/guides/20261026_상주형_조작실행기_사용법.md) 6절이다. 주문이 시작되면 출발 전에 양팔을 이동 자세(`place_end`)로 접고, `/api/state`의 `execution_backend.drive`로 다음 목적지와 출발 가능 여부를 알린다. 시연 절차는 [최종 시연 운영 절차](../docs/guides/20261029_최종시연_운영절차.md)다.

### Netlify 배포

Netlify에는 화면 파일만 올라간다. 주문 서버(파이썬)는 올라가지 않으므로 두 방식 중 하나로 쓴다.

1. **데모(기본)**: 서버 없이 브라우저 안에서 주문 흐름을 흉내 낸다. 화면에 "체험용 데모" 표시가 뜨고,
   주문은 같은 브라우저의 탭끼리만 공유된다(휴대폰 주문이 노트북 관제 화면에 보이지 않는다).
2. **실서버 연결**: 노트북에서 주문 서버를 `--cors-origin https://<사이트>.netlify.app`으로 실행하고
   HTTPS 주소(예: 터널)로 열어 둔 뒤, 화면 주소에 `?api=https://<서버 주소>`를 붙인다. 한 번 붙이면 그 브라우저가
   기억하며 `?api=`(빈 값)로 지운다. Netlify 화면은 HTTPS라 `http://` 서버 주소는 브라우저가 막는다.

```bash
python3 -m apps.service_order.build_netlify --out /tmp/netlify_order          # 데모 묶음
npx netlify-cli deploy --dir /tmp/netlify_order --prod                          # 로그인 필요
```

`/`는 손님 주문 화면(`/order`)으로 넘어가고, 관제 화면은 `/control`이다.

공개 데모: https://ms-water-serving.netlify.app (손님 `/order`, 관제 `/control`). 상세는 [주문 웹 문서](../docs/guides/20261026_주문웹_손님용_관제용_Netlify배포.md).
