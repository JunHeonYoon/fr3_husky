# FR3 Action Server TODO Framework

`fr3_action_controller` + `ActionServerBase` 구조에서 헷갈리는 지점은 대부분 **스레드 경계**입니다.
이 문서는 기존 구조를 유지하면서 TODO를 어디에 작성해야 하는지 빠르게 판단하기 위한 프레임워크입니다.

## 1) 한 줄 원칙

- `handle_*` 계열: **Action server callback thread**에서 실행 (요청/취소/수락 처리, 신호 전송).
- `onActivated`, `compute`, `onDeactivated`: **controller update thread**에서 실행 (실제 제어 로직/명령 출력).

즉,
- 서버 콜백은 “요청 접수/검증 + 컨트롤러에게 신호 전달”
- 컨트롤러 루프는 “로봇 제어 ownership을 가진 상태에서 실제 계산/출력”

## 2) 실행 흐름 (현재 코드 기준)

1. 액션 goal 도착 → `handle_goal()`에서 수락 여부 결정.
2. goal 수락됨 → `handle_accepted()`에서 goal 저장 + `requestActivate()`.
3. `FR3ActionController::update()`가 `consumeActivateRequest()`를 확인.
4. owner 선정 후 `onActivated()` 1회 호출.
5. owner 유지 동안 매 tick `compute()` 호출.
6. cancel 요청 또는 goal 비활성 상태가 되면 `onDeactivated()` 호출 후 owner 해제.

## 3) TODO 배치 규칙 (권장)

### A. `handle_goal()` (SERVER CALLBACK THREAD)
- 해야 할 일
  - goal 파라미터 범위/유효성 검사.
  - ACCEPT/REJECT 결정.
- 하지 말아야 할 일
  - 무거운 model 계산, command write.

### B. `handle_accepted()` (SERVER CALLBACK THREAD)
- 해야 할 일
  - `active_goal_` 저장.
  - `compute()`가 필요로 하는 **작고 불변**인 goal 파라미터 캐시.
  - `requestActivate()` 호출.
- 하지 말아야 할 일
  - 모션 계산 루프 실행.

### C. `handle_cancel()` (SERVER CALLBACK THREAD)
- 해야 할 일
  - `requestCancel()` 호출.
- 하지 말아야 할 일
  - 직접 goal 종료/명령 정지 처리 (종료 처리는 `onDeactivated()`에서 일원화).

### D. `onActivated()` (CONTROLLER THREAD)
- 해야 할 일
  - owner 획득 시점의 one-shot 초기화.
  - 현재 state 기준 목표/타이머 초기화.

### E. `compute()` (CONTROLLER THREAD)
- 해야 할 일
  - `model_updater_`에서 state 읽기.
  - 명령 생성 후 `model_updater_`를 통해 write.
  - 필요시 feedback publish.

### F. `onDeactivated()` (CONTROLLER THREAD)
- 해야 할 일
  - goal 종료 상태 정리(succeed/abort/canceled 중 정책 선택).
  - `active_goal_` 정리.
- 권장
  - owner release 시 종료 정책을 여기서만 관리.

## 4) 구현 체크리스트

- [ ] Goal validation 정책이 `handle_goal()`에만 있는가?
- [ ] goal 파라미터 캐시가 `handle_accepted()`에 모여 있는가?
- [ ] 제어 계산과 command write가 `compute()`에만 있는가?
- [ ] goal 종료 로직이 `onDeactivated()`에 일관되게 있는가?
- [ ] cancel은 `requestCancel()` 신호만 보내고, 정리/종료는 controller thread에서 하는가?

## 5) 코드 작성 템플릿 (요약)

- `handle_goal`: 파라미터 검사 후 ACCEPT/REJECT
- `handle_accepted`: goal 캐시 + `requestActivate()`
- `compute`: 상태 읽기 → 제어 계산 → 명령 출력 → feedback
- `onDeactivated`: result 마무리 + goal reset

이 템플릿만 지켜도 “서버 콜백 vs 컨트롤러 루프” 역할 충돌을 크게 줄일 수 있습니다.
