# visitor_follower 실행하면 일어나는 일

`src/arti_follow/arti_follow/visitor_follower.py` 한 파일. 로봇 1대(`/robot1`)가 관람객 1명을 맡아
**관람 시작 → 등록 → 추종 → 분실 처리 → 관람 끝**까지 움직임을 담당한다.
작품 설명 판단은 `artwork_zone`, 음성은 `arti_audio`의 `guide_audio_node`, 사람 인식은 `person_tracker`가 한다.

## 1. 켜자마자

```
노드 시작
  ├ init_pose.yaml   → 출발 지점 (x, y, yaw)        (robots.<네임스페이스>.undock)
  ├ artworks.yaml    → 바라볼 곳 (kiosk 구역 중심), 작품 수 (feedback total)
  ├ 구독·발행 준비, arti/start_tour 액션 서버 열기
  └ 상태 READY  → cmd_vel을 보내지 않고 액션을 기다림
```

`auto_start:=true`면 액션 없이 1초 뒤 바로 관람을 시작한다 (테스트용).

## 2. 키오스크가 `start_tour`를 보내면

```
goal {mode: FREE}
  ├ 이미 관람 중      → 바로 끝냄: success=false, message=busy (진행 중 관람은 그대로)
  ├ mode가 COURSE 등  → 바로 끝냄: success=false, message=unsupported_mode
  └ FREE 또는 ""      → 관람 시작
```

## 3. 관람 흐름 (상태)

```
READY
  │ start_tour
  ▼
UNDOCK        도크에 붙어 있으면 Create 3 undock 액션 (로봇이 스스로 움직임). 도크 밖이면 건너뜀
  │             실패 → 관람 끝 (undock_failed / no_undock_server)
  ▼
LOCALIZE      AMCL에 initialpose(출발 지점)를 보내고 map 위치가 근처(0.3 m)로 잡힐 때까지 기다림
  │             10 s 안에 map 위치가 아예 없으면 → 관람 끝 (no_localization)
  ▼
START_FACE    제자리 회전으로 키오스크 쪽을 봄 (15 s 안에 못 돌면 그냥 넘어감)
  ▼
WAIT_VISITOR  관람객 인식 기다림. 이 전까지 들어온 좌표는 버림
  │             좌표가 1 s 동안 끊김 없이(간격 0.5 s 이하) 들어오면 → 등록 완료
  │             확인 중에는 따라가지 않고 바라보기만. 끊기면 처음부터 다시
  ▼ on_registered(): registered=true, guide_message "recognized" ("인식되었습니다")
FOLLOW        관람객과 0.8 m 거리 유지하며 이동
  │ 좌표가 3 s(lost_timeout) 안 옴
  ▼
SEARCH        마지막 좌표까지 이동 → 거기서 10 s 동안 한 바퀴 회전 탐색
  ▼
WAIT          정지 대기 10 s
  ▼
LOST          관람 끝 (lost)

SEARCH·WAIT 중 좌표가 다시 오면 → FOLLOW
액션 취소(cancel)는 언제든 → 정지 → READY (canceled)
```

관람이 끝나면(취소·LOST·실패) 정지하고 `arti/tour_end`를 한 번 보낸다.
인식 모듈·`artwork_zone`·이 노드가 그 관람객 데이터(re-ID 특징, 좌표 기록, 필터)를 지운다 (로봇 1대 = 관람객 1명, 개인정보).

## 4. FOLLOW에서 움직이는 방식

```
좌표   칼만 필터(등속)로 위치·속도를 매끄럽게. 예측과 너무 다른 좌표는 버림 (5번 연속이면 다시 시작)
목표   관람객이 0.5 s 뒤 있을 곳에서 0.8 m 앞 지점
멈춤   0.85 m 안이면 감속해 서서 관람객 쪽으로 돎, 0.95 m 밖이 되면 다시 출발 (섰다 갔다 방지)
회피   LiDAR(벽·기둥) + OAK-D depth(바닥 물체 5~30 cm, 1.2 m 안)로 10 cm 격자
       → 칸마다 목표까지 돌아가는 거리(BFS) → (v, w) 후보를 1.5 s 굴려 보고 안 부딪히며 가장 가까워지는 것 (DWA)
속도   최대 0.3 m/s, 1.5 rad/s. 후진하지 않음
안전   TF(odom→base_link)가 없거나 scan이 0.5 s 넘게 안 오면 정지
```

`follow/enable`이 false면 계산은 하지만 `cmd_vel`을 보내지 않는다 (다른 노드에 주행을 넘길 때).

## 5. 키오스크가 받는 것 (`start_tour` feedback, 0.2 s마다)

| 내부 상태 | feedback `state` | `registered` |
|---|---|---|
| READY·UNDOCK·LOCALIZE·START_FACE | MOVING | false |
| WAIT_VISITOR | ARRIVED | false |
| FOLLOW·SEARCH·WAIT | FOLLOW | true |
| 위 상태에서 작품 설명 중 | EXPLAINING | true |

- `current_artwork_id`: 설명 중인 작품 (art_02 → 2, 없으면 0)
- `current_index` / `total`: 끝까지 설명한 작품 수 / artworks.yaml 작품 수
- result: `success`(취소로 끝나면 true), `message`(끝난 이유), `visited_ids`(끝까지 설명한 작품, 순서대로)

**등록 성공 보고 = `registered`가 처음 true가 되는 feedback.**

## 6. 주고받는 토픽·액션 (`/robot1` 아래)

| 방향 | 이름 | 타입 | 상대 |
|---|---|---|---|
| 액션 서버 | `arti/start_tour` | arti_interfaces/StartTour | 키오스크·관제 |
| 입력 | `target_person/point_map` | PointStamped (map) | person_tracker |
| 입력 | `scan` | LaserScan | 로봇 LiDAR |
| 입력 | `oakd/stereo/image_raw/compressedDepth`, `oakd/stereo/camera_info` | | 로봇 OAK-D |
| 입력 | `tf`, `tf_static` | | 로봇, AMCL |
| 입력 | `dock_status` | DockStatus | 로봇 |
| 입력 | `follow/enable` | Bool | (필요할 때) |
| 입력 | `artwork_zone/explaining`, `artwork_zone/explained` | String | artwork_zone |
| 액션 클라이언트 | `undock` | irobot_create_msgs/Undock | 로봇 |
| 출력 | `cmd_vel` | TwistStamped | 로봇 |
| 출력 | `initialpose` | PoseWithCovarianceStamped | AMCL |
| 출력 | `follow/state` | String (latched, 바뀔 때만) | artwork_zone, 확인용 |
| 출력 | `guide_message` | String | guide_audio_node |
| 출력 | `arti/tour_end` | Empty | person_tracker, artwork_zone |

## 7. 실행과 확인 (하나씩)

전체 관람 (인식 + 추종 + 작품 구역 + 음성). AMCL은 미리 떠 있어야 한다.
```bash
ros2 launch arti_follow tour.launch.py
```

관람 시작 (**로봇이 undock하고 회전하니 주변이 안전할 때**). Ctrl+C = 취소
```bash
ros2 action send_goal --feedback /robot1/arti/start_tour arti_interfaces/action/StartTour "{mode: FREE}"
```

상태 보기
```bash
ros2 topic echo /robot1/follow/state
```

## 8. 자주 바꾸는 파라미터 (`-p 이름:=값`, 전체는 노드 맨 위 `PARAMS`)

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `target_distance` | 0.8 | 유지 거리 [m] |
| `register_confirm_time` / `register_gap` | 1.0 / 0.5 | 등록 확인 시간 / 끊김으로 보는 간격 [s] |
| `lost_timeout` | 3.0 | 좌표가 이만큼 안 오면 SEARCH [s] |
| `search_time` / `wait_time` | 10 / 10 | 회전 탐색 / 정지 대기 [s] |
| `max_linear` / `max_angular` | 0.3 / 1.5 | 최대 속도 [m/s] / [rad/s] |
| `face_zone` | kiosk | 시작 때 바라볼 구역 |
| `undock` | true | 시작 때 undock |
| `use_depth` | true | 바닥 물체 회피 (빈 바닥에서 점이 잡히면 `floor_min_height` 올리기) |
| `auto_start` | false | 액션 없이 바로 시작 (테스트) |

## 한계

- 후진하지 않는다. U자 막다른 곳은 못 빠져나올 수 있다
- depth가 3.2 Hz라 바닥 물체 반응이 늦을 수 있다
- 등록은 "1 s 동안 계속 보임"까지만 본다. 특징 수집·한 명 확인(T13)은 아직 없다
- 관람 끝에 도크로 돌아가는 기능(RETURNING)은 아직 없다
