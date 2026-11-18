# 사용자 추종 (follow)

- 로봇: **`/robot1`** (`env.bash`의 `ARTI_ROBOT1_NS`). 코스주행은 `/robot4`
- 패키지: `src/arti_follow` (주행), `src/person_tracker` (사람 인식) (인터페이스는 `src/common/arti_interfaces`)
- 일정 (노션 간트): 11/16 ~ 11/18 코딩 → 11/19 ~ 11/20 테스트
- **작업 목록: [tasks.md](tasks.md)** (노션 T01~T46)
- **visitor_follower 동작 정리: [visitor_follower.md](visitor_follower.md)** (켜면 무슨 일이 일어나는지, 상태·feedback·토픽)
- 관련 요구사항: [common/requirements.md](../common/requirements.md)
- 원본 플로우차트: 노션 `tracking_pc_ver3.drawio`

## 범위

관람객을 추적 대상으로 등록하고, 지도 위 위치를 계속 추정해서 약 0.8 m 거리를 유지하며 따라간다.
작품 반원 구역에서 5초 멈추면 해설하고, 대상을 놓치면 분실 처리, 배터리 부족·종료 구역이면 도킹한다.
상태는 중앙 서버가 아니라 **로봇 노드가 직접** 관리한다. 웹(Flask)은 다른 팀이고 ROS 2로 데이터만 주고받는다.

## 추종 주행 노드: `visitor_follower` (한 파일)

`src/arti_follow/arti_follow/visitor_follower.py`. 플로우차트 "추적 모듈"의 **움직임**만 맡는다.
팀 결정(11/16): 관람객 좌표는 **인식 모듈이 LiDAR로 구해서 `/robot1/target_person/point_map`(map 좌표)으로 준다** → 이 노드는 좌표를 받아 주행만 한다.
**localization(AMCL)이 `/robot1`로 떠 있어야 한다** (map→odom 변환).
장애물은 LiDAR(벽·기둥)와 **OAK-D Depth(바닥에 떨어진 물체)**로 보고 돌아서 간다.

| 방향 | 토픽 | 타입 | 내용 |
|---|---|---|---|
| 입력 | `target_person/point_map` | PointStamped (`frame_id: map`) | 관람객 좌표 (인식 모듈). TF로 odom 좌표로 바꿔 씀 |
| 입력 | `scan` | LaserScan | 벽·기둥 등 장애물 |
| 입력 | `oakd/stereo/image_raw/compressedDepth`, `oakd/stereo/camera_info` | | 바닥 물체 (base_link 높이 5~30 cm, 1.2 m 안) |
| 입력 | `tf`, `tf_static` | | odom, base_link, 센서 좌표계 |
| 입력 | `follow/enable` | Bool | false면 `cmd_vel`을 보내지 않음 (작품 설명 중, 도킹 등) |
| 입력 | `dock_status` | irobot_create_msgs/DockStatus | 도크에 붙어 있을 때만 undock |
| 액션 서버 | `arti/start_tour` | arti_interfaces/StartTour | 관람 시작 (goal `mode` FREE만, COURSE·관람 중이면 바로 `success=false`, `message` unsupported_mode·busy). 취소하면 관람 끝. feedback `state`(MOVING·ARRIVED·FOLLOW·EXPLAINING), `registered`(첫 인식 = 등록 완료), `current_artwork_id`, `current_index`/`total`. result `success`(취소로 끝남), `message`(끝난 이유), `visited_ids`(끝까지 설명한 작품, art_02 → 2) |
| 입력 | `artwork_zone/explaining`, `artwork_zone/explained` | String | `artwork_zone`이 알려 주는 설명 중 작품(latched) / 끝까지 설명한 작품 → feedback·result |
| 액션 클라이언트 | `undock` | irobot_create_msgs/Undock | Create 3 언독 |
| 출력 | `initialpose` | PoseWithCovarianceStamped | AMCL 초기 위치 = `arti_common/config/init_pose.yaml`의 undock 지점 |
| 출력 | `guide_message` | String | 관람객 인식 때 `recognized` → arti_audio가 "인식되었습니다…" |
| 출력 | `arti/tour_end` | Empty | 관람 끝(취소·LOST·실패)에 한 번. 로봇 1대 = 관람객 1명 → 인식 모듈(re-ID 특징·타겟)·`artwork_zone`(좌표 기록·설명한 작품)·이 노드(필터)가 그 관람객 데이터를 지운다 (개인정보) |
| 출력 | `cmd_vel` | TwistStamped | 주행 (Nav2 안 씀) |
| 출력 | `follow/state` | String | 상태가 바뀔 때마다 |

| 상태 | 언제 | 동작 (플로우차트) |
|---|---|---|
| READY | 처음 | `arti/start_tour` 액션 기다림. `cmd_vel`을 보내지 않음 |
| UNDOCK | 액션 받음 | 도크에 붙어 있으면 undock (도크 밖이면 건너뜀) |
| LOCALIZE | undock 끝 | `initialpose`를 보내고 map 위치가 출발 지점 0.3 m 안으로 잡힐 때까지 (최대 10 s) |
| START_FACE | 위치 잡힘 | 제자리 회전으로 키오스크(`artworks.yaml`의 `kiosk` 구역 중심) 쪽을 봄 (`face_zone` 파라미터) |
| WAIT_VISITOR | 다 돎 | 관람객 인식 기다림 (이전까지 들어온 좌표는 버림). 좌표가 `register_confirm_time`(1 s) 동안 끊김 없이(`register_gap` 0.5 s 안) 들어오면 등록 완료(`on_registered`): feedback `registered=true`, "인식되었습니다" 안내 → FOLLOW. 확인 중에는 따라가지 않고 바라보기만, 끊기면 다시 기다림 |
| FOLLOW | 좌표가 들어옴 | 80 cm 유지하며 이동. 0.85 m 안이면 감속해 서고(0.95 m 밖이 되면 다시 출발), 관람객 쪽으로 돎 |
| SEARCH | 좌표가 3 s 안 옴 | 최종 좌표에 도착할 때까지 이동 → 거기서 10 s 동안 한 바퀴 회전 탐색 |
| WAIT | 회전 탐색 끝 | 정지 대기 (10 s) — 상태 노드가 "로봇 쪽으로 와 주세요" 안내 |
| LOST | 대기도 지남 | 정지, 액션 끝(`reason: lost`) — 재등록은 인식 모듈·상태 노드, 다시 시작은 액션을 새로 |

SEARCH·WAIT 중 좌표가 다시 들어오면 FOLLOW. 액션을 취소하면 언제든 정지 → READY.

```bash
ros2 action send_goal /robot1/arti/start_tour arti_interfaces/action/StartTour "{mode: FREE}" --feedback   # 관람 시작 (Ctrl+C = 취소)
```

```
좌표: 칼만 필터(등속)로 위치·속도를 매끄럽게, 예측에서 너무 먼 좌표는 버림
목표: 관람객이 0.5 s 뒤 있을 곳에서 0.8 m 앞 지점
회피: LiDAR + Depth 장애물로 10 cm 격자 → 칸마다 목표까지 "돌아가는 거리"(BFS)
      → (v, w) 후보를 1.5 s 굴려 보고, 안 부딪히면서(로봇 반지름 0.2 + 여유 0.05) 끝점의 그 거리가 가장 작은 것 (DWA)
```

- 파라미터는 노드 맨 위 `PARAMS`에 설명과 함께 있다. 바꿀 때는 `-p 이름:=값`
- **테스트 실행 (한 번에)**: `ros2 launch arti_follow follow_test.launch.py`
  → 웹캠 인식(person_tracker, 카메라 + `multi_tracker_node`, 카메라는 `phone_camera.yaml`의 노트북 웹캠 0) + `visitor_follower`(실제 주행). 화면은 인식 창 하나(사람 박스 + 거리 m).
  `arti/start_tour` 액션을 보낸 뒤 인식 창에서 따라갈 사람을 클릭. AMCL은 미리 떠 있어야 한다. 옵션: `auto_start:=true`(액션 없이 바로 시작), `drive:=false`, `test:=false`(bag 안 함), `bag_dir:=`, `tracker:=camera|none`
- 테스트 저장(`test:=true`, 기본): **ros2 bag 하나만** `~/arti_bags/follow/<날짜_시각>/`에 mcap으로 녹화. 로그·CSV 파일은 따로 만들지 않는다.
  토픽은 launch의 `BAG_TOPICS`(scan, tf, odom, amcl_pose, target_person/*, cmd_vel, follow/*, guide_message 등) + `/rosout`(노드 로그 메시지).
  **카메라 영상·depth는 넣지 않는다**(개인정보, 팀 합의 전).
  다시 보기: `ros2 bag info <폴더>`, `ros2 bag play <폴더>` + RViz, 로그는 재생하면서 `ros2 topic echo /rosout`. Discovery Server가 안 떠 있으면 0개로 녹화된다
- 끊김 원인을 화면 로그로 볼 때만: `ros2 run arti_follow visitor_follower --ros-args -p diag:=true -p diag_events:=true`
- **관람 안내 실행 (추종 + 작품 설명)**: `ros2 launch arti_follow tour.launch.py`
  → `follow_test.launch.py` 전부 + `artwork_zone`(작품 구역 판정, T32) + `guide_audio_node`(arti_audio, 음성).
  옵션은 follow_test와 같고 `audio:=false`(음성 노드 따로), `player_cmd:=`, `zones_file:=` 추가.
  구역은 `arti_common/config/artworks.yaml`(map 좌표 반원: 중심·정면 방향·반지름 0.7 m).
  관람객이 작품 반원 안에서 5초간 0.6 m 안에 머무르면 `explain_artwork`, 1 m 벗어나거나 1초 끊기면 취소, 끝까지 들은 작품은 다시 안 함
  (관람이 끝나면 `arti/tour_end`로 비움). 출구·키오스크 구역은 들어가면 바로 `guide_message`(`tour_end`, `kiosk`).
  `artwork_zone`은 `follow/state`가 FOLLOW·SEARCH·WAIT(등록 뒤 추종 중)일 때만 판정한다 (그 밖에는 `off (follow ...)`, 단독 테스트는 `need_follow:=false`).
  확인: `ros2 topic echo /robot1/artwork_zone/state`
- **인식 노드만 빼고 실행**: `ros2 launch arti_follow tour.launch.py tracker:=camera` → `multi_tracker_node`만 뺌 (카메라는 띄움).
  관람객 좌표(`target_person/point_map`)는 다른 PC의 인식 노드·bag 재생·`ros2 topic pub`에서 와야 한다.
- **구역 RViz로 보기**: `ros2 launch arti_follow zone_view.launch.py` (localization의 `<ns>/map` 위에), 로봇 없이 `standalone:=true`(지도 파일로 `/zone_view` map_server).
  `zone_viz`가 `artwork_zone/markers`(파랑 = 작품, 주황 = 출구·키오스크, 화살표 = 정면) 발행. "Publish Point"로 클릭 → `<ns>/clicked_point`로 좌표 재기
- 인식 노드는 네임스페이스를 붙여 띄워야 한다. 안 붙이면 설치본에서 설정 파일을 못 찾아 `/top_cam/...`(네임스페이스 없음)을 구독 → 영상이 안 와서 인식 창이 까맣다
- 확인한 것 (가상 LiDAR, 2026-11-16): 0.25 m/s 직진 시 0.80~0.85 m 유지, 길 위 바닥 물체(10 cm) 회피,
  상자 뒤로 사라지면 SEARCH → 돌아가서 재발견, 인식이 끊기면 SEARCH(이동→회전) → WAIT → 재인식 시 FOLLOW,
  map이 odom과 어긋나 있어도(1 m, 17°) map 좌표 입력으로 동일하게 추종
- 실측 (로봇 정지, 도크 앞): depth는 1 m 너머에서 높이 잡음이 ±0.1~0.3 m → 바닥 물체는 1.2 m 안, 10 cm 칸에 점 3개 이상만 씀.
  **현장에서 앞이 빈 바닥일 때 바닥 물체 점이 안 잡히는지 먼저 확인**한다 (잡히면 `floor_min_height` 올리기, 끄려면 `use_depth:=false`)
- 플로우차트와 다른 점: `navigate_to_pose`·`goal_update`·`spin` 액션 대신 노드가 직접 `cmd_vel`로 같은 순서를 수행
- 한계: 후진하지 않는다. U자 막다른 곳은 못 빠져나올 수 있다. depth 3.2 Hz라 바닥 물체 반응이 늦을 수 있다

## 하드웨어 (SRD 기준)

- 상판 노트북 웹캠(top_cam, Victus 15: 1280×720, 화각 88°): 사람 인식·재식별
  - 이유: OAK-D는 낮아서 가까운 사람의 하반신만 보인다
- OAK-D Pro Depth + LiDAR: 거리 측정
- 웹캠 박스 → Depth 매칭은 캘리브레이션(호모그래피 + 외부 파라미터)으로 한다 → T05
- 음성 출력: 우선 **스피커**(노트북, `pw-play`)로 한다 (11/17 결정, 원래 기획은 에어팟). 작품 설명·안내 음성을 `arti_audio`의 `guide_audio_node`가 재생 → T35. 에어팟은 `player_cmd` 파라미터로 바꿀 수 있게 해 두었고, 연결 끊김 알림(`/arti/event`)은 나중에

## 노드 구성

| 노드 | 하는 일 | 입력 | 출력 | 작업 |
|---|---|---|---|---|
| `laptop_cam_node` | 노트북 웹캠 → 압축 이미지 | 웹캠 | 압축 이미지 (640×480 JPEG) | T10 |
| `visitor_perception` | 사람 추적·재식별, 등록, 손바닥 제스처 | 압축 이미지 | `VisitorDetection`, 디버그 이미지, palm / `RegisterVisitor` 액션 서버 | T12, T13, T14 |
| `visitor_localizer` | 웹캠 박스 + Depth → map 좌표, 칼만 필터 | `VisitorDetection`, Depth, camera_info, `/scan`, TF | `visitor/pose`, 마커 | T15, T16, T17 |
| `amr_controller` | 명령 실행 (UNDOCK·FACE·FOLLOW·GOTO·SEARCH·STOP·DOCK), 안전 워치독 | `AmrCommand`, `visitor/pose`, 범퍼·낭떠러지 | `AmrStatus`, Nav2 목표, `cmd_vel` | T20–T27 |
| 로봇 상태 노드 (이름 미정) | 상태 전환·판정 (등록 → 추적 → 설명 → 분실 → 종료) | `visitor/pose`(+속도), `AmrStatus`, `battery_state`, `artworks.yaml` | `AmrCommand`, `ArtiEvent`, `explain_artwork` 호출, `guide_message` / `StartSession` 서버 | T30–T34 |
| `guide_audio_node` | 작품 설명·안내 음성 재생 | `guide_message` | `ExplainArtwork` 액션 서버 | T35 |

## 인터페이스 (arti_interfaces에 추가할 것)

| 이름 | 종류 | 내용 | 작업 |
|---|---|---|---|
| `AmrCommand` / `AmrStatus` | msg | 로봇 명령(이동·정지·도킹 등) / 실행 결과·도착 여부·에러 코드 | T20 |
| `StartSession` | srv | 웹 버튼 → 로봇이 관람 세션 시작, 결과 응답 | T30 |
| `ArtiEvent` | msg | 등록 성공·분실·배터리 부족·관람 종료(사유) 등 상태 변화 | T30 |
| `RegisterVisitor` | action | 관람객 등록 (진행률·취소·타임아웃) | T13 |
| `VisitorDetection` | msg | 찾음 여부·ID·bbox | T12 |
| `ExplainArtwork` | action | 작품 설명 재생 (진행률·취소·완료) | T35 |

## 다른 팀과 주고받는 것

| 이름 | 종류 | 방향 | 상대 | 내용 |
|---|---|---|---|---|
| `/arti/start_session` | srv (`StartSession`) | 받음 | 웹(Flask) | 관람 등록 요청 |
| `/arti/event` | topic (`ArtiEvent`) | 보냄 | 웹(Flask) | 등록 성공·분실·배터리 부족·관람 종료 |
| 로봇 상태 (이름 미정) | topic | 보냄 | 웹(Flask) | 배터리·현재 상태 주기 발행 |
| 상대 로봇 위치 (미정) | topic | 받음 | 코스주행 | 경로 겹침 대응 (T28) |

## 로봇 내부 토픽·액션 (네임스페이스 아래)

노션 작업 목록의 `/robot2/...`는 모두 `/robot1/...`로 읽는다. 코드에서는 상대 이름(`visitor/pose`)으로 쓰고 launch에서 네임스페이스를 붙인다.

`visitor/pose`, `goal_update`, `explain_artwork`, `guide_message`, `spin`, `navigate_to_pose`

## 설정 파일 (공용 좌표는 `arti_common/config/`)

| 파일 | 내용 | 작업 |
|---|---|---|
| `artworks.yaml` | 작품별 좌표·정면 방향·반원 반경·음성 파일, 종료 구역 | T04 |
| `init_pose.yaml` | 로봇별 초기 위치·도킹 앞 좌표 | T06 |
| `calib.yaml`, top_cam camera_info | 웹캠↔Depth 호모그래피, 웹캠 내부 파라미터 | T05 |

## 파라미터 (기본값은 노션 작업 목록 기준)

| 이름 | 기본값 | 설명 |
|---|---|---|
| 추종 거리 | 0.8 m | 더 가까우면 정지 |
| 목표 갱신 거리 / 주기 | 0.3 m / 초당 2회 이하 | 로봇 떨림 방지 |
| 최대 속도 | 0.3 m/s | 예시 값 |
| 정지 판정 | 5 s, 반경 0.6 m | `is_dwelling` |
| 설명 중 이탈 | 1 m | 넘으면 설명 취소 |
| 인식 끊김 → 분실 | 1 s | 이하면 마지막 목표 유지 |
| 유효 거리 | 0.3–4 m | Depth 값 필터 |
| 등록 | 30 프레임, 10 s 타임아웃, 3회 재시도 | T13, T31 |
| 분실 처리 | 탐색 10 s, 대기 10 s, 재등록 20 s | T33 |
| 배터리 종료 기준 | 20% | 예시 값 |
| FACE 허용 오차 | ±5° | T23 |

## ⚠️ 확인 필요

- **탑CAM:** 11/15 메모는 "휴대폰", SRD·작업 목록은 "노트북 웹캠" → 작업 목록(노트북) 기준으로 정리함.
- **T18 bag 녹화:** 요구사항상 관람객 영상은 저장하지 않는다 → 테스트용 녹화 범위를 팀에서 합의.

## 참고

- 기존 실습 코드: `~/rokey_ws/src/yolo_follow` (YOLO → Depth → TF(map) → Nav2 목표), `~/rokey_ws/src/day3/depth_to_3d*`
- 트러블슈팅: [../troubleshooting/](../troubleshooting/README.md) 의 `follow/` 폴더
