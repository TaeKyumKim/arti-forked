# arti

TurtleBot 4 두 대로 갤러리 관람객을 안내하는 큐레이터 로봇 (ROKEY 10기 지능1 팀 프로젝트). 기능 개요는 `README.md`, 개발용 요구사항(수치·제약·기능→패키지 매핑)은 `docs/common/requirements.md` 참고. 원본 기획은 팀 노션.

## 진행 방식

- 사용자는 로보틱스를 배우는 중이다. 데이터가 어떻게 오가는지 이해하는 것이 목표다.
- 명령어는 한 번에 하나씩, 짧은 설명과 함께. 새 토픽/서비스/액션은 `ros2 topic info/echo`, `ros2 interface show`, `rqt_graph`로 직접 확인하게 한다.
- 팀 저장소다. `main`에 직접 push하지 않고 브랜치 → PR로 올린다. push/PR은 사용자 확인 후에.
- 로봇(TurtleBot 4) SSH 접속은 사용자가 그 작업에 명시적으로 허락할 때만. 이동 명령은 보내지 않는다.
- OAK-D RGB는 raw 말고 `/compressed` 토픽 사용. 사람 인식용 탑CAM은 로봇 위 노트북 웹캠(`video_source:=0`, 기본값). 휴대폰 DroidCam URL로도 바꿀 수 있다.
- 관람객 영상은 저장하지 않는다(개인정보 요구사항). 카메라 토픽 rosbag 녹화는 팀 합의 후에만.
- 로봇·지도·작품 위치·관람 구역은 코드에 하드코딩하지 말고 YAML 파라미터로 둔다(확장성 요구사항).
- 답변은 한국어, 수식은 LaTeX 없이 평문/코드 블록.

## 트러블슈팅 기록

- 에러를 해결하면 (또는 해결하지 못해도) `docs/troubleshooting/<폴더>/YYYY-MM-DD-slug.md`에 남긴다. 양식은 `TEMPLATE.md`, 폴더 기준과 목록은 `docs/troubleshooting/README.md`.
- 기록을 추가하면 README 목록에도 한 줄을 추가한다.

## 환경

- Ubuntu 24.04, ROS 2 Jazzy. `~/turtlebot4_ws`는 `.bashrc`에서 이미 source됨.
- 팀 공통 `ROS_DOMAIN_ID=1` (로봇 discovery 설정과 같음). 로봇 네임스페이스: `/robot1`(follow), `/robot4`(코스주행) (`env.bash`의 `ARTI_ROBOT1_NS`, `ARTI_ROBOT2_NS`).
- 저장소 루트(`~/project/arti`) = colcon 워크스페이스 루트. `pb`로 빌드.
- Python venv: `~/venvs/arti_venv` (`scripts/make_venv.sh`, `--system-site-packages`). pip 의존성은 `requirements.txt`에 버전 고정. pytest·colcon은 pip로 설치하지 않는다(ROS와 충돌).
- `.envrc`(direnv, 커밋됨)가 들어올 때 `env.bash` → arti venv → `install/setup.bash`를 자동 로드한다. `ws arti`도 같은 효과. 팀원 세팅 절차는 `docs/common/setup.md`, 주행 띄우기는 `docs/common/navigation.md`.
- ROS 네트워크(RMW, Discovery Server)는 노션 CASE 1-3: **관제 PC(host, discovery server ID 0)에 로봇 PC 2대(guest, 각자 로봇 1대)가 붙는** 구성. `scripts/ros_network.bash`가 `env.bash`의 `ARTI_PC_ROLE`(host/guest), `ARTI_HOST_PC_IP`, `ARTI_DISCOVERY_SERVERS`(자기 로봇만 "서버ID:IP")로 만든다. 서버 ID 0 = `192.168.107.42`(관제 PC, `scripts/run_discovery_server.sh`를 켜 둠). 이 PC(`192.168.107.11`, mu-01, robot1 담당)는 guest → `192.168.107.42:11811;192.168.107.101:11811;` (`/etc/turtlebot4_discovery/setup.bash`와 같은 값). 원본은 `/etc/turtlebot4_discovery/setup.bash`(robot1만). robot4 IP 미정.
- `env.bash`는 로컬 전용(gitignore). `env.bash.example`을 복사해서 `ROS_DOMAIN_ID`, 로봇 네임스페이스 설정.

## 패키지 구성

공용 지도는 패키지 밖 저장소 루트 `maps/gallery.yaml` (`.envrc`가 `$ARTI_MAP`으로 설정).
역할(노션 간트 차트)별 폴더. 두 역할 이상이 쓰는 것은 `src/common/`. 규칙은 `src/README.md`.

| 폴더 | 패키지 | 빌드 | 역할 | 상태 |
|---|---|---|---|---|
| common | arti_interfaces | ament_cmake | 커스텀 msg/srv/action (역할 간 데이터는 여기로만) | `VisitorDetection` msg, `ExplainArtwork` action |
| common | arti_bringup | ament_python | 로봇별 launch·파라미터 (localization, nav2, /robot1·/robot4) | `localization.launch.py`(AMCL 시작 위치 = `arti_common/config/init_pose.yaml` undock 자동) |
| common | arti_common | ament_python | 공용 좌표 config, 유틸 | `config/artworks.yaml` (작품 4·출구·키오스크 반원 구역) |
| (src 바로 아래) | arti_follow | ament_python | 사용자 추종: 등록·재식별, 위치 추정, 80cm 추종, 분실 처리 | `visitor_follower`(인식 모듈의 `target_person/point_map`으로 플로우차트대로 주행 + LiDAR·depth 회피, 한 파일), `artwork_zone`(작품 반원 구역 5초 정지 → `explain_artwork`, 출구·키오스크 → `guide_message`). 통합 실행 `tour.launch.py`(인식 노드만 빼려면 `tracker:=camera`), 구역 RViz `zone_view.launch.py`(`zone_viz` 마커) |
| (src 바로 아래) | person_tracker | ament_python | 사람 인식: 탑캠 YOLO 추적·재식별 + LiDAR → `target_person/point_map` (jsbzzz/turtlebot_project에서 가져옴) | `multi_tracker_node`, `camera_pub`, `top_cam_tf` 등 |
| (src 바로 아래) | arti_audio | ament_python | 작품 설명·안내 음성 (T35). 작품 정보(artwork_id·artwork_title·artist·location·theme·description·x·y, 지금 `config/artworks_sample.yaml`, 나중에 DB)와 안내 문구(`config/guide_lines.yaml`)를 실행 중에 edge-tts로 합성(캐시 `~/.cache/arti_audio/tts`) → 스피커(`pw-play`) 재생 | `guide_audio_node` (`explain_artwork` 액션, `guide_message` 토픽) |

예정 역할 폴더 (필요해지면 추가): `course/`(코스주행, 현재 다른 담당), `docent/`(작품 해설), `fleet/`(배정·통행 순서·도킹·비상 정지), `marking/`(AprilTag 작품 마킹), 키오스크/관제 UI(Figma 설계 중).
역할 패키지끼리 직접 import하지 않는다.
문서도 같은 구분: `docs/common/`, `docs/follow/` (`docs/README.md`). 역할 설계(노드·토픽·파라미터)가 정해지면 해당 역할 README 표에 적는다.

## 미정

- robot4 IP·discovery server ID
- package.xml maintainer (현재 자리표시자 `arti@todo.invalid`)
