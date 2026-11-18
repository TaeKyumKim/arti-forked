# src 폴더 구조

역할(간트 차트의 작업)별로 폴더를 나눈다. 두 역할 이상이 쓰는 것은 `common/`에 둔다.
colcon은 하위 폴더까지 찾아서 빌드하므로 폴더를 나눠도 `pb` 한 번으로 모두 빌드된다.

공용 지도는 패키지가 아니라 저장소 루트 `maps/`에 있다.

```
src/
├── common/            모두가 쓰는 것
│   ├── arti_interfaces    커스텀 msg/srv/action
│   ├── arti_bringup       로봇별 launch·파라미터 (localization, nav2, /robot1·/robot4)
│   └── arti_common        공용 좌표(config/: 작품·만남·충전 위치, 구역), 공용 Python 유틸
├── arti_follow/       사용자 추종 로봇 (패키지) — 관람객 등록·재식별, 위치 추정, 80cm 추종, 분실 처리
├── arti_audio/        작품 설명·안내 음성 (패키지) — explain_artwork 액션 + guide_message 토픽 → 스피커 재생
└── person_tracker/    사람 인식 (패키지) — 탑캠 YOLO 추적·재식별 + LiDAR → target_person/point_map
                       (jsbzzz/turtlebot_project에서 가져옴, 모델은 yolo26n-pose.pt만 포함, 지도는 루트 maps/ 사용)
```

## 규칙

- **내 역할 폴더 안에서는 자유롭게** 바꾼다. 다른 역할 폴더는 그 담당자와 이야기한 뒤 바꾼다.
- **`common/`을 바꿀 때는** PR 설명에 적고, 다른 역할 담당자 한 명 이상의 리뷰를 받는다. 모두에게 영향이 가기 때문이다.
- 역할 폴더에서 만든 코드를 다른 역할도 쓰게 되면 `common/`으로 옮긴다.
  예: 사람 인식(YOLO)을 코스주행에서도 쓰게 되면 `arti_follow`에서 공용 패키지로 분리한다.
- 역할 패키지끼리 직접 import하지 않는다. 주고받을 데이터는 `arti_interfaces`의 msg/srv/action으로 정의한다.
- 새 역할이 생기면 같은 방식으로 폴더를 추가한다 (예: `course/` 코스주행, `docent/` 작품 해설, `fleet/` 배정·관제, `marking/` AprilTag 작품 마킹).
