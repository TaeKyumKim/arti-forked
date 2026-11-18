"""관람객 추종 주행 — 플로우차트(tracking_pc_ver3) '추적 모듈'의 움직임.

입력  target_person/point_map (PointStamped, frame_id map)  인식 모듈이 준 관람객 좌표
                                  → TF(map→odom)로 odom 좌표로 바꿔 씀 (localization 필요)
      scan (LaserScan)            벽·기둥 등 장애물
      oakd/stereo/image_raw/compressedDepth, oakd/stereo/camera_info   바닥에 떨어진 물체
      tf, tf_static               좌표 변환
      follow/enable (Bool)        false면 cmd_vel을 보내지 않음 (작품 설명 중, 도킹 등)
      dock_status (irobot_create_msgs/DockStatus)   도크에 붙어 있는지 (붙어 있을 때만 undock)
      arti/start_tour (arti_interfaces/StartTour, 액션 서버)   관람 시작 (mode FREE만). 취소하면 관람 끝
                                  feedback: state(MOVING/ARRIVED/FOLLOW/EXPLAINING), registered, 설명 중 작품, 설명한 수/전체
                                  result: success, message(끝난 이유), visited_ids(끝까지 설명한 작품)
      artwork_zone/explaining (String)   설명 중인 artwork_id ('' = 없음) → feedback EXPLAINING, current_artwork_id
      artwork_zone/explained (String)    끝까지 설명한 artwork_id → visited_ids
출력  cmd_vel (TwistStamped), follow/state (String)
      undock (irobot_create_msgs/Undock, 액션 클라이언트)     Create 3 언독
      initialpose (PoseWithCovarianceStamped)      AMCL 초기 위치 = init_pose.yaml의 출발 지점(undock)
      guide_message (String)                       관람객을 인식하면 recognized_guide 키 → arti_audio가 말함
      arti/tour_end (Empty)                        관람이 끝나면(취소·LOST·실패) 한 번 → 인식 모듈·artwork_zone이
                                                   그 관람객 데이터(re-ID 특징, 좌표 기록 등)를 지운다. 로봇 1대 = 관람객 1명
진단  diag일 때 diag_period마다 토픽별 수신 주기·지연·콜백 시간, 제어 주기·계산 시간, 정지 이유를 로그로 낸다.
      (화면 로그만. 테스트 기록은 follow_test.launch.py의 ros2 bag)

상태 (플로우차트 순서). 시작은 READY — arti/start_tour 액션이 와야 움직인다 (auto_start면 켜자마자)
  READY       액션 기다림. cmd_vel을 보내지 않음 (텔레옵 등 다른 노드와 안 겹침)
  UNDOCK      도크에 붙어 있으면 Create 3 undock 액션 (로봇이 스스로 움직임 → cmd_vel 안 보냄). 도크 밖이면 건너뜀
  LOCALIZE    AMCL에 initialpose(출발 지점 x, y, yaw_deg)를 보내고 map 위치가 그 근처로 잡힐 때까지 기다림
  START_FACE  제자리 회전으로 face_zone(기본 키오스크, arti_common artworks.yaml) 중심 쪽을 봄.
              face_zone이 없으면 출발 지점의 yaw_deg. 다 돌거나 face_timeout 지나면 WAIT_VISITOR
  WAIT_VISITOR 관람객 인식 기다림 (이 전까지 들어온 좌표는 버림). 좌표가 register_confirm_time 동안 끊김 없이
              (register_gap 안) 들어오면 등록 완료(on_registered: feedback registered, "인식되었습니다") → FOLLOW.
              확인하는 동안은 따라가지 않고 그 자리에서 바라본다. 끊기면 처음부터 다시 기다림
  FOLLOW  좌표가 들어옴 → 80 cm 거리 유지하며 이동
  SEARCH  lost_timeout 동안 안 들어옴 → 최종 좌표에 도착할 때까지 이동 → 거기서 search_time 동안 한 바퀴 회전 탐색
  WAIT    회전 탐색 끝 → 정지 대기 (상태 노드가 '로봇 쪽으로 와 주세요' 안내)
  LOST    wait_time 지남 → 정지, 액션 끝 (reason lost). 다시 시작하려면 액션을 새로 보냄
  SEARCH·WAIT 중 좌표가 다시 들어오면 FOLLOW
  액션을 취소하면 언제든 정지 → READY

관람객 좌표: 인식 좌표는 튀므로 칼만 필터(등속 모델)로 위치·속도를 매끄럽게 추정하고,
      예측에서 너무 먼 좌표(이상값)는 버린다. 연속으로 버려지면 사람이 정말 옮긴 것으로 보고 다시 시작.
멈춤: 0.85 m 안에서 서고 0.95 m 밖에서 다시 출발 (경계를 둘로 나눠 섰다 갔다 방지), 감속은 linear_decel로 부드럽게.
주행: 장애물로 격자를 만들어 칸마다 목표까지 '돌아가는 거리'를 구하고(BFS),
      (v, w) 후보를 sim_time 동안 굴려 보며 안 부딪히고 끝점의 그 거리가 가장 작은 것을 고른다(DWA).
"""
import math
import os
import re
import signal
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from arti_interfaces.action import StartTour
import cv2
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TwistStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import yaml

PARAMS = {
    'odom_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
    'auto_start': False,            # true면 액션 없이 켜자마자 관람 시작 (테스트용)
    'undock': True,                 # 관람 시작 때 도크에 붙어 있으면 undock
    'undock_timeout': 60.0,         # undock 액션을 이만큼 기다림 [s]
    'set_initial_pose': True,       # undock 뒤 AMCL initialpose를 출발 지점으로 보냄
    'localize_timeout': 10.0,       # map 위치가 출발 지점 근처로 잡히길 이만큼 기다림 [s]
    'localize_tolerance': 0.3,      # initialpose 뒤 map 위치가 출발 지점에서 이 안이면 잡힌 것 [m]
    'recognized_guide': 'recognized',   # 관람객 인식 때 guide_message로 보낼 키 (arti_audio guide_lines.yaml, '' = 말 안 함)
    'start_pose_file': '',          # '' = arti_common/config/init_pose.yaml
    'zones_file': '',               # '' = arti_common/config/artworks.yaml (face_zone을 찾는 곳)
    'face_zone': 'kiosk',           # 시작 때 바라볼 곳 = artworks.yaml 구역(zone_id·artwork_id)의 중심 ('' = 출발 지점 yaw_deg)
    'start_point': 'undock',        # init_pose.yaml robots.<네임스페이스>.<이것> = AMCL 초기 위치
    'face_tolerance': 0.1,          # 이 각도 안이면 다 돈 것 [rad] (약 6°)
    'register_confirm_time': 1.0,   # 관람객이 이만큼 끊김 없이 보여야 등록 완료 (순간 오인식 방지) [s]
    'register_gap': 0.5,            # 등록 확인 중 좌표 간격이 이보다 길면 끊긴 것 → 다시 기다림 [s]
    'face_timeout': 15.0,           # 이 시간 안에 못 돌면 포기하고 WAIT_VISITOR [s] (map TF가 없을 때 등)
    'target_distance': 0.8,         # 유지 거리 [m]
    'distance_tolerance': 0.05,     # 이 안이면 전진 안 함 [m]
    'restart_margin': 0.15,         # 멈춘 뒤에는 target_distance + 이만큼 멀어져야 다시 출발 [m] (섰다 갔다 방지)
    'lead_time': 0.5,               # 관람객 속도로 이만큼 앞을 보고 목표를 잡음 [s] (길면 좌표 잡음이 커짐)
    'kf_accel_std': 1.0,            # 칼만: 관람객 가속도 잡음 (클수록 빨리 따라가고 덜 매끄러움) [m/s^2]
    'kf_meas_std': 0.15,            # 칼만: 인식 좌표 잡음 (클수록 더 매끄럽고 느림) [m]
    'kf_gate': 9.21,                # 칼만: 예측과 이만큼(마할라노비스 거리^2, 2차원 99 %) 넘게 다르면 이상값으로 버림
    'kf_max_rejects': 5,            # 연속으로 이만큼 버리면 사람이 정말 옮긴 것으로 보고 필터를 새로 시작
    'max_person_speed': 1.5,        # 추정 속도 상한 [m/s]
    'face_deadband': 0.15,          # 가까이 서 있을 때 이 각도 안이면 돌지 않음 [rad] (약 9°)
    'lost_timeout': 3.0,            # 좌표가 이만큼 안 오면 분실 → SEARCH [s]
    'search_time': 10.0,            # 최종 좌표에 도착한 뒤 회전 탐색 시간 (플로우차트 10초) [s]
    'search_move_time': 0.0,        # 최종 좌표로 이동하는 시간 상한. 0이면 도착할 때까지 [s]
    'wait_time': 10.0,              # 그다음 정지 대기 (플로우차트 10초) [s]
    'search_spin': 0.6,             # 회전 탐색 최소 속도. 남은 시간에 한 바퀴가 안 되면 더 빨리 돎 [rad/s]
    'max_linear': 0.3, 'max_angular': 1.5,          # [m/s], [rad/s] (TurtleBot 4 최대 약 0.31 m/s)
    'linear_accel': 0.5, 'angular_accel': 3.0,      # [m/s^2], [rad/s^2]
    'linear_decel': 0.4,            # 멈출 때 감속 [m/s^2] (0.3 m/s → 0.75 s, 약 11 cm 더 감)
    'control_period': 0.1, 'sim_time': 1.5,         # 제어 주기, 후보를 굴려 볼 시간 [s]
    'v_samples': 7, 'w_samples': 15,
    'robot_radius': 0.2, 'safety_margin': 0.05,     # [m]
    'grid_resolution': 0.1,         # 길 찾기 격자 칸 [m]
    'w_heading': 0.3, 'w_clearance': 1.0, 'clearance_zone': 0.4,   # 비용: 방향, 장애물 근접(이 거리 안)
    'w_turn': 0.02,                 # 비용: 회전 (같은 조건이면 직진)
    'scan_min_range': 0.2,          # 이보다 가까운 LiDAR 점은 로봇 몸체(상판 기둥) [m]
    'scan_timeout': 0.5,            # scan이 이보다 오래 안 오면 정지 [s]
    'use_depth': True,              # 바닥 물체 회피 켜기
    'depth_step': 8,                # depth 몇 픽셀마다 한 점
    'depth_max_range': 1.2,         # 이보다 먼 depth는 잡음이 커서 안 씀 (실측: 1 m 너머 ±0.1~0.3 m) [m]
    'floor_min_height': 0.05,       # 바닥 물체 높이 범위 (base_link 기준, 아래는 바닥 잡음) [m]
    'floor_max_height': 0.3,        #   (위는 LiDAR가 봄)
    'floor_min_points': 3,          # 10 cm 칸에 점이 이만큼 모여야 물체 (흩어진 잡음 제거)
    'depth_keep': 1.0,              # depth 장애물을 이 시간 동안 기억 [s]
    'start_enabled': True,          # 시작할 때 cmd_vel을 보낼지 (false면 follow/enable true가 올 때까지 계산만)
    'diag': False,                  # 데이터 흐름 진단 로그 (끊김 원인 찾기)
    'diag_period': 2.0,             # 진단 요약을 내는 주기 [s]
    'diag_events': False,           # 끊김이 생긴 순간마다 바로 로그 (토픽 늦음, 제어 밀림, 정지 이유 바뀜)
}


def artwork_number(artwork_id):
    """artwork_id 문자열 → 액션에서 쓰는 int 작품 ID.
    받음: artwork_id  "art_02" (artworks.yaml 키)
    돌려줌: 끝 숫자 2 (숫자가 없으면 0)"""
    m = re.search(r'(\d+)$', artwork_id)
    return int(m.group(1)) if m else 0


class Diag:
    """진단 통계 (diag:=true일 때만): 이름별로 (개수, 합, 최대, 최소)를 모았다가 take()로 꺼내고 비운다."""

    def __init__(self):
        """빈 통계로 시작. 모으기 시작한 시각을 기록한다."""
        self.s, self.start = {}, time.monotonic()

    def add(self, key, value=1.0):
        """통계에 값 하나 더하기.
        받음: key    이름 (예: 'scan.gap')
              value  값 (기본 1 = 개수 세기)"""
        c = self.s.setdefault(key, [0, 0.0, -math.inf, math.inf])
        c[0], c[1], c[2], c[3] = c[0] + 1, c[1] + value, max(c[2], value), min(c[3], value)

    def take(self):
        """모은 통계를 꺼내고 비운다 (report가 diag_period마다 부름).
        돌려줌: (통계 dict {이름: [개수, 합, 최대, 최소]}, 모은 시간 [s])"""
        s, span = self.s, time.monotonic() - self.start
        self.s, self.start = {}, time.monotonic()
        return s, span


def wrap(a):
    """각도를 -π ~ π 범위로 맞춘다 (350° → -10°). 회전할 때 짧은 쪽으로 돌게 하려고.
    받음: a  각도 [rad] (숫자 또는 numpy 배열)
    돌려줌: -π ~ π 각도 [rad]"""
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    """쿼터니언에서 바닥 위 방향(yaw)만 뽑는다. 로봇은 바닥에서만 움직이므로 기울기(roll, pitch)는 버린다.
    받음: q  geometry_msgs/Quaternion (x, y, z, w)
    돌려줌: yaw [rad] (0 = x축 방향, + = 왼쪽으로 돈 쪽)"""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def apply(f, pts):
    """점들을 다른 좌표계로 옮긴다 (2D 회전 + 이동). 예: 라이다 좌표 → odom 좌표.
    받음: f    변환 (x, y, yaw)  lookup()의 결과
          pts  점들 (N, 2) numpy 배열
    돌려줌: 옮긴 점들 (N, 2)"""
    c, s = math.cos(f[2]), math.sin(f[2])
    return np.stack([f[0] + c * pts[:, 0] - s * pts[:, 1], f[1] + s * pts[:, 0] + c * pts[:, 1]], 1)


class PersonKF:
    """관람객 위치 칼만 필터 (등속 모델: 사람이 같은 속도로 걷는다고 가정).
    상태 x = [px, py, vx, vy] (odom 좌표, m · m/s). 튀는 인식 좌표를 매끄럽게 하고, 위치만 받아도 속도를 추정한다."""

    def __init__(self, accel_std, meas_std, gate, max_rejects):
        """필터 준비. 아직 사람 정보 없음 (x = None).
        받음: accel_std    사람 가속도 잡음 [m/s²] (클수록 빨리 따라가고 덜 매끄러움)
              meas_std     인식 좌표 잡음 [m] (클수록 매끄럽고 느림)
              gate         이상값 기준 (예측과 이만큼 넘게 다르면 버림)
              max_rejects  연속으로 이만큼 버리면 사람이 정말 옮긴 것으로 보고 새로 시작"""
        self.q, self.r, self.gate, self.max_rejects = accel_std ** 2, meas_std ** 2, gate, max_rejects
        self.x, self.P, self.t, self.rejects = None, None, None, 0

    def reset(self, z, t):
        """이 좌표로 필터를 새로 시작한다 (처음 보거나 오래 못 봤을 때). 속도는 모르니 0, 불확실도는 크게.
        받음: z  좌표 (x, y) odom
              t  시각 [s]
        바뀜: x = [x, y, 0, 0], P, t"""
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 1.0, 1.0])   # 속도는 모름 → 크게
        self.t, self.rejects = t, 0

    def predict(self, t):
        """지난 위치 + 속도 × 지난 시간 → 지금쯤 위치를 예측한다 (불확실도도 시간만큼 커짐).
        받음: t  예측할 시각 [s]
        바뀜: x, P, t"""
        dt = max(t - self.t, 0.0)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + self.q * g @ g.T, t

    def update(self, z, t):
        """좌표 하나 반영 (reset 뒤에만): 예측 → 받은 좌표와 비교 → 너무 다르면 버림, 아니면 섞어서 보정.
        받음: z  좌표 (x, y) odom
              t  받은 시각 [s]
        돌려줌: 반영했으면 True, 이상값으로 버렸으면 False (예측은 반영됨)
              연속 max_rejects번 버리면 사람이 정말 옮긴 것으로 보고 reset"""
        self.predict(t)
        y = np.asarray(z, float) - self.x[:2]
        S = self.P[:2, :2] + self.r * np.eye(2)
        if y @ np.linalg.solve(S, y) > self.gate:
            self.rejects += 1
            if self.rejects < self.max_rejects:
                return False
            self.reset(z, t)                            # 계속 다른 곳 → 사람이 정말 옮김
            return True
        K = self.P[:, :2] @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ np.eye(2, 4)) @ self.P    # np.eye(2, 4) = H (상태에서 위치만 꺼냄)
        self.rejects = 0
        return True


class VisitorFollower(Node):

    """관람객 추종 노드: 키오스크 액션(start_tour) 응대 + 상태 기계(step) + 주행(dwa)."""
    def __init__(self):
        """노드 준비: 파라미터·설정 파일 읽기, 구독·발행·액션 서버·타이머 등록. 상태 READY.
        구독  tf, tf_static, target_person/point_map, scan, oakd camera_info·compressedDepth,
              follow/enable, dock_status, artwork_zone/explaining·explained
        발행  cmd_vel, follow/state, initialpose, guide_message, arti/tour_end
        액션  서버 arti/start_tour, 클라이언트 undock
        타이머 control_period(0.1 s)마다 control()"""
        super().__init__('visitor_follower')
        self.p = {q.name: q.value for q in self.declare_parameters('', list(PARAMS.items()))}
        self.tf = Buffer()
        self.seen = -1e9                            # 관람객 좌표를 마지막으로 받은 시각
        self.kf = PersonKF(self.p['kf_accel_std'], self.p['kf_meas_std'], self.p['kf_gate'], self.p['kf_max_rejects'])
        self.lost_at, self.turn, self.spin_at, self.spin_speed, self.holding = 0.0, 1.0, None, 0.0, False
        self.scan_pts, self.scan_time = np.zeros((0, 2)), -1e9
        self.floor_pts, self.floor_time = np.zeros((0, 2)), -1e9
        self.cam_k = self.cam_tf = None
        self.state, self.enabled, self.cmd = None, self.p['start_enabled'], (0.0, 0.0)
        self.dbg = {}                               # 이번 주기의 판단 과정 (diag)
        self.diag = Diag() if self.p['diag'] else None
        self.arrive, self.last_tick, self.last_cmd = {}, None, None
        self.gap_avg, self.last_reason = {}, None    # 토픽별 평소 수신 간격, 지난 주기 이유 (diag_events)
        if self.diag is not None:
            self.get_logger().info('파라미터 ' + ', '.join(f'{k}={v}' for k, v in self.p.items()))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def sub(kind, topic, fn, qos):               # 진단 켜면 수신 간격·지연·콜백 시간을 잰다
            return self.create_subscription(kind, topic, self.timed(topic, fn), qos)
        # tf는 네임스페이스 안(/robot1/tf)에 있으므로 상대 이름으로 받아 Buffer에 넣는다
        sub(TFMessage, 'tf', lambda m: [self.tf.set_transform(t, 'tf') for t in m.transforms], 100)
        sub(TFMessage, 'tf_static', lambda m: [self.tf.set_transform_static(t, 'tf') for t in m.transforms],
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(PointStamped, 'target_person/point_map', self.on_person, 10)
        sub(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        sub(CameraInfo, 'oakd/stereo/camera_info',
            lambda m: setattr(self, 'cam_k', (m.k[0], m.k[4], m.k[2], m.k[5])), 10)
        if self.p['use_depth']:
            sub(CompressedImage, 'oakd/stereo/image_raw/compressedDepth', self.on_depth, qos_profile_sensor_data)
        sub(Bool, 'follow/enable', self.on_enable, 10)
        sub(DockStatus, 'dock_status', lambda m: setattr(self, 'docked', m.is_docked), qos_profile_sensor_data)
        sub(String, 'artwork_zone/explaining', self.on_explaining, latched)
        sub(String, 'artwork_zone/explained', self.on_explained, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'follow/state', latched)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.guide_pub = self.create_publisher(String, 'guide_message', 10)
        self.tour_end_pub = self.create_publisher(Empty, 'arti/tour_end', 10)
        self.start = self.load_start_point()        # 초기 위치 (x, y, yaw) map
        self.face_point = self.load_face_point()    # 시작 때 바라볼 점 (x, y) map
        self.artwork_total = self.count_artworks()  # feedback total (FREE)

        # 관람 (start_tour 액션). 액션 처리는 다른 스레드에서 기다리므로 제어 주기와 겹치는 상태 변경은 lock으로 막는다
        self.lock = threading.Lock()
        self.docked = None                          # dock_status (아직 못 받으면 None)
        self.active, self.end_reason = False, ''    # 관람 중인지, 끝난 이유 (액션 result)
        self.registered, self.explaining, self.visited = False, '', []   # 이번 관람: 인식됨, 설명 중 작품, 설명한 작품
        self.seen_since = None                      # 등록 확인: 끊김 없이 보이기 시작한 시각
        self.face_until = self.loc_at = self.pose_sent = None
        group = ReentrantCallbackGroup()
        self.undock_client = ActionClient(self, Undock, 'undock', callback_group=group)
        ActionServer(self, StartTour, 'arti/start_tour', self.run_tour, callback_group=group,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.set_state('READY')
        if self.p['auto_start']:
            def once():
                timer.cancel()
                self.run_tour(None)
            timer = self.create_timer(1.0, once, callback_group=group)
        else:
            self.get_logger().info(f'관람 시작 액션 기다림: {self.get_namespace().rstrip("/")}/arti/start_tour')

        self.create_timer(self.p['control_period'], self.control)
        if self.diag is not None:
            self.create_timer(self.p['diag_period'], self.report)

    def config_file(self, param, name):
        """설정 파일 경로: 파라미터에 경로가 있으면 그것, 비었으면 arti_common/config/<name>.
        받음: param  파라미터 이름 (예: 'zones_file')
              name   기본 파일 이름 (예: 'artworks.yaml')
        돌려줌: 파일 경로"""
        return self.p[param] or os.path.join(get_package_share_directory('arti_common'), 'config', name)

    def load_start_point(self):
        """init_pose.yaml에서 이 로봇의 출발 지점을 읽는다 (LOCALIZE에서 AMCL에 알려 줄 위치).
        yaw_deg가 없으면 dock → 출발 지점 방향, dock도 없으면 0.
        받음: (파라미터) start_pose_file, start_point, 노드 네임스페이스
        돌려줌: (x, y, yaw) map 좌표, 못 읽으면 None (초기 위치 안 보냄)"""
        path = self.p['start_pose_file']
        try:
            path = self.config_file('start_pose_file', 'init_pose.yaml')
            with open(path, encoding='utf-8') as f:
                robot = (yaml.safe_load(f) or {}).get('robots', {}).get(self.get_namespace(), {})
            target = robot[self.p['start_point']]
        except (OSError, KeyError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'출발 지점을 못 읽음 ({path}, {self.get_namespace()}: {e!r}) → 초기 위치 안 보냄')
            return None
        x, y, dock = float(target['x']), float(target['y']), robot.get('dock')
        if 'yaw_deg' in target:
            yaw = math.radians(float(target['yaw_deg']))
        else:
            yaw = math.atan2(y - float(dock['y']), x - float(dock['x'])) if dock else 0.0
        self.get_logger().info(f'출발 지점 {self.p["start_point"]} ({x}, {y}, {math.degrees(yaw):.0f}°)')
        return x, y, yaw

    def load_face_point(self):
        """artworks.yaml에서 face_zone(기본 kiosk) 구역의 중심을 읽는다 (START_FACE에서 바라볼 곳).
        받음: (파라미터) face_zone, zones_file
        돌려줌: (x, y) map 좌표, 없으면 None (출발 지점 방향으로 돎)"""
        zid = self.p['face_zone']
        if not zid:
            return None
        path = self.p['zones_file']
        try:
            path = self.config_file('zones_file', 'artworks.yaml')
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            rows = [r for r in (data.get('guide_zones') or []) + (data.get('artworks') or [])
                    if zid in (r.get('zone_id'), r.get('artwork_id'))]
            r = rows[0]
            if 'x' in r:
                point = (float(r['x']), float(r['y']))
            else:                                   # 직사각형 구역 → 가운데
                point = ((float(r['x_min']) + float(r['x_max'])) / 2, (float(r['y_min']) + float(r['y_max'])) / 2)
        except (OSError, KeyError, IndexError, PackageNotFoundError, yaml.YAMLError, ValueError) as e:
            self.get_logger().warn(f'바라볼 구역 {zid}을 못 읽음 ({path}: {e!r}) → 출발 지점 방향으로 돎')
            return None
        self.get_logger().info(f'시작 때 바라볼 곳: {zid} {point}')
        return point

    def count_artworks(self):
        """artworks.yaml의 작품 수를 센다 (feedback total).
        받음: (파라미터) zones_file
        돌려줌: 작품 수 (못 읽으면 0)"""
        try:
            with open(self.config_file('zones_file', 'artworks.yaml'), encoding='utf-8') as f:
                return len((yaml.safe_load(f) or {}).get('artworks') or [])
        except (OSError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'작품 수를 못 읽음 ({e!r}) → total 0')
            return 0

    # ---------- 입력 ----------

    def now(self):
        """지금 시각 (ROS 시계).
        돌려줌: 초 [s] (float)"""
        return self.get_clock().now().nanoseconds / 1e9

    def lookup(self, target, source, stamp=None):
        """TF에서 source 좌표계가 target 좌표계의 어디에 있는지 꺼낸다 (쓰기 편하게 2D만).
        받음: target  기준 좌표계 (예: 'odom')
              source  찾을 좌표계 (예: 'base_link')
              stamp   (선택) 이 시각의 값. 없으면 최신 값으로 다시 시도
        돌려줌: (x, y, yaw), 변환이 없으면 None (예외 대신)"""
        # odom 하고 base_link 시각을 받아서 x, y , vaw 만 뽑아서 
        for t in ([stamp] if stamp is not None else []) + [Time()]:
            try:
                tr = self.tf.lookup_transform(target, source, t).transform
                if t is not stamp and stamp is not None and self.diag is not None:
                    self.diag.add('tf_fallback:' + source)      # 그 시각 TF가 없어 최신 값으로 대신함
                return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
            except Exception:
                pass
        if self.diag is not None:
            self.diag.add('tf_fail:' + source)
        return None

    def on_person(self, msg):
        """[구독] target_person/point_map (PointStamped, map) — 관람객 좌표가 올 때마다.
        받아도 되는 상태(WAIT_VISITOR, FOLLOW, SEARCH, WAIT)일 때만: map → odom 변환 → 칼만 필터.
        받음: msg.header.frame_id, msg.header.stamp (그 시각 TF로 변환), msg.point.x, y
        바뀜: kf.x (위치·속도), seen (마지막으로 본 시각), seen_since (등록 확인: 계속 보이기 시작한 시각)
        로봇을 움직이거나 state를 바꾸지는 않는다 (그건 step)."""
        if self.state not in ('WAIT_VISITOR', 'FOLLOW', 'SEARCH', 'WAIT'):
            return                                  # 관람 전·undock·초기 위치·출발 방향 보기 중 좌표는 버림
        f = self.lookup(self.p['odom_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if f is None:
            return
        q = apply(f, np.array([[msg.point.x, msg.point.y]]))[0]
        now = self.now()
        if self.state == 'WAIT_VISITOR' and (self.seen_since is None or now - self.seen > self.p['register_gap']):
            self.seen_since = now                   # 등록 확인 시작 (처음이거나 끊겼다 다시 보임)
        if self.kf.x is None or now - self.seen >= self.p['lost_timeout']:
            self.kf.reset(q, now)                   # 처음이거나 한동안 못 봄 → 새로 시작
        else:
            self.kf.update(q, now)                  # 이상값이면 버리고 예측만 (사람은 보이는 중)
        self.seen = now

    def person(self):
        """칼만 필터에서 관람객 위치·속도를 꺼낸다. 속도는 max_person_speed(1.5 m/s)로 자름 (튀는 값 막기).
        받음: (kf.x)
        돌려줌: (위치 [x, y], 속도 [vx, vy])  odom, kf.t 시각 기준"""
        pos, vel = self.kf.x[:2].copy(), self.kf.x[2:].copy()
        speed = np.linalg.norm(vel)
        if speed > self.p['max_person_speed']:
            vel *= self.p['max_person_speed'] / speed
        return pos, vel

    def on_scan(self, msg):
        """[구독] scan (LaserScan) — 라이다가 올 때마다.
        거리 → (x, y) 점, inf·nan·로봇 몸(0.2 m 안)·범위 밖은 버림 → 그 시각 TF로 odom 좌표.
        받음: msg.ranges, angle_min, angle_increment, range_max, header
        바뀜: scan_pts (장애물 점 N×2), scan_time (받은 시각 → step이 라이다 끊김 확인)"""
        f = self.lookup(self.p['odom_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if f is None:
            return
        r = np.asarray(msg.ranges)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > self.p['scan_min_range']) & (r < msg.range_max)
        self.scan_pts = apply(f, np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1))
        self.scan_time = self.now()

    def on_depth(self, msg):
        """[구독] oakd/stereo/image_raw/compressedDepth — 깊이 영상이 올 때마다.
        깊이 → 3D 점 → base_link 높이 floor_min~floor_max(5~30 cm)만 = 바닥 물체 → 10 cm 칸에 3점 이상만 → odom.
        받음: msg.data (12바이트 헤더 + 16비트 PNG, mm), (cam_k: camera_info의 fx, fy, cx, cy)
        바뀜: floor_pts (바닥 물체 점), floor_time"""
        p = self.p
        if self.cam_tf is None:
            try:
                tr = self.tf.lookup_transform(p['base_frame'], msg.header.frame_id, Time()).transform
            except Exception:
                return
            x, y, z, w = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
            rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            self.cam_tf = (rot, np.array([tr.translation.x, tr.translation.y, tr.translation.z]))
        f = self.lookup(p['odom_frame'], p['base_frame'], Time.from_msg(msg.header.stamp))
        # compressedDepth = 12바이트 헤더 + PNG(16비트, mm)
        depth = cv2.imdecode(np.frombuffer(bytes(msg.data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
        if self.cam_k is None or f is None or depth is None:
            return
        s, (fx, fy, cx, cy) = p['depth_step'], self.cam_k
        z = depth[::s, ::s] / 1000.0
        v, u = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
        ok = (z > 0.2) & (z < p['depth_max_range'])
        cam = np.stack([(u[ok] - cx) * z[ok] / fx, (v[ok] - cy) * z[ok] / fy, z[ok]], 1)
        base = cam @ self.cam_tf[0].T + self.cam_tf[1]
        base = base[(base[:, 2] > p['floor_min_height']) & (base[:, 2] < p['floor_max_height'])]
        if len(base):                               # 10 cm 칸에 floor_min_points개 이상 모인 점만
            _, inv, cnt = np.unique(np.floor(base[:, :2] / 0.1).astype(int), axis=0,
                                    return_inverse=True, return_counts=True)
            base = base[cnt[inv.ravel()] >= p['floor_min_points']]
        self.floor_pts, self.floor_time = apply(f, base[:, :2]), self.now()

    def on_enable(self, msg):
        """[구독] follow/enable (Bool) — false면 바로 정지하고 이후 cmd_vel을 보내지 않음 (다른 노드에 주행을 넘길 때).
        받음: msg.data
        바뀜: enabled"""
        if not msg.data:
            self.drive(0.0, 0.0)                    # 멈춘 뒤 cmd_vel을 넘겨줌
        self.enabled = msg.data

    def on_explaining(self, msg):
        """[구독] artwork_zone/explaining (String) — 설명 중인 작품. 관람 중일 때만 저장.
        받음: msg.data  "art_02" 또는 "" (설명 끝)
        바뀜: explaining → send_feedback의 EXPLAINING"""
        self.explaining = msg.data if self.active else ''

    def on_explained(self, msg):
        """[구독] artwork_zone/explained (String) — 끝까지 설명한 작품. 관람 중이고 처음일 때만 추가.
        받음: msg.data  "art_02"
        바뀜: visited → feedback current_index, result visited_ids"""
        if self.active and msg.data not in self.visited:
            self.visited.append(msg.data)

    # ---------- 상태 (플로우차트) ----------

    def control(self):
        """[타이머] control_period(0.1 s)마다. step()을 감싸는 껍데기:
        lock을 걸어 run_tour(다른 스레드)와 동시에 상태를 바꾸지 않게 하고, (diag) 걸린 시간을 기록한다."""
        t0 = time.monotonic()
        self.dbg = {}
        with self.lock:
            self.step()
        if self.diag is not None:
            self.record(t0)

    def step(self):
        """0.1 s마다 한 번: 지금 상태를 위에서부터 차례로 확인하고, 처음 맞는 곳에서 행동한 뒤 return.
        순서가 중요하다 (안전 확인이 따라가기보다 먼저). if마다 [번호] 주석 = 무엇을 묻고 어디로 가는지.
        받음: (state, kf.x, seen, scan_time, TF odom→base_link)
        내보냄: drive() → cmd_vel, set_state() → follow/state, 필요하면 end_tour()"""
        p, now = self.p, self.now()
        # [1] 지금 상태가 READY·UNDOCK·LOST인가? → 예: 아무것도 안 하고 끝 / 아니오: 아래로
        # 관람 전(READY)·관람 끝(LOST)·undock 중이면 손 놓기:
        # undock은 Create 3가 스스로 움직이므로 cmd_vel을 같이 보내면 둘이 부딪힌다. 텔레옵 등 다른 노드와도 안 겹치게
        if self.state in ('READY', 'UNDOCK', 'LOST'):
            self.dbg['reason'] = self.state.lower()
            return
        # [2] 지금 상태가 LOCALIZE인가? → 예: localize() / 아니오: 아래로
        # 초기 위치 잡는 중: 아직 map 위치를 믿을 수 없어서 움직이지 않고 AMCL만 기다린다
        if self.state == 'LOCALIZE':
            return self.localize(now)
        
        pose = self.lookup(p['odom_frame'], p['base_frame']) #로봇 위치가 나옴
        # [3] 센서 정상인가? (로봇 위치 TF가 있고, 라이다가 0.5 s 안에 왔나) → 아니오: 바로 정지 / 예: 아래로
        # 안전: 내 위치(TF)를 모르거나 라이다가 0.5 s 넘게 끊기면 장애물을 못 보는 상태 → 계산하지 말고 바로 정지
        if pose is None or now - self.scan_time > p['scan_timeout']: # 센서 확인
            self.dbg['reason'] = 'no_pose' if pose is None else 'scan_timeout' # 0.5초 넘게 안바뀌었으면 라이다 끊긴 것
            return self.drive(0.0, 0.0)
        here = np.array(pose[:2])

        # [4] 지금 상태가 START_FACE인가? → 예: face_start() (키오스크 쪽으로 돌기) / 아니오: 아래로
        # 출발 직후: 관람객이 키오스크 쪽에서 오므로 그쪽을 먼저 바라본다 (카메라에 잡히게)
        if self.state == 'START_FACE':
            return self.face_start(now)
        # [5] 지금 상태가 WAIT_VISITOR인가? → 예: wait_visitor() (1 s 확인 → 등록) / 아니오: 아래로
        # 등록 확인 중: 1 s 동안 계속 보여야 등록 (순간 오인식으로 엉뚱한 사람을 따라가지 않게). 확인 중에는 안 따라감
        if self.state == 'WAIT_VISITOR':
            return self.wait_visitor(now, pose, here)
        # [6] 관람객 위치 수신 중인가? (kf.x가 있고 now - seen < 3 s) → 예: FOLLOW / 아니오: 아래로(놓침)
        # 따라가기: 사람 위치를 알고, 마지막으로 본 지 3 s 안이면 FOLLOW.
        # 3 s는 카메라가 잠깐 놓쳐도(가려짐·흔들림) 바로 탐색으로 넘어가지 않게 둔 여유. 그동안은 칼만 예측으로 따라감
        if self.kf.x is not None and now - self.seen < p['lost_timeout']:
            self.set_state('FOLLOW')
            last, vel = self.person()
            person = last + vel * (now - self.kf.t)     # 마지막 좌표 뒤 흐른 시간만큼 → 지금 위치
            ahead = person + vel * p['lead_time']       # 0.5 s 뒤 위치: 인식·통신·가속이 늦는 만큼 미리 향함
            to = ahead - here
            d = np.linalg.norm(to)
            stop = p['target_distance'] + p['distance_tolerance']
            # [7] 사람까지 거리 d가 0.85 m 이하인가? → 예: holding = True (서기)
            # [8] 아니면 0.95 m 넘나? → 예: holding = False (다시 출발) / 사이: 그대로
            # 0.85 m 안이면 서기, 0.95 m 밖이면 다시 출발. 기준을 둘로 나눠
            # 0.8 m 근처에서 좌표가 조금 흔들릴 때 섰다 갔다 덜덜 떨지 않게 한다 (사이 구간은 하던 대로)
            if d <= stop:
                self.holding = True
            elif d > p['target_distance'] + p['restart_margin']:
                self.holding = False
            # [9] 서 있는 중이 아닌가? (holding == False) → 예: dwa로 따라가기 / 아니오: 아래 face()
            # 멀면: 사람에서 로봇 쪽으로 0.8 m 돌아온 점을 목표로 장애물 피해서 이동 (사람 자체를 목표로 하면 부딪힘)
            if not self.holding:
                goal = ahead - to / d * p['target_distance']
                self.dbg['reason'] = 'follow'
                return self.drive(*self.dwa(pose, goal, ahead))
            # 가까우면: 앞으로는 안 가고 사람 쪽으로 몸만 돌림 (작품 감상 중 등)
            self.dbg['reason'] = 'hold'
            return self.face(pose, here, person)
        self.holding = False
        # [10] 사람 정보가 아예 없나? (kf.x == None) → 예: 정지 / 아니오: 아래로(탐색)
        # 사람을 한 번도 못 봤으면 탐색할 기준(마지막 위치)이 없다 → 그냥 정지
        if self.kf.x is None: #kf.x 없음 정지해야함
            self.dbg['reason'] = 'idle'
            return self.drive(*self.ramp(0.0, 0.0))

        # ---- 여기부터 = 사람을 놓침 (3 s 넘게 좌표가 안 옴) ----
        last, vel = self.person()
        # [11] 방금 놓쳤나? (직전 상태가 FOLLOW) → 예: 놓친 시각·회전 방향 기억 (한 번만) / 아니오: 건너뜀
        # 방금 놓친 순간(직전까지 FOLLOW)에만 한 번: 놓친 시각과 사람이 가던 쪽(왼/오른)을 기억 → 나중에 그쪽으로 돈다
        if self.state == 'FOLLOW': # 놓친 순간 1회 
            self.lost_at, self.spin_at = now, None # lost-at은 언제 놓쳤나? spin_at은 회전 탐색은 아직 안 시작함
            d = last + vel - here
            self.turn = 1.0 if wrap(math.atan2(d[1], d[0]) - pose[2]) >= 0 else -1.0
        # [12] 아직 회전 탐색 전인가? (spin_at == None) → 예: 마지막 위치로 이동 또는 회전 준비 / 아니오: 아래로
        # 1) 아직 회전 탐색 전: 마지막으로 본 곳까지 먼저 간다 (기둥·모퉁이 뒤로 사라졌으면 가야 보인다)
        if self.spin_at is None:
            cap = p['search_move_time']             # 0이면 시간 제한 없음
            # [12-1] 마지막 위치까지 0.5 m 넘게 남았나? (시간 제한 안인가?) → 예: 그쪽으로 이동 / 아니오: 도착 → 회전 시작 시각 기록
            # 아직 0.5 m 넘게 남았고 (시간 제한이 있으면) 그 안이면 계속 이동
            if np.linalg.norm(last - here) > 0.5 and (cap <= 0 or now - self.lost_at < cap):
                self.set_state('SEARCH')
                self.dbg['reason'] = 'search_move'
                return self.drive(*self.dwa(pose, last, last))
            self.spin_at = now                      # 도착 → 여기서부터 search_time 동안 한 바퀴 (가속분 5 % 여유)
            self.spin_speed = float(np.clip(1.05 * 2 * math.pi / max(p['search_time'], 1e-3),
                                            p['search_spin'], p['max_angular']))
        t = now - self.spin_at
        # [13] 회전 탐색 10 s가 안 지났나? (t = now - spin_at < search_time) → 예: 제자리 회전 / 아니오(10 s 이상): 아래로
        # 2) 도착 뒤 10 s 동안 제자리 한 바퀴: 카메라 화각(88°) 밖으로 나간 사람을 다시 화면에 넣으려고.
        #    사람이 가던 쪽부터 돈다 (그쪽에 있을 가능성이 크다)
        if t < p['search_time']:
            self.set_state('SEARCH') # SEARCH 상태로 바꿈 
            self.dbg['reason'] = 'search_spin'
            return self.drive(*self.ramp(0.0, self.turn * self.spin_speed))
        # [14] 회전 10 s + 대기 10 s가 다 지났나? (t >= search_time + wait_time) → 예: 관람 끝 LOST / 아니오: 아래 WAIT
        # 4) 회전 10 s + 대기 10 s 다 지남 → 더 기다려도 못 찾는다고 보고 관람 끝 (키오스크에 result lost)
        if t >= p['search_time'] + p['wait_time']:
            return self.end_tour('LOST', 'lost')
        # 3) 회전으로도 못 찾음 → 멈춰서 기다림 (관람객이 로봇 쪽으로 와 주길). 좌표가 다시 오면 위 FOLLOW로 돌아감
        self.set_state('WAIT')
        self.dbg['reason'] = 'wait'
        self.drive(*self.ramp(0.0, 0.0))

    def face(self, pose, here, person):
        """제자리에서 감속해 서서 사람 쪽으로 몸만 돌린다 (face_deadband 9° 안이면 안 돎 → 꿈틀거림 방지).
        받음: pose    로봇 (x, y, yaw) odom
              here    로봇 (x, y)
              person  사람 (x, y)
        내보냄: drive(ramp(0, w)) → cmd_vel (전진 0, 회전 w)"""
        turn = wrap(math.atan2(*(person - here)[::-1]) - pose[2])
        w = float(np.clip(1.5 * turn, -self.p['max_angular'], self.p['max_angular']))
        if abs(turn) < self.p['face_deadband']:
            w = 0.0
        return self.drive(*self.ramp(0.0, w))

    def wait_visitor(self, now, pose, here):
        """[WAIT_VISITOR] 관람객이 register_confirm_time(1 s) 동안 끊김 없이(register_gap 0.5 s 안) 보이면 등록 → FOLLOW.
        확인하는 동안은 따라가지 않고 바라보기만 한다 (등록 안 된 사람을 따라가지 않게). 끊기면 처음부터 다시.
        받음: now, pose (x, y, yaw), here (x, y), (kf.x, seen, seen_since)
        내보냄: 끊김 → 정지 / 확인 중 → face() / 1 s 채움 → on_registered(), state FOLLOW"""
        p = self.p
        if self.kf.x is None or now - self.seen > p['register_gap']:
            if self.kf.x is not None:
                self.get_logger().info('등록 확인 중 끊김 → 다시 기다림')
            self.kf.x, self.seen_since = None, None
            self.dbg['reason'] = 'wait_visitor'
            return self.drive(*self.ramp(0.0, 0.0))
        if now - self.seen_since < p['register_confirm_time']:
            self.dbg['reason'] = 'register_confirm'
            last, vel = self.person()
            return self.face(pose, here, last + vel * (now - self.kf.t))
        self.on_registered(now - self.seen_since)
        self.set_state('FOLLOW')                    # 다음 주기부터 추종
        return self.drive(*self.ramp(0.0, 0.0))

    def on_registered(self, confirmed):
        """등록 완료 순간에 할 일 (한곳에). 나중에 인식 모듈이 등록 신호를 주면 부르는 조건만 바꾼다.
        받음: confirmed  끊김 없이 보인 시간 [s] (로그용)
        바뀜: registered = True → run_tour의 다음 feedback으로 키오스크에 보고
        내보냄: guide_message recognized_guide("recognized") → 음성 '인식되었습니다'"""
        self.registered = True                      # → start_tour feedback registered (키오스크 보고)
        self.get_logger().info(f'관람객 등록 완료 ({confirmed:.1f} s 끊김 없이 보임)')
        if self.p['recognized_guide']:
            self.guide_pub.publish(String(data=self.p['recognized_guide']))   # "인식되었습니다"

    def face_start(self, now):
        """[START_FACE] 제자리 회전으로 face_zone(키오스크) 쪽을 본다. 관람객이 키오스크에서 오므로 카메라에 잡히게.
        map 좌표 필요 (그래서 LOCALIZE 뒤). face_timeout(15 s) 안에 못 돌면 포기하고 WAIT_VISITOR.
        받음: now, (face_point, start, TF map→base_link)
        내보냄: drive(ramp(0, w)), 다 돌면 state WAIT_VISITOR"""
        p = self.p
        if self.face_point is None and self.start is None:   # 바라볼 곳을 모름 → 돌지 않음
            return self.set_state('WAIT_VISITOR')
        if self.face_until is None:
            self.face_until = now + p['face_timeout']
        pose = self.lookup(p['map_frame'], p['base_frame'])
        if pose is None or now > self.face_until:
            if now > self.face_until:
                self.get_logger().warn(f'시작 회전 {p["face_timeout"]} s 안에 못 끝냄'
                                       + (' (map TF 없음: localization 확인)' if pose is None else '')
                                       + ' → WAIT_VISITOR')
                self.set_state('WAIT_VISITOR')
            self.dbg['reason'] = 'start_face_wait'
            return self.drive(*self.ramp(0.0, 0.0))
        x, y, yaw = pose
        if self.face_point is not None:             # 바라볼 곳(키오스크 등) 쪽
            want = math.atan2(self.face_point[1] - y, self.face_point[0] - x)
        else:                                       # 없으면 출발 지점의 방향
            want = self.start[2]
        err = float(wrap(want - yaw))
        self.dbg['reason'] = 'start_face'
        if abs(err) < p['face_tolerance'] and abs(self.cmd[1]) < 0.2:
            self.get_logger().info(f'시작 회전 끝 (오차 {math.degrees(err):.0f}°) → 관람객 인식 기다림')
            self.set_state('WAIT_VISITOR')
            return self.drive(*self.ramp(0.0, 0.0))
        w = float(np.clip(1.5 * err, -p['max_angular'], p['max_angular']))
        return self.drive(*self.ramp(0.0, w))

    def localize(self, now):
        """[LOCALIZE] AMCL에 initialpose(출발 지점)를 보내고 map 위치가 그 근처로 잡힐 때까지 기다린다.
        로봇을 움직이지 않는다 (지도에 '여기 있다' 핀만 꽂음). 로봇이 이미 출발 지점(도크 앞)에 있다는 전제.
        받음: now, (start, TF map→base_link)
        내보냄: initialpose 한 번 → 잡히면 state START_FACE, 위치를 아예 못 잡으면 end_tour(no_localization)"""
        p = self.p
        if self.start is None or not p['set_initial_pose']:
            return self.set_state('START_FACE')     # 초기 위치를 안 보냄 → 지금 AMCL 위치 그대로
        if self.loc_at is None:
            self.loc_at = now
        x, y, yaw = self.start
        if self.pose_sent is None:
            if self.pose_pub.get_subscription_count() == 0:     # AMCL이 아직 initialpose를 구독하지 않음
                self.dbg['reason'] = 'localize_wait_amcl'
                if now - self.loc_at > p['localize_timeout']:
                    self.end_tour('READY', 'no_localization')
                return
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = p['map_frame']
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.05                  # x, y 분산 [m^2]
            cov[35] = 0.05                          # yaw 분산 [rad^2]
            msg.pose.covariance = cov
            self.pose_pub.publish(msg)
            self.pose_sent = now
            self.get_logger().info(f'초기 위치 보냄 (map {x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        self.dbg['reason'] = 'localize'
        pose = self.lookup(p['map_frame'], p['base_frame'])
        near = pose is not None and math.hypot(pose[0] - x, pose[1] - y) < p['localize_tolerance']
        if near and now - self.pose_sent > 1.0:     # AMCL이 새 위치로 map→odom을 낼 시간
            self.get_logger().info('초기 위치 잡힘 → 그 자리에서 바라볼 곳(face_zone) 쪽으로 회전')
            return self.set_state('START_FACE')
        if now - self.loc_at > p['localize_timeout']:
            if pose is None:
                return self.end_tour('READY', 'no_localization')
            self.get_logger().warn(f'map 위치가 출발 지점에서 {math.hypot(pose[0] - x, pose[1] - y):.2f} m '
                                   '떨어져 있음 (AMCL 확인) → 그래도 바라볼 곳 쪽으로 회전')
            self.set_state('START_FACE')

    # ---------- 관람 시작 (start_tour 액션) ----------

    def on_goal(self, _goal):
        """[액션] start_tour goal이 오면 일단 다 받는다.
        여기서 거절하면 키오스크가 이유를 모르므로, 검사는 run_tour에서 하고 이유(message)와 함께 끝낸다.
        돌려줌: GoalResponse.ACCEPT"""
        return GoalResponse.ACCEPT                  # 받을 수 없는 goal은 run_tour에서 이유(message)와 함께 끝냄

    def goal_problem(self, goal):
        """이 goal로 관람을 시작할 수 없으면 이유를 돌려준다 (검사만 하고 아무것도 안 바꿈). lock 안에서 부른다.
        받음: goal (goal.request.mode), (active)
        돌려줌: '' (문제 없음) / 'busy' (이미 관람 중) / 'unsupported_mode' (FREE·"" 아님)"""
        if self.active:
            return 'busy'
        mode = (goal.request.mode if goal is not None else '').strip().upper()
        if mode not in ('', 'FREE'):
            return 'unsupported_mode'
        return ''

    def begin_tour(self):
        """관람 시작 준비: 지난 관람객 데이터(위치, 등록, 본 작품)를 지우고 active = True, 첫 상태로.
        바뀜: kf.x, seen, holding, 단계 타이머, registered, explaining, visited, seen_since, active, end_reason
              state → UNDOCK (undock 파라미터 false면 LOCALIZE)"""
        self.kf.x, self.seen, self.holding = None, -1e9, False
        self.face_until = self.loc_at = self.pose_sent = None
        self.active, self.end_reason = True, ''
        self.registered, self.explaining, self.visited = False, '', []
        self.seen_since = None
        self.get_logger().info('관람 시작 (FREE)')
        self.set_state('UNDOCK' if self.p['undock'] else 'LOCALIZE')

    def end_tour(self, state, reason):
        """관람 끝: 정지 → 관람객 데이터 삭제 → arti/tour_end 발행(다른 노드도 삭제) → state.
        받음: state   다음 상태 'READY' (취소·실패) 또는 'LOST' (분실)
              reason  끝난 이유 → 액션 result.message ('canceled', 'lost', 'undock_failed' ...)
        바뀜: active = False → run_tour 루프가 끝나 result를 보냄
        내보냄: cmd_vel 0, arti/tour_end, follow/state"""
        self.active, self.end_reason = False, reason
        self.drive(0.0, 0.0)
        self.kf.x, self.kf.P, self.seen, self.holding = None, None, -1e9, False
        self.tour_end_pub.publish(Empty())
        self.get_logger().info(f'관람 끝 ({reason}) → 관람객 데이터 삭제 (arti/tour_end)')
        self.set_state(state)

    def run_tour(self, goal):
        """[액션 실행, 관람 내내] start_tour goal 하나를 처음부터 끝까지 맡는다 (goal None = auto_start).
        검사(goal_problem) → begin_tour → undock → 0.2 s마다 feedback (취소 확인) → active가 내려가면 result.
        움직임은 step()이 하고, 여기서는 시작·끝과 키오스크 보고만 한다.
        받음: goal (mode, artwork_ids, is_cancel_requested)
        돌려줌: StartTour.Result(success = 취소로 끝남, message = 끝난 이유, visited_ids = 끝까지 설명한 작품)"""
        with self.lock:
            problem = self.goal_problem(goal)
            if not problem:
                self.begin_tour()
        if problem:
            mode = goal.request.mode if goal is not None else ''
            self.get_logger().warn(f'관람 시작 거절 ({problem}, mode "{mode}") — 이 로봇은 FREE만, 한 번에 한 관람')
            if goal is not None:
                goal.abort()
            return StartTour.Result(success=False, message=problem)
        if self.state == 'UNDOCK':
            self.send_feedback(goal)
            why = self.undock(goal)
            with self.lock:
                if why:
                    self.end_tour('READY', why)
                elif self.active:
                    self.set_state('LOCALIZE')
        while self.active:
            if goal is not None and goal.is_cancel_requested:
                with self.lock:
                    self.end_tour('READY', 'canceled')
                break
            self.send_feedback(goal)
            time.sleep(0.2)
        reason = self.end_reason
        result = StartTour.Result(success=reason == 'canceled', message=reason,
                                  visited_ids=[artwork_number(a) for a in self.visited])
        self.visited = []                           # 관람 기록은 result로 넘기고 지운다 (관람객 데이터 남기지 않음)
        self.get_logger().info(f'관람 결과: success={result.success}, message={reason}, '
                               f'visited_ids={list(result.visited_ids)}')
        if goal is not None:
            goal.canceled() if reason == 'canceled' else goal.abort()
        return result

    def send_feedback(self, goal):
        """내부 상태 → 키오스크용 feedback (StartTour.action 주석 참고).
        READY·UNDOCK·LOCALIZE·START_FACE → MOVING, WAIT_VISITOR → ARRIVED, FOLLOW·SEARCH·WAIT·LOST → FOLLOW, 설명 중 → EXPLAINING.
        받음: goal, (state, registered, explaining, visited, artwork_total)
        내보냄: feedback (state, registered, current_artwork_id, current_index, total)"""
        if goal is None:
            return
        explaining = self.explaining if self.state in ('FOLLOW', 'SEARCH', 'WAIT') else ''
        if explaining:
            state = 'EXPLAINING'
        elif self.state == 'WAIT_VISITOR':
            state = 'ARRIVED'
        elif self.state in ('FOLLOW', 'SEARCH', 'WAIT', 'LOST'):
            state = 'FOLLOW'
        else:                                       # READY·UNDOCK·LOCALIZE·START_FACE
            state = 'MOVING'
        goal.publish_feedback(StartTour.Feedback(
            state=state, registered=self.registered,
            current_artwork_id=artwork_number(explaining) if explaining else 0,
            current_index=len(self.visited), total=self.artwork_total))

    def wait_future(self, future, timeout, goal):
        """future(서비스·액션 응답)가 끝날 때까지 기다린다 (다른 스레드가 채움).
        받음: future, timeout [s], goal (취소되면 그만 기다림)
        돌려줌: 끝났으면 True, 시간 초과·취소면 False"""
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end or (goal is not None and goal.is_cancel_requested):
                return False
            time.sleep(0.1)
        return True

    def undock(self, goal):
        """도크에 붙어 있으면 Create 3에 undock 액션을 보내고 끝날 때까지 기다린다 (로봇이 스스로 움직임).
        받음: goal (취소 확인용), (docked: dock_status)
        돌려줌: '' (성공 또는 이미 도크 밖) / 'no_undock_server' / 'undock_failed' / 'canceled'"""
        if self.docked is False:
            self.get_logger().info('이미 도크 밖 → undock 건너뜀')
            return ''
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'undock 액션 서버 없음 ({self.get_namespace().rstrip("/")}/undock)')
            return 'no_undock_server'
        self.get_logger().info('undock 시작')
        sent = self.undock_client.send_goal_async(Undock.Goal())
        if not self.wait_future(sent, 5.0, goal) or not sent.result().accepted:
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'undock_failed'
        handle = sent.result()
        done = handle.get_result_async()
        if not self.wait_future(done, self.p['undock_timeout'], goal):
            handle.cancel_goal_async()
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'undock_failed'
        res = done.result()
        ok = (not res.result.is_docked) if res.status == GoalStatus.STATUS_SUCCEEDED else self.docked is False
        self.get_logger().info('undock 끝' if ok else f'undock 실패 (status {res.status})')
        return '' if ok else 'undock_failed'

    # ---------- 주행 ----------

    def obstacles(self):
        """지금 피해야 할 장애물 점 = 라이다 점 + depth_keep(1 s) 안의 바닥 물체 점.
        받음: (scan_pts, floor_pts, floor_time)
        돌려줌: 장애물 점 (N, 2) odom"""
        fresh = self.now() - self.floor_time < self.p['depth_keep']
        return np.concatenate([self.scan_pts] + ([self.floor_pts] if fresh else []))

    def dwa(self, pose, goal, face):
        """(v, w) 후보를 굴려 보고, 안 부딪히면서 목표까지 '돌아가는 거리'가 가장 줄어드는 것을 고른다 (DWA).
        ① 후보 105개 ② 1.5 s 굴려 봄 ③ 위험한 후보 버림 ④ 벌점 최소 — 단계별 입출력은 본문 주석.
        받음: pose (x, y, yaw) odom, goal 목표 (x, y), face 바라볼 곳 (x, y), (self.cmd: 지난 명령 = 지금 속도로 가정)
        돌려줌: (v [m/s], w [rad/s]). 모든 후보가 막혔으면 (0, face 쪽 회전)"""
        # ── 입력 풀기 ──────────────────────────────────────────────────────────────
        # pose      → x0, y0 [m], th0 [rad]   로봇 위치·방향 (odom)            예) 1.0, 0.0, 0.0
        # self.cmd  → v0 [m/s], w0 [rad/s]    지난번 보낸 명령 = 지금 속도로 가정  예) 0.20, 0.0
        p, (x0, y0, th0), (v0, w0) = self.p, pose, self.cmd
        dt = p['control_period']
        # ── ① 후보 만들기 ──────────────────────────────────────────────────────────
        # 입력: v0, w0, 가속 한계(linear_accel 0.5, angular_accel 3.0), 최대 속도(0.3, 1.5)
        # 출력: vs (7,)  0.1 s 안에 낼 수 있는 전진 속도 7개   예) 0.15 ~ 0.25
        #       ws (15,) 0.1 s 안에 낼 수 있는 회전 속도 15개  예) -0.3 ~ 0.3
        vs = np.linspace(max(0.0, v0 - p['linear_accel'] * dt),
                         min(p['max_linear'], v0 + p['linear_accel'] * dt), p['v_samples'])
        ws = np.linspace(max(-p['max_angular'], w0 - p['angular_accel'] * dt),
                         min(p['max_angular'], w0 + p['angular_accel'] * dt), p['w_samples'])
        # 출력: V, W (105,)  모든 조합을 한 줄로 편 것. 후보 i = (V[i], W[i])
        V, W = (g.ravel() for g in np.meshgrid(vs, ws))
        # ── ② 굴려 보기 (sim_time 1.5 s를 0.1 s씩 15칸) ─────────────────────────────
        # 입력: V, W, x0, y0, th0
        # 출력: th (105, 15)  후보별·시간별 방향 [rad]
        #       xs, ys (105, 15)  후보별·시간별 위치 [m]   행 = 후보, 열 = 0.1 ~ 1.5 s
        th = th0 + W[:, None] * np.arange(1, int(p['sim_time'] / 0.1) + 1) * 0.1
        xs = x0 + np.cumsum(V[:, None] * np.cos(th) * 0.1, 1)
        ys = y0 + np.cumsum(V[:, None] * np.sin(th) * 0.1, 1)

        # ── ③ 위험한 후보 버리기 ────────────────────────────────────────────────────
        # 입력: obstacles() → obs (N, 2)  라이다 + 바닥 물체 점 (odom)
        # 출력: near (M, 2)  로봇에서 1.05 m(0.3 × 1.5 + 0.6) 안의 점만 (먼 점은 궤적과 상관없어 계산 줄임)
        obs = self.obstacles()
        near = obs[np.hypot(obs[:, 0] - x0, obs[:, 1] - y0) < p['max_linear'] * p['sim_time'] + 0.6]
        # 출력: clear (105,)   후보별로 궤적 15점 중 장애물에 가장 가까이 간 거리 [m] (장애물 없으면 inf)
        #       here_clear     지금 로봇 위치에서 가장 가까운 장애물 거리 [m]
        clear, here_clear = np.full(V.size, np.inf), np.inf
        if len(near):
            clear = np.hypot(xs[..., None] - near[:, 0], ys[..., None] - near[:, 1]).min((1, 2))
            here_clear = np.hypot(near[:, 0] - x0, near[:, 1] - y0).min()
        # 여유를 지키는 경로, 또는 이미 붙어 있을 때 더 가까워지지 않는 경로(빠져나오기). 로봇 반지름 안은 금지
        # 출력: safe (105,) True/False  통과 조건 둘 중 하나
        #   - clear > 0.25 (로봇 반지름 0.2 + 여유 0.05): 여유 있게 지나감
        #   - 이미 벽에 붙어 있을 때: 몸(0.2)이 닿지 않고 지금보다 안 가까워짐 → 빠져나가는 길 (없으면 꼼짝 못 함)
        r = p['robot_radius']
        safe = (clear > r + p['safety_margin']) | ((clear > r) & (clear >= here_clear - 0.005))
        # 전부 위험 → 앞으로 안 가고 face(사람) 쪽으로 제자리 회전만. 출력: (0.0, w)  w = 1.5 × 어긋난 각도 (최대 ±1.5)
        if not safe.any():                          # 막힘 → 제자리에서 대상 쪽으로
            self.dbg['blocked'] = True
            turn = wrap(math.atan2(face[1] - y0, face[0] - x0) - th0)
            return 0.0, float(np.clip(1.5 * turn, -p['max_angular'], p['max_angular']))

        # ── ④ 점수(벌점) 매기고 고르기 — 작을수록 좋음 ─────────────────────────────
        # 입력: goal, 로봇 위치, obs
        # 출력: field (n, n)  칸마다 goal까지 장애물을 돌아가는 거리 [m],  origin 격자 원점 (x, y)
        t0 = time.monotonic()
        field, origin = self.field(np.asarray(goal, float), np.array([x0, y0]), obs)
        self.dbg['field_ms'] = (time.monotonic() - t0) * 1000
        # 출력: ex, ey, eth (105,)  후보별 1.5 s 뒤 도착 위치·방향 (② 결과의 마지막 열)
        ex, ey, eth = xs[:, -1], ys[:, -1], th[:, -1]
        # 출력: cost (105,)  후보별 벌점 =
        #   sample(field, 도착점)               도착점에서 goal까지 돌아가는 거리 [m]   (가장 중요)
        #   + 1.0  × max(0.4 - clear, 0)        장애물에 0.4 m 안으로 붙은 만큼
        #   + 0.02 × |W|                        회전 (비슷하면 직진)
        #   + (goal 0.3 m 안이면) 0.3 × |사람 쪽과 어긋난 각도|  사람을 바라보며 서게
        cost = self.sample(field, origin, ex, ey) \
            + p['w_clearance'] * np.clip(p['clearance_zone'] - clear, 0, None) \
            + p['w_turn'] * np.abs(W)                       # 비용이 같으면 덜 도는 쪽
        if math.hypot(goal[0] - x0, goal[1] - y0) < 0.3:   # 거의 도착 → 관람객 쪽을 바라봄
            cost += p['w_heading'] * np.abs(wrap(np.arctan2(face[1] - ey, face[0] - ex) - eth))
        cost[~safe] = np.inf
        # 출력: i  벌점이 가장 작은 후보 번호 (0 ~ 104). 위험한 후보는 cost = inf라 절대 안 뽑힘
        # 결과: (v [m/s], w [rad/s])  예) (0.25, 0.3) → drive()가 cmd_vel로 보냄. 다음 0.1 s만 쓰고 다시 계산
        i = int(np.argmin(cost))
        return float(V[i]), float(W[i])

    def field(self, goal, here, obs):
        """격자 칸마다 goal까지 장애물을 돌아가는 거리 [m] (dwa의 벌점용).
        10 cm 격자 → 장애물 칸을 로봇 반지름+여유만큼 부풀림 → 목표에서 BFS로 퍼져 나가며 거리 채움.
        받음: goal (x, y), here 로봇 (x, y), obs 장애물 점 (N, 2)
        돌려줌: (거리 지도 n×n, 지도 원점 (x, y)). 못 가는 칸은 50"""
        p = self.p
        res = p['grid_resolution']
        size = max(6.0, np.linalg.norm(goal - here) + 3.0)
        n, origin = int(size / res), (goal + here) / 2 - size / 2
        grid = np.zeros((n, n), np.uint8)
        c = np.floor((obs - origin) / res).astype(int)
        c = c[((c >= 0) & (c < n)).all(1)]
        grid[c[:, 1], c[:, 0]] = 1
        k = int(math.ceil((p['robot_radius'] + p['safety_margin']) / res))
        raw, grid = grid, cv2.dilate(grid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
        gx, gy = np.floor((goal - origin) / res).astype(int)
        rx, ry = np.floor((here - origin) / res).astype(int)
        grid[max(gy - 1, 0):gy + 2, max(gx - 1, 0):gx + 2] = 0      # 목표 자리는 열어 둔다
        win = np.s_[max(ry - k, 0):ry + k + 1, max(rx - k, 0):rx + k + 1]
        grid[win] = raw[win]                        # 로봇 주변은 실제 장애물 칸만 막음 (붙어 있어도 출발 가능)
        dist = np.full((n, n), np.inf)
        dist[gy, gx] = 0.0
        steps = [(dy, dx, math.hypot(dy, dx)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        frontier = [(gy, gx)]
        while frontier:                             # 목표에서부터 넓혀 가며 거리 채우기
            nxt = []
            for y, x in frontier:
                for dy, dx, w in steps:
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < n and 0 <= xx < n and not grid[yy, xx] and dist[y, x] + w < dist[yy, xx]:
                        dist[yy, xx] = dist[y, x] + w
                        nxt.append((yy, xx))
            frontier = nxt
        dist *= res
        dist[~np.isfinite(dist)] = 50.0
        return dist, origin

    def sample(self, field, origin, x, y):
        """거리 지도에서 점들의 값을 읽는다. 칸 사이는 주변 4칸을 섞어서(bilinear) → 후보끼리 점수가 같아지지 않게.
        받음: field 거리 지도, origin 지도 원점, x, y 점들 (배열)
        돌려줌: 점마다 거리 [m]"""
        n = field.shape[0]
        g = (np.stack([x, y], 1) - origin) / self.p['grid_resolution'] - 0.5
        i = np.clip(np.floor(g).astype(int), 0, n - 2)
        t = np.clip(g - i, 0.0, 1.0)
        f = [field[i[:, 1] + dy, i[:, 0] + dx] for dy in (0, 1) for dx in (0, 1)]
        return (f[0] * (1 - t[:, 0]) + f[1] * t[:, 0]) * (1 - t[:, 1]) + (f[2] * (1 - t[:, 0]) + f[3] * t[:, 0]) * t[:, 1]

    def ramp(self, v, w):
        """속도를 한 번에 바꾸지 않고 0.1 s에 바꿀 수 있는 만큼만 바꾼다 (덜컹 방지).
        감속 linear_decel, 가속 linear_accel, 회전 angular_accel 한도.
        받음: v, w  원하는 속도, (self.cmd: 지난 명령)
        돌려줌: (v, w)  이번에 보낼 속도"""
        #속도를 한번에 확 바꾸지 않고 조금씩 바꿔 주는 함
        p, (v0, w0) = self.p, self.cmd
        dt = p['control_period']
        dv = p['linear_decel'] if abs(v) < abs(v0) else p['linear_accel']
        v = v0 + float(np.clip(v - v0, -dv * dt, dv * dt))
        w = w0 + float(np.clip(w - w0, -p['angular_accel'] * dt, p['angular_accel'] * dt))
        return v, w

    def drive(self, v, w):
        """속도 명령을 로봇에 보낸다. enabled가 false면 보내지 않음. 보낸 값을 self.cmd에 기억 (ramp·dwa가 '지금 속도'로 씀).
        받음: v  전진 속도 [m/s], w  회전 속도 [rad/s] (+ 왼쪽)
        내보냄: cmd_vel (TwistStamped: twist.linear.x = v, twist.angular.z = w)"""
        self.cmd = (v, w)
        if not self.enabled:
            self.cmd = (0.0, 0.0)
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['base_frame']
        msg.twist.linear.x, msg.twist.angular.z = v, w
        self.cmd_pub.publish(msg)
        if self.diag is not None:
            t = time.monotonic()
            if self.last_cmd is not None:
                self.diag.add('cmd_vel.gap', t - self.last_cmd)
            self.last_cmd = t

    # ---------- 진단 (끊김 원인 찾기) ----------

    def timed(self, name, fn):
        """(diag) 구독 콜백을 감싸서 수신 간격, 지연(받은 시각 - header.stamp), 콜백 시간을 잰다. diag가 꺼져 있으면 그대로.
        받음: name 토픽 이름, fn 원래 콜백
        돌려줌: 감싼 콜백"""
        def cb(msg):
            if self.diag is None:
                return fn(msg)
            t0 = time.monotonic()
            if name in self.arrive:
                gap = t0 - self.arrive[name]
                self.diag.add(name + '.gap', gap)
                avg = self.gap_avg.get(name)
                if self.p['diag_events'] and avg is not None and gap > max(3 * avg, 0.25) and name != 'tf_static':
                    self.get_logger().warn(f'[늦음] {name}: {gap:.2f} s 만에 도착 (평소 {avg:.2f} s)')
                self.gap_avg[name] = gap if avg is None else 0.9 * avg + 0.1 * min(gap, 3 * avg)
            self.arrive[name] = t0
            stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
            if stamp is not None and (stamp.sec or stamp.nanosec):
                self.diag.add(name + '.lat', self.now() - (stamp.sec + stamp.nanosec * 1e-9))
            fn(msg)
            self.diag.add(name + '.cb', time.monotonic() - t0)
        return cb

    def record(self, t0):
        """(diag) 제어 한 주기를 진단 통계에 넣고, diag_events면 끊긴 순간을 바로 로그로.
        받음: t0  control() 시작 시각"""
        d, now, t1 = self.dbg, self.now(), time.monotonic()
        gap = t1 - self.last_tick if self.last_tick is not None else 0.0
        self.last_tick = t1
        step_ms = (t1 - t0) * 1000
        self.diag.add('control.gap', gap)
        self.diag.add('control.ms', step_ms)
        if 'field_ms' in d:
            self.diag.add('field.ms', d['field_ms'])
        reason = d.get('reason', '')
        if d.get('blocked'):
            reason += '+blocked'
        self.diag.add('reason:' + reason)
        if self.p['diag_events']:
            period = self.p['control_period']
            if gap > 1.5 * period:
                self.get_logger().warn(f'[제어 밀림] 주기 {gap:.3f} s (목표 {period}), 이번 계산 {step_ms:.1f} ms')
            if reason != self.last_reason:
                bad = reason.startswith(('no_pose', 'scan_timeout')) or 'blocked' in reason
                def ago(t):
                    return f'{now - t:.2f} s 전' if t > 0 else '아직 없음'
                self.say(bad, f'[이유] {self.last_reason} → {reason}  상태 {self.state}, '
                              f'v={self.cmd[0]:.2f} w={self.cmd[1]:.2f}, scan {ago(self.scan_time)}, '
                              f'좌표 {ago(self.seen)}')
        self.last_reason = reason

    def report(self):
        """(diag) diag_period(2 s)마다 요약 로그: 토픽별 수신 주기·지연, 제어 주기·계산 시간, 정지 이유.
        끊김과 관련 있어 보이는 값은 WARN으로."""
        s, span = self.diag.take()
        log, period = self.get_logger(), self.p['control_period']

        def stat(key, scale=1.0):
            n, total, mx, _ = s.get(key, (0, 0.0, 0.0, 0.0))
            return n, (total / n * scale if n else 0.0), mx * scale
        topics = sorted({k.rsplit('.', 1)[0] for k in s if k.endswith('.cb')})
        for name in topics:
            n, _, _ = stat(name + '.cb')
            _, _, gap = stat(name + '.gap')
            _, lat, lat_max = stat(name + '.lat')
            _, cb, cb_max = stat(name + '.cb', 1000)
            text = (f'[입력] {name}: {n / span:.1f} Hz, 간격 최대 {gap:.2f} s, 지연 평균/최대 {lat:.3f}/{lat_max:.3f} s, '
                    f'콜백 평균/최대 {cb:.1f}/{cb_max:.1f} ms')
            slow = gap > 3 * span / max(n, 1) or lat_max > 0.3 or cb_max > period * 1000 / 2
            self.say(slow, text)
            if name + '.lat' in s:                  # 가장 빨리 온 메시지의 지연 ≈ 두 컴퓨터 시계 차이 + 최소 전송 시간
                lat_min = s[name + '.lat'][3]
                if abs(lat_min) > 0.05:
                    ahead = '로봇 시계가 빠름 (stamp가 미래)' if lat_min < 0 else '로봇 시계가 느리거나 전송이 늦음'
                    self.say(True, f'[시간] {name}: 최소 지연 {lat_min * 1000:.0f} ms → {ahead}. '
                                   f'PC·로봇 시간 동기화(chrony/NTP) 확인')
        expected = ['scan', 'target_person/point_map', 'tf']
        if self.p['use_depth']:
            expected.append('oakd/stereo/image_raw/compressedDepth')
        for name in [t for t in expected if t not in topics]:
            log.warn(f'[입력] {name}: {span:.1f} s 동안 한 번도 안 옴')
        n, _, gap = stat('control.gap')
        _, ms, ms_max = stat('control.ms')
        _, fms, fms_max = stat('field.ms')
        text = (f'[제어] {n / span:.1f} Hz (목표 {1 / period:.0f}), 주기 최대 {gap:.3f} s, '
                f'계산 평균/최대 {ms:.1f}/{ms_max:.1f} ms (그중 field {fms:.1f}/{fms_max:.1f} ms)')
        self.say(gap > 1.5 * period or ms_max > 0.8 * period * 1000, text)
        nc, _, cgap = stat('cmd_vel.gap')
        reasons = {k.split(':', 1)[1]: v[0] for k, v in s.items() if k.startswith('reason:')}
        sent = f'cmd_vel 간격 최대 {cgap:.3f} s' if nc else 'cmd_vel 안 보냄 (enabled=false)'
        text = f'[출력] {sent}, 이유별 주기 수 {reasons}'
        bad = any(r.startswith(('no_pose', 'scan_timeout')) or 'blocked' in r for r in reasons)
        self.say(bad or cgap > 1.5 * period, text)
        tf = {k: v[0] for k, v in s.items() if k.startswith('tf_')}
        if tf:
            log.warn(f'[TF] 그 시각 값 없음(fallback)/실패 횟수 {tf}')

    def say(self, warn, text):
        # rclpy는 한 줄(호출 위치)에서 INFO/WARN을 섞어 쓰면 에러를 내서 따로 부른다
        """(diag) 로그 한 줄 출력.
        받음: warn  True면 WARN, 아니면 INFO
              text  문장"""
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def set_state(self, state):
        """상태를 바꾼다. 바뀔 때만 로그를 남기고 follow/state를 발행 (같은 상태로 또 불러도 아무것도 안 함).
        받음: state  'READY' / 'UNDOCK' / 'LOCALIZE' / 'START_FACE' / 'WAIT_VISITOR' / 'FOLLOW' / 'SEARCH' / 'WAIT' / 'LOST'
        내보냄: follow/state (latched) → artwork_zone 등"""
        if state != self.state:
            self.get_logger().info(f'상태 {self.state} → {state}')
            self.state = state
            self.state_pub.publish(String(data=state))


def main():
    # Ctrl+C 뒤에도 정지 명령을 보낼 수 있게 rclpy의 시그널 처리를 끈다
    """프로그램 시작점: 노드를 만들고 여러 스레드(MultiThreadedExecutor)로 돌린다
    (run_tour가 관람 내내 기다리는 동안에도 step·구독이 돌게). 끝날 때 정지 명령을 보낸다."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VisitorFollower()
    executor = MultiThreadedExecutor()              # 액션(관람)이 기다리는 동안에도 제어 주기·구독이 돌게
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C가 두 번 와도 정지 명령은 끝까지 보냄
    finally:
        node.active = False                         # 관람 액션 대기 루프 끝냄
        executor.shutdown(timeout_sec=1.0)
        node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
