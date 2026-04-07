"""
Isaac Sim 위치 드라이브 기반 로봇 제어

제어 구조:
    set_joint_position_targets(q_d)
    → Isaac Sim 내부 Drive(Kp, Kd)가 자동으로 토크 생성
    → PhysX가 중력/마찰/충돌 물리 처리

충돌 감지:
    장애물 충돌 → 로봇 속도 ≈ 0 이면서 위치 오차 지속
    → 정지 + /tmp/isaac_collision.json 기록
"""

import json
import math
import os
import time
import numpy as np

import omni.kit.app
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

CMD_FILE       = "/tmp/isaac_joint_cmd.json"
COLLISION_FILE = "/tmp/isaac_collision.json"


class FileWatcher:

    # 이동 시작 직후 충돌 감지 스킵 프레임 (궤도 초기 가속 구간)
    COLLISION_COOLDOWN_FRAMES = 30

    # 충돌 판정: 위치 오차 大 + 속도 ≈ 0 이 N프레임 지속
    POS_ERR_THRESH_DEG   = 15.0   # 위치 오차 임계값 (deg)
    VEL_STUCK_THRESH_DEG = 1.0    # "정지" 판정 속도 (deg/s)
    POS_ERR_FRAMES_LIMIT = 15     # 지속 프레임 수

    def __init__(self):
        self._robot       = None
        self._last_mtime  = 0
        self._q_start     = None
        self._q_target    = None
        self._q_d         = None
        self._duration    = 0.0
        self._elapsed     = 0.0
        self._moving      = False
        self._collision   = False
        self._cooldown    = 0
        self._pos_err_cnt = 0
        self._q_hold      = None   # 모션 완료 후 유지할 위치

    # M1013 위치 드라이브 게인 (PhysX 직접 설정)
    # 순서: J1, J2, J3, J4, J5, J6  단위: Nm/rad, Nm·s/rad
    _KP = np.array([[10000, 15000, 12000, 5000, 5000, 3000]], dtype=np.float64)
    _KD = np.array([[500,   800,   600,   100,  100,  50  ]], dtype=np.float64)

    def _init_robot(self):
        try:
            robot = SingleArticulation("/World/m1013")
            robot.initialize()

            # ── PhysX에 직접 게인 설정 ───────────────────────────
            # USD 속성이 아닌 ArticulationView를 통해 PhysX 씬에 직접 반영
            try:
                if hasattr(robot, 'set_gains'):
                    robot.set_gains(kps=self._KP, kds=self._KD)
                    print(f"[OK] set_gains (direct)")
                elif hasattr(robot, '_articulation_view'):
                    robot._articulation_view.set_gains(
                        kps=self._KP, kds=self._KD)
                    print(f"[OK] set_gains (_articulation_view)")
                else:
                    print("[WARN] set_gains API 없음 → 기존 USD 게인 사용")
                print(f"     kp={self._KP.tolist()}  kd={self._KD.tolist()}")
            except Exception as e:
                print(f"[WARN] set_gains 실패: {e}")

            self._robot = robot
            print("[OK] Robot ready (position drive control)")
        except Exception as e:
            print(f"[WARN] Robot init failed: {e}")
            self._robot = None

    def _reset_robot(self):
        self._robot   = None
        self._moving  = False
        self._q_start = None
        print("[WARN] Robot reset (simulation view invalidated)")

    # ── 충돌 감지 ─────────────────────────────────────────────────
    def _check_collision(self, q_d, q_rad, qd_deg):
        """
        위치 오차 大 + 로봇 속도 ≈ 0 → 충돌 (장애물에 막힘)
        위치 오차 大 + 로봇 속도 ≠ 0 → 추적 지연 (정상)
        """
        max_err = 0.0
        worst_j = 0
        for i in range(6):
            err = abs(math.degrees(q_d[i] - q_rad[i]))
            if err > max_err:
                max_err = err
                worst_j = i

        if max_err < self.POS_ERR_THRESH_DEG:
            return False, worst_j

        max_vel = max(abs(v) for v in qd_deg)
        return max_vel < self.VEL_STUCK_THRESH_DEG, worst_j

    def _report_collision(self, joint_idx, pos_err_deg):
        data = {
            "collision":     True,
            "joint":         joint_idx,
            "pos_error":     [round(e, 3) for e in pos_err_deg],
            "threshold_deg": self.POS_ERR_THRESH_DEG,
            "timestamp":     time.time(),
        }
        with open(COLLISION_FILE, "w") as f:
            json.dump(data, f)
        print(f"[COLLISION] joint_{joint_idx+1} 충돌  "
              f"err={[round(e,2) for e in pos_err_deg]} deg")

    # ── 메인 루프 ──────────────────────────────────────────────────
    def on_update(self, e):
        dt = e.payload["dt"]

        if self._robot is None:
            self._init_robot()
            return

        # ── 현재 상태 읽기 ────────────────────────────────────────
        try:
            q_rad  = self._robot.get_joint_positions().tolist()
            qd_rad = self._robot.get_joint_velocities().tolist()
        except Exception as ex:
            if "invalidated" in str(ex):
                self._reset_robot()
            return

        qd_deg = [math.degrees(v) for v in qd_rad]

        # ── 새 명령 확인 ──────────────────────────────────────────
        if os.path.exists(CMD_FILE):
            try:
                mtime = os.path.getmtime(CMD_FILE)
                if mtime > self._last_mtime:
                    self._last_mtime = mtime
                    with open(CMD_FILE) as f:
                        cmd = json.load(f)
                    self._q_start   = q_rad[:]
                    self._q_target  = cmd["target"]   # rad
                    self._q_hold    = cmd["target"][:]  # 완료 후 유지 위치
                    self._duration  = max(cmd["duration"], 0.1)
                    self._elapsed   = 0.0
                    self._moving    = True
                    self._collision = False
                    self._cooldown  = self.COLLISION_COOLDOWN_FRAMES
                    self._pos_err_cnt = 0
                    if os.path.exists(COLLISION_FILE):
                        os.remove(COLLISION_FILE)
            except Exception:
                pass

        # ── 목표 궤적 계산 (5차 다항식 보간) ─────────────────────
        if self._moving and self._q_target is not None:
            self._elapsed += dt
            t = min(self._elapsed / self._duration, 1.0)

            t3, t4, t5 = t**3, t**4, t**5
            s = 10*t3 - 15*t4 + 6*t5

            q_d = [self._q_start[i] + (self._q_target[i] - self._q_start[i]) * s
                   for i in range(6)]

            if t >= 1.0:
                self._moving = False
                q_d = self._q_target[:]
        else:
            # 정지: 마지막 명령 위치 유지 (현재 위치 따라가면 중력에 의해 낙하)
            q_d = self._q_hold[:] if self._q_hold else q_rad[:]

        # ── 충돌 감지 ─────────────────────────────────────────────
        if self._cooldown > 0:
            self._cooldown -= 1

        if self._moving and not self._collision and self._cooldown == 0:
            blocked, jidx = self._check_collision(q_d, q_rad, qd_deg)
            if blocked:
                self._pos_err_cnt += 1
                if self._pos_err_cnt >= self.POS_ERR_FRAMES_LIMIT:
                    pos_err = [math.degrees(q_d[i] - q_rad[i]) for i in range(6)]
                    self._collision   = True
                    self._moving      = False
                    self._pos_err_cnt = 0
                    self._report_collision(jidx, pos_err)
            else:
                self._pos_err_cnt = 0

        # ── 목표 위치 적용 ────────────────────────────────────────
        target = q_rad if self._collision else q_d
        try:
            # ArticulationAction: Isaac Sim 표준 제어 API
            # set_joint_position_targets 대신 사용 (드라이브 모드 자동 처리)
            self._robot.apply_action(
                ArticulationAction(
                    joint_positions=np.array(target, dtype=np.float64)
                )
            )
        except Exception as ex:
            if "invalidated" in str(ex):
                self._reset_robot()
            else:
                # fallback: 직접 위치 설정 (물리 없이 텔레포트)
                try:
                    self._robot.set_joint_positions(np.array(target, dtype=np.float64))
                except Exception:
                    pass


# ── 실행 ───────────────────────────────────────────────────────────
watcher = FileWatcher()
sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    watcher.on_update, name="joint_cmd_watcher"
)
print("FileWatcher ready - position drive control + collision detection")

