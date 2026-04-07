"""
DSR Isaac Bridge Node
- dsr01/motion/move_joint : 관절각도 직접 지정 → Isaac Sim
- dsr01/motion/move_line  : Cartesian 포즈 → 수치 IK → Isaac Sim
- /joint_states (Isaac Sim) 구독 → dsr01/joint_states 재발행
"""

import rclpy
from rclpy.node import Node

import json
import math
import os
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from dsr_msgs2.srv import MoveJoint, MoveLine
from sensor_msgs.msg import JointState

import sys
import importlib.util as _ilu
import glob as _glob

_koras = None
_KORAS_AVAILABLE = False

# 현재 Python 버전에 맞는 .so만 로드
_PY_VER = f"python{sys.version_info.major}.{sys.version_info.minor}"
_SO_CANDIDATES = _glob.glob(os.path.expanduser(
    f"~/hye/isaac_bridge_ws/install/dsr_isaac_bridge"
    f"/lib/{_PY_VER}/site-packages/dsr_isaac_bridge/koras_kinematics*.so"
))
if _SO_CANDIDATES:
    try:
        _spec = _ilu.spec_from_file_location("koras_kinematics", _SO_CANDIDATES[0])
        _koras = _ilu.module_from_spec(_spec)
        _spec.loader.exec_module(_koras)
        _KORAS_AVAILABLE = True
    except Exception as _e:
        print(f"[WARN] koras_kinematics 로드 실패: {_e}")


ROBOT_ID    = "dsr01"
JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"
]

# 작업공간 Z 최소값 (충돌 방지), 단위: m
# 테스트용: 바닥 여유만 (드럼 충돌은 물리엔진이 처리)
WORKSPACE_Z_MIN = 0.05

# DH 파라미터 파일 경로
DH_PARAM_PATH = os.path.expanduser(
    "~/hye/hanyang_ui_ws/src/robot_control_config/"
    "controller_config/dynamics_param.json"
)


# ──────────────────────────────────────────────────────────────────────────────
# 기구학 유틸리티
# ──────────────────────────────────────────────────────────────────────────────

def _load_dh_params(path: str) -> dict:
    """dynamics_param.json에서 DH 파라미터 로드"""
    try:
        with open(path) as f:
            data = json.load(f)
        dh = data["dh"]
        return {
            "a":     [x * 0.001 for x in dh["a"]],      # mm → m
            "alpha": [math.radians(x) for x in dh["alpha"]],
            "d":     [x * 0.001 for x in dh["d"]],       # mm → m
            "theta": [math.radians(x) for x in dh["theta"]],  # offset
        }
    except Exception as e:
        # 로드 실패 시 M1013 공칭 DH 파라미터 사용 (mm → m)
        print(f"[WARN] DH param load failed ({e}), using nominal values")
        return {
            "a":     [0.0,    -0.6142, -0.5710, 0.0,   0.0,    0.0  ],
            "alpha": [math.pi/2, 0.0,  0.0, math.pi/2, -math.pi/2, 0.0],
            "d":     [0.1770,  0.0,    0.0,   0.1746,  0.1190, 0.1165],
            "theta": [0.0] * 6,
        }


def _load_koras_model(path: str):
    """
    dynamics_param.json을 읽어 RobotModel 인스턴스를 설정하고 반환한다.
    koras_kinematics 모듈이 없으면 None을 반환한다.
    """
    if not _KORAS_AVAILABLE:
        return None
    try:
        with open(path) as f:
            data = json.load(f)

        model = _koras.RobotModel()

        # DH 파라미터: [alpha, a, d, theta] 순서로 전달 (단위: deg, mm, mm, deg)
        dh = data["dh"]
        dh_list = [
            [math.radians(dh["alpha"][i]),  # alpha: deg→rad (C++에서 sin/cos 직접 사용)
             dh["a"][i],                    # a: mm
             dh["d"][i],                    # d: mm
             dh["theta"][i]]                # theta: 내부 미사용
            for i in range(6)
        ]
        model.setDH(dh_list)

        # 동역학 파라미터
        mass_g   = data["link_mass"]["value"]           # g
        mass_kg  = [m * 1e-3 for m in mass_g]           # kg

        com_mm   = data["center_of_mass"]["value"]       # mm (DH와 동일 단위, 변환 없음)
        com_m    = [list(row) for row in com_mm]

        inertia_g_mm2 = data["inertia"]["value"]         # g·mm²
        inertia_kg_m2 = [
            [v * 1e-3 for v in row] for row in inertia_g_mm2    # g·mm² → kg·mm²
        ]

        # 로터 관성 — json에 없으면 0으로 초기화
        inertia_rotor = data.get("inertia_rotor", {}).get("value", [0.0] * 6)

        model.setDynamicParameters(mass_kg, com_m, inertia_kg_m2, inertia_rotor)

        # 마찰 모델 — json에 있는 경우만 설정
        if "friction" in data:
            fr = data["friction"]
            coulomb  = fr.get("coulomb",  [0.0] * 6)
            deadzone = fr.get("deadzone", [0.0] * 6)
            n_elem   = fr.get("n_elem",   [1]   * 6)
            viscous_raw = fr.get("viscous", [[0.0] * 5] * 6)
            # viscous를 각 관절별 5-element list로 맞춘다
            viscous = []
            for row in viscous_raw:
                padded = list(row) + [0.0] * (5 - len(row))
                viscous.append(padded[:5])
            model.setFrictionModel(coulomb, viscous, deadzone, n_elem)

        print(f"[INFO] koras_kinematics RobotModel loaded from {path}")
        return model

    except Exception as e:
        print(f"[WARN] _load_koras_model failed ({e}), koras IK unavailable")
        return None


def _dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """표준 DH 변환 행렬 (4x4)"""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d     ],
        [0.0, 0.0,      0.0,     1.0   ]
    ])


def fk(q_rad: list, dh: dict) -> np.ndarray:
    """순기구학: 관절각도(rad) → 말단 포즈 (4x4 변환행렬)"""
    T = np.eye(4)
    for i in range(6):
        theta = q_rad[i] + dh["theta"][i]
        T = T @ _dh_matrix(dh["a"][i], dh["alpha"][i], dh["d"][i], theta)
    return T


def jacobian(q_rad: list, dh: dict) -> np.ndarray:
    """
    기하학적 야코비안 (6x6), 베이스 좌표계 기준
    robot_model.cpp calJacobian() 과 동일한 기하학적 야코비안
    J[:,i] = [z_{i-1} × (p_ee - p_{i-1})]  ← 선속도
             [z_{i-1}                     ]  ← 각속도
    """
    T = np.eye(4)
    frames = [T.copy()]           # frames[0] = 베이스 프레임
    for i in range(6):
        theta = q_rad[i] + dh["theta"][i]
        T = T @ _dh_matrix(dh["a"][i], dh["alpha"][i], dh["d"][i], theta)
        frames.append(T.copy())

    p_ee = frames[6][:3, 3]      # 말단 위치 (베이스 좌표계)

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = frames[i][:3, 2]   # 관절 i의 z축 (베이스 좌표계)
        p_i = frames[i][:3, 3]   # 관절 i의 원점 (베이스 좌표계)
        J[:3, i] = np.cross(z_i, p_ee - p_i)
        J[3:,  i] = z_i
    return J




# M1013 관절 범위 (deg → rad)
_Q_MIN = [math.radians(v) for v in [-360, -360, -360, -360, -360, -360]]
_Q_MAX = [math.radians(v) for v in [ 360,  360,  360,  360,  360,  360]]


def ik(target_T: np.ndarray, q_init: list, dh: dict,
       max_iter: int = 100, tol: float = 1e-6) -> tuple:
    """
    Levenberg-Marquardt 역기구학 + 관절 범위 클램핑
    반환: (q_rad[6], success)
    """
    q        = np.clip(np.array(q_init, dtype=float), _Q_MIN, _Q_MAX)
    lam      = 0.01
    R_d      = target_T[:3, :3]
    p_d      = target_T[:3, 3]
    err_prev = np.inf

    for _ in range(max_iter):
        T   = fk(q.tolist(), dh)
        R_c = T[:3, :3]
        p_c = T[:3, 3]

        pos_err = p_d - p_c
        rot_err = 0.5 * (
            np.cross(R_c[:, 0], R_d[:, 0]) +
            np.cross(R_c[:, 1], R_d[:, 1]) +
            np.cross(R_c[:, 2], R_d[:, 2])
        )

        err      = np.concatenate([pos_err, rot_err])
        err_norm = np.linalg.norm(err)

        if err_norm < tol:
            return q.tolist(), True

        J   = jacobian(q.tolist(), dh)
        JTJ = J.T @ J
        JTJ[np.diag_indices(6)] += lam

        dq  = np.linalg.solve(JTJ, J.T @ err)
        q   = np.clip(q + dq, _Q_MIN, _Q_MAX)   # 관절 범위 클램핑

        if err_norm < err_prev:
            lam /= 2.0
        else:
            lam *= 2.0
        err_prev = err_norm

    return q.tolist(), False


def pose_to_matrix(pos_m: list, rot_deg: list) -> np.ndarray:
    """
    [x,y,z](m) + [rx,ry,rz](deg) → 4x4 변환행렬
    두산 회전 규약: 고정축 ZYZ (extrinsic)
    """
    T = np.eye(4)
    T[:3, 3] = pos_m
    T[:3, :3] = Rotation.from_euler(
        "ZYZ", rot_deg, degrees=True
    ).as_matrix()
    return T


# ──────────────────────────────────────────────────────────────────────────────
# Bridge Node
# ──────────────────────────────────────────────────────────────────────────────

class DsrIsaacBridge(Node):

    def __init__(self):
        super().__init__("dsr_isaac_bridge")

        self._joint_pos_rad = [0.0] * 6
        self._lock = threading.Lock()
        self._dh   = _load_dh_params(DH_PARAM_PATH)
        self._koras_model = _load_koras_model(DH_PARAM_PATH)

        # Isaac Sim joint_states 구독
        self.sub_joint_states = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

        # Isaac Sim 관절 명령 발행
        self.pub_joint_cmd = self.create_publisher(
            JointState, "/joint_command", 10
        )

        # UI 서비스 수신
        self.srv_move_joint = self.create_service(
            MoveJoint, f"{ROBOT_ID}/motion/move_joint", self._on_move_joint
        )
        self.srv_move_line = self.create_service(
            MoveLine, f"{ROBOT_ID}/motion/move_line", self._on_move_line
        )

        # UI에 joint_states 발행
        self.pub_joint_states = self.create_publisher(
            JointState, f"{ROBOT_ID}/joint_states", 10
        )
        self.create_timer(0.05, self._publish_joint_states)

        # 충돌 감지 모니터 (Isaac Sim filewatcher → /tmp/isaac_collision.json)
        # 이전 세션 파일 초기화
        _col_file = "/tmp/isaac_collision.json"
        if os.path.exists(_col_file):
            os.remove(_col_file)
        self._last_collision_mtime = 0.0
        self.create_timer(0.05, self._monitor_collision)

        ik_backend = "koras C++ IK" if self._koras_model is not None else "Python LM IK"
        self.get_logger().info("DSR Isaac Bridge started")
        self.get_logger().info(f"  move_joint : {ROBOT_ID}/motion/move_joint")
        self.get_logger().info(f"  move_line  : {ROBOT_ID}/motion/move_line  [{ik_backend}]")

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self._joint_pos_rad[i] = msg.position[idx]

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        with self._lock:
            msg.position = [math.degrees(r) for r in self._joint_pos_rad]
        self.pub_joint_states.publish(msg)

    def _on_move_joint(self, request: MoveJoint.Request,
                       response: MoveJoint.Response):
        self.get_logger().info(
            f"move_joint: {[round(p,2) for p in request.pos]} deg  "
            f"vel={request.vel}"
        )
        target_rad = [math.radians(d) for d in request.pos]
        duration   = self._calc_duration(self._joint_pos_rad, target_rad, request.vel)
        self._move_to(target_rad, duration)
        response.success = True
        return response

    def _on_move_line(self, request: MoveLine.Request,
                      response: MoveLine.Response):
        """
        Cartesian 포즈 → 수치 IK → 관절각도 → Isaac Sim
        request.pos : [x, y, z, rx, ry, rz]  (mm, deg)
        request.vel : [linear mm/s, angular deg/s]
        """
        pos_m   = [p * 0.001 for p in request.pos[:3]]   # mm → m
        rot_deg = list(request.pos[3:6])

        self.get_logger().info(
            f"move_line: pos={[round(p,1) for p in request.pos[:3]]} mm  "
            f"rot={[round(r,1) for r in rot_deg]} deg"
        )

        # 바닥 충돌 사전 검사
        if pos_m[2] < WORKSPACE_Z_MIN:
            self.get_logger().error(
                f"move_line: 목표 Z={pos_m[2]*1000:.1f}mm 가 바닥 한계 "
                f"({WORKSPACE_Z_MIN*1000:.0f}mm) 미만 → 거부"
            )
            response.success = False
            return response

        target_T = pose_to_matrix(pos_m, rot_deg)

        with self._lock:
            q_init = list(self._joint_pos_rad)

        if self._koras_model is not None:
            # DSR 회전 규약(ZYZ extrinsic) → koras IK 입력 규약(ZYX/RPY: roll,pitch,yaw)으로 변환
            R_mat = Rotation.from_euler("ZYZ", rot_deg, degrees=True).as_matrix()
            rpy   = Rotation.from_matrix(R_mat).as_euler("xyz", degrees=True)  # [roll,pitch,yaw]
            x_target   = [p * 1000.0 for p in pos_m] + list(rpy)
            self.get_logger().info(
                f"  koras IK target: pos_mm={[round(v,1) for v in x_target[:3]]}  "
                f"ZYZ={[round(v,1) for v in rot_deg]} → RPY={[round(v,1) for v in rpy]}"
            )
            q_init_deg = [math.degrees(r) for r in q_init]
            q_sol_deg, success = self._koras_model.inverseKinematics(x_target, q_init_deg)
            if not success:
                # 초기각이 나쁠 경우 M1013 대표 자세로 재시도
                for seed_deg in [
                    [0.0,  -90.0,  90.0,  0.0,   0.0,  0.0],
                    [0.0,  -70.0, 110.0,  0.0, -40.0,  0.0],
                    [0.0,  -60.0,  90.0,  0.0, -30.0,  0.0],
                    [-10.0, -90.0, 90.0,  0.0,   0.0,  0.0],
                    [10.0,  -90.0, 90.0,  0.0,   0.0,  0.0],
                    [0.0,  -80.0, 130.0,  0.0, -50.0,  0.0],
                    [0.0,  -45.0,  60.0,  0.0, -15.0,  0.0],
                ]:
                    q_sol_deg, success = self._koras_model.inverseKinematics(
                        x_target, seed_deg)
                    if success:
                        break
            # koras IK는 관절 범위 비제한 → [-180, 180]으로 정규화
            if success:
                q_sol_deg = [((v + 180.0) % 360.0) - 180.0 for v in q_sol_deg]
            q_sol = [math.radians(d) for d in q_sol_deg]
        else:
            q_sol, success = ik(target_T, q_init, self._dh)
            if not success:
                for seed in [
                    [0.0, math.radians(-60), math.radians(90),  0.0, math.radians(-30), 0.0],
                    [0.0, math.radians(-45), math.radians(60),  0.0, math.radians(-15), 0.0],
                ]:
                    q_sol, success = ik(target_T, seed, self._dh)
                    if success:
                        break

        if not success:
            self.get_logger().warn("move_line: IK 수렴 실패 → 이동 취소")
            response.success = False
            return response

        # 속도: vel[0]=선속도(mm/s), vel[1]=각속도(deg/s) 중 큰 값 기준
        vel_deg = request.vel[1] if len(request.vel) > 1 and request.vel[1] > 0 else 30.0
        duration = self._calc_duration(q_init, q_sol, vel_deg)

        self.get_logger().info(
            f"move_line: IK sol={[round(math.degrees(q),1) for q in q_sol]} deg  "
            f"duration={duration:.2f}s  success={success}"
        )

        self._move_to(q_sol, duration)
        response.success = True
        return response

    def _monitor_collision(self):
        """Isaac Sim 충돌 파일 감시 → ROS2 로그"""
        path = "/tmp/isaac_collision.json"
        if not os.path.exists(path):
            return
        try:
            mtime = os.path.getmtime(path)
            if mtime <= self._last_collision_mtime:
                return
            self._last_collision_mtime = mtime
            with open(path) as f:
                data = json.load(f)
            jidx    = data.get("joint", -1)
            pos_err = data.get("pos_error", [])
            thr     = data.get("threshold_deg", 8.0)
            self.get_logger().error(
                f"[COLLISION] joint_{jidx+1} 충돌 감지  "
                f"pos_err={[round(e,2) for e in pos_err]} deg  "
                f"threshold={thr} deg"
            )
        except Exception:
            pass

    # ── 헬퍼 ──────────────────────────────────────────────────────────────────

    def _move_to(self, target_rad: list, duration_sec: float):
        """목표 각도 + duration → JSON 파일 → Isaac Sim 보간"""
        cmd = {"target": target_rad, "duration": duration_sec}
        with open("/tmp/isaac_joint_cmd.json", "w") as f:
            json.dump(cmd, f)
        time.sleep(duration_sec)

    def _calc_duration(self, current_rad: list, target_rad: list,
                       vel_deg_per_sec: float) -> float:
        if vel_deg_per_sec <= 0:
            vel_deg_per_sec = 30.0
        max_delta = max(
            abs(math.degrees(t - c))
            for t, c in zip(target_rad, current_rad)
        )
        return max(max_delta / vel_deg_per_sec, 0.5)


def main():
    rclpy.init()
    node = DsrIsaacBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
