"""
DSR Isaac Bridge Node
- dsr01/motion/move_joint  서비스 수신 → /joint_command 토픽 발행 → Isaac Sim
- dsr01/motion/move_line   서비스 수신 → IK 미구현
- /joint_states (Isaac Sim) 구독 → dsr01/joint_states 재발행
"""

import rclpy
from rclpy.node import Node

import math
import threading
import time

from dsr_msgs2.srv import MoveJoint, MoveLine
from sensor_msgs.msg import JointState


ROBOT_ID    = "dsr01"
JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"
]


class DsrIsaacBridge(Node):

    def __init__(self):
        super().__init__("dsr_isaac_bridge")

        # 현재 관절 각도 (rad) - Isaac Sim으로부터 수신
        self._joint_pos_rad = [0.0] * 6
        self._lock = threading.Lock()

        # ── Isaac Sim joint_states 구독 ──
        self.sub_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10
        )

        # ── Isaac Sim으로 관절 명령 발행 ──
        self.pub_joint_cmd = self.create_publisher(
            JointState,
            "/joint_command",
            10
        )

        # ── UI로부터 move_joint 서비스 수신 ──
        self.srv_move_joint = self.create_service(
            MoveJoint,
            f"{ROBOT_ID}/motion/move_joint",
            self._on_move_joint
        )

        # ── UI로부터 move_line 서비스 수신 ──
        self.srv_move_line = self.create_service(
            MoveLine,
            f"{ROBOT_ID}/motion/move_line",
            self._on_move_line
        )

        # ── UI에 joint_states 발행 ──
        self.pub_joint_states = self.create_publisher(
            JointState,
            f"{ROBOT_ID}/joint_states",
            10
        )

        self.create_timer(0.05, self._publish_joint_states)  # 20Hz

        self.get_logger().info("DSR Isaac Bridge started")
        self.get_logger().info(f"  move_joint service: {ROBOT_ID}/motion/move_joint")
        self.get_logger().info(f"  joint_command topic: /joint_command")

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

    def _on_move_joint(self, request: MoveJoint.Request, response: MoveJoint.Response):
        self.get_logger().info(
            f"move_joint: {[round(p,2) for p in request.pos]} deg  "
            f"vel={request.vel} acc={request.acc}"
        )

        target_rad = [math.radians(d) for d in request.pos]
        duration_sec = self._calc_duration(self._joint_pos_rad, target_rad, request.vel)

        self._move_to(target_rad, duration_sec)
        response.success = True
        return response

    def _on_move_line(self, request: MoveLine.Request, response: MoveLine.Response):
        self.get_logger().warn("move_line: IK not implemented yet. Skipping.")
        response.success = True
        return response

    def _move_to(self, target_rad: list, duration_sec: float):
        """목표 각도와 duration을 파일로 전달 → Isaac Sim에서 보간"""
        import json
        cmd = {"target": target_rad, "duration": duration_sec}
        with open("/tmp/isaac_joint_cmd.json", "w") as f:
            json.dump(cmd, f)
        time.sleep(duration_sec)

    def _calc_duration(self, current_rad, target_rad, vel_deg_per_sec) -> float:
        if vel_deg_per_sec <= 0:
            vel_deg_per_sec = 30.0
        max_delta_deg = max(
            abs(math.degrees(t - c))
            for t, c in zip(target_rad, current_rad)
        )
        return max(max_delta_deg / vel_deg_per_sec, 0.5)


def main():
    rclpy.init()
    node = DsrIsaacBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
