"""onnx_nav_target_tracker.py

役割:
- `/goal_pose` と `/amcl_pose` と `/clicked_point` の状態管理を担当
- ウェイポイント到達判定と次ターゲット決定を担当
- 行動モデル向けベクトル観測（角度・距離）の計算を担当
"""

import math
from typing import Any

import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class TargetTracker:
    def __init__(self, vec_obs_dim: int, waypoint_reach_threshold_m: float, initial_now_ns: int) -> None:
        self.vec_obs_dim = vec_obs_dim
        self.waypoint_reach_threshold_m = waypoint_reach_threshold_m

        self.goal_xy: np.ndarray | None = None
        self.robot_xyyaw: np.ndarray | None = None
        self.waypoints_xy: list[np.ndarray] = []
        self.current_waypoint_index = 0

        self._warned_goal_frame_mismatch = False
        self._warned_amcl_frame_mismatch = False
        self._warned_clicked_point_frame_mismatch = False

        self.goal_count = 0
        self.amcl_count = 0
        self.last_goal_stamp = None
        self.last_amcl_stamp = None
        self._last_goal_log_ns = initial_now_ns
        self._last_amcl_log_ns = initial_now_ns

    def handle_goal_pose(
        self,
        msg: PoseStamped,
        logger: Any,
        *,
        goal_pose_topic: str,
        debug: bool,
        log_period_sec: float,
        now_ns: int,
    ) -> None:
        frame = (msg.header.frame_id or '').strip()
        if frame != 'map':
            if not self._warned_goal_frame_mismatch:
                logger.warn(f"Ignoring {goal_pose_topic} frame_id='{frame}' (expected: 'map').")
                self._warned_goal_frame_mismatch = True
            return

        goal_x = float(msg.pose.position.x)
        goal_y = float(msg.pose.position.y)
        self.goal_xy = np.array([goal_x, goal_y], dtype=np.float32)
        self.last_goal_stamp = msg.header.stamp
        self.goal_count += 1

        if debug and (
            self.goal_count == 1
            or (now_ns - self._last_goal_log_ns) / 1e9 >= log_period_sec
        ):
            logger.info(
                f"goal updated #{self.goal_count}: frame_id='{frame}', "
                f'goal=({goal_x:.3f},{goal_y:.3f}), '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            )
            self._last_goal_log_ns = now_ns

    def handle_amcl_pose(
        self,
        msg: PoseWithCovarianceStamped,
        logger: Any,
        *,
        amcl_pose_topic: str,
        debug: bool,
        log_period_sec: float,
        now_ns: int,
    ) -> None:
        frame = (msg.header.frame_id or '').strip()
        if frame != 'map':
            if not self._warned_amcl_frame_mismatch:
                logger.warn(f"Ignoring {amcl_pose_topic} frame_id='{frame}' (expected: 'map').")
                self._warned_amcl_frame_mismatch = True
            return

        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        self.robot_xyyaw = np.array([px, py, yaw], dtype=np.float32)
        self.last_amcl_stamp = msg.header.stamp
        self.amcl_count += 1

        if debug and (
            self.amcl_count == 1
            or (now_ns - self._last_amcl_log_ns) / 1e9 >= log_period_sec
        ):
            logger.info(
                f"amcl updated #{self.amcl_count}: frame_id='{frame}', "
                f'pose=({px:.3f},{py:.3f},{yaw:.3f}), '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            )
            self._last_amcl_log_ns = now_ns

    def handle_clicked_point(self, msg: PointStamped, logger: Any, *, clicked_point_topic: str) -> None:
        frame = (msg.header.frame_id or '').strip()
        if frame != 'map':
            if not self._warned_clicked_point_frame_mismatch:
                logger.warn(
                    f"Ignoring {clicked_point_topic} frame_id='{frame}' (expected: 'map')."
                )
                self._warned_clicked_point_frame_mismatch = True
            return

        x = float(msg.point.x)
        y = float(msg.point.y)
        self.waypoints_xy.append(np.array([x, y], dtype=np.float32))

        total = len(self.waypoints_xy)
        remaining = max(0, total - self.current_waypoint_index)
        logger.info(f'waypoint added: ({x:.3f},{y:.3f}), total={total}, remaining={remaining}')

    @staticmethod
    def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(math.atan2(siny_cosp, cosy_cosp))

    @staticmethod
    def wrap_to_pi(angle: float) -> float:
        wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
        return float(wrapped)

    @staticmethod
    def wrap_to_180_deg(angle_deg: float) -> float:
        wrapped = (angle_deg + 180.0) % 360.0 - 180.0
        return float(wrapped)

    def _advance_waypoint_if_reached(self, logger: Any) -> None:
        if self.robot_xyyaw is None:
            return

        rx = float(self.robot_xyyaw[0])
        ry = float(self.robot_xyyaw[1])

        while self.current_waypoint_index < len(self.waypoints_xy):
            waypoint = self.waypoints_xy[self.current_waypoint_index]
            dx = float(waypoint[0]) - rx
            dy = float(waypoint[1]) - ry
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > self.waypoint_reach_threshold_m:
                break

            reached_index = self.current_waypoint_index + 1
            self.current_waypoint_index += 1
            remaining = max(0, len(self.waypoints_xy) - self.current_waypoint_index)
            logger.info(f'waypoint reached: index={reached_index}, dist={dist:.3f} m, remaining={remaining}')

    def get_target_xy(self, logger: Any) -> tuple[np.ndarray | None, str]:
        self._advance_waypoint_if_reached(logger)

        if self.current_waypoint_index < len(self.waypoints_xy):
            return self.waypoints_xy[self.current_waypoint_index], 'waypoint'
        return self.goal_xy, 'goal'

    def compute_goal_vector(self, logger: Any) -> np.ndarray:
        if self.goal_xy is None or self.robot_xyyaw is None:
            return np.zeros((1, self.vec_obs_dim), dtype=np.float32)

        target_xy, _ = self.get_target_xy(logger)
        if target_xy is None:
            return np.zeros((1, self.vec_obs_dim), dtype=np.float32)

        goal_x, goal_y = float(target_xy[0]), float(target_xy[1])
        rx, ry, yaw = (
            float(self.robot_xyyaw[0]),
            float(self.robot_xyyaw[1]),
            float(self.robot_xyyaw[2]),
        )

        dx = goal_x - rx
        dy = goal_y - ry
        dist = math.sqrt(dx * dx + dy * dy)

        goal_heading = math.atan2(dy, dx)
        signed_rad = self.wrap_to_pi(goal_heading - yaw)
        signed_deg = self.wrap_to_180_deg(math.degrees(signed_rad))

        base = np.array([[signed_deg, dist]], dtype=np.float32)
        if self.vec_obs_dim == 2:
            return base
        if self.vec_obs_dim < 2:
            return base[:, : self.vec_obs_dim]
        return np.pad(base, ((0, 0), (0, self.vec_obs_dim - 2)), constant_values=0.0)

    def remaining_waypoints(self) -> int:
        return max(0, len(self.waypoints_xy) - self.current_waypoint_index)
