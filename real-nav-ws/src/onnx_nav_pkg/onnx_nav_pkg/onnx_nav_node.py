import math
import os
import json
from datetime import datetime
from pathlib import Path
from collections import deque

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class RealOnnxNavNode(Node):
    def __init__(self) -> None:
        super().__init__('real_onnx_nav_node')

        self.bridge = CvBridge()

        # ゴール/自己位置キャッシュ（map frame前提）
        self.goal_xy = None
        self.robot_xyyaw = None
        self._warned_goal_frame_mismatch = False
        self._warned_amcl_frame_mismatch = False
        self.goal_count = 0
        self.amcl_count = 0
        self._warned_no_goal = False
        self._warned_no_amcl = False
        self.last_goal_stamp = None
        self.last_amcl_stamp = None
        self._last_goal_log_time = self.get_clock().now()
        self._last_amcl_log_time = self.get_clock().now()

        # clicked_point で追加するウェイポイント列（map frame前提）
        self.waypoints_xy = []
        self.current_waypoint_index = 0
        self._warned_clicked_point_frame_mismatch = False

        # ノード設定パラメータ
        self.declare_parameter('debug', False)
        self.declare_parameter('log_model_io', False)
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('write_model_io_file', False)
        self.declare_parameter('model_io_log_dir', 'logs/model_io')
        self.declare_parameter('model_io_log_every_n', 1)
        self.declare_parameter('image_topic', '/cb_img')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('action_topic', '/agent/cmd')
        self.declare_parameter('max_inference_hz', 10.0)
        self.declare_parameter('model_file_name', 'balance.onnx')
        self.declare_parameter('img_width', 112)
        self.declare_parameter('img_height', 84)
        self.declare_parameter('stack_size', 5)
        self.declare_parameter('vec_obs_dim', 2)
        self.declare_parameter('waypoint_reach_threshold_m', 0.6)

        self.debug = bool(self.get_parameter('debug').value)
        self.log_model_io = bool(self.get_parameter('log_model_io').value)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.write_model_io_file = bool(self.get_parameter('write_model_io_file').value)
        self.model_io_log_dir = str(self.get_parameter('model_io_log_dir').value)
        self.model_io_log_every_n = int(self.get_parameter('model_io_log_every_n').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.goal_pose_topic = str(self.get_parameter('goal_pose_topic').value)
        self.clicked_point_topic = str(self.get_parameter('clicked_point_topic').value)
        self.amcl_pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.action_topic = str(self.get_parameter('action_topic').value)
        self.max_inference_hz = float(self.get_parameter('max_inference_hz').value)
        self.img_width = int(self.get_parameter('img_width').value)
        self.img_height = int(self.get_parameter('img_height').value)
        self.stack_size = int(self.get_parameter('stack_size').value)
        self.vec_obs_dim = int(self.get_parameter('vec_obs_dim').value)
        self.waypoint_reach_threshold_m = float(self.get_parameter('waypoint_reach_threshold_m').value)

        if self.max_inference_hz < 0.0:
            raise ValueError(f'max_inference_hz must be >= 0.0: {self.max_inference_hz}')

        self._min_infer_period_sec = (1.0 / self.max_inference_hz) if self.max_inference_hz > 0.0 else 0.0
        self._last_infer_accept_ns = 0

        if self.img_width <= 0 or self.img_height <= 0:
            raise ValueError(f'img_width/img_height must be > 0: ({self.img_width}, {self.img_height})')
        if self.stack_size <= 0:
            raise ValueError(f'stack_size must be > 0: {self.stack_size}')
        if self.vec_obs_dim <= 0:
            raise ValueError(f'vec_obs_dim must be > 0: {self.vec_obs_dim}')
        if self.waypoint_reach_threshold_m <= 0.0:
            raise ValueError(
                f'waypoint_reach_threshold_m must be > 0.0: {self.waypoint_reach_threshold_m}'
            )
        if self.model_io_log_every_n <= 0:
            raise ValueError(f'model_io_log_every_n must be > 0: {self.model_io_log_every_n}')

        # フレームスタックバッファ（最新 stack_size 枚）
        self.frame_buffer = deque(maxlen=self.stack_size)
        self._infer_count = 0
        self._last_log_time = self.get_clock().now()
        self._file_log_count = 0
        self._file_log_path = None

        # 入力トピック（画像/ゴール/自己位置）
        self.sub_cam = self.create_subscription(
            Image,
            self.image_topic,
            self.cb_cam,
            qos_profile_sensor_data,
        )
        self.sub_goal_pose = self.create_subscription(PoseStamped, self.goal_pose_topic, self.cb_goal_pose, 10)
        self.sub_clicked_point = self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self.cb_clicked_point,
            10,
        )
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.cb_amcl_pose,
            qos_profile_sensor_data,
        )

        # 出力トピック（行動/デバッグ画像）
        self.pub_act = self.create_publisher(Float32MultiArray, self.action_topic, 10)
        self.pub_debug_stack = self.create_publisher(Image, '/debug/stacked_image', 10)

        # ONNXモデルを share/onnx_nav_pkg/models からロード
        share_dir = get_package_share_directory('onnx_nav_pkg')
        model_file_name = str(self.get_parameter('model_file_name').value or '').strip()
        if not model_file_name:
            model_file_name = 'balance.onnx'
        model_path = os.path.join(share_dir, 'models', model_file_name)
        self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

        self.inputs = self.session.get_inputs()
        self.input_names = [i.name for i in self.inputs]
        self.input_shapes = [i.shape for i in self.inputs]
        self.get_logger().info(f'ONNX inputs: {list(zip(self.input_names, self.input_shapes))}')

        self.declare_parameter('action_output_name', '')
        self.outputs = self.session.get_outputs()
        self.output_names = [o.name for o in self.outputs]
        self.output_shapes = [o.shape for o in self.outputs]
        self.get_logger().info(f'ONNX outputs: {list(zip(self.output_names, self.output_shapes))}')

        # 出力候補から continuous action を優先選択
        preferred = ['continuous_actions', 'deterministic_continuous_actions']
        param_action = str(self.get_parameter('action_output_name').value or '').strip()
        if param_action:
            if 'discrete' in param_action:
                raise RuntimeError(
                    'discrete action outputs are not supported. '
                    f"requested action_output_name='{param_action}'"
                )
            if param_action not in self.output_names:
                raise RuntimeError(
                    f"action_output_name='{param_action}' is not in model outputs. "
                    f'available_outputs={self.output_names}'
                )
            self.action_output_name = param_action
        else:
            self.action_output_name = next((n for n in preferred if n in self.output_names), None)
            if self.action_output_name is None:
                raise RuntimeError(
                    f'action output not found. preferred={preferred}, '
                    f'available_outputs={self.output_names}'
                )

        if 'discrete' in self.action_output_name:
            raise RuntimeError(
                'discrete action outputs are not supported. '
                f"selected action_output_name='{self.action_output_name}', "
                f'available_outputs={self.output_names}'
            )

        self.get_logger().info(
            f'action_output_name={self.action_output_name}, available_outputs={self.output_names}'
        )

        if self.write_model_io_file:
            self._init_model_io_log_file()

        self.get_logger().info(f'Subscribed image topic: {self.image_topic}')
        self.get_logger().info(f'Subscribed goal topic: {self.goal_pose_topic}')
        self.get_logger().info(f'Subscribed waypoint topic: {self.clicked_point_topic}')
        self.get_logger().info(f'Subscribed amcl topic: {self.amcl_pose_topic}')
        self.get_logger().info(f'Published action topic: {self.action_topic}')
        self.get_logger().info(f'max_inference_hz={self.max_inference_hz:.3f} (0.0 means unlimited)')

    def _init_model_io_log_file(self) -> None:
        log_dir = Path(self.model_io_log_dir).expanduser()
        if not log_dir.is_absolute():
            log_dir = Path.cwd() / log_dir

        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._file_log_path = log_dir / f'model_io_{timestamp}.jsonl'

        start_record = {
            'event': 'start',
            'node': self.get_name(),
            'action_output_name': self.action_output_name,
            'input_names': self.input_names,
            'output_names': self.output_names,
        }

        with self._file_log_path.open('w', encoding='utf-8') as fp:
            fp.write(f'{json.dumps(start_record, ensure_ascii=False)}\n')

        self.get_logger().info(f'model_io file logging enabled: {self._file_log_path}')

    @staticmethod
    def _arr_stats(arr: np.ndarray) -> dict:
        flat = arr.reshape(-1)
        sample = flat[:8].tolist()
        return {
            'shape': list(arr.shape),
            'dtype': str(arr.dtype),
            'min': float(np.min(arr)),
            'max': float(np.max(arr)),
            'mean': float(np.mean(arr)),
            'sample': [float(v) for v in sample],
        }

    def _write_model_io_record(self, feed: dict, vec: np.ndarray, action: np.ndarray) -> None:
        if not self.write_model_io_file or self._file_log_path is None:
            return

        self._file_log_count += 1
        if self._file_log_count % self.model_io_log_every_n != 0:
            return

        feed_stats = {}
        for name, arr in feed.items():
            if isinstance(arr, np.ndarray):
                feed_stats[name] = self._arr_stats(arr)
            else:
                feed_stats[name] = {'type': type(arr).__name__}

        record = {
            'event': 'inference',
            'stamp_ns': int(self.get_clock().now().nanoseconds),
            'goal_xy': self.goal_xy.tolist() if self.goal_xy is not None else None,
            'robot_xyyaw': self.robot_xyyaw.tolist() if self.robot_xyyaw is not None else None,
            'vec_obs': [float(v) for v in vec.reshape(-1).tolist()],
            'feed': feed_stats,
            'action_output_name': self.action_output_name,
            'action': [float(v) for v in action.tolist()],
        }

        try:
            with self._file_log_path.open('a', encoding='utf-8') as fp:
                fp.write(f'{json.dumps(record, ensure_ascii=False)}\n')
        except Exception as exc:
            self.get_logger().error(f'Failed to write model_io log file: {exc}')

    def cb_goal_pose(self, msg: PoseStamped) -> None:
        # 目標は map frame のみ受理（TF変換は本ノードで未実装）
        frame = (msg.header.frame_id or '').strip()
        if frame != 'map':
            if not self._warned_goal_frame_mismatch:
                self.get_logger().warn(f"Ignoring {self.goal_pose_topic} frame_id='{frame}' (expected: 'map').")
                self._warned_goal_frame_mismatch = True
            return

        goal_x = float(msg.pose.position.x)
        goal_y = float(msg.pose.position.y)
        self.goal_xy = np.array([goal_x, goal_y], dtype=np.float32)
        self.last_goal_stamp = msg.header.stamp
        self.goal_count += 1

        now = self.get_clock().now()
        if self.debug and (
            self.goal_count == 1
            or (now - self._last_goal_log_time).nanoseconds / 1e9 >= self.log_period_sec
        ):
            self.get_logger().info(
                f"goal updated #{self.goal_count}: frame_id='{frame}', "
                f'goal=({goal_x:.3f},{goal_y:.3f}), '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            )
            self._last_goal_log_time = now

    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # 自己位置も map frame を前提とする
        frame = (msg.header.frame_id or '').strip()
        if frame != 'map':
            if not self._warned_amcl_frame_mismatch:
                self.get_logger().warn(f"Ignoring {self.amcl_pose_topic} frame_id='{frame}' (expected: 'map').")
                self._warned_amcl_frame_mismatch = True
            return

        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        self.robot_xyyaw = np.array([px, py, yaw], dtype=np.float32)
        self.last_amcl_stamp = msg.header.stamp
        self.amcl_count += 1

        now = self.get_clock().now()
        if self.debug and (
            self.amcl_count == 1
            or (now - self._last_amcl_log_time).nanoseconds / 1e9 >= self.log_period_sec
        ):
            self.get_logger().info(
                f"amcl updated #{self.amcl_count}: frame_id='{frame}', "
                f'pose=({px:.3f},{py:.3f},{yaw:.3f}), '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            )
            self._last_amcl_log_time = now

    def cb_clicked_point(self, msg: PointStamped) -> None:
        # RViz Publish Point をウェイポイントとして追加
        frame = (msg.header.frame_id or '').strip()
        if frame != 'map':
            if not self._warned_clicked_point_frame_mismatch:
                self.get_logger().warn(
                    f"Ignoring {self.clicked_point_topic} frame_id='{frame}' (expected: 'map')."
                )
                self._warned_clicked_point_frame_mismatch = True
            return

        x = float(msg.point.x)
        y = float(msg.point.y)
        self.waypoints_xy.append(np.array([x, y], dtype=np.float32))

        total = len(self.waypoints_xy)
        remaining = max(0, total - self.current_waypoint_index)
        self.get_logger().info(
            f'waypoint added: ({x:.3f},{y:.3f}), total={total}, remaining={remaining}'
        )

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

    def _advance_waypoint_if_reached(self) -> None:
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
            self.get_logger().info(
                f'waypoint reached: index={reached_index}, dist={dist:.3f} m, remaining={remaining}'
            )

    def get_target_xy(self) -> tuple[np.ndarray, str]:
        self._advance_waypoint_if_reached()

        if self.current_waypoint_index < len(self.waypoints_xy):
            return self.waypoints_xy[self.current_waypoint_index], 'waypoint'
        return self.goal_xy, 'goal'

    def compute_goal_vector(self) -> np.ndarray:
        # [goal方向(deg), goal距離(m)] をベクトル観測として返す
        if self.goal_xy is None or self.robot_xyyaw is None:
            return np.zeros((1, self.vec_obs_dim), dtype=np.float32)

        target_xy, _ = self.get_target_xy()
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

    def cb_cam(self, msg: Image) -> None:
        # 受信画像ベースで推論周期を制限（0.0=無制限）
        now_ns = self.get_clock().now().nanoseconds
        if self._min_infer_period_sec > 0.0 and self._last_infer_accept_ns > 0:
            elapsed_sec = (now_ns - self._last_infer_accept_ns) / 1e9
            if elapsed_sec < self._min_infer_period_sec:
                return
        self._last_infer_accept_ns = now_ns

        try:
            img_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        # 前処理: resize -> [0,1]正規化 -> CHW
        img_rgb = cv2.resize(img_rgb, (self.img_width, self.img_height), interpolation=cv2.INTER_AREA)
        img_f = img_rgb.astype(np.float32) / 255.0
        img_chw = np.transpose(img_f, (2, 0, 1))

        if len(self.frame_buffer) == 0:
            for _ in range(self.stack_size):
                self.frame_buffer.append(img_chw)
        else:
            self.frame_buffer.append(img_chw)

        # モデル入力用に NCHW/NHWC の両方を準備
        stacked_chw = np.concatenate(list(self.frame_buffer), axis=0)
        img_nchw = np.expand_dims(stacked_chw, axis=0)
        img_nhwc = np.transpose(img_nchw, (0, 2, 3, 1))

        if self.goal_xy is None:
            if not self._warned_no_goal and self.debug:
                self.get_logger().info(f'waiting for {self.goal_pose_topic}')
                self._warned_no_goal = True
            return
        self._warned_no_goal = False

        if self.robot_xyyaw is None:
            if not self._warned_no_amcl and self.debug:
                self.get_logger().info(f'waiting for {self.amcl_pose_topic}')
                self._warned_no_amcl = True
        else:
            self._warned_no_amcl = False

        if self.debug:
            try:
                frames_vis = []
                for frame in self.frame_buffer:
                    f_hwc = np.transpose(frame, (1, 2, 0))
                    f_uint8 = (f_hwc * 255.0).astype(np.uint8)
                    frames_vis.append(f_uint8)
                stacked_vis = np.concatenate(frames_vis, axis=1)
                debug_msg = self.bridge.cv2_to_imgmsg(stacked_vis, encoding='rgb8')
                self.pub_debug_stack.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warn(f'Failed to publish debug image: {exc}')

        vec = self.compute_goal_vector()

        # モデル入力shapeに合わせて feed を構築
        feed = {}
        for name, shape in zip(self.input_names, self.input_shapes):
            if len(shape) == 4:
                ch_dim1 = shape[1]
                if isinstance(ch_dim1, int) and (ch_dim1 == 3 or ch_dim1 == 3 * self.stack_size):
                    feed[name] = img_nchw
                else:
                    feed[name] = img_nhwc
            else:
                target_dim = shape[-1] if isinstance(shape[-1], int) else vec.shape[1]
                vector = vec
                if isinstance(target_dim, int) and vector.shape[1] != target_dim:
                    if vector.shape[1] < target_dim:
                        vector = np.pad(
                            vector,
                            ((0, 0), (0, target_dim - vector.shape[1])),
                            constant_values=0.0,
                        )
                    else:
                        vector = vector[:, :target_dim]
                feed[name] = vector.astype(np.float32)

        try:
            # 決定済み output からアクションを取得
            action = self.session.run([self.action_output_name], feed)[0]
            action = np.asarray(action, dtype=np.float32).reshape(-1)
            action = np.clip(action, -1.0, 1.0)
        except Exception as exc:
            self.get_logger().error(f'Inference failed: {exc}')
            return

        self._write_model_io_record(feed, vec, action)

        if action.size >= 1:
            # 前進成分は負値を許可しない（既定ポリシー）
            action[0] = max(0.0, float(action[0]))

        act_msg = Float32MultiArray()
        act_msg.data = action.tolist()
        self.pub_act.publish(act_msg)

        self._infer_count += 1
        if self.debug:
            now = self.get_clock().now()
            elapsed = (now - self._last_log_time).nanoseconds / 1e9
            if elapsed >= max(0.1, self.log_period_sec):
                rate = self._infer_count / max(1e-6, elapsed)
                act_preview = ','.join([f'{value:.3f}' for value in action[:4]])

                img_shape = None
                vec_shape = None
                for arr in feed.values():
                    if isinstance(arr, np.ndarray) and arr.ndim == 4:
                        img_shape = arr.shape
                    elif isinstance(arr, np.ndarray) and arr.ndim == 2:
                        vec_shape = arr.shape

                angle_to_goal = float(vec[0, 0]) if vec.shape[1] >= 1 else 0.0
                distance_to_goal = float(vec[0, 1]) if vec.shape[1] >= 2 else 0.0
                _, target_kind = self.get_target_xy()
                remaining_waypoints = max(0, len(self.waypoints_xy) - self.current_waypoint_index)

                self.get_logger().info(
                    f'infer_rate={rate:.1f} Hz, img={img_shape}, vec={vec_shape}, '
                    f'target={target_kind}, remaining_waypoints={remaining_waypoints}, '
                    f'vec_obs=[{angle_to_goal:.2f}°, {distance_to_goal:.3f}m], '
                    f'action=[{act_preview}]'
                )

                if self.log_model_io:
                    feed_summary = []
                    for name, arr in feed.items():
                        if isinstance(arr, np.ndarray):
                            feed_summary.append(
                                f"{name}:shape={arr.shape},dtype={arr.dtype},min={float(np.min(arr)):.3f},max={float(np.max(arr)):.3f}"
                            )
                        else:
                            feed_summary.append(f'{name}:type={type(arr).__name__}')

                    out_preview = ','.join([f'{value:.3f}' for value in action[:6]])
                    self.get_logger().info(
                        f"model_io feed=[{'; '.join(feed_summary)}], "
                        f'output_name={self.action_output_name}, '
                        f'output_shape={tuple(action.shape)}, '
                        f'output_preview=[{out_preview}]'
                    )

                self._infer_count = 0
                self._last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealOnnxNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
