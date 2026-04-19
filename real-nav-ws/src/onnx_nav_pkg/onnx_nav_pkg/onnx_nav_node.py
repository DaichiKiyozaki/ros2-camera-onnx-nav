"""onnx_nav_node.py

役割:
- ROS2ノードとして入出力トピックを接続
- 前処理/ターゲット管理/ONNX推論/モデルI/O記録の各責務を統合
- 推論結果を `/agent/cmd` にpublish
"""

import os

import numpy as np
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

from .onnx_nav_config import declare_nav_parameters, load_nav_config
from .onnx_nav_inference import (
    FrameStackPreprocessor,
    build_model_feed,
    create_onnx_session,
    select_action_output_name,
)
from .onnx_nav_model_io import ModelIoFileLogger
from .onnx_nav_target_tracker import TargetTracker


class RealOnnxNavNode(Node):
    @staticmethod
    def _format_array_preview(arr: np.ndarray, sample_size: int = 8) -> str:
        flat = arr.reshape(-1)
        sample = ','.join([f'{float(v):.4f}' for v in flat[:sample_size]])
        return f'shape={arr.shape},dtype={arr.dtype},sample=[{sample}]'

    def __init__(self) -> None:
        super().__init__('real_onnx_nav_node')

        self.bridge = CvBridge()

        # ノード設定
        declare_nav_parameters(self)
        cfg = load_nav_config(self)

        self.debug = cfg.debug
        self.io_debug = cfg.io_debug
        self.log_model_io = cfg.log_model_io
        self.log_period_sec = cfg.log_period_sec
        self.write_model_io_file = cfg.write_model_io_file
        self.model_io_log_dir = cfg.model_io_log_dir
        self.model_io_log_every_n = cfg.model_io_log_every_n
        self.image_topic = cfg.image_topic
        self.goal_pose_topic = cfg.goal_pose_topic
        self.clicked_point_topic = cfg.clicked_point_topic
        self.amcl_pose_topic = cfg.amcl_pose_topic
        self.action_topic = cfg.action_topic
        self.max_inference_hz = cfg.max_inference_hz
        self.stack_size = cfg.stack_size

        self._min_infer_period_sec = (1.0 / self.max_inference_hz) if self.max_inference_hz > 0.0 else 0.0
        self._last_infer_accept_ns = 0

        # 状態管理
        initial_now_ns = self.get_clock().now().nanoseconds
        self.target_tracker = TargetTracker(
            vec_obs_dim=cfg.vec_obs_dim,
            waypoint_reach_threshold_m=cfg.waypoint_reach_threshold_m,
            initial_now_ns=initial_now_ns,
        )
        self._warned_no_goal = False
        self._warned_no_amcl = False

        # 前処理
        self.frame_preprocessor = FrameStackPreprocessor(
            img_width=cfg.img_width,
            img_height=cfg.img_height,
            stack_size=cfg.stack_size,
        )

        # デバッグ統計
        self._infer_count = 0
        self._last_log_time = self.get_clock().now()
        self._last_io_debug_log_time = self.get_clock().now()

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
        self.session = create_onnx_session(model_path, logger=self.get_logger())

        self.inputs = self.session.get_inputs()
        self.input_names = [i.name for i in self.inputs]
        self.input_shapes = [i.shape for i in self.inputs]
        self.get_logger().info(f'ONNX inputs: {list(zip(self.input_names, self.input_shapes))}')

        self.declare_parameter('action_output_name', '')
        self.outputs = self.session.get_outputs()
        self.output_names = [o.name for o in self.outputs]
        self.output_shapes = [o.shape for o in self.outputs]
        self.get_logger().info(f'ONNX outputs: {list(zip(self.output_names, self.output_shapes))}')

        param_action = str(self.get_parameter('action_output_name').value or '').strip()
        self.action_output_name = select_action_output_name(self.output_names, param_action)

        self.get_logger().info(
            f'action_output_name={self.action_output_name}, available_outputs={self.output_names}'
        )

        self.model_io_logger = ModelIoFileLogger(
            enabled=self.write_model_io_file,
            log_dir=self.model_io_log_dir,
            every_n=self.model_io_log_every_n,
            node_name=self.get_name(),
            action_output_name=self.action_output_name,
            input_names=self.input_names,
            output_names=self.output_names,
            logger=self.get_logger(),
            stamp_ns_provider=lambda: int(self.get_clock().now().nanoseconds),
        )

        self.get_logger().info(f'Subscribed image topic: {self.image_topic}')
        self.get_logger().info(f'Subscribed goal topic: {self.goal_pose_topic}')
        self.get_logger().info(f'Subscribed waypoint topic: {self.clicked_point_topic}')
        self.get_logger().info(f'Subscribed amcl topic: {self.amcl_pose_topic}')
        self.get_logger().info(f'Published action topic: {self.action_topic}')
        self.get_logger().info(f'max_inference_hz={self.max_inference_hz:.3f} (0.0 means unlimited)')
        self.get_logger().info(f'io_debug={self.io_debug}')

    def cb_goal_pose(self, msg: PoseStamped) -> None:
        self.target_tracker.handle_goal_pose(
            msg,
            self.get_logger(),
            goal_pose_topic=self.goal_pose_topic,
            debug=self.debug,
            log_period_sec=self.log_period_sec,
            now_ns=int(self.get_clock().now().nanoseconds),
        )

    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.target_tracker.handle_amcl_pose(
            msg,
            self.get_logger(),
            amcl_pose_topic=self.amcl_pose_topic,
            debug=self.debug,
            log_period_sec=self.log_period_sec,
            now_ns=int(self.get_clock().now().nanoseconds),
        )

    def cb_clicked_point(self, msg: PointStamped) -> None:
        self.target_tracker.handle_clicked_point(
            msg,
            self.get_logger(),
            clicked_point_topic=self.clicked_point_topic,
        )

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

        img_nchw, img_nhwc = self.frame_preprocessor.push_rgb_frame(img_rgb)

        if self.target_tracker.goal_xy is None:
            if not self._warned_no_goal and self.debug:
                self.get_logger().info(f'waiting for {self.goal_pose_topic}')
                self._warned_no_goal = True
            return
        self._warned_no_goal = False

        if self.target_tracker.robot_xyyaw is None:
            if not self._warned_no_amcl and self.debug:
                self.get_logger().info(f'waiting for {self.amcl_pose_topic}')
                self._warned_no_amcl = True
        else:
            self._warned_no_amcl = False

        if self.debug:
            try:
                stacked_vis = self.frame_preprocessor.get_stacked_debug_rgb()
                debug_msg = self.bridge.cv2_to_imgmsg(stacked_vis, encoding='rgb8')
                self.pub_debug_stack.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warn(f'Failed to publish debug image: {exc}')

        vec = self.target_tracker.compute_goal_vector(self.get_logger())

        feed = build_model_feed(
            input_names=self.input_names,
            input_shapes=self.input_shapes,
            img_nchw=img_nchw,
            img_nhwc=img_nhwc,
            vec=vec,
            stack_size=self.stack_size,
        )

        try:
            # 決定済み output からアクションを取得
            action = self.session.run([self.action_output_name], feed)[0]
            action = np.asarray(action, dtype=np.float32).reshape(-1)
            action = np.clip(action, -1.0, 1.0)
        except Exception as exc:
            self.get_logger().error(f'Inference failed: {exc}')
            return

        self.model_io_logger.write(
            feed,
            vec,
            action,
            goal_xy=self.target_tracker.goal_xy,
            robot_xyyaw=self.target_tracker.robot_xyyaw,
            logger=self.get_logger(),
        )

        if self.io_debug:
            now = self.get_clock().now()
            io_elapsed = (now - self._last_io_debug_log_time).nanoseconds / 1e9
            if io_elapsed >= max(0.1, self.log_period_sec):
                feed_preview = []
                for name, arr in feed.items():
                    if isinstance(arr, np.ndarray):
                        feed_preview.append(f'{name}:{self._format_array_preview(arr)}')
                    else:
                        feed_preview.append(f'{name}:type={type(arr).__name__}')

                vec_values = ','.join([f'{float(v):.4f}' for v in vec.reshape(-1)])
                action_values = ','.join([f'{float(v):.4f}' for v in action.reshape(-1)])

                self.get_logger().info(
                    f'io_debug vec_obs=[{vec_values}], '
                    f'output_name={self.action_output_name}, '
                    f'action=[{action_values}]'
                )
                self.get_logger().info(f"io_debug feed=[{'; '.join(feed_preview)}]")
                self._last_io_debug_log_time = now

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
                _, target_kind = self.target_tracker.get_target_xy(self.get_logger())
                remaining_waypoints = self.target_tracker.remaining_waypoints()

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
