"""onnx_nav_config.py

役割:
- `real_onnx_nav_node` が使うROS2パラメータ定義を集約
- パラメータ読み出しと型変換を一元化
- 実行前バリデーション（値域チェック）を担当
"""

from dataclasses import dataclass

from rclpy.node import Node


@dataclass(frozen=True)
class OnnxNavConfig:
    debug: bool
    log_model_io: bool
    log_period_sec: float
    write_model_io_file: bool
    model_io_log_dir: str
    model_io_log_every_n: int
    image_topic: str
    goal_pose_topic: str
    clicked_point_topic: str
    amcl_pose_topic: str
    action_topic: str
    max_inference_hz: float
    model_file_name: str
    img_width: int
    img_height: int
    stack_size: int
    vec_obs_dim: int
    waypoint_reach_threshold_m: float


def declare_nav_parameters(node: Node) -> None:
    node.declare_parameter('debug', False)
    node.declare_parameter('log_model_io', False)
    node.declare_parameter('log_period_sec', 1.0)
    node.declare_parameter('write_model_io_file', False)
    node.declare_parameter('model_io_log_dir', 'logs/model_io')
    node.declare_parameter('model_io_log_every_n', 1)
    node.declare_parameter('image_topic', '/cb_img')
    node.declare_parameter('goal_pose_topic', '/goal_pose')
    node.declare_parameter('clicked_point_topic', '/clicked_point')
    node.declare_parameter('amcl_pose_topic', '/amcl_pose')
    node.declare_parameter('action_topic', '/agent/cmd')
    node.declare_parameter('max_inference_hz', 10.0)
    node.declare_parameter('model_file_name', 'balance.onnx')
    node.declare_parameter('img_width', 112)
    node.declare_parameter('img_height', 84)
    node.declare_parameter('stack_size', 5)
    node.declare_parameter('vec_obs_dim', 2)
    node.declare_parameter('waypoint_reach_threshold_m', 0.6)


def load_nav_config(node: Node) -> OnnxNavConfig:
    cfg = OnnxNavConfig(
        debug=bool(node.get_parameter('debug').value),
        log_model_io=bool(node.get_parameter('log_model_io').value),
        log_period_sec=float(node.get_parameter('log_period_sec').value),
        write_model_io_file=bool(node.get_parameter('write_model_io_file').value),
        model_io_log_dir=str(node.get_parameter('model_io_log_dir').value),
        model_io_log_every_n=int(node.get_parameter('model_io_log_every_n').value),
        image_topic=str(node.get_parameter('image_topic').value),
        goal_pose_topic=str(node.get_parameter('goal_pose_topic').value),
        clicked_point_topic=str(node.get_parameter('clicked_point_topic').value),
        amcl_pose_topic=str(node.get_parameter('amcl_pose_topic').value),
        action_topic=str(node.get_parameter('action_topic').value),
        max_inference_hz=float(node.get_parameter('max_inference_hz').value),
        model_file_name=str(node.get_parameter('model_file_name').value or '').strip() or 'balance.onnx',
        img_width=int(node.get_parameter('img_width').value),
        img_height=int(node.get_parameter('img_height').value),
        stack_size=int(node.get_parameter('stack_size').value),
        vec_obs_dim=int(node.get_parameter('vec_obs_dim').value),
        waypoint_reach_threshold_m=float(node.get_parameter('waypoint_reach_threshold_m').value),
    )
    validate_nav_config(cfg)
    return cfg


def validate_nav_config(cfg: OnnxNavConfig) -> None:
    if cfg.max_inference_hz < 0.0:
        raise ValueError(f'max_inference_hz must be >= 0.0: {cfg.max_inference_hz}')
    if cfg.img_width <= 0 or cfg.img_height <= 0:
        raise ValueError(f'img_width/img_height must be > 0: ({cfg.img_width}, {cfg.img_height})')
    if cfg.stack_size <= 0:
        raise ValueError(f'stack_size must be > 0: {cfg.stack_size}')
    if cfg.vec_obs_dim <= 0:
        raise ValueError(f'vec_obs_dim must be > 0: {cfg.vec_obs_dim}')
    if cfg.waypoint_reach_threshold_m <= 0.0:
        raise ValueError(
            f'waypoint_reach_threshold_m must be > 0.0: {cfg.waypoint_reach_threshold_m}'
        )
    if cfg.model_io_log_every_n <= 0:
        raise ValueError(f'model_io_log_every_n must be > 0: {cfg.model_io_log_every_n}')
