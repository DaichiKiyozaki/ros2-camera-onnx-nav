from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ActionToTwistNode(Node):
    def __init__(self) -> None:
        super().__init__('action_to_twist_node')

        # 入出力トピック、アクション配列の参照インデックス、速度スケーリングを設定
        self.declare_parameter('action_topic', '/agent/cmd')
        self.declare_parameter('cmd_vel_topic', '/commands/velocity')
        self.declare_parameter('linear_index', 0)
        self.declare_parameter('angular_index', 1)
        self.declare_parameter('linear_scale', 6.0)
        self.declare_parameter('angular_scale', 3.0)
        self.declare_parameter('max_linear_x', 3.0)
        self.declare_parameter('max_angular_z', 6.0)
        self.declare_parameter('allow_backward', False)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('command_timeout_sec', 0.5)

        self.action_topic = str(self.get_parameter('action_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.linear_index = int(self.get_parameter('linear_index').value)
        self.angular_index = int(self.get_parameter('angular_index').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.allow_backward = bool(self.get_parameter('allow_backward').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        if self.linear_index < 0 or self.angular_index < 0:
            raise ValueError('linear_index/angular_index must be >= 0')
        if self.publish_rate_hz <= 0.0:
            raise ValueError(f'publish_rate_hz must be > 0.0: {self.publish_rate_hz}')
        if self.command_timeout_sec <= 0.0:
            raise ValueError(f'command_timeout_sec must be > 0.0: {self.command_timeout_sec}')

        # /agent/cmd(Float32MultiArray) を受けて /commands/velocity(Twist) を出力
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub_action = self.create_subscription(Float32MultiArray, self.action_topic, self.cb_action, 10)

        # 最新の行動指令を保持し、タイマ周期で安定 publish する
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_action_time: Optional[rclpy.time.Time] = None

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info(f'Subscribed action topic: {self.action_topic}')
        self.get_logger().info(f'Published cmd_vel topic: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'scale(linear,angular)=({self.linear_scale:.3f},{self.angular_scale:.3f}), '
            f'max(linear,angular)=({self.max_linear_x:.3f},{self.max_angular_z:.3f})'
        )

    def cb_action(self, msg: Float32MultiArray) -> None:
        # 配列サイズ不足は安全側で無視
        data = msg.data
        need_size = max(self.linear_index, self.angular_index) + 1
        if len(data) < need_size:
            self.get_logger().warn(
                f'Ignoring action: data length {len(data)} < required {need_size}'
            )
            return

        linear_raw = float(data[self.linear_index])
        angular_raw = float(data[self.angular_index])

        # モデル出力 [-1,1] を実機速度レンジへ変換
        linear_x = linear_raw * self.linear_scale
        # 実機系の回転方向に合わせるため、角速度の符号を反転
        angular_z = -angular_raw * self.angular_scale

        # 後進を許可しない場合は linear.x を 0 以上に制限
        if not self.allow_backward:
            linear_x = max(0.0, linear_x)

        # 最終的な安全クリップ
        linear_x = max(-self.max_linear_x, min(self.max_linear_x, linear_x))
        angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))

        self.last_linear_x = linear_x
        self.last_angular_z = angular_z
        self.last_action_time = self.get_clock().now()

    def on_timer(self) -> None:
        now = self.get_clock().now()

        twist = Twist()
        # まだ有効な行動が無い間は停止コマンドを送る
        if self.last_action_time is None:
            self.pub_cmd_vel.publish(twist)
            return

        # 行動指令が途絶えた場合は自動停止（フェイルセーフ）
        age_sec = (now - self.last_action_time).nanoseconds / 1e9
        if age_sec > self.command_timeout_sec:
            self.last_linear_x = 0.0
            self.last_angular_z = 0.0

        twist.linear.x = float(self.last_linear_x)
        twist.angular.z = float(self.last_angular_z)
        self.pub_cmd_vel.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionToTwistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
