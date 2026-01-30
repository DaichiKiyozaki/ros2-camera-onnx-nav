# unity-nav-ws

## 概要
- Unity 上のエージェントを ROS2 ノードで推論・制御するワークスペース
- [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) により Unity と ROS2 間で通信
- ROS2 ノード上で ONNX モデル推論を実行
- 自己位置推定にamclを用いるため、環境地図を事前に用意する必要がある

## 開発環境
- ROS2 jazzy
- Python 3.12

## セットアップ

1. 環境地図とrvizの設定ファイルを追加
   - 環境地図
      - 配置先： `unity-nav-ws/src/model_in_ros2node_pkg/map`
      - .yamlと.pgmのセット
      - 推論環境の地図は launch 呼び出し時に指定、または launch ファイルのデフォルト値を変更
   - rviz設定ファイル
      - 配置先： `unity-nav-ws/src/model_in_ros2node_pkg/rviz`

2. ワークスペースのビルド
   ```bash
   cd ~/ros_pj/unity-nav-ws
   colcon build --symlink-install
   ```

3. オーバーレイ
   ```bash
   source install/setup.bash
   ```

## 使い方

### 1. Unityの再生

### 2. AMCL + Map Server の起動
`unity_amcl.launch.py` で地図と自己位置推定を起動し、`/amcl_pose` を publish する。

```bash
ros2 launch model_in_ros2node_pkg unity_amcl.launch.py
```

主な引数:
- `use_sim_time` (default: `true`)
- `map`
- `rviz` (default: `true`)
- `rviz_config`
- `global_frame_id` (default: `map`)
- `odom_frame_id` (default: `odom`)
- `base_frame_id` (default: `base_link`)
- `scan_topic` (default: `/scan`)

### 3. rviz2で初期位置・目標位置を設定

- 初期位置：2D Pose Estimate
- 目標位置：2D Goal Pose

### 4. ノードの起動
推論エージェントノードを起動する。

```bash
ros2 run model_in_ros2node_pkg agent_node
```

### 通信仕様 (ROS Topics)

#### Subscribed Topics (入力)
| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `/unity/camera/image_raw` | `sensor_msgs/Image` | エージェントの視覚情報 (RGB)。ノード内で **112x84** にリサイズ。 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 の 2D Nav Goal。ゴール位置として使用。 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL 推定の自己位置。ゴール相対角の計算に使用。 |

#### Published Topics (出力)
| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `/agent/cmd` | `std_msgs/Float32MultiArray` | 推論された行動コマンド (continuous)。 |
| `/debug/stacked_image` | `sensor_msgs/Image` | デバッグ用。スタックしたフレームを横並びで可視化。 |

## Unity側の設定
Unity 側は `ros2-for-unity` を使用し、以下を publish 対象とする。

1. **カメラ画像**: RGB 形式。解像度は **112x84** を推奨。
2. **自己位置とゴール情報**: `/amcl_pose` と `/goal_pose` は ROS 側（AMCL/RViz2 など）で用意。
3. **TF/座標系**: 整合性が取れた`map`/`odom`/`base_link` をpublishする必要がある。

## パラメータ
| 名前 | デフォルト | 説明 |
| --- | --- | --- |
| `debug` | `true` | デバッグログと `/debug/stacked_image` の publish を有効化。 |
| `log_period_sec` | `1.0` | デバッグログの周期 (秒)。 |
