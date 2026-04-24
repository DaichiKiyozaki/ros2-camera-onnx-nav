# real-nav-ws

行動モデルを実機に適用して目的地まで自律走行させるためのワークスペース

## img_seg_pkg

### 概要

- カメラ画像から「走行可能領域（床）」と「歩行者（同方向/同方向以外）」を検出し、4値化セグメンテーション画像を出力する。
- 学習環境を4値化し、実環境でも本パッケージで4値化した画像を行動モデルに入力することで、sim2realの視覚ギャップを軽減する。

使用モデル：

- セマンティックセグメンテーション（走行可能領域）
  - 画像をピクセル単位で分類し、走行可能領域（床）マスクを推定する。
  - 実装上は PyTorch のセマンティックセグメンテーションモデル（`best_model_house2.pth`）で床マスクを生成する。
- YOLO-seg（歩行者 + 向きカテゴリ）
  - [yolo26s-seg](https://docs.ultralytics.com/ja/models/yolo26/)ベースでファインチューニングした2クラスのインスタンスセグメンテーションモデル（`yolo26s-seg_pedflow2cls.pt`）。
  - 歩行者領域を検出し、ピクセル単位のマスクを推定する。
  - クラス定義（実装の `class 0/1`）
    - class 0: 同方向歩行者（カメラとの相対角度が 45° 以内）
    - class 1: 同方向以外歩行者

出力（`/gb_img`）の色（BGR）は以下。

| クラス | 色(BGR) |
| --- | --- |
| 走行可能領域（床） | 緑（0, 255, 0） |
| 同方向歩行者 | 青（255, 0, 0） |
| 同方向以外歩行者 | 赤（0, 0, 255） |
| その他 | シアン（255, 255, 0） |

### 開発環境

- ROS2 jazzy
- Python 3.12.3
- CUDA Version: 13.0
- Python依存は [../requirements.txt](../requirements.txt) を参照

モデルファイル（要配置）：

- セマンティックセグメンテーションモデル
  - 例：`models/best_model_house2.pth`
- YOLO-seg（歩行者 + 向きカテゴリ）
  - 例：`models/yolo26s-seg_pedflow2cls.pt`

### 使用方法

#### 仮想環境の運用

- 仮想環境はリポジトリ直下の `../.venv` を共通利用
- 実行時は `source ../.venv/bin/activate` 後に `python -m colcon build` / `ros2 run` を実行

#### 1) セットアップ & ビルド

```bash
cd real-nav-ws

source ../.venv/bin/activate

python -m pip install --upgrade pip
python -m pip install -r ../requirements.txt

python -m colcon build --symlink-install
source install/setup.bash
```

#### 2) 起動

カメラ（例）：

```bash
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
    -p video_device:="/dev/video0" \
    -p image_size:="[640,480]" \
    -p pixel_format:="YUYV"
```

セグメンテーション：

```bash
cd real-nav-ws
source ../.venv/bin/activate
source install/setup.bash
ros2 run img_seg_pkg pedflow_4cls_seg_node
```

デフォルトは 10 Hz。周期を変更する場合の例：

```bash
ros2 run img_seg_pkg pedflow_4cls_seg_node --ros-args \
  -p max_segmentation_hz:=10.0
```

トピック：

- Subscribe: `/image_raw`（sensor_msgs/Image, BGR8, 640×480想定）
- Publish: `/gb_img`（sensor_msgs/Image, BGR8, 112×84）

### 処理フロー

1. `/image_raw` を受信
2. 床セグメンテーション（PyTorchモデルで床マスク生成）
3. 歩行者seg（YOLO-segで class 0/1 のマスク生成、複数人はクラス別に統合）
4. 4値化画像を作成（優先度: その他 → 床 → class 0 → class 1）
5. 行動モデルの入力画像サイズにリサイズして `/gb_img` を publish

## onnx_nav_pkg

### 概要

- 実機向け ONNX 行動モデル推論パッケージ
- 推論には AMCL による自己位置推定（`/amcl_pose`）が前提
- ノード名: `real_onnx_nav_node`
- パッケージ名: `onnx_nav_pkg`

### 必須前提（AMCL + 地図）

- `real_onnx_nav_node` は `/goal_pose` と `/amcl_pose` が揃って初めて目標相対ベクトルを計算
- `/amcl_pose` が無い場合は自己位置を 0 埋め扱いで推論し、挙動が不安定になりやすい
- 実運用は map_server + amcl を先に起動する
- 推論トリガーは従来どおり `/goal_pose`
- RViz2 の Publish Point（`/clicked_point`）でウェイポイントを任意個追加可能（0個でも動作）
- 距離・角度は「次の未到達ウェイポイント（無ければ最終目的地）」に対して計算

### リソース配置

- 行動モデル（ONNX）
  - `src/onnx_nav_pkg/models/*.onnx`
- 地図（AMCL 用）
  - `src/onnx_nav_pkg/map/my_map.yaml`
  - `src/onnx_nav_pkg/map/my_map.pgm`（または `.png`）
- RViz 設定
  - `src/onnx_nav_pkg/rviz/real_nav_default.rviz`

### AMCL 起動

- launch ファイル: `real_amcl.launch.py`
- 役割: `map_server` + `amcl` + `nav2_lifecycle_manager`（必要時 `rviz2`）

```bash
cd real-nav-ws
source ../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch onnx_nav_pkg real_amcl.launch.py \
  map:=src/onnx_nav_pkg/map/my_map.yaml \
  rviz:=true
```

### 実機ドライバ起動（Kobuki + URG）

- `kobuki_node` はワークスペース外の環境オーバーレイを利用
- URG のパラメータは `src/onnx_nav_pkg/urg/urg.yaml` を使用
- 先に Kobuki と URG を起動し、`/odom` と `/scan` を有効化してから AMCL / ナビゲーションを起動

ターミナル1（Kobuki）:

```bash
cd real-nav-ws
source ../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source kobuki-jazzy-env/install/setup.bash

ros2 run kobuki_node kobuki_ros_node --ros-args \
  -p device_port:=/dev/ttyUSB0
```

ターミナル2（URG）:

```bash
cd real-nav-ws
source ../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source kobuki-jazzy-env/install/setup.bash

ros2 run urg_node urg_node_driver --ros-args \
  --params-file src/onnx_nav_pkg/urg/urg.yaml
```

### 推論ノード起動

```bash
cd real-nav-ws
source ../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run onnx_nav_pkg real_onnx_nav_node --ros-args \
  -p model_file_name:=balance.onnx \
  -p image_topic:=/cb_img \
  -p action_topic:=/agent/cmd \
  -p stack_size:=5
```

### 一括起動（カメラ + セグメンテーション + AMCL + ONNX-nav）

- launch ファイル
  - kobuki: `kobuki_real_nav_bringup.launch.py`
- 起動対象
  - `v4l2_camera_node`
  - `img_seg_pkg/pedflow_4cls_seg_node`
  - `onnx_nav_pkg/real_amcl.launch.py`（map_server + amcl + lifecycle_manager）
  - `onnx_nav_pkg/real_onnx_nav_node`

```bash
cd real-nav-ws
source ../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch onnx_nav_pkg kobuki_real_nav_bringup.launch.py \
  map:=src/onnx_nav_pkg/map/my_map.yaml \
  model_file_name:=0419.onnx \
  video_device:=/dev/video2 \
  image_size:="[640,480]" \
  pixel_format:=YUYV \
  max_segmentation_hz:=10.0
```

主な起動引数:

- `use_camera` / `use_segmentation` / `use_localization` / `use_navigation`
- `camera_topic`（default: `/image_raw`）
- `cb_topic`（default: `/cb_img`）
- `scan_topic`（default: `/scan`）
- `base_frame_id`（default: `base`）
- `use_base_to_base_link_alias_tf`（default: `true`）
- `base_link_frame_id`（default: `base_link`）
- `rviz`（default: `true`）
- `map`（default: `src/onnx_nav_pkg/map/my_map.yaml` を想定）
- `model_file_name`（default: `balance.onnx`）
- `use_static_base_to_laser_tf`（default: `true`）
- `laser_frame_id`（default: `laser`）
- `static_tf_x` / `static_tf_y` / `static_tf_z` / `static_tf_yaw` / `static_tf_pitch` / `static_tf_roll`

`No tf data. Actual error: Frame [map] does not exist` が出る場合の例（LiDAR取付TFを仮設定）:


### パラメータ（推論ノード）

- `image_topic`（default: `/cb_img`）
- `goal_pose_topic`（default: `/goal_pose`）
- `clicked_point_topic`（default: `/clicked_point`）
- `amcl_pose_topic`（default: `/amcl_pose`）
- `action_topic`（default: `/agent/cmd`）
- `max_inference_hz`（default: `10.0`、0.0 は無制限）
- `io_debug`（default: `false`、`true` で入力/出力の具体値プレビューをログ出力）
- `waypoint_reach_threshold_m`（default: `0.6`）

### 通信仕様（推論ノード）

| 種別 | トピック | 型 | 用途 |
| --- | --- | --- | --- |
| Subscribe | `/cb_img` | `sensor_msgs/Image` | セグメンテーション後の入力画像 |
| Subscribe | `/goal_pose` | `geometry_msgs/PoseStamped` | 目標位置 |
| Subscribe | `/clicked_point` | `geometry_msgs/PointStamped` | RViz2 Publish Pointで追加するウェイポイント |
| Subscribe | `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 自己位置 |
| Publish | `/agent/cmd` | `std_msgs/Float32MultiArray` | 推論行動 |
| Publish | `/debug/stacked_image` | `sensor_msgs/Image` | デバッグ用スタック画像 |
