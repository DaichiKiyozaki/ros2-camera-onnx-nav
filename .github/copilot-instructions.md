# Copilot 指示書 (ros_pj)

このリポジトリは **ROS 2 (Jazzy) のマルチワークスペース・モノレポ**。
原則として編集対象は各ワークスペース配下の `src/` のみ（`build/`, `install/`, `log/` は生成物なので編集しない）。

## 言語・文体ルール
- 基本言語：日本語（レビュー・コメント等）
- ログ出力：英語
- README は「です・ます調」を避け、箇条書きやテーブルを用いて見やすく書く

## 全体像（ワークスペースと役割）
- `unity-inference-ws/`: Unity（ros2-for-unity）↔ ROS 2 通信 + ONNX 推論ノード
  - 主要パッケージ: `unity-inference-ws/src/model_in_ros2node_pkg/`
- `seg-ws/`: カメラ → YOLOv8-seg + MEBOW（歩行者向き推定）→ セグメンテーション画像 publish
  - 主要パッケージ: `seg-ws/src/ped_orient_pkg/`
- `real-nav-ws/`: 実機ナビ用ワークスペース（このチェックアウトでは不完全な可能性あり）
  - 注意: `real-nav-ws/src/ped_road_seg_pkg` は `build/` 側に broken symlink があり、ソース欠落の疑い。編集前に `src/` の実体確認を行う
- `slam-params/`: Nav2 / AMCL / SLAM Toolbox などの YAML パラメータ置き場

## 共通ワークフロー（各ワークスペース共通）
- ビルド: ワークスペース直下で `colcon build --symlink-install`
- 実行前: `source install/setup.bash` で overlay
- リソース（models/map/rviz 等）は `setup.py` の `data_files` で `share/<pkg>/…` にインストールし、実行時は `ament_index_python` で探索する

## パッケージ別の慣例
### `ped_orient_pkg`（seg-ws）
- 起動: `ros2 run ped_orient_pkg ped_orient_node`（`seg-ws/src/ped_orient_pkg/setup.py` の `console_scripts`）
- リソース探索: `get_package_share_path("ped_orient_pkg")` を優先し、開発時はパッケージ直下へフォールバック（`seg-ws/src/ped_orient_pkg/ped_orient_pkg/ped_orient_node.py`）
- 必須ファイル: `MEBOW/` ツリーと `yolov8n-seg.pt`（`seg-ws/src/ped_orient_pkg/README.md`）
- ROS I/O:
  - Subscribe: パラメータ `input_image_topic`（default: `/img`）, `sensor_msgs/Image`
  - Publish: `/ped_orient/segmentation`（`bgr8`, 112×84 にダウンスケール）
- 将来的には4値化画像をML-Agentsで学習させたモデルに入力する予定

### `model_in_ros2node_pkg`（unity-inference-ws）
- AMCL+Map 起動: `ros2 launch model_in_ros2node_pkg unity_amcl.launch.py`
- 推論ノード起動: `ros2 run model_in_ros2node_pkg agent_node`
- フレーム前提: `/goal_pose` と `/amcl_pose` は `frame_id == "map"` を想定（不一致は警告して無視、TF 変換は未実装）
  - 実装箇所: `cb_goal_pose` / `cb_amcl_pose`（`unity-inference-ws/src/model_in_ros2node_pkg/model_in_ros2node_pkg/agent_node.py`）
- QoS: カメラ画像と AMCL は `qos_profile_sensor_data` を使用（遅延/取りこぼし対策）
- モデル配置: ONNX は `share/<pkg>/models/` から `get_package_share_directory` 経由でロード
  - 実行: `colcon test --packages-select model_in_ros2node_pkg` → `log/latest_test/` を確認

## 変更時の注意
- ROS API（トピック名・メッセージ型・QoS）は既存互換を優先して維持する
- リソース追加（map/rviz/models 等）をしたら、各パッケージの `setup.py` の `data_files` も更新して `share/<pkg>/…` に確実にインストールする
