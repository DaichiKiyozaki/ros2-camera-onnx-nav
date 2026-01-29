# Copilot Instructions

このリポジトリは **ROS 2 (Jazzy) のマルチワークスペース・モノレポ**。
原則として編集対象は各ワークスペース配下の `src/` のみ（`build/`, `install/`, `log/` は生成物なので編集しない）。

## 言語・文体ルール
- 基本言語：日本語（レビュー・コメント等）
- ログ出力：英語
- README は「です・ます調」を避け、箇条書きやテーブルを用いて見やすく書く

## 全体像（ワークスペースと役割）
- `unity-inference-ws/`: ROS2-For-Unityを用いてUnity上でモデル推論する用のワークスペース
- `real-nav-ws/`: 学習モデルよる実機ナビゲーション用ワークスペース
- `slam-params/`: Nav2 / AMCL / SLAM Toolbox などの YAML パラメータ置き場

## 共通ワークフロー（各ワークスペース共通）
- ビルド: ワークスペース直下で `colcon build --symlink-install`
- 実行前: `source install/setup.bash` で overlay
- リソース（models/map/rviz 等）は `setup.py` の `data_files` で `share/<pkg>/…` にインストールし、実行時は `ament_index_python` で探索する

## 変更時の注意
- ROS API（トピック名・メッセージ型・QoS）は既存互換を優先して維持する
- リソース追加（map/rviz/models 等）をしたら、各パッケージの `setup.py` の `data_files` も更新して `share/<pkg>/…` に確実にインストールする
- 不必要になったファイル等は許可を取って適宜削除する
