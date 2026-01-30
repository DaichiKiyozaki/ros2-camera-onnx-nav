# Copilot Instructions

## リポジトリ制約
- このリポジトリは ROS2 (Jazzy) のマルチワークスペース・モノレポ。
- 生成物は絶対に編集しない: **/build/**, **/install/**, **/log/**。
- ドキュメント/設定（README, yaml, launch, rviz）は版管理対象で編集可。

## 言語・文体
- レビュー/コメント: 日本語
- 実行時ログ: 英語
- README: です・ます調を避け、箇条書き/テーブル中心で書く
- 用語: 文書中は `ROS2` に統一（引用/固有名詞は例外）

## ワークスペース
- unity-nav-ws/: Unity (ROS2-For-Unity) 推論ワークスペース
- real-nav-ws/: 実機ナビゲーション用ワークスペース

## 必須ワークフロー（各ワークスペース共通）
- コマンドは必ず対象ワークスペース直下で実行する（リポジトリ直下で実行しない）。
- ビルド: `colcon build --symlink-install`
- overlay順（必須）:
  - `source /opt/ros/jazzy/setup.bash`
  - `source <ws>/install/setup.bash`
- リソース（models/map/rviz 等）:
  - `setup.py:data_files` で `share/<pkg>/...` にインストールする
  - 実行時は `ament_index_python` で探索する

## 検証ルール（変更提案時）
- 検証コマンドを必ず提示する:
  - `colcon build --packages-select <pkg>`
  - `colcon test --packages-select <pkg> && colcon test-result --verbose`
  - （該当する場合）`ros2 launch <pkg> <file>.launch.py`

## Python環境メモ
- `python -m pip ...` を優先する。
- `.venv/` がある場合、明示がない限り ROS2 とは別の補助ツール用途にのみ使用する。
  - （rclpy 等、ROS2提供のPythonパッケージを壊さない）

## ドキュメント表記ルール
- 絶対パス（例: `/home/...`）を書かない。相対パスまたは `~/ros2-workspaces/...` を使う。

## 変更の安全性
- 明示がない限り ROS API互換（トピック名・メッセージ型・QoS）を維持する。
- リソース追加（map/rviz/models 等）をしたら、`setup.py:data_files` を更新する。
- ファイル削除は明示的な確認が必要。

## 依存関係/設定の変更
- Python依存: `requirements.txt` と/または `setup.py` の依存指定を更新し、再現手順も必要に応じてREADMEへ反映する。
- ROS依存: `package.xml` を更新する（必要なら CMakeLists/setup も更新）。
- 設定変更（yaml/launch）: 実行引数/手順/前提が変わる場合は該当READMEも更新する。
