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
- ビルド: `python -m colcon build --symlink-install`
- overlay順（必須）:
  - `source /opt/ros/jazzy/setup.bash`
  - `source <ws>/install/setup.bash`
- リソース（models/map/rviz 等）:
  - `setup.py:data_files` で `share/<pkg>/...` にインストールする
  - 実行時は `ament_index_python` で探索する
  - 行動モデルや画像認識モデルは `models/` 配下で管理し、`share/<pkg>/models` にインストールする
  - `resource/` は ament index 用マーカーファイルのみを置く

## 検証ルール（変更提案時）
- 検証コマンドを必ず提示する:
  - `python -m colcon build --packages-select <pkg>`
  - `python -m colcon test --packages-select <pkg> && colcon test-result --verbose`
  - （該当する場合）`ros2 launch <pkg> <file>.launch.py`

## Python環境メモ
- `python -m pip ...` を優先する。
- `.venv/` はワークスペース単位で作成し、ROS2推論/開発でも基本的に使用する。
- `.venv/` 有効化後に `python -m colcon build` を実行する。

## ドキュメント表記ルール
- 絶対パス（例: `/home/...`）を書かない。リポジトリルートからの相対パスのみを使う。

## 変更の安全性
- 明示がない限り ROS API互換（トピック名・メッセージ型・QoS）を維持する。
- リソース追加（map/rviz/models 等）をしたら、`setup.py:data_files` を更新する。
- ファイル削除は明示的な確認が必要。

## 依存関係/設定の変更
- Python依存: `requirements.txt` と/または `setup.py` の依存指定を更新し、再現手順も必要に応じてREADMEへ反映する。
- ROS依存: `package.xml` を更新する（必要なら CMakeLists/setup も更新）。
- 設定変更（yaml/launch）: 実行引数/手順/前提が変わる場合は該当READMEも更新する。

## Code Rules（Python/ROS2）

### Pythonスタイル
- PEP 8 準拠

### 命名規則
- Python:
  - module/file/function/variable: `snake_case`
  - class: `PascalCase`
  - constant: `UPPER_SNAKE_CASE`
- ROS2:
  - topic / parameter: `snake_case`（スラッシュ区切り）
  - node名: `snake_case`
  - frame_id: `base_link`, `odom`, `map` 等の慣例に寄せる

### 型・Docstring
- public関数（他ファイルから呼ぶ想定）には型ヒント必須。
- `Optional` は `None` 処理を必ず明示し、早期returnで分岐を浅くする。
- 複雑な関数のみ docstring（Args/Returns/Raises）を付与。

### ログ・例外処理
- ログ文言は英語。数値/ID（frame_id, topic, param名）を含めて調査可能にする。
- 例外の握りつぶし禁止（`except Exception: pass` 禁止）。
- 例外時は `logger.error(..., exc_info=True)` 相当の情報を残す。

### ROS2（rclpy）実装ポリシー
- パラメータは `declare_parameter` → `get_parameter` の順で扱う。
- QoS は原則として明示（sensor系は SensorDataQoS 等）。
- callback は軽量に保つ（推論/IOが重い場合はキュー/別スレッド等を検討し、少なくともブロッキングを避ける）。
