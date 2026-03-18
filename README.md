# ros2-camera-onnx-nav

画像を入力とした行動モデルによる移動ロボットの自律走行を目的とする

## 構成

- `unity-nav-ws/` : Unity 環境で ROS2-For-Unity を使用して行動モデルの推論を行うワークスペース
- `real-nav-ws/` : 実機移動ロボットへ行動モデルを適用して自律走行させるワークスペース

## 開発環境
- ROS2 jazzy
- Python 3.12

## Python 仮想環境（共通）

- 仮想環境はリポジトリ直下の `.venv/` を共通利用
- 生成（初回のみ）

```bash
cd .
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -m pip install --upgrade pip
```

- 各ワークスペースで作業するときは、対象ワークスペース直下へ移動してから有効化

```bash
cd unity-nav-ws
source ../.venv/bin/activate
```

```bash
cd real-nav-ws
source ../.venv/bin/activate
```
```

## 想定する行動モデル

- Unity ML-Agents を用いて学習した深層強化学習モデル (ONNX)
- 入力
  - カメラ画像
    - 複数枚でも可
    - 基本的には４値化環境で学習させる想定
  - 目的地までの距離(m)
  - 目的地までの角度
    - -180 ~ 180(deg)
- 出力
  - 速度 $v$ [m/s]
    - -1<= $v$ <=1
    - **0以下は0にクリップ**（安全のため後進は禁止）
  - 角速度 $\omega$ [rad/s]
    - -1<= $\omega$ <=1
