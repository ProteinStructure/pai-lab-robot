
# Multi-Robot Laboratory Automation

## 概要

Python、Pygame、ROS 2を用いた研究室向け自律ロボットシミュレーションです。

2台の移動ロボットが協調し、検体搬送、実験工程の実行、障害物への対応を行います。

## 主な機能

- A*探索アルゴリズムによる経路計画
- ROS 2 Topicによる複数ロボット間通信
- Robot Aによる検体搬送と実験工程の実行
- Robot Bによる障害物の退避
- 通路閉鎖時の支援要求と経路再計画
- Pygameによるシミュレーションの可視化

## 使用技術

- Python 3
- ROS 2 Humble
- Pygame 2.6.1
- Docker
- A* path planning

## システム構成

### Robot A: Laboratory workflow robot

検体を回収し、以下の工程を実行します。

1. 検体回収
2. 秤量
3. 試薬添加
4. 混合
5. 分析

通路が障害物によって塞がれた場合、Robot Bに支援を要求します。

### Robot B: Support robot

Robot Aから支援要求を受け取り、障害物となる搬送カートを退避場所へ移動させます。

作業完了後、Robot Aに結果を通知します。

### ROS 2 Communication

主に以下のTopicを使用します。

- `/lab/robot_a/state`
- `/lab/robot_a/assistance_request`
- `/lab/robot_b/state`
- `/lab/robot_b/status`
- `/lab/environment/state`

## 実行環境

ROS 2 Humbleを含むDocker環境を使用します。

Dockerイメージ:

`airobotbook/ros2-desktop-ai-robot-book-humble:latest`

Pygameがインストールされていない場合は、コンテナ内で以下を実行します。

```bash
python3 -m pip install --user pygame==2.6.1
```

## 実行方法

ROS 2の環境を読み込みます。

```bash
source /opt/ros/humble/setup.bash
cd ~/pai_multi_robot
```

3つのターミナルでそれぞれ実行します。

ターミナル1: Visualizer

```bash
python3 lab_system.py view
```

ターミナル2: Robot B

```bash
python3 lab_system.py B
```

ターミナル3: Robot A

```bash
python3 lab_system.py A
```

## 操作方法

Visualizerの終了: `ESC`

## ファイル構成

- `main.py`: 基本的な2D研究室シミュレーション
- `lab_system.py`: ROS 2による複数ロボット協調システム
- `requirements.txt`: Python依存ライブラリ
- `.gitignore`: Git管理対象外ファイルの設定

## 実装上の制限

本プロジェクトは2Dシミュレーションです。

ロボットの移動、障害物の退避、秤量、試薬添加、混合、分析はシミュレーション上の状態変化として表現しています。

実機センサーによる障害物認識、物理的な把持動作、実際の試薬計量や分析は実装していません。
