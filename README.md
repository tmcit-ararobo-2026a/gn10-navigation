# gn10-navigation

NHKロボコン2026等の自動走行要求を満たすナビゲーションパッケージです。
単一責任の原則（KISS原則）およびDAG構造のデータフローに基づき、4つの独立したノード群で構成されます。

## 目次

1. [概要](#1-概要)
2. [システムアーキテクチャ](#2-システムアーキテクチャ)
3. [通信仕様](#3-通信仕様)
4. [ビルド・実行方法](#4-ビルド実行方法)
5. [パラメータ設定](#5-パラメータ設定)
6. [コントリビューション](#6-コントリビューション)
7. [ライセンス](#7-ライセンス)

---

## 1. 概要

- **target_manager_node**: メインマイコンからの指示に応じた最適な投擲・目標座標（`/goal_pose`）の管理と発行。VSCode Extension等からのメモリ更新およびYAML保存サービスを提供。
- **costmap_generator_node**: フィールド定義（`nhk2026_map.json`）に基づくMulti-Source BFSを用いた波及コスト付き地図データ（`nav_msgs/msg/OccupancyGrid`）の生成。
- **path_planner_node**: コストマップと目標姿勢に基づく8方向A*標準実装による最短・低コストな格子経路（`nav_msgs/msg/Path`）の算定。
- **path_follower_node**: ナビゲーション状態（`NavigationStatus`）のステートマシン管理、Pure Pursuitおよび終端15cm位置偏差フィードバックによる運動制御（3cm以下の目標停止精度）。

---

## 2. システムアーキテクチャ

```
メイン基板 (udp_node)
   │  /robot/navigation_command (UInt8)
   │  /robot/target_id (UInt8)
   ├──────────────────────────────┬──────────────────────────────┐
   ▼                              ▼                              │
[target_manager_node]     [path_follower_node]                   │
   │ /goal_pose                   ▲                              │
   │ (PoseStamped)                │ /planned_path (Path)         │
   ▼                              │                              │
[path_planner_node] ──────────────┘                              │
   ▲                                                             │
   │ /costmap (OccupancyGrid)                                    │
[costmap_generator_node]                                         │
                                                                 │
[path_follower_node] ───────────────────────────────────────────┘
   │ /robot/operation/cmd_vel (Twist: 機体ローカル速度)
   │ /robot/operation/navigation_status (UInt8)
   ▼
メイン基板 (udp_node)
```

---

## 3. 通信仕様

### トピック・サービス

| 通信名 | 種別 | 型 / フレーム | 発行元 | 受信元 | 概要 |
| --- | --- | --- | --- | --- | --- |
| `/robot/navigation_command` | Topic | `std_msgs/msg/UInt8` | udp_node | TargetMgr / Follower | ナビゲーション動作命令 (Sleep=0, Stanby=1, Move=2) |
| `/robot/target_id` | Topic | `std_msgs/msg/UInt8` | udp_node | TargetMgr / Follower | 目的地・追従対象ID (0..7) |
| `/robot/operation/navigation_status` | Topic | `std_msgs/msg/UInt8` | PathFollower | udp_node | ナビゲーション現在状態 (Stanby=0, Ready=1, Moving=2, Tracking=3, Goal=4, Fail=5) |
| `/robot/operation/cmd_vel` | Topic | `geometry_msgs/msg/Twist` | PathFollower | udp_node | 機体ローカル座標系速度指示値 |
| `/costmap` | Topic | `nav_msgs/msg/OccupancyGrid` | CostmapGen | PathPlanner | 障害物コスト・波及勾配付きコストマップ |
| `/goal_pose` | Topic | `geometry_msgs/msg/PoseStamped` | TargetMgr | PathPlanner | 投擲ポイント（目標姿勢） |
| `/planned_path` | Topic | `nav_msgs/msg/Path` | PathPlanner | PathFollower | A*生成経路 |
| `/target_manager/update_point` | Service | `gn10_navigation/srv/UpdatePoint` | VSCode Ext | TargetManager | 投擲ポイント（座標・角度・メモ）のメモリ更新 |
| `/target_manager/save_points` | Service | `std_srvs/srv/Trigger` | VSCode Ext | TargetManager | メモリ上の座標データをYAMLへ書き出し |
| `/debug/lookahead_point` | Topic | `visualization_msgs/msg/Marker` | PathFollower | RViz2 (Debug) | Pure Pursuit 注視点可視化 |
| `map -> base_link` | TF | TransformStamped | Localizer | Planner / Follower | 機体現在位置・姿勢 |
| `map -> bucket_bottom` | TF | TransformStamped | Perception | PathFollower | 移動バケツ目標姿勢 |

---

## 4. ビルド・実行方法

ROS 2 Humble環境でビルドします。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gn10_navigation
source install/setup.bash
```

全ノードを一括起動する場合:

```bash
ros2 launch gn10_navigation navigation.launch.py
```

RViz2を同時に起動する場合:

```bash
ros2 launch gn10_navigation navigation.launch.py use_rviz:=true
```

---

## 5. パラメータ設定

- `config/costmap_params.yaml`: 解像度、マップ限界、インフレーション半径 (0.35m)・マージン (0.50m)、基準コスト設定
- `config/path_planner_params.yaml`: map/baseフレーム、A*ペナルティ倍率
- `config/path_follower_params.yaml`: 制御周期 (50Hz)、最高速度 (1.5m/s)、最大加速度 (1.0m/s^2)、注視距離 (0.4m)、停止許容誤差 (0.03m)
- `config/shooting_points.yaml`: 各TargetId (0〜7) の投擲・目標座標、姿勢、メモ

---

## 6. コントリビューション

[CONTRIBUTING.md](CONTRIBUTING.md) を参照してください。

## 7. ライセンス

MIT License
