# gn10-navigation

NHKロボコン2026向けの自動走行ナビゲーションパッケージです。
疎結合なトピック通信に基づき、5つのノードで構成されています。

## 1. ノード構成

- **`target_manager_node`**: Target IDに応じた目標座標（`/goal_pose`）および射出ベルト速度（`/robot/operation/belt_launcher_speed`）を発行します（移動バケツ追従時は射出速度発行をスキップ）。
- **`costmap_generator_node`**: フィールド定義（`nhk2026_map.json`）からMulti-Source BFSを用いてインフレーション付きコストマップ（`/costmap`）を生成します。
- **`path_planner_node`**: 8方向A*探索、B-spline平滑化、および旋回区間のフットプリント安全判定を行い、計画経路（`/planned_path`）を発行します。
- **`path_follower_node`**: 速度プロファイル計画、接線フィードフォワード＋直交位置フィードバック制御、1次最大加速度リミッタ、および停止判定に基づき経路追従制御を行います。また、移動バケツ追従（Tracking）時はTF基準の目標位置EMA平滑化、PID制御、不感帯およびタイムアウト処理により追従姿勢制御を行います。
- **`moving_bucket_launcher_node`**: 移動バケツ（Target ID 7: `MovingBucket`）追従かつ動作命令 `Move` 時に、TF（既定: `bucket_bottom` + `bucket_height`）またはLiDAR Perceptionトピックから `base_link` 基準で弾道射出速度を計算し、EMA平滑化を経て `/robot/operation/belt_launcher_speed` へ発行します（固定ターゲット時や `Sleep`/`Stanby` 時は配信停止）。

---

## 2. システムアーキテクチャ

```
メイン基板 (udp_node)
   │  /robot/navigation_command (UInt8)
   │  /robot/target_id (UInt8)
   ├──────────────────────────────┬──────────────────────────────┐
   ▼                              ▼                              ▼
[target_manager_node]     [path_follower_node]       [moving_bucket_launcher_node]
   │ /goal_pose                   ▲                              ▲
   │ (PoseStamped)                │ /planned_path (Path)         │ /bucket_opening_tracker/target
   ▼                              │                              │ (MovingBucketTrack)
[path_planner_node] ──────────────┘                  [LiDAR-Perception-System]
   ▲
   │ /costmap (OccupancyGrid)
[costmap_generator_node]

※ /robot/operation/belt_launcher_speed 配信の排他制御:
- target_id != 7: target_manager_node が Move 遷移時に配信
- target_id == 7 (MovingBucket) かつ navigation_command == Move (2): moving_bucket_launcher_node が動的配信
```

詳細な処理フローとパラメータ一覧は [docs/README.md](docs/README.md) を参照してください。

---

## 3. 通信仕様

| 通信名 | 種別 | 型 | 発行元 | 受信元 | 概要 |
| --- | --- | --- | --- | --- | --- |
| `/robot/navigation_command` | Topic | `std_msgs/msg/UInt8` | udp_node | TargetMgr / Follower / MovingBucketLauncher | 動作命令 (`Sleep=0, Stanby=1, Move=2`) |
| `/robot/target_id` | Topic | `std_msgs/msg/UInt8` | udp_node | TargetMgr / Follower / MovingBucketLauncher | 目標ID (`0..7`, `MovingBucket=7`) |
| `/robot/operation/navigation_status` | Topic | `std_msgs/msg/UInt8` | PathFollower | udp_node | 現在状態 (`Stanby=0, Ready=1, Moving=2, Tracking=3, Goal=4, Fail=5`) |
| `/robot/operation/cmd_vel` | Topic | `geometry_msgs/msg/Twist` | PathFollower | udp_node | 機体ローカル速度指令 |
| `/robot/operation/belt_launcher_speed` | Topic | `std_msgs/msg/Float32` | TargetMgr / MovingBucketLauncher | udp_node | 射出ベルト目標速度 [m/s]（target_id・動作命令に応じて排他配信） |
| `/bucket_opening_tracker/target` | Topic | `lidar_perception_system/msg/MovingBucketTrack` | LPS | MovingBucketLauncher | 移動バケツ上部開口部中心位置・トラッキング情報 |
| `/bucket_opening_tracker/bottom` | Topic | `lidar_perception_system/msg/BucketBottom` | LPS | MovingBucketLauncher | 移動バケツ底位置（`use_bottom_topic=true` 時のフォールバック） |
| `/costmap` | Topic | `nav_msgs/msg/OccupancyGrid` | CostmapGen | PathPlanner | 障害物インフレーション付きコストマップ |
| `/goal_pose` | Topic | `geometry_msgs/msg/PoseStamped` | TargetMgr | PathPlanner | 目標位置・姿勢 |
| `/planned_path` | Topic | `nav_msgs/msg/Path` | PathPlanner | PathFollower | 姿勢計画付き平滑化経路 |
| `/debug/reference_point` | Topic | `visualization_msgs/msg/Marker` | PathFollower | RViz2 | 遅延補償付き参照点の可視化マーカー |
| `/target_manager/get_points` | Service | `gn10_navigation/srv/GetPoints` | Tool | TargetManager | 投擲ポイントデータの取得 |
| `/target_manager/update_point` | Service | `gn10_navigation/srv/UpdatePoint` | Tool | TargetManager | 投擲ポイントのメモリ更新 |
| `/target_manager/save_points` | Service | `std_srvs/srv/Trigger` | Tool | TargetManager | 投擲ポイントのYAML保存 |
| `map -> base_link` | TF | `TransformStamped` | Localizer | Planner / Follower / MovingBucketLauncher | 機体現在位置・姿勢 |
| `map -> bucket_bottom` | TF | `TransformStamped` | Perception | Follower / MovingBucketLauncher | 移動バケツ底面位置 |

---

## 4. ビルド・実行方法

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gn10_navigation
source install/setup.bash

# 全ノード起動
ros2 launch gn10_navigation navigation.launch.py

# RViz2付きで起動
ros2 launch gn10_navigation navigation.launch.py use_rviz:=true
```

---

## 5. 設定ファイル

- [`config/costmap_params.yaml`](config/costmap_params.yaml): 解像度、マップ範囲、インフレーション設定
- [`config/path_planner_params.yaml`](config/path_planner_params.yaml): A*重み、B-spline平滑化度合い、機体外形、安全旋回設定
- [`config/path_follower_params.yaml`](config/path_follower_params.yaml): 物理速度・加速度制限、位置・姿勢フィードバックゲイン、ゴール許容誤差
- [`config/moving_bucket_launcher_params.yaml`](config/moving_bucket_launcher_params.yaml): 移動バケツの高さ・上部開口部弾道計算パラメータ
- [`config/shooting_points_blue.yaml`](config/shooting_points_blue.yaml) / [`config/shooting_points_red.yaml`](config/shooting_points_red.yaml): 各Target IDの目標座標

## 6. ライセンス

MIT License
