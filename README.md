# gn10-navigation

NHKロボコン2026向けの自動走行ナビゲーションパッケージです。
単一責任の原則（KISS原則）と疎結合なトピック通信に基づき、4つの独立したノードで構成されています。

## 1. ノード構成

- **`target_manager_node`**: Target IDに応じた目標座標（`/goal_pose`）および射出ベルト速度（`/robot/operation/belt_launcher_speed`）を管理・発行します。
- **`costmap_generator_node`**: フィールド定義（`nhk2026_map.json`）からMulti-Source BFSを用いてインフレーション付きコストマップ（`/costmap`）を生成します。
- **`path_planner_node`**: 8方向A*探索、反復ラプラシアン付き動的B-spline平滑化、および旋回区間全域のフットプリント安全判定を行い、量子化誤差のない計画経路（`/planned_path`）を発行します。
- **`path_follower_node`**: 接線フィードフォワード速度と直交位置フィードバック速度の単一統合制御則、1次最大加速度リミッタ、およびゴール近傍の静止摩擦補償により、高速追従と±5mm・±1.0度以内の精密停止を実現します。

---

## 2. システムアーキテクチャ

```
メイン基板 (udp_node)
   │  /robot/navigation_command (UInt8)
   │  /robot/target_id (UInt8)
   ├──────────────────────────────┐
   ▼                              ▼
[target_manager_node]     [path_follower_node]
   │ /goal_pose                   ▲
   │ (PoseStamped)                │ /planned_path (Path)
   ▼                              │
[path_planner_node] ──────────────┘
   ▲
   │ /costmap (OccupancyGrid)
[costmap_generator_node]
```

詳細な処理フローとパラメータ一覧は [docs/README.md](docs/README.md) を参照してください。

---

## 3. 通信仕様

| 通信名 | 種別 | 型 | 発行元 | 受信元 | 概要 |
| --- | --- | --- | --- | --- | --- |
| `/robot/navigation_command` | Topic | `std_msgs/msg/UInt8` | udp_node | TargetMgr / Follower | 動作命令 (`Sleep=0, Stanby=1, Move=2`) |
| `/robot/target_id` | Topic | `std_msgs/msg/UInt8` | udp_node | TargetMgr / Follower | 目標ID (`0..7`) |
| `/robot/operation/navigation_status` | Topic | `std_msgs/msg/UInt8` | PathFollower | udp_node | 現在状態 (`Stanby=0, Ready=1, Moving=2, Tracking=3, Goal=4, Fail=5`) |
| `/robot/operation/cmd_vel` | Topic | `geometry_msgs/msg/Twist` | PathFollower | udp_node | 機体ローカル速度指令 |
| `/robot/operation/belt_launcher_speed` | Topic | `std_msgs/msg/Float32` | TargetMgr | udp_node | 射出ベルト目標速度 [m/s] |
| `/costmap` | Topic | `nav_msgs/msg/OccupancyGrid` | CostmapGen | PathPlanner | 障害物インフレーション付きコストマップ |
| `/goal_pose` | Topic | `geometry_msgs/msg/PoseStamped` | TargetMgr | PathPlanner | 目標位置・姿勢 |
| `/planned_path` | Topic | `nav_msgs/msg/Path` | PathPlanner | PathFollower | 姿勢計画付き平滑化経路 |
| `/debug/reference_point` | Topic | `visualization_msgs/msg/Marker` | PathFollower | RViz2 | 遅延補償付き参照点の可視化マーカー |
| `/target_manager/get_points` | Service | `gn10_navigation/srv/GetPoints` | Tool | TargetManager | 投擲ポイントデータの取得 |
| `/target_manager/update_point` | Service | `gn10_navigation/srv/UpdatePoint` | Tool | TargetManager | 投擲ポイントのメモリ更新 |
| `/target_manager/save_points` | Service | `std_srvs/srv/Trigger` | Tool | TargetManager | 投擲ポイントのYAML保存 |
| `map -> base_link` | TF | `TransformStamped` | Localizer | Planner / Follower | 機体現在位置・姿勢 |
| `map -> bucket_bottom` | TF | `TransformStamped` | Perception | PathFollower | 移動バケツ位置 |

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
- [`config/shooting_points_blue.yaml`](config/shooting_points_blue.yaml) / [`config/shooting_points_red.yaml`](config/shooting_points_red.yaml): 各Target IDの目標座標

## 6. ライセンス

MIT License
