# gn10-navigation

コストマップ生成、経路生成、経路追従などのナビゲーション機能を提供するROS 2パッケージです。

## 目次

1. [概要](#1-概要)
2. [コントリビューション](#2-コントリビューション)
3. [ビルド・使い方](#3-ビルド使い方)
4. [システム構成](#4-システム構成)
5. [ライセンス](#5-ライセンス)

## 1. 概要

- コストマップ生成：フィールド地図と障害物から走行コストを生成
- 経路生成：2D A*による経路計画
- 経路追従：Pure Pursuitによる並進と、移動バケツ方向への常時旋回

経路計画はService、追従は標準型のPathトピックで利用します。追従ノードは標準型のTwist速度とLED用BoolをUDPノードへ配信します。
経路計画は標準型のゴール・経路トピックで利用します。追従はActionで利用し、速度は`PathVelocity`として配信します。アクチュエータへの送信は呼出側が担当します。

## 2. コントリビューション

[CONTRIBUTING.md](CONTRIBUTING.md)を参照してください。

## 3. ビルド・使い方

ROS 2 Humbleを使用します。ワークスペースで依存関係をインストールします。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y nlohmann-json3-dev
rosdep install --from-paths src/gn10-navigation --ignore-src -y --rosdistro humble
colcon build --symlink-install --packages-select gn10_navigation
source install/setup.bash
```

コストマップ生成ノードだけを起動する場合:

```bash
ros2 launch gn10_navigation costmap_generator.launch.py
```

コストマップ・経路計画・追従をまとめて起動する場合:

```bash
ros2 launch gn10_navigation navigation.launch.py
```

RVizも起動する場合:

```bash
ros2 launch gn10_navigation navigation.launch.py use_rviz:=true
```

自己位置推定の`map → base_link`と、LiDAR認識の`map → bucket_bottom` TFを別途用意してください。bag再生では`use_sim_time:=true`を指定します。バケツTFが有効になると、経路未受信でも旋回指令を配信します。

地図は[config/nhk2026_map.json](config/nhk2026_map.json)、各ノードの設定は[コストマップ](config/costmap_params.yaml)、[経路計画](config/path_planner_params.yaml)、[経路追従](config/path_follower_params.yaml)を参照してください。`navigation.launch.py`の`map_file_path`、`costmap_params`、`planner_params`、`follower_params`で変更できます。

型のみをビルドする場合は`--cmake-args -DBUILD_RUNTIME_NODES=OFF`を指定します。ローカルに`test/`ファイルを配置した場合、試験は以下で実行できます。

```bash
colcon test --packages-select gn10_navigation
colcon test-result --verbose
```

## 4. システム構成

### ノードとインターフェース

| ノード | インターフェース | 役割 |
| :--- | :--- | :--- |
| `costmap_generator_node` | `/costmap` | 独自の0〜254コストを持つコストマップ生成 |
| `path_planner_node` | `/goal_pose` (`geometry_msgs/msg/PoseStamped`) | goal managerからmap座標のゴールを受信 |
| `path_planner_node` | `/costmap` (`nav_msgs/msg/OccupancyGrid`)・TF `map → base_link` | 地図と現在の開始位置を取得 |
| `path_planner_node` | `/planned_path` (`nav_msgs/msg/Path`) | 生成した経路。失敗時は空Path |
| `path_planner_node` | `/navigation/compute_path` (`ComputePath`) | 開始・目標Poseから経路を計画 |
| `path_follower_node` | `/robot/command/cmd_vel` (`geometry_msgs/msg/Twist`) | `base_link`基準の並進・旋回速度 |
| `path_follower_node` | `/robot/command/can_move` (`std_msgs/msg/Bool`) | LED用の正常／失敗表示 |
| `path_follower_node` | `/speed_path` (`nav_msgs/msg/Path`) | 速度計画の可視化 |

### 経路計画

goal managerが`/goal_pose`へゴールを送信すると、その受信時の`map → base_link` TFを開始位置として、指定したXY位置までの経路を1回生成します。RVizの「2D Goal Pose」も利用できます。ゴールの`header.frame_id`は`map_configs.map_frame`（既定`map`）と一致させ、有限の位置と正規化したquaternionを指定してください。他の座標系への変換は行いません。Pathの末尾姿勢にはゴールの姿勢を格納しますが、planner自身は旋回や到着判定を行いません。

```bash
ros2 launch gn10_navigation path_planner.launch.py
# 別端末で環境をsourceし、通行可能なmap座標を指定
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}'
```

地図・TFが未取得、不正Pose、占有端点、経路なし、障害物検査失敗の場合は空Pathを公開し、理由をログへ記録します。ゴールを保持しないため、地図・TFの到着や更新だけでは再計画しません。準備ができたらゴールを再指定してください。同じ位置の再指定でも、その時点の自己位置から計画します。

ゴール入力と経路出力はreliable/volatile/depth 1、地図入力はreliable/transient_local/depth 1です。既存のA*とB-splineを使用し、コストはunsignedとして比較します。セル境界・角・端点を含む線分の通行検査と、実際の開始・ゴール位置への端点補正を行います。占有開始位置から自由セルへの単純な移動補正は行いません。

plannerの独自`ComputePath`サービス・型定義と`enable_legacy_goal_input`は撤去しました。plannerは標準ROS型だけを使用します。旧Service呼出側は`/goal_pose`へ移行し、旧生成型を残さないようクリーンなbuild/installを使用してください。followerの`FollowPath`・`ControlLease`・`PathVelocity`はこの変更では維持します。

### 経路追従とバケツ方向への旋回

`/planned_path`は常時受信し、新しいPathで並進経路と並進制御履歴を置き換えます。空Pathで並進を取り消します。到着は位置だけで判定し、Pathの姿勢は旋回に使用しません。既存の速度・加減速・jerk・車輪速度の制限を使用します。

旋回は並進から独立し、起動後・走行中・到着後も`bucket_frame`（既定`bucket_bottom`）のXY位置へ`base_link`の正面+Xを向けます。底面TFの姿勢は使いません。`heading_gain`、`max_angular_speed`、`max_angular_acceleration`で調整します。

バケツTFが未受信・不正・`max_bucket_age`（既定0.3秒）を超過した場合やXY方向が定義できない場合は、並進を続けて旋回だけゼロにし、`can_move=false`を配信します。TFの更新停止はsteady時計でも判定し、復帰後は旋回を再開します。自己位置TF異常では全速度をゼロにして並進経路を解除し、経路不正・逸脱でも並進を終了します。解除した経路の再開には新しいPathが必要です。

`can_move`はLED専用で、正常な待機・到着時もtrueです。経路異常の表示は有効な新Pathまたは空Pathで解除します。安全許可やNavigationStatusの代わりには使いません。UDP側の速度受理に必要な`localization_valid`と`navigation_status`は自己位置推定・goal manager等から別途配信してください。

plannerの`/goal_pose`入力は常に有効です。このブランチのfollowerで`/planned_path`入力を使用する場合は`enable_legacy_path_input`を有効にします。Path入力とActionは排他的に使用します。followerの標準型化は別の[PR #19](https://github.com/tmcit-ararobo-2026a/gn10-navigation/pull/19)で扱います。
followerの`FollowPath`・`ControlLease`・`PathVelocity`と、`heading_mode`・`enable_legacy_path_input`・lease関連パラメータは撤去しました。旧呼出側との互換性はありません。plannerの独自`ComputePath`は維持します。plannerを`/goal_pose`で操作する場合は、従来どおり`enable_legacy_goal_input`を有効にしてください。旧生成型が残らないよう、移行時はクリーンなbuild/installを使用してください。


## 5. ライセンス

本リポジトリは[MITライセンス](LICENSE)のもとで公開されています。
