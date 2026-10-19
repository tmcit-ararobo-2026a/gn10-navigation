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
- 経路追従：Pure Pursuit / PID制御による並進速度・Yaw速度の計算

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

自己位置推定を別途起動し、点群・Poseの時刻に対応する`map → base_link` TFを用意してください。bag再生では`use_sim_time:=true`を指定します。

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
| `path_follower_node` | `/navigation/follow_path` (`FollowPath`) | 経路追従と進捗・終了結果の通知 |
| `path_follower_node` | `/navigation/control_lease` (`ControlLease`) | 制御許可の受信 |
| `path_follower_node` | `/navigation/path_velocity` (`PathVelocity`) | `base_link`基準の速度と有効性の配信 |

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

### 経路追従と取消

`FollowPath`はPath・`producer_session`・`generation`を受け取り、進捗とSUCCESS、CANCELED、INVALID_PATH、LEASE_EXPIRED、TF_ERROR、PATH_DEVIATIONを返します。開始前に同じsession・世代の有効なleaseが必要です。

呼出側は新しいstamp・`update_sequence`のleaseを20ms周期で配信します。追従ノードは生成時点から100msの単調期限を監視し、重複・逆行・未来時刻・期限超過を拒否します。実行中の別goalは受け付けず、取消後に新世代で差し替えます。

取消・lease失効・TF異常・経路逸脱では、ゼロ速度の配信、経路と制御履歴の消去、終了結果の通知を行います。`control_history_cleared`で履歴消去を確認できます。TFやleaseの復旧だけでは旧経路を再開しません。到着は位置と最終Yawで判定します。閾値は実機での調整・確認が必要です。

plannerの`/goal_pose`入力は常に有効です。このブランチのfollowerで`/planned_path`入力を使用する場合は`enable_legacy_path_input`を有効にします。Path入力とActionは排他的に使用します。followerの標準型化は別の[PR #19](https://github.com/tmcit-ararobo-2026a/gn10-navigation/pull/19)で扱います。


## 5. ライセンス

本リポジトリは[MITライセンス](LICENSE)のもとで公開されています。
