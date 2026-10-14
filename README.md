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

経路計画はService、追従はActionで利用します。速度は`PathVelocity`として配信し、アクチュエータへの送信は呼出側が担当します。

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
ros2 launch gn10_navigation navigation.launch.py use_rviz:=true
```

自己位置推定を別途起動し、点群・Poseの時刻に対応する`map → base_link` TFを用意してください。bag再生では`use_sim_time:=true`を指定します。

地図は[config/nhk2026_map.json](config/nhk2026_map.json)、各ノードの設定は[コストマップ](config/costmap_params.yaml)、[経路計画](config/path_planner_params.yaml)、[経路追従](config/path_follower_params.yaml)を参照してください。`navigation.launch.py`の`map_file_path`、`costmap_params`、`planner_params`、`follower_params`で変更できます。

型のみをビルドする場合は`--cmake-args -DBUILD_RUNTIME_NODES=OFF`を指定します。試験は以下で実行できます。

```bash
colcon test --packages-select gn10_navigation
colcon test-result --verbose
```

## 4. システム構成

### ノードとインターフェース

| ノード | インターフェース | 役割 |
| :--- | :--- | :--- |
| `costmap_generator_node` | `/costmap` | 独自の0〜254コストを持つコストマップ生成 |
| `path_planner_node` | `/navigation/compute_path` (`ComputePath`) | 開始・目標Poseから経路を計画 |
| `path_follower_node` | `/navigation/follow_path` (`FollowPath`) | 経路追従と進捗・終了結果の通知 |
| `path_follower_node` | `/navigation/control_lease` (`ControlLease`) | 制御許可の受信 |
| `path_follower_node` | `/navigation/path_velocity` (`PathVelocity`) | `base_link`基準の速度と有効性の配信 |

### 経路計画

`ComputePath`はmap座標の開始・目標Poseと`generation`を受け取り、同じ世代番号とPathまたはエラーを返します。地図の未取得、不正Pose、占有端点、経路なし、安全性検査失敗などを区別します。計画失敗時は空Pathを公開し、古い経路を残しません。

コストはunsignedとして比較し、セル境界・角・端点を含めて線分が通るセルを検査します。占有開始位置から自由セルへの単純な移動補正は行いません。ServiceはTFを要求しないため、呼出側で開始Poseの鮮度と遅延結果の世代を確認してください。

### 経路追従と取消

`FollowPath`はPath・`producer_session`・`generation`を受け取り、進捗とSUCCESS、CANCELED、INVALID_PATH、LEASE_EXPIRED、TF_ERROR、PATH_DEVIATIONを返します。開始前に同じsession・世代の有効なleaseが必要です。

呼出側は新しいstamp・`update_sequence`のleaseを20ms周期で配信します。追従ノードは生成時点から100msの単調期限を監視し、重複・逆行・未来時刻・期限超過を拒否します。実行中の別goalは受け付けず、取消後に新世代で差し替えます。

取消・lease失効・TF異常・経路逸脱では、ゼロ速度の配信、経路と制御履歴の消去、終了結果の通知を行います。`control_history_cleared`で履歴消去を確認できます。TFやleaseの復旧だけでは旧経路を再開しません。到着は位置と最終Yawで判定します。閾値は実機での調整・確認が必要です。

従来の`/goal_pose`・`/planned_path`入力は既定で無効です。単独操作で使用する場合は、それぞれ`enable_legacy_goal_input`・`enable_legacy_path_input`を有効にします。従来のPath入力とActionは排他的に使用します。

試験の詳細は[経路・追従の検証記録](docs/new_navigation_verification.md)を参照してください。

## 5. ライセンス

本リポジトリは[MITライセンス](LICENSE)のもとで公開されています。
