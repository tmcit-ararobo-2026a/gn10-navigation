# gn10-navigation

コストマップ生成、経路生成、経路追従などのナビゲーション機能を提供するROS2パッケージです。

## 目次

1. [概要](#1-概要)
2. [コントリビューション](#2-コントリビューション)
3. [ビルド・使い方](#3-ビルド使い方)
4. [システム構成](#4-システム構成)
5. [ライセンス](#5-ライセンス)

## 1. 概要

経路生成：2D A*
経路追従：Pure Pursuit / PID制御

## 2. コントリビューション

[CONTRIBUTING.md](./CONTRIBUTING.md) を参照してください。

## 3. ビルド・使い方

必要なパッケージをインストール

```bash
sudo apt update
sudo apt install -y nlohmann-json3-dev
```

rosdepで依存関係をインストール

```bash
rosdep update --rosdistro humble
rosdep install --from-paths . --ignore-src -y --rosdistro humble
```

ビルド

```bash
colcon build --symlink-install --package-select gn10_navigation
```

読み込み

```bash
source install/setup.bash
```

コストマップ生成ノードの起動

```bash
# Blue チームとして起動
ros2 launch gn10_navigation costmap_generator.launch.py team_color:=blue

# Red チームとして起動
ros2 launch gn10_navigation costmap_generator.launch.py team_color:=red
```

## 4. システム構成

<!-- ROS2ノード構成・STM32との通信方式・ハードウェア構成 等 -->
<!-- 依存クラスが3つ以上ある場合は docs/uml/ にUML図を作成し、ここにリンクする -->

## 5. ライセンス

本リポジトリは [MITライセンス](./LICENSE) のもとで公開されています。

## STM32新仕様向けAPI整備（T01）

ROS型は本パッケージが所有し、manager・robot_portには依存しない。
`-DBUILD_RUNTIME_NODES=OFF` で実行ノードのビルドを分離できる。
Service・Actionの型と取消・lease対応は同じ作業ブランチで段階的に追加する。

## 経路安全修正（T06）

独自0〜254コストをunsignedとして比較し、描画順で高コストを下げない。
要求端点をPathへのコピー前に補正し、全線分をsupercoverで検査する。
セル角抜け、境界の両隣、占有終端、開始補正区間の障害物通過を許可しない。
占有開始位置から最寄り自由セルへの単純snapは安全な接続を保証できないため拒否する。
同一セル内の短距離も実際の開始・目標座標を使用する。

## ComputePath Service（T07）

`/navigation/compute_path` はmap座標の開始・目標Poseとgenerationを受け、Pathまたは
NO_MAP / INVALID_START / INVALID_GOAL / START_BLOCKED / GOAL_BLOCKED /
NO_PATH / PATH_UNSAFE / INTERNAL_ERRORを返す。generationはそのまま返す。
Quaternion・有限座標・地図サイズ・軸に平行な地図原点を検査する。
計画失敗は空Pathを公開し、古い計画を残さない。Service自身はTFを要求せず、
開始Poseの鮮度と遅延結果の世代判定は呼出側が実行前に検査する。
旧 `/goal_pose` 入力は既定無効で、単独の従来操作時のみ `enable_legacy_goal_input:=true` を指定する。
