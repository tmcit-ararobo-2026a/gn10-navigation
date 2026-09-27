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
