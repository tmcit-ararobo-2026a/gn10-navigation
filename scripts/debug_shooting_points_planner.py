#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
debug_shooting_points_planner.py
---------------------------------
自己位置推定や他ノードが動いていない単体環境下で、
shooting-point の目標地点 YAML ファイル (red / blue) を読み込み、
path_planner_node に各目的地間のパスを順次計算させて RViz 等に可視化・検証するデバッグスクリプト。
"""

import math
import os
import sys
import time
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


def get_default_shooting_points_path(zone: str = 'blue', custom_path: str = '') -> str:
    """shooting-point の YAML ファイルパスを探索・解決する"""
    if custom_path and Path(custom_path).exists():
        return str(Path(custom_path).resolve())

    env_path = os.environ.get('GN10_SHOOTING_POINTS_PATH')
    if env_path and Path(env_path).exists():
        return str(Path(env_path).resolve())

    zone = zone.strip().lower()
    filenames = []
    if zone == 'red':
        filenames = ['shooting_points_red.yaml', 'shooting_point_red.yaml']
    else:
        filenames = ['shooting_points_blue.yaml', 'shooting_point_blue.yaml', 'shooting_points.yaml', 'shooting_point.yaml']

    candidates = []
    for fn in filenames:
        candidates.extend([
            Path.home() / 'ros2_ws/data/shooting-point' / fn,
            Path.home() / 'ros2_ws/data/shooting-points' / fn,
            Path(f'data/shooting-point/{fn}').resolve(),
            Path(f'data/shooting-points/{fn}').resolve(),
            Path.home() / 'ros2_ws/data/shooting-point-config' / fn,
        ])

    for c in candidates:
        if c.exists():
            return str(c.resolve())

    # フォールバック (パッケージ share ディレクトリ)
    from ament_index_python.packages import get_package_share_directory
    try:
        share = Path(get_package_share_directory('gn10_navigation'))
        if zone == 'red':
            p = share / 'config/shooting_points_red.yaml'
        else:
            p = share / 'config/shooting_points_blue.yaml'
        if p.exists():
            return str(p.resolve())
    except Exception:
        pass

    raise FileNotFoundError(f"Shooting points file for zone '{zone}' could not be found.")


def euler_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Yaw [rad] から (x, y, z, w) クォータニオンを算出"""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def compute_path_metrics(path_msg: NavPath, costmap: Optional[OccupancyGrid] = None) -> Dict[str, Any]:
    """パスの総距離、最大曲率、推定最小旋回半径、最大通過コストを算出"""
    poses = path_msg.poses
    if len(poses) < 2:
        return {'length': 0.0, 'max_curvature': 0.0, 'min_radius': float('inf'), 'max_cost': 0}

    total_len = 0.0
    pts = [(p.pose.position.x, p.pose.position.y) for p in poses]

    # 1. 経路長
    for i in range(len(pts) - 1):
        total_len += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])

    # 2. 幾何曲率 (3点からの方向変化率)
    max_k = 0.0
    for i in range(1, len(pts) - 1):
        d1x = pts[i][0] - pts[i - 1][0]
        d1y = pts[i][1] - pts[i - 1][1]
        d2x = pts[i + 1][0] - pts[i][0]
        d2y = pts[i + 1][1] - pts[i][1]
        l1 = math.hypot(d1x, d1y)
        l2 = math.hypot(d2x, d2y)
        if l1 > 1e-4 and l2 > 1e-4:
            th1 = math.atan2(d1y, d1x)
            th2 = math.atan2(d2y, d2x)
            dth = th2 - th1
            while dth > math.pi:
                dth -= 2.0 * math.pi
            while dth < -math.pi:
                dth += 2.0 * math.pi
            ds = 0.5 * (l1 + l2)
            curv = abs(dth) / ds
            if curv > max_k:
                max_k = curv

    min_r = (1.0 / max_k) if max_k > 1e-4 else float('inf')

    # 3. 最大通過コスト
    max_cost = 0
    if costmap and len(costmap.data) > 0:
        res = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        width = costmap.info.width
        height = costmap.info.height

        for x, y in pts:
            gx = int(round((x - origin_x) / res))
            gy = int(round((y - origin_y) / res))
            if 0 <= gx < width and 0 <= gy < height:
                c = costmap.data[gy * width + gx]
                if c > max_cost:
                    max_cost = c

    return {
        'length': total_len,
        'max_curvature': max_k,
        'min_radius': min_r,
        'max_cost': max_cost
    }


class DebugShootingPointsPlannerNode(Node):
    def __init__(self):
        super().__init__('debug_shooting_points_planner')

        # パラメータ宣言
        self.declare_parameter('zone', 'blue')
        self.declare_parameter('shooting_points_file', '')
        self.declare_parameter('mode', 'tour')  # 'tour', 'from_start', 'all_pairs', 'both'
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.zone = self.get_parameter('zone').get_parameter_value().string_value.strip().lower()
        custom_file = self.get_parameter('shooting_points_file').get_parameter_value().string_value.strip()
        self.mode = self.get_parameter('mode').get_parameter_value().string_value.strip().lower()
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # 目標地点 YAML ファイルの読み込み
        try:
            self.yaml_path = get_default_shooting_points_path(self.zone, custom_file)
            self.get_logger().info(f"Loaded shooting points file: {self.yaml_path}")
            with open(self.yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.shooting_points: List[Dict[str, Any]] = data.get('shooting_points', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load shooting points: {e}")
            self.shooting_points = []

        self.get_logger().info(f"Target count: {len(self.shooting_points)} targets loaded for zone '{self.zone}'.")

        # QoS
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # パブリッシャー
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', volatile_qos)
        self.target_markers_pub = self.create_publisher(MarkerArray, '/shooting_points_markers', latched_qos)
        self.path_markers_pub = self.create_publisher(MarkerArray, '/debug_paths_markers', latched_qos)
        self.selected_path_pub = self.create_publisher(NavPath, '/debug_selected_path', latched_qos)

        # サブスクライバー
        self.costmap: Optional[OccupancyGrid] = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/costmap', self.on_costmap, latched_qos
        )
        # スレッド同期・共有変数
        import threading
        self.lock = threading.Lock()
        self.path_event = threading.Event()
        self.latest_path: Optional[NavPath] = None
        self.path_received = False
        self.planned_path_sub = self.create_subscription(
            NavPath, '/planned_path', self.on_planned_path, volatile_qos
        )

        # TF ブロードキャスター
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 計画結果格納
        self.planned_results: List[Dict[str, Any]] = []

        # 定期パブリッシュ用タイマー (RViz用リフレッシュ 1Hz)
        self.timer = self.create_timer(1.0, self.on_timer_publish)

        # 計画実行スレッド起動
        self.planning_done = False
        self.worker_thread = threading.Thread(target=self.run_planning_worker, daemon=True)
        self.worker_thread.start()

    def on_costmap(self, msg: OccupancyGrid):
        if self.costmap is None:
            self.get_logger().info(f"Received initial Costmap ({msg.info.width}x{msg.info.height}, res={msg.info.resolution}m)")
        self.costmap = msg

    def on_planned_path(self, msg: NavPath):
        with self.lock:
            # 深いコピーを作成してスレッド間のデータ競合を確実に防止
            copied = NavPath()
            copied.header = msg.header
            copied.poses = list(msg.poses)
            self.latest_path = copied
            self.path_received = True
            self.path_event.set()

    def broadcast_robot_tf(self, x: float, y: float, yaw: float):
        """指定位置に map -> base_link TF をブロードキャスト"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def run_planning_worker(self):
        """ワーカースレッドで計画シーケンスを実行 (メインスレッドの spin を妨げない)"""
        # コストマップ受信待機
        while rclpy.ok() and self.costmap is None:
            self.get_logger().info("Waiting for /costmap topic before planning...", throttle_duration_sec=2.0)
            time.sleep(0.1)

        if not rclpy.ok():
            return

        time.sleep(0.5)  # コストマップ確定のための微小待機

        self.get_logger().info("================================================================================")
        self.get_logger().info(f"Starting Shooting Points Path Planning Sequence (Zone: {self.zone.upper()}, Mode: {self.mode})")
        self.get_logger().info("================================================================================")

        # ターゲットマーカー作成＆パブリッシュ
        self.publish_target_markers()

        # ペアの生成
        pairs_to_plan: List[Tuple[Dict[str, Any], Dict[str, Any]]] = []
        n = len(self.shooting_points)
        if n < 2:
            self.get_logger().warn("Less than 2 shooting points loaded. Nothing to plan.")
            self.planning_done = True
            return

        start_pt = self.shooting_points[0]  # 通常 ID 0 (StartZone)

        if self.mode == 'from_start':
            # スタートから全目的地へ
            for i in range(1, n):
                pairs_to_plan.append((start_pt, self.shooting_points[i]))
        elif self.mode == 'all_pairs':
            # 全ペア (n * (n-1))
            for i in range(n):
                for j in range(n):
                    if i != j:
                        pairs_to_plan.append((self.shooting_points[i], self.shooting_points[j]))
        elif self.mode == 'both':
            # スタートから全目的地へ ＋ 順次巡回
            for i in range(1, n):
                pairs_to_plan.append((start_pt, self.shooting_points[i]))
            for i in range(1, n - 1):
                pairs_to_plan.append((self.shooting_points[i], self.shooting_points[i + 1]))
            pairs_to_plan.append((self.shooting_points[n - 1], start_pt))
        else:  # 'tour' (default)
            # 順次巡回: 0 -> 1 -> 2 -> ... -> N-1 -> 0
            for i in range(n - 1):
                pairs_to_plan.append((self.shooting_points[i], self.shooting_points[i + 1]))
            pairs_to_plan.append((self.shooting_points[-1], self.shooting_points[0]))

        self.get_logger().info(f"Total pairs to compute: {len(pairs_to_plan)} paths.")

        # 各ペアについて順次計画を実行
        for idx, (p_start, p_goal) in enumerate(pairs_to_plan):
            if not rclpy.ok():
                break

            pair_name = f"{p_start['name']} -> {p_goal['name']}"
            self.get_logger().info(f"[{idx + 1}/{len(pairs_to_plan)}] Planning path: {pair_name} ...")

            # スタート地点のTFを何度かパブリッシュしてバッファを満たす
            for _ in range(8):
                self.broadcast_robot_tf(p_start['x'], p_start['y'], p_start['yaw'])
                time.sleep(0.02)

            # ゴールメッセージ発行
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = self.map_frame
            goal_msg.pose.position.x = float(p_goal['x'])
            goal_msg.pose.position.y = float(p_goal['y'])
            goal_msg.pose.position.z = 0.0
            gx, gy, gz, gw = euler_to_quaternion(p_goal['yaw'])
            goal_msg.pose.orientation.x = gx
            goal_msg.pose.orientation.y = gy
            goal_msg.pose.orientation.z = gz
            goal_msg.pose.orientation.w = gw

            with self.lock:
                self.path_received = False
                self.latest_path = None
                self.path_event.clear()

            self.goal_pub.publish(goal_msg)

            # パス受信待機 (最大 1.5 秒)
            start_wait = time.time()
            received_path = None
            while (time.time() - start_wait < 1.5) and rclpy.ok():
                self.broadcast_robot_tf(p_start['x'], p_start['y'], p_start['yaw'])
                if self.path_event.wait(timeout=0.03):
                    with self.lock:
                        if self.latest_path is not None and len(self.latest_path.poses) > 0:
                            received_path = self.latest_path
                            break

            if received_path is None or len(received_path.poses) == 0:
                self.get_logger().error(f"  FAILED: No path generated for {pair_name} (Timeout or Obstacle Blocked)")
                self.planned_results.append({
                    'start': p_start,
                    'goal': p_goal,
                    'name': pair_name,
                    'path': None,
                    'metrics': {'length': 0.0, 'max_curvature': 0.0, 'min_radius': 0.0, 'max_cost': 100},
                    'success': False
                })
            else:
                metrics = compute_path_metrics(received_path, self.costmap)
                self.get_logger().info(
                    f"  SUCCESS: length={metrics['length']:.2f}m, "
                    f"k_max={metrics['max_curvature']:.2f}/m (R_min={metrics['min_radius']:.2f}m), "
                    f"max_cost={metrics['max_cost']}"
                )
                self.planned_results.append({
                    'start': p_start,
                    'goal': p_goal,
                    'name': pair_name,
                    'path': received_path,
                    'metrics': metrics,
                    'success': True
                })

            time.sleep(0.08)

        self.planning_done = True
        self.print_summary_table()
        self.publish_path_markers()

    def print_summary_table(self):
        """コンソールに綺麗な結果サマリー表を出力"""
        lines = [
            "",
            "=" * 98,
            f" SHOOTING POINTS PATH PLANNING SUMMARY (Zone: {self.zone.upper()}, Targets: {len(self.shooting_points)})",
            "=" * 98,
            f"{'No.':<4} {'Route (From -> To)':<36} {'Length':<10} {'Max Curv':<12} {'Min Radius':<13} {'Max Cost':<10} {'Status':<8}",
            "-" * 98
        ]

        for i, res in enumerate(self.planned_results):
            m = res['metrics']
            status = "OK" if res['success'] else "FAILED"
            r_str = f"{m['min_radius']:.2f} m" if m['min_radius'] < 100.0 else "inf (Straight)"
            if not res['success']:
                lines.append(f"{i + 1:<4} {res['name']:<36} {'---':<10} {'---':<12} {'---':<13} {'---':<10} {status:<8}")
            else:
                lines.append(
                    f"{i + 1:<4} {res['name']:<36} "
                    f"{m['length']:>6.2f} m   "
                    f"{m['max_curvature']:>6.2f} /m    "
                    f"{r_str:>11}  "
                    f"{m['max_cost']:>5}      "
                    f"{status:<8}"
                )
        lines.append("=" * 98)
        lines.append("")

        summary_text = "\n".join(lines)
        print(summary_text, flush=True)
        self.get_logger().info(summary_text)

    def publish_target_markers(self):
        """目標地点のマーカー (円柱 + 矢印 + 名前テキスト) を生成してパブリッシュ"""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 色設定 (Blue または Red)
        is_blue = (self.zone == 'blue')
        base_color = (0.2, 0.5, 1.0) if is_blue else (1.0, 0.3, 0.3)

        for pt in self.shooting_points:
            tid = pt.get('target_id', 0)
            name = pt.get('name', f"T{tid}")
            x = float(pt.get('x', 0.0))
            y = float(pt.get('y', 0.0))
            yaw = float(pt.get('yaw', 0.0))

            # 1. 位置シリンダー (足元の円盤)
            cyl = Marker()
            cyl.header.frame_id = self.map_frame
            cyl.header.stamp = now
            cyl.ns = "targets_cylinder"
            cyl.id = tid * 3
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = x
            cyl.pose.position.y = y
            cyl.pose.position.z = 0.05
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.4
            cyl.scale.y = 0.4
            cyl.scale.z = 0.1
            cyl.color.r = base_color[0]
            cyl.color.g = base_color[1]
            cyl.color.b = base_color[2]
            cyl.color.a = 0.7
            ma.markers.append(cyl)

            # 2. 姿勢矢印
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = now
            arrow.ns = "targets_arrow"
            arrow.id = tid * 3 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.12
            qx, qy, qz, qw = euler_to_quaternion(yaw)
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = 0.5
            arrow.scale.y = 0.08
            arrow.scale.z = 0.08
            arrow.color.r = 1.0
            arrow.color.g = 0.9
            arrow.color.b = 0.1
            arrow.color.a = 0.9
            ma.markers.append(arrow)

            # 3. テキストラベル
            txt = Marker()
            txt.header.frame_id = self.map_frame
            txt.header.stamp = now
            txt.ns = "targets_text"
            txt.id = tid * 3 + 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = 0.35
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.22
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = f"[{tid}] {name}"
            ma.markers.append(txt)

        self.target_markers_pub.publish(ma)

    def publish_path_markers(self):
        """全パスを色分けして MarkerArray としてパブリッシュ"""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        valid_results = [r for r in self.planned_results if r['success'] and r['path'] is not None]
        num_paths = len(valid_results)

        for i, res in enumerate(valid_results):
            path: NavPath = res['path']
            # カラーパレット (HSV 色相環による鮮やかな色分け)
            hue = (float(i) / max(1, num_paths)) * 360.0
            r, g, b = hsv_to_rgb(hue, 0.85, 1.0)

            # 1. パスライン (LINE_STRIP)
            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = now
            line.ns = "debug_path_lines"
            line.id = i * 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.05  # 太さ 5cm
            line.color.r = r
            line.color.g = g
            line.color.b = b
            line.color.a = 0.85

            for pose in path.poses:
                p = Point()
                p.x = pose.pose.position.x
                p.y = pose.pose.position.y
                p.z = 0.04 + (i * 0.005)  # 重なり防止の微小オフセット
                line.points.append(p)
            ma.markers.append(line)

            # 2. パス中間点のテキストラベル (ルート名 + 距離 + 最小R)
            if len(path.poses) > 0:
                mid_idx = len(path.poses) // 2
                mid_pose = path.poses[mid_idx]

                label = Marker()
                label.header.frame_id = self.map_frame
                label.header.stamp = now
                label.ns = "debug_path_labels"
                label.id = i * 2 + 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = mid_pose.pose.position.x
                label.pose.position.y = mid_pose.pose.position.y
                label.pose.position.z = 0.15 + (i * 0.02)
                label.pose.orientation.w = 1.0
                label.scale.z = 0.16
                label.color.r = r
                label.color.g = g
                label.color.b = b
                label.color.a = 0.95
                m = res['metrics']
                label.text = f"{res['name']} ({m['length']:.1f}m, Rmin={m['min_radius']:.1f}m)"
                ma.markers.append(label)

        self.path_markers_pub.publish(ma)

        # 最初の有効パスを `/debug_selected_path` にも配信
        if valid_results:
            self.selected_path_pub.publish(valid_results[0]['path'])

    def on_timer_publish(self):
        """定期的にマーカーを再送信し、後から開いた RViz でも表示されるようにする"""
        if self.planning_done:
            self.publish_target_markers()
            self.publish_path_markers()
            # 最初の地点のTFを流し続ける
            if self.shooting_points:
                p0 = self.shooting_points[0]
                self.broadcast_robot_tf(p0['x'], p0['y'], p0['yaw'])


def hsv_to_rgb(h: float, s: float, v: float) -> Tuple[float, float, float]:
    """HSV (h: 0..360, s: 0..1, v: 0..1) から RGB (0..1) を算出"""
    c = v * s
    x = c * (1.0 - abs((h / 60.0) % 2 - 1.0))
    m = v - c
    if 0 <= h < 60:
        rp, gp, bp = c, x, 0.0
    elif 60 <= h < 120:
        rp, gp, bp = x, c, 0.0
    elif 120 <= h < 180:
        rp, gp, bp = 0.0, c, x
    elif 180 <= h < 240:
        rp, gp, bp = 0.0, x, c
    elif 240 <= h < 300:
        rp, gp, bp = x, 0.0, c
    else:
        rp, gp, bp = c, 0.0, x
    return (rp + m, gp + m, bp + m)


def main(args=None):
    rclpy.init(args=args)
    node = DebugShootingPointsPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
