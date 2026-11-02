#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Tracking and Velocity Debugger GUI for gn10-navigation
- Compares 3 velocities:
  1. Command velocity (/robot/operation/cmd_vel)
  2. Wheel forward kinematics (/robot/feedback/wheel_angular_velocity)
  3. Real effective velocity from TF (map -> base_link)
- Visualizes 2D path tracking, trajectory, and overlaid velocity vectors
- Supports both Live ROS 2 subscription and Rosbag2 (.db3) direct loading
"""

import sys
import os
import math
import time
import sqlite3
import threading
from collections import deque
from typing import List, Tuple, Optional, Dict, Any

import numpy as np

# PyQt5 & Matplotlib
from PyQt5 import QtCore, QtGui, QtWidgets
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

# ROS 2 imports (optional fallback for offline analysis if rclpy not available)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32MultiArray
    from nav_msgs.msg import Path
    from tf2_msgs.msg import TFMessage
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class Omni3Kinematics:
    """
    Kinematics for 3-wheel omni robot.
    Geometry matching gn10-mainboard ThreeWheelOmni:
      - 0: Front
      - 1: Back Left
      - 2: Back Right
      - M3508 Gear Ratio: 19.0
      - Wheel Radius: 0.065 m (130 mm diameter)
      - Chassis Radius: 0.40 m
    """
    def __init__(self, gear_ratio: float = 19.0, wheel_radius: float = 0.065, chassis_radius: float = 0.40):
        self.gear_ratio = gear_ratio
        self.wheel_radius = wheel_radius
        self.chassis_radius = chassis_radius

    def forward_kinematics(self, w_front_raw: float, w_left_raw: float, w_right_raw: float) -> Tuple[float, float, float]:
        """
        Calculates robot base_link velocity from motor shaft angular velocities [rad/s].
        Returns: (vx, vy, omega_z) in robot base_link frame [m/s, m/s, rad/s]
        """
        # Wheel linear peripheral speeds [m/s]
        vf = (w_front_raw / self.gear_ratio) * self.wheel_radius
        vbl = (w_left_raw / self.gear_ratio) * self.wheel_radius
        vbr = (w_right_raw / self.gear_ratio) * self.wheel_radius

        # Inverse of ThreeWheelOmni equations:
        # vf = -x + R * omega
        # vbl = 0.5 * x - (sqrt(3)/2) * y + R * omega
        # vbr = 0.5 * x + (sqrt(3)/2) * y + R * omega
        # with:
        # op.vel_x = y, op.vel_y = x, op.vel_yaw = -omega
        vx = (vbr - vbl) / math.sqrt(3.0)
        vy = (-2.0 * vf + vbl + vbr) / 3.0
        omega_z = -(vf + vbl + vbr) / (3.0 * self.chassis_radius)
        return vx, vy, omega_z


class DataRecord:
    """Time-stamped measurement point"""
    def __init__(self, t: float):
        self.t: float = t  # Relative time [s]
        # Command velocity (base_link)
        self.cmd_vx: float = 0.0
        self.cmd_vy: float = 0.0
        self.cmd_norm: float = 0.0
        self.cmd_w: float = 0.0
        # Wheel FK velocity (base_link)
        self.wheel_vx: float = 0.0
        self.wheel_vy: float = 0.0
        self.wheel_norm: float = 0.0
        self.wheel_w: float = 0.0
        # TF effective velocity (base_link)
        self.tf_vx: float = 0.0
        self.tf_vy: float = 0.0
        self.tf_norm: float = 0.0
        self.tf_w: float = 0.0
        # Robot Pose (map frame)
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_yaw: float = 0.0
        # Tracking errors
        self.error_lat: float = 0.0   # Cross-track error [m]
        self.error_yaw: float = 0.0   # Heading error [rad]


class LiveRosSubscriber:
    """ROS 2 subscriber node running in background thread"""
    def __init__(self, callback_data_update):
        self.callback_data_update = callback_data_update
        self.node: Optional[Node] = None
        self.thread: Optional[threading.Thread] = None
        self.running = False
        self.kinematics = Omni3Kinematics()

        # State storage
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_wheel = (0.0, 0.0, 0.0)
        self.last_tf_pose: Optional[Tuple[float, float, float, float]] = None  # (t, x, y, yaw)
        self.tf_lpf_vx = 0.0
        self.tf_lpf_vy = 0.0
        self.tf_lpf_w = 0.0
        self.current_path: List[Tuple[float, float]] = []

    def start(self):
        if not ROS2_AVAILABLE:
            return
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("tracking_debugger_listener")
        self.node.create_subscription(Twist, "/robot/operation/cmd_vel", self._on_cmd_vel, 10)
        self.node.create_subscription(Float32MultiArray, "/robot/feedback/wheel_angular_velocity", self._on_wheel, 10)
        self.node.create_subscription(Path, "/planned_path", self._on_path, 10)
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 50)

        self.running = True
        self.thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.thread.start()

    def _spin_loop(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        while self.running and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
        executor.remove_node(self.node)

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.node:
            self.node.destroy_node()
            self.node = None

    def _on_cmd_vel(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z
        self.last_cmd = (vx, vy, w)

    def _on_wheel(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            w_front = msg.data[0]
            w_left = msg.data[1]
            w_right = msg.data[2]
            vx, vy, w = self.kinematics.forward_kinematics(w_front, w_left, w_right)
            self.last_wheel = (vx, vy, w)

    def _on_path(self, msg: Path):
        pts = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            pts.append((p.x, p.y))
        self.current_path = pts

    def _on_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "base_link":
                t = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                rot = transform.transform.rotation
                yaw = quat_to_yaw(rot.x, rot.y, rot.z, rot.w)

                tf_vx = 0.0
                tf_vy = 0.0
                tf_w = 0.0

                if self.last_tf_pose is not None:
                    prev_t, prev_x, prev_y, prev_yaw = self.last_tf_pose
                    dt = t - prev_t
                    if 0.001 < dt < 0.2:
                        # World linear velocity
                        world_vx = (tx - prev_x) / dt
                        world_vy = (ty - prev_y) / dt
                        world_w = normalize_angle(yaw - prev_yaw) / dt

                        # Transform to robot base_link frame
                        cos_y = math.cos(yaw)
                        sin_y = math.sin(yaw)
                        raw_vx = cos_y * world_vx + sin_y * world_vy
                        raw_vy = -sin_y * world_vx + cos_y * world_vy

                        # Low pass filter (alpha ~ 0.25)
                        alpha = 0.25
                        self.tf_lpf_vx += alpha * (raw_vx - self.tf_lpf_vx)
                        self.tf_lpf_vy += alpha * (raw_vy - self.tf_lpf_vy)
                        self.tf_lpf_w += alpha * (world_w - self.tf_lpf_w)

                        tf_vx = self.tf_lpf_vx
                        tf_vy = self.tf_lpf_vy
                        tf_w = self.tf_lpf_w

                self.last_tf_pose = (t, tx, ty, yaw)

                # Compute tracking cross-track error
                err_lat = 0.0
                err_yaw = 0.0
                if len(self.current_path) >= 2:
                    min_dist_sq = float('inf')
                    best_proj = (0.0, 0.0)
                    best_tangent = (1.0, 0.0)
                    for i in range(len(self.current_path) - 1):
                        p1 = self.current_path[i]
                        p2 = self.current_path[i + 1]
                        seg_x = p2[0] - p1[0]
                        seg_y = p2[1] - p1[1]
                        seg_len_sq = seg_x * seg_x + seg_y * seg_y
                        if seg_len_sq < 1e-6:
                            continue
                        proj_u = ((tx - p1[0]) * seg_x + (ty - p1[1]) * seg_y) / seg_len_sq
                        proj_u = max(0.0, min(1.0, proj_u))
                        qx = p1[0] + proj_u * seg_x
                        qy = p1[1] + proj_u * seg_y
                        d_sq = (tx - qx) ** 2 + (ty - qy) ** 2
                        if d_sq < min_dist_sq:
                            min_dist_sq = d_sq
                            best_proj = (qx, qy)
                            seg_len = math.sqrt(seg_len_sq)
                            best_tangent = (seg_x / seg_len, seg_y / seg_len)

                    # Signed cross-track error (positive if robot is left of path)
                    dx = tx - best_proj[0]
                    dy = ty - best_proj[1]
                    err_lat = best_tangent[0] * dy - best_tangent[1] * dx
                    path_yaw = math.atan2(best_tangent[1], best_tangent[0])
                    err_yaw = normalize_angle(yaw - path_yaw)

                # Trigger GUI update callback
                self.callback_data_update(
                    t=t,
                    cmd_v=self.last_cmd,
                    wheel_v=self.last_wheel,
                    tf_v=(tf_vx, tf_vy, tf_w),
                    pose=(tx, ty, yaw),
                    path=self.current_path,
                    errors=(err_lat, err_yaw)
                )


class RosbagLoader:
    """Loads and synchronizes cmd_vel, wheel_angular_velocity, and TF from Rosbag2 sqlite3 database"""
    @staticmethod
    def load_bag(bag_path: str, kinematics: Omni3Kinematics) -> Tuple[List[DataRecord], List[Tuple[float, float]]]:
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy, rosidl_runtime_py) is required to parse rosbag2 messages.")

        # Find db3 file
        db_file = bag_path
        if os.path.isdir(bag_path):
            candidates = [os.path.join(bag_path, f) for f in os.listdir(bag_path) if f.endswith(".db3")]
            if not candidates:
                raise FileNotFoundError(f"No .db3 file found in directory: {bag_path}")
            db_file = candidates[0]

        con = sqlite3.connect(db_file)
        cur = con.cursor()

        # Get topic mapping
        cur.execute("SELECT id, name, type FROM topics")
        topics = {row[0]: (row[1], row[2]) for row in cur.fetchall()}
        topic_name_to_id = {name: tid for tid, (name, _) in topics.items()}

        target_topics = [
            "/robot/operation/cmd_vel",
            "/robot/feedback/wheel_angular_velocity",
            "/tf",
            "/planned_path"
        ]
        active_ids = [topic_name_to_id[name] for name in target_topics if name in topic_name_to_id]
        if not active_ids:
            con.close()
            raise ValueError("None of the required topics found in the specified bag.")

        placeholders = ",".join("?" * len(active_ids))
        cur.execute(
            f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY timestamp ASC",
            active_ids
        )

        msg_classes = {tid: get_message(type_str) for tid, (_, type_str) in topics.items() if tid in active_ids}

        records: List[DataRecord] = []
        path_points: List[Tuple[float, float]] = []

        last_cmd = (0.0, 0.0, 0.0)
        last_wheel = (0.0, 0.0, 0.0)
        last_tf: Optional[Tuple[float, float, float, float]] = None
        tf_lpf_vx = 0.0
        tf_lpf_vy = 0.0
        tf_lpf_w = 0.0

        t0: Optional[float] = None

        for tid, ts_ns, data in cur.fetchall():
            t_sec = ts_ns * 1e-9
            if t0 is None:
                t0 = t_sec
            rel_t = t_sec - t0

            topic_name, _ = topics[tid]
            msg_cls = msg_classes[tid]
            msg = deserialize_message(data, msg_cls)

            if topic_name == "/robot/operation/cmd_vel":
                last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

            elif topic_name == "/robot/feedback/wheel_angular_velocity":
                if len(msg.data) >= 3:
                    w_front = msg.data[0]
                    w_left = msg.data[1]
                    w_right = msg.data[2]
                    last_wheel = kinematics.forward_kinematics(w_front, w_left, w_right)

            elif topic_name == "/planned_path":
                pts = []
                for ps in msg.poses:
                    p = ps.pose.position
                    pts.append((p.x, p.y))
                if pts:
                    path_points = pts

            elif topic_name == "/tf":
                for tr in msg.transforms:
                    if tr.header.frame_id == "map" and tr.child_frame_id == "base_link":
                        tx = tr.transform.translation.x
                        ty = tr.transform.translation.y
                        rot = tr.transform.rotation
                        yaw = quat_to_yaw(rot.x, rot.y, rot.z, rot.w)

                        tf_vx = 0.0
                        tf_vy = 0.0
                        tf_w = 0.0

                        if last_tf is not None:
                            prev_t, prev_x, prev_y, prev_yaw = last_tf
                            dt = rel_t - prev_t
                            if 0.002 < dt < 0.2:
                                world_vx = (tx - prev_x) / dt
                                world_vy = (ty - prev_y) / dt
                                world_w = normalize_angle(yaw - prev_yaw) / dt

                                cos_y = math.cos(yaw)
                                sin_y = math.sin(yaw)
                                raw_vx = cos_y * world_vx + sin_y * world_vy
                                raw_vy = -sin_y * world_vx + cos_y * world_vy

                                alpha = 0.25
                                tf_lpf_vx += alpha * (raw_vx - tf_lpf_vx)
                                tf_lpf_vy += alpha * (raw_vy - tf_lpf_vy)
                                tf_lpf_w += alpha * (world_w - tf_lpf_w)

                                tf_vx = tf_lpf_vx
                                tf_vy = tf_lpf_vy
                                tf_w = tf_lpf_w

                        last_tf = (rel_t, tx, ty, yaw)

                        # Create a record at each TF update
                        rec = DataRecord(rel_t)
                        rec.cmd_vx = last_cmd[0]
                        rec.cmd_vy = last_cmd[1]
                        rec.cmd_norm = math.hypot(rec.cmd_vx, rec.cmd_vy)
                        rec.cmd_w = last_cmd[2]

                        rec.wheel_vx = last_wheel[0]
                        rec.wheel_vy = last_wheel[1]
                        rec.wheel_norm = math.hypot(rec.wheel_vx, rec.wheel_vy)
                        rec.wheel_w = last_wheel[2]

                        rec.tf_vx = tf_vx
                        rec.tf_vy = tf_vy
                        rec.tf_norm = math.hypot(rec.tf_vx, rec.tf_vy)
                        rec.tf_w = tf_w

                        rec.robot_x = tx
                        rec.robot_y = ty
                        rec.robot_yaw = yaw

                        # Tracking errors
                        if len(path_points) >= 2:
                            min_dist_sq = float('inf')
                            best_proj = (0.0, 0.0)
                            best_tan = (1.0, 0.0)
                            for i in range(len(path_points) - 1):
                                p1 = path_points[i]
                                p2 = path_points[i + 1]
                                sx = p2[0] - p1[0]
                                sy = p2[1] - p1[1]
                                slen_sq = sx * sx + sy * sy
                                if slen_sq < 1e-6:
                                    continue
                                u = max(0.0, min(1.0, ((tx - p1[0]) * sx + (ty - p1[1]) * sy) / slen_sq))
                                qx = p1[0] + u * sx
                                qy = p1[1] + u * sy
                                d_sq = (tx - qx) ** 2 + (ty - qy) ** 2
                                if d_sq < min_dist_sq:
                                    min_dist_sq = d_sq
                                    best_proj = (qx, qy)
                                    slen = math.sqrt(slen_sq)
                                    best_tan = (sx / slen, sy / slen)

                            rec.error_lat = best_tan[0] * (ty - best_proj[1]) - best_tan[1] * (tx - best_proj[0])
                            path_yaw = math.atan2(best_tan[1], best_tan[0])
                            rec.error_yaw = normalize_angle(yaw - path_yaw)

                        records.append(rec)

        con.close()
        return records, path_points


class TrackingDebuggerMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("gn10 Tracking & Velocity Debugger")
        self.resize(1380, 880)

        self.kinematics = Omni3Kinematics(gear_ratio=19.0, wheel_radius=0.065, chassis_radius=0.40)
        self.records: List[DataRecord] = []
        self.path_points: List[Tuple[float, float]] = []
        self.live_subscriber: Optional[LiveRosSubscriber] = None

        self.is_live = False
        self.t_start_live: Optional[float] = None
        self.history_window_sec = 15.0  # Sliding time window for live plots

        self.current_playback_index = 0
        self.playback_timer = QtCore.QTimer(self)
        self.playback_timer.timeout.connect(self._on_playback_step)

        self._init_ui()

        # Update timer for live display (20 Hz)
        self.ui_update_timer = QtCore.QTimer(self)
        self.ui_update_timer.timeout.connect(self._on_ui_tick)
        self.ui_update_timer.start(50)

    def _init_ui(self):
        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # --- Top Control Bar ---
        top_bar = QtWidgets.QHBoxLayout()

        self.btn_live = QtWidgets.QPushButton("● Live ROS 2 Start")
        self.btn_live.setCheckable(True)
        self.btn_live.setStyleSheet("font-weight: bold; background-color: #e1f5fe;")
        self.btn_live.clicked.connect(self._toggle_live)
        top_bar.addWidget(self.btn_live)

        self.btn_open_bag = QtWidgets.QPushButton("📂 Open Rosbag (.db3)")
        self.btn_open_bag.clicked.connect(self._open_bag_dialog)
        top_bar.addWidget(self.btn_open_bag)

        self.btn_export_csv = QtWidgets.QPushButton("💾 Export CSV")
        self.btn_export_csv.clicked.connect(self._export_csv)
        top_bar.addWidget(self.btn_export_csv)

        top_bar.addSpacing(20)

        self.lbl_status = QtWidgets.QLabel("Status: Standby")
        self.lbl_status.setStyleSheet("font-weight: bold; color: #37474f;")
        top_bar.addWidget(self.lbl_status)

        top_bar.addStretch()

        # Kinematic parameter adjustments
        top_bar.addWidget(QtWidgets.QLabel("Gear Ratio:"))
        self.spn_gear = QtWidgets.QDoubleSpinBox()
        self.spn_gear.setRange(1.0, 100.0)
        self.spn_gear.setValue(19.0)
        self.spn_gear.valueChanged.connect(self._on_param_changed)
        top_bar.addWidget(self.spn_gear)

        top_bar.addWidget(QtWidgets.QLabel("Wheel Radius [m]:"))
        self.spn_wheel_r = QtWidgets.QDoubleSpinBox()
        self.spn_wheel_r.setRange(0.01, 0.50)
        self.spn_wheel_r.setSingleStep(0.005)
        self.spn_wheel_r.setValue(0.065)
        self.spn_wheel_r.valueChanged.connect(self._on_param_changed)
        top_bar.addWidget(self.spn_wheel_r)

        main_layout.addLayout(top_bar)

        # --- Middle Splitter (2D Map on Left, Time-Series on Right) ---
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # 1. Left Panel: 2D Map View
        map_container = QtWidgets.QWidget()
        map_layout = QtWidgets.QVBoxLayout(map_container)
        map_layout.setContentsMargins(0, 0, 0, 0)

        self.fig_map = Figure(figsize=(5, 6), dpi=100)
        self.canvas_map = FigureCanvasQTAgg(self.fig_map)
        self.ax_map = self.fig_map.add_subplot(111)
        self.toolbar_map = NavigationToolbar2QT(self.canvas_map, self)
        map_layout.addWidget(self.toolbar_map)
        map_layout.addWidget(self.canvas_map)

        splitter.addWidget(map_container)

        # 2. Right Panel: Time-series Plots (4 subplots)
        plots_container = QtWidgets.QWidget()
        plots_layout = QtWidgets.QVBoxLayout(plots_container)
        plots_layout.setContentsMargins(0, 0, 0, 0)

        self.fig_plots = Figure(figsize=(7, 7), dpi=100)
        self.canvas_plots = FigureCanvasQTAgg(self.fig_plots)
        self.toolbar_plots = NavigationToolbar2QT(self.canvas_plots, self)
        plots_layout.addWidget(self.toolbar_plots)
        plots_layout.addWidget(self.canvas_plots)

        # Subplots: 1: Total Speed, 2: Vx & Vy, 3: Omega_z, 4: Tracking Error
        self.ax_v = self.fig_plots.add_subplot(411)
        self.ax_vxvy = self.fig_plots.add_subplot(412, sharex=self.ax_v)
        self.ax_w = self.fig_plots.add_subplot(413, sharex=self.ax_v)
        self.ax_err = self.fig_plots.add_subplot(414, sharex=self.ax_v)
        self.fig_plots.tight_layout()

        splitter.addWidget(plots_container)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 6)
        main_layout.addWidget(splitter)

        # --- Playback Seek Bar (For Rosbag) ---
        playback_box = QtWidgets.QHBoxLayout()
        self.btn_play_pause = QtWidgets.QPushButton("▶ Play")
        self.btn_play_pause.clicked.connect(self._toggle_playback)
        playback_box.addWidget(self.btn_play_pause)

        self.slider_time = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_time.setRange(0, 1000)
        self.slider_time.setValue(0)
        self.slider_time.sliderMoved.connect(self._on_slider_moved)
        playback_box.addWidget(self.slider_time)

        self.lbl_time = QtWidgets.QLabel("Time: 0.00 / 0.00 s")
        playback_box.addWidget(self.lbl_time)
        main_layout.addLayout(playback_box)

        # --- Bottom Metrics Dashboard ---
        metrics_group = QtWidgets.QGroupBox("Real-time Tracking & Velocity Evaluation Metrics")
        metrics_layout = QtWidgets.QHBoxLayout(metrics_group)

        self.lbl_metric_v = QtWidgets.QLabel("Velocity [m/s]\nCmd: 0.00 | Wheel: 0.00 | TF: 0.00")
        self.lbl_metric_v.setStyleSheet("font-size: 11pt;")
        metrics_layout.addWidget(self.lbl_metric_v)

        self.lbl_metric_slip = QtWidgets.QLabel("Estimated Slip (|Wheel - TF|):\n0.00 m/s")
        self.lbl_metric_slip.setStyleSheet("font-size: 11pt; color: #b71c1c;")
        metrics_layout.addWidget(self.lbl_metric_slip)

        self.lbl_metric_lag = QtWidgets.QLabel("Motor Lag (|Cmd - Wheel|):\n0.00 m/s")
        self.lbl_metric_lag.setStyleSheet("font-size: 11pt; color: #e65100;")
        metrics_layout.addWidget(self.lbl_metric_lag)

        self.lbl_metric_err = QtWidgets.QLabel("Cross-track Error:\n0.000 m (Max: 0.000 m)")
        self.lbl_metric_err.setStyleSheet("font-size: 11pt; color: #1b5e20;")
        metrics_layout.addWidget(self.lbl_metric_err)

        main_layout.addWidget(metrics_group)

    def _on_param_changed(self):
        self.kinematics.gear_ratio = self.spn_gear.value()
        self.kinematics.wheel_radius = self.spn_wheel_r.value()

    def _toggle_live(self):
        if not self.btn_live.isChecked():
            # Stop live
            self.is_live = False
            self.btn_live.setText("● Live ROS 2 Start")
            self.btn_live.setStyleSheet("font-weight: bold; background-color: #e1f5fe;")
            self.lbl_status.setText("Status: Live stopped")
            if self.live_subscriber:
                self.live_subscriber.stop()
                self.live_subscriber = None
        else:
            if not ROS2_AVAILABLE:
                QtWidgets.QMessageBox.critical(self, "Error", "ROS 2 Python (rclpy) is not available.")
                self.btn_live.setChecked(False)
                return
            # Start live
            self.is_live = True
            self.records.clear()
            self.t_start_live = None
            self.btn_live.setText("■ Stop Live ROS 2")
            self.btn_live.setStyleSheet("font-weight: bold; background-color: #ffebee; color: #c62828;")
            self.lbl_status.setText("Status: Live streaming...")
            self.live_subscriber = LiveRosSubscriber(self._on_live_data_arrived)
            self.live_subscriber.start()

    def _on_live_data_arrived(self, t: float, cmd_v, wheel_v, tf_v, pose, path, errors):
        if self.t_start_live is None:
            self.t_start_live = t
        rel_t = t - self.t_start_live

        rec = DataRecord(rel_t)
        rec.cmd_vx, rec.cmd_vy, rec.cmd_w = cmd_v
        rec.cmd_norm = math.hypot(rec.cmd_vx, rec.cmd_vy)

        rec.wheel_vx, rec.wheel_vy, rec.wheel_w = wheel_v
        rec.wheel_norm = math.hypot(rec.wheel_vx, rec.wheel_vy)

        rec.tf_vx, rec.tf_vy, rec.tf_w = tf_v
        rec.tf_norm = math.hypot(rec.tf_vx, rec.tf_vy)

        rec.robot_x, rec.robot_y, rec.robot_yaw = pose
        rec.error_lat, rec.error_yaw = errors

        self.records.append(rec)
        if path:
            self.path_points = path

    def _open_bag_dialog(self):
        bag_dir = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Rosbag2 Directory or File", "/home/gn10"
        )
        if not bag_dir:
            return

        if self.is_live:
            self.btn_live.setChecked(False)
            self._toggle_live()

        self.lbl_status.setText(f"Loading bag: {os.path.basename(bag_dir)}...")
        QtWidgets.QApplication.processEvents()

        try:
            records, path_pts = RosbagLoader.load_bag(bag_dir, self.kinematics)
            if not records:
                QtWidgets.QMessageBox.warning(self, "Warning", "No velocity/TF records found in bag.")
                return
            self.records = records
            self.path_points = path_pts
            self.current_playback_index = len(self.records) - 1
            self.slider_time.setValue(1000)
            self.lbl_status.setText(f"Loaded {len(records)} records ({records[-1].t:.1f} s)")
            self._redraw_all(self.current_playback_index)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error loading bag", str(e))
            self.lbl_status.setText("Failed to load bag")

    def _toggle_playback(self):
        if self.playback_timer.isActive():
            self.playback_timer.stop()
            self.btn_play_pause.setText("▶ Play")
        else:
            if not self.records:
                return
            if self.current_playback_index >= len(self.records) - 1:
                self.current_playback_index = 0
            self.playback_timer.start(40)  # ~25 Hz playback
            self.btn_play_pause.setText("❚❚ Pause")

    def _on_playback_step(self):
        if self.current_playback_index < len(self.records) - 1:
            self.current_playback_index += 1
            progress = int((self.current_playback_index / (len(self.records) - 1)) * 1000)
            self.slider_time.blockSignals(True)
            self.slider_time.setValue(progress)
            self.slider_time.blockSignals(False)
            self._redraw_all(self.current_playback_index)
        else:
            self.playback_timer.stop()
            self.btn_play_pause.setText("▶ Play")

    def _on_slider_moved(self, value: int):
        if not self.records:
            return
        idx = int((value / 1000.0) * (len(self.records) - 1))
        self.current_playback_index = max(0, min(len(self.records) - 1, idx))
        self._redraw_all(self.current_playback_index)

    def _on_ui_tick(self):
        if self.is_live and self.records:
            self._redraw_all(len(self.records) - 1)

    def _redraw_all(self, active_idx: int):
        if not self.records:
            return

        rec = self.records[active_idx]
        total_time = self.records[-1].t
        self.lbl_time.setText(f"Time: {rec.t:.2f} / {total_time:.2f} s")

        # Update metrics label
        slip = abs(rec.wheel_norm - rec.tf_norm)
        lag = abs(rec.cmd_norm - rec.wheel_norm)
        max_err = max(abs(r.error_lat) for r in self.records[:active_idx + 1]) if active_idx > 0 else abs(rec.error_lat)
        self.lbl_metric_v.setText(
            f"Velocity [m/s]\nCmd: {rec.cmd_norm:.2f} | Wheel: {rec.wheel_norm:.2f} | TF: {rec.tf_norm:.2f}"
        )
        self.lbl_metric_slip.setText(f"Estimated Slip (|Wheel - TF|):\n{slip:.2f} m/s")
        self.lbl_metric_lag.setText(f"Motor Lag (|Cmd - Wheel|):\n{lag:.2f} m/s")
        self.lbl_metric_err.setText(f"Cross-track Error:\n{rec.error_lat:.3f} m (Max: {max_err:.3f} m)")

        # 1. Redraw 2D Map
        self.ax_map.clear()
        self.ax_map.set_title("2D Path Tracking & Velocity Vectors", fontsize=11)
        self.ax_map.set_xlabel("X [m]")
        self.ax_map.set_ylabel("Y [m]")
        self.ax_map.grid(True, linestyle="--", alpha=0.5)
        self.ax_map.set_aspect("equal", "datalim")

        # Draw planned path
        if len(self.path_points) >= 2:
            px = [p[0] for p in self.path_points]
            py = [p[1] for p in self.path_points]
            self.ax_map.plot(px, py, "b--", alpha=0.6, label="Planned Path")

        # Draw trajectory up to active_idx
        traj_x = [r.robot_x for r in self.records[:active_idx + 1]]
        traj_y = [r.robot_y for r in self.records[:active_idx + 1]]
        self.ax_map.plot(traj_x, traj_y, "gray", linewidth=1.5, alpha=0.7, label="Trajectory (TF)")

        # Draw robot body (circle of radius 0.40 m)
        rx, ry, ryaw = rec.robot_x, rec.robot_y, rec.robot_yaw
        circle = matplotlib.patches.Circle(
            (rx, ry), self.kinematics.chassis_radius, fill=False, color="black", linestyle="-", linewidth=1.2
        )
        self.ax_map.add_patch(circle)
        # Orientation heading line
        head_len = 0.40
        self.ax_map.plot([rx, rx + head_len * math.cos(ryaw)], [ry, ry + head_len * math.sin(ryaw)], "k-", linewidth=2.0)

        # Draw 3 Velocity Vectors transformed to world frame
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)

        arrow_scale = 0.6  # Arrow scale length factor
        # 1. Cmd velocity (Blue)
        cmd_wx = cos_y * rec.cmd_vx - sin_y * rec.cmd_vy
        cmd_wy = sin_y * rec.cmd_vx + cos_y * rec.cmd_vy
        if rec.cmd_norm > 0.02:
            self.ax_map.annotate(
                "", xy=(rx + cmd_wx * arrow_scale, ry + cmd_wy * arrow_scale), xytext=(rx, ry),
                arrowprops=dict(arrowstyle="->", color="#1976d2", lw=2.5, mutation_scale=15),
            )
        # 2. Wheel FK velocity (Green)
        wheel_wx = cos_y * rec.wheel_vx - sin_y * rec.wheel_vy
        wheel_wy = sin_y * rec.wheel_vx + cos_y * rec.wheel_vy
        if rec.wheel_norm > 0.02:
            self.ax_map.annotate(
                "", xy=(rx + wheel_wx * arrow_scale, ry + wheel_wy * arrow_scale), xytext=(rx, ry),
                arrowprops=dict(arrowstyle="->", color="#388e3c", lw=2.5, mutation_scale=15),
            )
        # 3. TF effective velocity (Red)
        tf_wx = cos_y * rec.tf_vx - sin_y * rec.tf_vy
        tf_wy = sin_y * rec.tf_vx + cos_y * rec.tf_vy
        if rec.tf_norm > 0.02:
            self.ax_map.annotate(
                "", xy=(rx + tf_wx * arrow_scale, ry + tf_wy * arrow_scale), xytext=(rx, ry),
                arrowprops=dict(arrowstyle="->", color="#d32f2f", lw=2.5, mutation_scale=15),
            )

        # Legend proxy elements
        legend_lines = [
            matplotlib.lines.Line2D([0], [0], color="#1976d2", lw=2, label=f"Cmd ({rec.cmd_norm:.2f} m/s)"),
            matplotlib.lines.Line2D([0], [0], color="#388e3c", lw=2, label=f"Wheel FK ({rec.wheel_norm:.2f} m/s)"),
            matplotlib.lines.Line2D([0], [0], color="#d32f2f", lw=2, label=f"TF Real ({rec.tf_norm:.2f} m/s)")
        ]
        self.ax_map.legend(handles=legend_lines, loc="upper right", fontsize=9)
        self.canvas_map.draw_idle()

        # 2. Redraw Time-Series Plots
        # Filter window if in live mode
        if self.is_live:
            t_min = max(0.0, rec.t - self.history_window_sec)
            plot_records = [r for r in self.records if r.t >= t_min]
        else:
            plot_records = self.records

        ts = [r.t for r in plot_records]
        cmd_norms = [r.cmd_norm for r in plot_records]
        wheel_norms = [r.wheel_norm for r in plot_records]
        tf_norms = [r.tf_norm for r in plot_records]

        cmd_vxs = [r.cmd_vx for r in plot_records]
        wheel_vxs = [r.wheel_vx for r in plot_records]
        tf_vxs = [r.tf_vx for r in plot_records]

        cmd_vys = [r.cmd_vy for r in plot_records]
        wheel_vys = [r.wheel_vy for r in plot_records]
        tf_vys = [r.tf_vy for r in plot_records]

        cmd_ws = [r.cmd_w for r in plot_records]
        wheel_ws = [r.wheel_w for r in plot_records]
        tf_ws = [r.tf_w for r in plot_records]

        err_lats = [r.error_lat for r in plot_records]

        # Subplot 1: Total Speed Norm
        self.ax_v.clear()
        self.ax_v.set_ylabel("Speed [m/s]", fontsize=9)
        self.ax_v.plot(ts, cmd_norms, color="#1976d2", label="Cmd", lw=1.5)
        self.ax_v.plot(ts, wheel_norms, color="#388e3c", label="Wheel FK", lw=1.5)
        self.ax_v.plot(ts, tf_norms, color="#d32f2f", label="TF Real", lw=1.5)
        self.ax_v.axvline(rec.t, color="black", linestyle=":", alpha=0.7)
        self.ax_v.legend(loc="upper right", fontsize=8)
        self.ax_v.grid(True, linestyle="--", alpha=0.5)

        # Subplot 2: Forward (Vx) & Lateral (Vy)
        self.ax_vxvy.clear()
        self.ax_vxvy.set_ylabel("Vx / Vy [m/s]", fontsize=9)
        self.ax_vxvy.plot(ts, cmd_vxs, color="#1976d2", linestyle="-", label="Cmd Vx", lw=1.2)
        self.ax_vxvy.plot(ts, wheel_vxs, color="#388e3c", linestyle="-", label="Wheel Vx", lw=1.2)
        self.ax_vxvy.plot(ts, tf_vxs, color="#d32f2f", linestyle="-", label="TF Vx", lw=1.2)
        self.ax_vxvy.plot(ts, cmd_vys, color="#1976d2", linestyle="--", label="Cmd Vy", lw=1.2, alpha=0.7)
        self.ax_vxvy.plot(ts, wheel_vys, color="#388e3c", linestyle="--", label="Wheel Vy", lw=1.2, alpha=0.7)
        self.ax_vxvy.plot(ts, tf_vys, color="#d32f2f", linestyle="--", label="TF Vy", lw=1.2, alpha=0.7)
        self.ax_vxvy.axvline(rec.t, color="black", linestyle=":", alpha=0.7)
        self.ax_vxvy.legend(loc="upper right", fontsize=7, ncol=2)
        self.ax_vxvy.grid(True, linestyle="--", alpha=0.5)

        # Subplot 3: Yaw rate Omega_z
        self.ax_w.clear()
        self.ax_w.set_ylabel("Yaw Rate [rad/s]", fontsize=9)
        self.ax_w.plot(ts, cmd_ws, color="#1976d2", label="Cmd ω", lw=1.5)
        self.ax_w.plot(ts, wheel_ws, color="#388e3c", label="Wheel ω", lw=1.5)
        self.ax_w.plot(ts, tf_ws, color="#d32f2f", label="TF ω", lw=1.5)
        self.ax_w.axvline(rec.t, color="black", linestyle=":", alpha=0.7)
        self.ax_w.legend(loc="upper right", fontsize=8)
        self.ax_w.grid(True, linestyle="--", alpha=0.5)

        # Subplot 4: Cross-track Error
        self.ax_err.clear()
        self.ax_err.set_ylabel("Cross Error [m]", fontsize=9)
        self.ax_err.set_xlabel("Time [s]", fontsize=9)
        self.ax_err.plot(ts, err_lats, color="#7b1fa2", label="Cross-track Error", lw=1.5)
        self.ax_err.axhline(0.0, color="gray", linestyle="-", lw=0.8)
        self.ax_err.axvline(rec.t, color="black", linestyle=":", alpha=0.7)
        self.ax_err.legend(loc="upper right", fontsize=8)
        self.ax_err.grid(True, linestyle="--", alpha=0.5)

        self.canvas_plots.draw_idle()

    def _export_csv(self):
        if not self.records:
            QtWidgets.QMessageBox.warning(self, "Warning", "No data to export.")
            return

        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Tracking Log to CSV", "tracking_velocity_log.csv", "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                header = [
                    "time_s", "cmd_vx", "cmd_vy", "cmd_norm", "cmd_w",
                    "wheel_vx", "wheel_vy", "wheel_norm", "wheel_w",
                    "tf_vx", "tf_vy", "tf_norm", "tf_w",
                    "robot_x", "robot_y", "robot_yaw",
                    "error_lat", "error_yaw"
                ]
                f.write(",".join(header) + "\n")
                for r in self.records:
                    line = [
                        f"{r.t:.4f}", f"{r.cmd_vx:.4f}", f"{r.cmd_vy:.4f}", f"{r.cmd_norm:.4f}", f"{r.cmd_w:.4f}",
                        f"{r.wheel_vx:.4f}", f"{r.wheel_vy:.4f}", f"{r.wheel_norm:.4f}", f"{r.wheel_w:.4f}",
                        f"{r.tf_vx:.4f}", f"{r.tf_vy:.4f}", f"{r.tf_norm:.4f}", f"{r.tf_w:.4f}",
                        f"{r.robot_x:.4f}", f"{r.robot_y:.4f}", f"{r.robot_yaw:.4f}",
                        f"{r.error_lat:.4f}", f"{r.error_yaw:.4f}"
                    ]
                    f.write(",".join(line) + "\n")
            QtWidgets.QMessageBox.information(self, "Success", f"Log saved successfully to:\n{file_path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error saving CSV", str(e))

    def closeEvent(self, event):
        if self.live_subscriber:
            self.live_subscriber.stop()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = TrackingDebuggerMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
