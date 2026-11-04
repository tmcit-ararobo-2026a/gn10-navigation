# gn10_navigation 処理フローおよびパラメータガイド

`gn10_navigation` パッケージの各ノードの処理フローとパラメータを説明します。
各フロー図は同ディレクトリ内の PlantUML (`.pu`) ファイルに定義されています。

---

## 1. 全体アーキテクチャ (`navigation_system_architecture.pu`)

```
[ Upper Commands ]
       │ (/robot/target_id, /robot/navigation_command)
       ▼
[ target_manager_node ] ── (/goal_pose) ──► [ path_planner_node ] ── (/planned_path) ──► [ path_follower_node ] ── (/cmd_vel) ──► [ Base Actuators ]
                                                   ▲                                            ▲
                                                   │ (/costmap)                                 │ (TF: map -> base_link)
                                        [ costmap_generator_node ]                        [ TF Listener ]
```

- **`target_manager_node`**: Target IDに応じた目標座標（`/goal_pose`）と射出速度を発行します。
- **`costmap_generator_node`**: JSONマップから障害物インフレーション付きのコストマップを生成します。
- **`path_planner_node`**: 8方向A*探索、反復ラプラシアン付き動的B-spline平滑化、旋回区間全域のフットプリント安全判定により、始点・終点の量子化誤差がない経路を生成します。
- **`path_follower_node`**: 接線フィードフォワード速度と直交位置フィードバック速度の統合制御、および1次加速度リミッタにより、経路追従とゴール精密停止（±5mm, ±1.0度）を行います。

---

## 2. コストマップ生成 (`costmap_generator_flow.pu`)

### 処理の流れ
1. **JSONマップ読み込み**: `nhk2026_map.json` からフィールドオブジェクトを取得。
2. **高さフィルタ**: `robot_clearance.z_min` 〜 `z_max` 範囲外のオブジェクトを除外。
3. **インフレーション (Multi-Source BFS)**:
   - 障害物から `inflation.robot_radius` 以内: コスト 100 (侵入不可)
   - `robot_radius + inflation.margin` まで: 距離に応じて減衰するコスト (1〜99) を付与
4. **定期配信**: `publish_rate` [Hz] で `/costmap` をパブリッシュ。

### パラメータ (`config/costmap_params.yaml`)
| パラメータ名 | 既定値 | 役割 |
|---|---|---|
| `publish_rate` | 1.0 Hz | コストマップ配信周期 |
| `resolution` | 0.05 m | グリッド解像度 |
| `map_bounds.*` | -6.0〜6.0, -6.5〜6.5 | マップ範囲 [m] |
| `robot_clearance.z_min / z_max` | 0.0 / 1.4 m | 衝突判定を行う高さ範囲 |
| `inflation.robot_radius` | 0.45 m | 機体中心の不可侵半径 |
| `inflation.margin` | 0.50 m | 回避コストを波及させるマージン幅 |

---

## 3. 経路計画 (`path_planner_flow.pu`)

### 処理の流れ
1. **自己位置取得 & スナップ**:
   - 始点またはゴールが障害物セル (`cost >= 100`) にある場合、`a_star.snap_distance` 内の最寄り通行可能セルへスナップ。
   - ゴールが障害物内（教壇等への突き当て接触位置など）にある場合、最寄り通行可能セルを「安全アプローチ点」として設定しつつ、本来の目標座標も破棄せず保持。
2. **8方向 A* 探索**:
   - 移動距離＋`costmap_val * a_star.planning_cost` を最小化する格子経路を探索。
3. **動的適応型 B-spline 平滑化 & ドッキングアプローチ区間結合**:
   - 制御点の間引きステップ（最大 `bspline.max_stride`）と反復ラプラシアン平滑化（`bspline.smoothing_iterations` 回）を段階的に試行し、`bspline.smooth_max_cost` 以下で障害物に干渉しない最も滑らかな曲線を採用。
   - ゴールが障害物内にめり込んでいる場合は、安全アプローチ点から本来の目標地点への直線進入区間を平滑化経路の終端に付加（障害物直前まで安全移動した後に障害物へ直進アプローチ）。
4. **安全領域での旋回計画**:
   - 長さ `rotation_planning.turn_distance` の旋回区間ウィンドウを走査し、区間全域で全回転角度のフットプリントコストが `rotation_planning.turn_max_cost` 以下かつ最小となる最も安全な領域を選定して Smoothstep で目標Yaw角を補間。

### パラメータ (`config/path_planner_params.yaml`)
| パラメータ名 | 既定値 | 役割 |
|---|---|---|
| `a_star.planning_cost` | 1.5 | コストマップ値の重み（大きいほど障害物を遠回り） |
| `a_star.snap_distance` | 0.40 m | 障害物内セルからの最大スナップ探索半径 |
| `bspline.samples_per_segment` | 10 | B-spline の制御点間サンプル数 |
| `bspline.smoothing_strength` | 1.0 | 1反復あたりの内部制御点の中点ブレンド率 (0.0〜1.0) |
| `bspline.smoothing_iterations` | 5 | 内部制御点の反復ラプラシアン平滑化回数 (0〜50) |
| `bspline.max_stride` | 4 | 制御点間引きステップの最大値 (1〜10) |
| `bspline.smooth_max_cost` | 80 | 平滑化経路が許容する最大セルコスト (0〜99) |
| `footprint.polygon` | 8頂点座標列 | 機体の外形ポリゴン頂点 [m] |
| `rotation_planning.turn_max_cost` | 20 | 旋回区間全域で許容する最大フットプリントコスト |
| `rotation_planning.angle_step` | 0.1 rad | 旋回干渉チェックの角度ステップ |
| `rotation_planning.turn_distance` | 0.6 m | 旋回を行う移動区間長 |

---

## 4. 経路追従 (`path_follower_flow.pu`)

### 処理の流れ
1. **経路前処理 (`onPlannedPath`)**:
   - `path_interpolation_step` 間隔で経路を細分化し、単位接線ベクトルと曲率 $\kappa$ を計算。
   - 横加速度制限 $v \le \sqrt{a_{\text{lat,max}} / |\kappa|}$、旋回角速度要求 $|d\theta/ds|$ による並進速度制限（`turn_yaw_rate_ratio`, `min_turn_speed`）、S字速度プロファイル、ゴール近傍の位置P制御速度上限（$K_p \cdot d_{\text{rem}}$）、および前後方加減速走査から計画速度プロファイルを生成。
2. **周期制御ループ (`controlLoop`)**:
   - 現在位置を経路へ直交射影し、`max_path_error` 超過時は非常停止（`Fail`）。
   - **FF + FB 統合速度計算**:
     - 巡航中は遅延補償点 `ref_pt` の接線速度 $\boldsymbol{v}_{\text{ff}} = v_{\text{ref}} \boldsymbol{t}$ と、現在位置の直交射影点 `proj_pt` への横ズレ補正 $\boldsymbol{v}_{\text{fb}} = K_p (\boldsymbol{p}_{\text{proj}} - \boldsymbol{p}_{\text{robot}})$ を合成。
     - ゴール近傍（`settle_zone_distance` 以内）ではゴール座標へのPI制御と最小起動速度リフト（`min_docking_speed`）により静止摩擦を突破して収束。
   - **1次最大加速度リミッタ**: 1周期あたりの速度変化量を `accel_limit * dt` 以内に制限。
   - **姿勢PID制御**: 目標Yaw角へのPID制御＋ゴール近傍の最小角速度リフト（`min_docking_yaw_rate`）＋1次角加速度リミッタ。
   - **ゴール判定 (Settle Counter)**: 位置誤差 `goal_tolerance` 以下かつ角度誤差 `goal_yaw_tolerance` 以下を `goal_settle_count` 周期連続で維持したら `Goal` へ遷移して完全停止。

### パラメータ (`config/path_follower_params.yaml`)
| パラメータ名 | 既定値 | 役割 |
|---|---|---|
| `control_rate` | 50.0 Hz | 制御ループ周期 |
| `max_speed` | 2.0 m/s | 最大並進速度 |
| `max_acceleration / max_deceleration` | 3.0 m/s^2 | 最大並進加減速度（前処理および1次リミッタ） |
| `max_jerk` | 50.0 m/s^3 | 前処理S字プロファイルの最大躍度 |
| `max_lateral_acceleration` | 2.0 m/s^2 | カーブ通過時の最大横加速度 |
| `max_angular_velocity / acceleration` | 2.5 rad/s, 5.0 rad/s^2 | 最大角速度・最大角加速度 |
| `turn_yaw_rate_ratio` | 0.75 | 旋回角速度要求（\|dθ/ds\|）に対する並進速度制限時の許容角速度比率 |
| `min_turn_speed` | 0.3 m/s | 旋回制限時の最低並進速度下限値 |
| `path_interpolation_step` | 0.02 m | 経路補間ステップ間隔 |
| `curvature_window` | 0.10 m | 曲率計算の前後距離窓幅 |
| `decel_effective_ratio` | 0.85 | 後方減速走査の実効減速度比率 |
| `position_kp / position_ki` | 4.0 / 0.5 | 位置フィードバックのP・Iゲイン |
| `max_fb_speed` | 0.60 m/s | 位置フィードバック速度の上限 |
| `min_docking_speed` | 0.02 m/s | ゴール近傍の最小起動並進速度 |
| `settle_zone_distance` | 0.10 m | ゴール近傍の精密収束・積分有効化距離 |
| `heading_kp / ki / kd` | 3.0 / 0.2 / 0.1 | 姿勢(Yaw)制御のPIDゲイン |
| `heading_integral_max` | 0.3 | 姿勢I項の上限値 |
| `heading_d_filter_alpha` | 0.4 | 姿勢D項の1次LPF係数 |
| `min_docking_yaw_rate` | 0.03 rad/s | ゴール近傍の最小起動角速度 |
| `goal_tolerance / goal_yaw_tolerance` | 0.005 m / 0.0175 rad | ゴール到達許容誤差（±5mm / ±1.0度） |
| `goal_settle_count` | 5 | ゴール完了判定の連続維持周期数 |
| `max_path_error / max_initial_path_error` | 0.30 m / 0.45 m | 非常停止する経路逸脱距離上限 |
| `max_pose_age / control_comm_delay` | 0.2 s / 0.025 s | TF許容遅延と通信遅延補正時間 |

---

## 5. 移動バケツ追従射出速度計算 (`moving_bucket_launcher_flow.pu`)

相手ロボット上の可動バケツ（Target ID 0: `MovingBucket`）に対し、LiDAR Perception System の推定底面位置から直動ベルトの射出速度をリアルタイムに逆算します。

### 処理の流れ
1. **排他制御チェック**:
   - `target_id == MovingBucket (0)` の時のみ計算およびパブリッシュを実行。固定ターゲット時は `target_manager_node` とのトピック競合を防止するため完全停止。
2. **有効性・鮮度検証**:
   - `BucketBottom.valid == true` かつ `(now - last_observation_stamp) <= max_tracking_age_sec` の場合のみ処理を継続。
3. **TF相対座標変換**:
   - 計測座標を `base_link` へ変換し、ランチャー射出位置オフセット（`launcher_offset_x`, `launcher_offset_z`）を差し引いて相対距離 $L$ と高低差 $\Delta z$ を算出。
4. **3次元弾道方程式による初速逆算**:
   - 仰角 $\theta$（既定 $45^\circ$）による放物弾道式：
     $$v_0 = \sqrt{\frac{g L^2}{2 \cos^2\theta (L \tan\theta - \Delta z)}}$$
   - スリップ・抵抗補正: $v_{\text{target}} = v_0 \cdot \text{scale} + \text{offset}$
5. **バッテリー電圧低下補正 & クランプ**:
   - バッテリー電圧（`/robot/feedback/drive_battery_voltage`）が $15\text{V}$ 以上の正常値の場合、基準電圧（$20\text{V}$）との偏差に応じて速度を微調整。
   - 機構許容範囲（`min_velocity` 〜 `max_velocity`）へクランプして `/robot/operation/belt_launcher_speed` へ発行。

### パラメータ
| パラメータ名 | 既定値 | 役割 |
|---|---|---|
| `base_frame` | `base_link` | 基準ロボットフレーム |
| `launcher_offset_x / z` | 0.20 m / 0.35 m | 機体中心からの射出点オフセット |
| `launch_angle_deg` | 45.0 deg | ランチャー射出仰角 |
| `min_velocity / max_velocity` | 3.0 / 7.75 m/s | 機構速度上下限リミット |
| `velocity_scale / offset` | 1.05 / 0.10 m/s | 弾道スケール・切片補正係数 |
| `voltage_ref` | 20.0 V | 基準電源電圧 |
| `voltage_comp_gain` | 0.02 /V | 電圧低下1Vあたりの速度補正比率 |
| `max_tracking_age_sec` | 0.60 s | バケツ追従計測値の許容最大遅延 |

---

## 6. デバッグツール

速度指令値・ホイール順運動学速度・TF実効速度の比較および軌道追従の可視化には [tracking_debugger.md](tracking_debugger.md) を参照してください。

```bash
ros2 launch gn10_navigation tracking_debugger.launch.py
```
