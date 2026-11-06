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
3. **動的適応型 B-spline 平滑化・直線近似・ドッキングアプローチ区間結合**:
   - 制御点の間引きステップ（最大 `bspline.max_stride`）と反復ラプラシアン平滑化（`bspline.smoothing_iterations` 回）を段階的に試行し、`bspline.smooth_max_cost` 以下で障害物に干渉しない最も滑らかな曲線を採用。
   - 経路のうち、セルコストが `line_approximation.max_cost` 以下の領域を通過し、かつ長さが `line_approximation.min_distance` 以上ある区間を直線近似（接続点は微小ラウンディングで曲率スパイクを防止）。
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
| `line_approximation.max_cost` | 20 | 直線近似を許容する最大セルコスト (-1: 無効, 0〜99) |
| `line_approximation.min_distance` | 0.50 m | 直線化する最小セグメント距離 |
| `footprint.polygon` | 8頂点座標列 | 機体の外形ポリゴン頂点 [m] |
| `rotation_planning.turn_max_cost` | 20 | 旋回区間全域で許容する最大フットプリントコスト |
| `rotation_planning.angle_step` | 0.1 rad | 旋回干渉チェックの角度ステップ |
| `rotation_planning.turn_distance` | 0.6 m | 旋回を行う移動区間長 |

---

## 4. 経路追従 (`path_follower_flow.pu`)

### 処理の流れ
1. **経路前処理 (`onPlannedPath`)**:
   - `path_interpolation_step` 間隔で経路を細分化し、単位接線ベクトルと曲率 $\kappa$ を計算。
   - 横加速度制限 $v \le \sqrt{a_{\text{lat,max}} / |\kappa|}$、旋回角速度要求 $|d\theta/ds|$ による並進速度制限（`turn_yaw_rate_ratio`, `min_turn_speed`）、ロケットスタート初速ランプ＋S字速度プロファイル、ゴール近傍の位置P制御速度上限（$K_p \cdot d_{\text{rem}}$）、および前後方加減速走査から計画速度プロファイルを生成。
2. **周期制御ループ (`controlLoop`)**:
   - 現在位置を経路へ直交射影し、`max_path_error` 超過時は非常停止（`Fail`）。
   - **FF + FB 統合速度計算**:
     - 巡航中は遅延補償点 `ref_pt` の接線速度 $\boldsymbol{v}_{\text{ff}} = v_{\text{ref}} \boldsymbol{t}$ と、現在位置の直交射影点 `proj_pt` への横ズレPD補正 $\boldsymbol{v}_{\text{fb}} = K_p \boldsymbol{e} + K_d \dot{\boldsymbol{e}}$（`max_cruise_fb_speed` でクランプ）を合成。
     - ゴール近傍（`settle_zone_distance` 以内）ではゴール座標へのPID制御（`position_kp / ki / kd`、`max_fb_speed` でクランプ）と最小起動速度リフト（`min_docking_speed`）により振動を抑えつつ静止摩擦を突破して収束。
   - **1次最大加速度リミッタ**: 1周期あたりの速度変化量を `accel_limit * dt` 以内に制限（巡航発進時はロケットスタート初期ステップを付与）。
   - **姿勢PID制御**: 目標Yaw角へのPID制御＋ゴール近傍の最小角速度リフト（`min_docking_yaw_rate`）＋1次角加速度リミッタ。
   - **ゴール判定 (Settle Counter)**: 位置誤差 `goal_tolerance` 以下かつ角度誤差 `goal_yaw_tolerance` 以下を `goal_settle_count` 周期連続で維持したら `Goal` へ遷移して完全停止。

### パラメータ (`config/path_follower_params.yaml`)
| パラメータ名 | 既定値 | 役割 |
|---|---|---|
| `control_rate` | 50.0 Hz | 制御ループ周期 |
| `max_speed` | 3.0 m/s | 最大並進速度 |
| `max_acceleration / max_deceleration` | 8.0 / 6.0 m/s^2 | 最大並進加減速度（前処理および1次リミッタ） |
| `max_jerk` | 150.0 m/s^3 | 前処理S字プロファイルの最大躍度 |
| `max_lateral_acceleration` | 3.0 m/s^2 | カーブ通過時の最大横加速度 |
| `max_angular_velocity / acceleration` | 3.0 rad/s, 5.0 rad/s^2 | 最大角速度・最大角加速度 |
| `turn_yaw_rate_ratio` | 0.75 | 旋回角速度要求（\|dθ/ds\|）に対する並進速度制限時の許容角速度比率 |
| `min_turn_speed` | 0.3 m/s | 旋回制限時の最低並進速度下限値 |
| `path_interpolation_step` | 0.02 m | 経路補間ステップ間隔 |
| `curvature_window` | 0.50 m | 曲率計算の前後距離窓幅 |
| `decel_effective_ratio` | 0.70 | 後方減速走査の実効減速度比率 |
| `position_kp / ki / kd` | 20.0 / 0.0 / 2.5 | 位置フィードバックのPIDゲイン |
| `position_d_filter_alpha` | 0.4 | 位置D項の1次LPF係数 |
| `max_fb_speed` | 0.60 m/s | ゴール近傍（`in_settle_zone` / ドッキング時）の位置FB速度上限 |
| `max_cruise_fb_speed` | 1.50 m/s | 巡航中（`!in_settle_zone`）の位置FB速度上限 |
| `min_docking_speed` | 0.02 m/s | ゴール近傍の最小起動並進速度 |
| `settle_zone_distance` | 0.50 m | ゴール近傍の精密収束・積分有効化距離 |
| `heading_kp / ki / kd` | 3.0 / 0.2 / 0.1 | 姿勢(Yaw)制御のPIDゲイン |
| `heading_integral_max` | 0.3 | 姿勢I項の上限値 |
| `heading_d_filter_alpha` | 0.4 | 姿勢D項の1次LPF係数 |
| `min_docking_yaw_rate` | 0.03 rad/s | ゴール近傍の最小起動角速度 |
| `goal_tolerance / goal_yaw_tolerance` | 0.005 m / 0.0175 rad | ゴール到達許容誤差（±5mm / ±1.0度） |
| `goal_settle_count` | 5 | ゴール完了判定の連続維持周期数 |
| `max_path_error / max_initial_path_error` | 0.30 m / 0.45 m | 非常停止する経路逸脱距離上限 |
| `max_pose_age / control_comm_delay` | 0.2 s / 0.025 s | TF許容遅延と通信遅延補正時間 |
| `tracking_kp / ki / kd` | 3.5 / 0.0 / 0.15 | MovingBucket追従専用Yaw PIDゲイン |
| `tracking_integral_max` | 0.2 | MovingBucket追従I項上限 |
| `tracking_d_filter_alpha` | 0.2 | MovingBucket追従D項LPF係数 |
| `tracking_deadband` | 0.015 rad (約0.86度) | 追従目標角度誤差の不感帯（微小チャタリング・モータ鳴き防止） |
| `tracking_pos_filter_alpha` | 0.25 | TFバケツ目標位置（map系）のEMA平滑化係数 |
| `max_tracking_angular_velocity / acceleration` | 2.0 rad/s / 12.0 rad/s^2 | 移動バケツ追従時の最大角速度・角加速度制限 |
| `tracking_timeout` | 1.0 s | バケツTF一時ロスト時の追従猶予時間（超過時にFail遷移） |

---

## 5. 移動バケツ追従射出速度計算 (`moving_bucket_launcher_flow.pu`)

相手ロボット上の可動バケツ（Target ID 7: `MovingBucket`）に対し、TF（既定: `bucket_bottom`）または LiDAR Perception System の開口部トピックから `base_link` 基準で直動ベルトの射出速度をリアルタイムに逆算します。

### 処理の流れ
1. **排他制御および状態チェック**:
   - `target_id == MovingBucket (7)` かつ `navigation_command == Move (2)` の時のみ計算およびパブリッシュを実行。固定ターゲット時や `Sleep` / `Stanby` 時は `target_manager_node` との競合および不要な速度指令を防止するため完全停止。
2. **位置取得方式（TF同期またはトピックコールバック）**:
   - `use_tf == true`（既定）: `calculation_rate`（20Hz）で `bucket_frame`（既定: `bucket_bottom`）の `base_link` 相対座標を TF バッファから直接取得。`is_bottom_frame == true` の場合は Z 座標に `bucket_height` を自動加算して上部開口部高さを算出。
   - `use_tf == false`: `/bucket_opening_tracker/target` または `bottom` トピックを受信した際に `base_link` へ座標変換して計算。
3. **有効性・鮮度検証**:
   - 動的TF時は `(now - stamp) <= max_tracking_age_sec`（静的TFやタイムスタンプ0は有効扱い）を満たす場合のみ処理継続。
4. **3次元弾道方程式による初速逆算とリリース速度変換**:
   - `base_link` 原点からの水平距離 $L = \sqrt{x^2 + y^2}$ と有効リリース位置からの水平飛行距離 $\Delta x = L - x_0$、および目標高さ差 $\Delta z = (z_{\text{open}} + \text{aim\_z\_offset}) - z_0$ を算出。
   - `gn10-shot-manager` の軌道解析で同定した `base_link` 基準の放物弾道式から必要弾道初速 $v_0$ を逆算：
     $$v_0 = \sqrt{\frac{g \Delta x^2}{2 \cos^2\theta (\Delta x \tan\theta - \Delta z)}}$$
   - リリース速度変換式によりベルト目標速度を算出：
     $$v_{\text{belt}} = \text{velocity\_quad} \cdot v_0^2 + \text{velocity\_scale} \cdot v_0 + \text{velocity\_offset}$$
5. **クランプ & 指数平滑化 (EMA) & 配信**:
   - 機構許容範囲（`min_velocity` 〜 `max_velocity`）へクランプした後、`speed_filter_alpha`（既定: 0.25）のEMAフィルタで平滑化し、目標速度ジッターを抑制して `/robot/operation/belt_launcher_speed` へ発行。

### パラメータ (`config/moving_bucket_launcher_params.yaml`)
| パラメータ名 | 既定値 | 役割 |
|---|---|---|
| `base_frame` | `base_link` | 軌道計算を行う基準ロボットフレーム |
| `bucket_frame` | `bucket_bottom` | TF追従対象のバケツフレーム |
| `use_tf` | `true` | TFキャッシュから周期タイマーで目標位置を取得するかどうか |
| `is_bottom_frame` | `true` | `bucket_frame` が底面か（`true` の場合 Z 座標に `bucket_height` を加算） |
| `calculation_rate` | `20.0` Hz | TF基準の速度計算・配信タイマー周期 |
| `speed_filter_alpha` | `0.25` | 射出目標速度の1次EMAローパスフィルタ係数 |
| `opening_topic` | `/bucket_opening_tracker/target` | バケツ上部開口部トラッキングトピック（`use_tf=false` 時） |
| `bottom_topic` | `/bucket_opening_tracker/bottom` | バケツ底面トピック（`use_tf=false` かつ `use_bottom_topic=true` 時） |
| `use_bottom_topic` | `false` | `true` の場合は底面Z座標 + `bucket_height` を上部開口部として使用 |
| `bucket_height` | `0.255` m | バケツ底面から上部開口部までの高さ |
| `aim_z_offset` | `0.0` m | 上部開口部中心に対する狙い高さオフセット |
| `launcher_offset_x / z` | `0.348` m / `1.034` m | `base_link` 基準の有効リリース位置オフセット $(x_0, z_0)$ |
| `launch_angle_deg` | `45.88` deg | 有効リリース仰角 $\theta$ |
| `velocity_quad / scale / offset` | `0.0` / `0.806` / `-0.243` | 弾道初速 $v_0$ からベルトリリース速度 $v_{\text{belt}}$ への変換係数 |
| `min_velocity / max_velocity` | `2.5` / `7.75` m/s | 機構速度上下限リミット |
| `min_horizontal_dist / max_horizontal_dist` | `0.6` / `5.5` m | 計算対象とする水平距離範囲 |
| `max_tracking_age_sec` | `0.5` s | バケツ追従計測値の許容最大遅延 |

---

## 6. デバッグツール

速度指令値・ホイール順運動学速度・TF実効速度の比較および軌道追従の可視化には [tracking_debugger.md](tracking_debugger.md) を参照してください。

```bash
ros2 launch gn10_navigation tracking_debugger.launch.py
```
