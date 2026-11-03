# gn10_navigation 処理フローおよびパラメータ調整ガイド

本ドキュメントでは、`gn10_navigation` パッケージの内部処理フローと、各パラメータがロボットの動作や計画に及ぼす影響、および調整手順を解説します。

各処理フロー図は PlantUML (`.pu`) ファイルとして本ディレクトリ内に配置されています。

---

## 1. 全体アーキテクチャ (`navigation_system_architecture.pu`)

システムは以下の4ノードで構成されています：

```
[ Upper Commands ]
       │
       ▼ (/robot/target_id, /robot/navigation_command)
[ target_manager_node ] ─── (/goal_pose) ───► [ path_planner_node ] ─── (/planned_path) ───► [ path_follower_node ] ─── (/cmd_vel) ───► [ Base Actuators ]
                                                     ▲                                               ▲
                                                     │ (/costmap)                                    │ (TF: map -> base_link)
                                          [ costmap_generator_node ]                               [ TF Listener ]
```

- **`target_manager_node`**: Target IDに応じた投擲ポイント（位置・姿勢・射出速度）を管理・発行
- **`costmap_generator_node`**: JSONフィールドマップから障害物インフレーションコストマップを生成
- **`path_planner_node`**: A*探索、動的適応型B-spline平滑化、機体外形(Footprint)干渉判定付きYaw角計画
- **`path_follower_node`**: S字加減速プロファイル、Pure Pursuit軌道追従、方位PID制御、通信遅延補正

---

## 2. コストマップ生成 (`costmap_generator_flow.pu`)

### 処理の流れ
1. **JSONマップ読み込み**: `nhk2026_map.json` 等からフィールドオブジェクト（BOX, CYLINDER等）を取得。
2. **高さクリアランス判定**: `robot_clearance.z_min` 〜 `robot_clearance.z_max` の範囲外のオブジェクトは除外。
3. **不可侵障害物とペナルティ領域の分離**: 通過可能なペナルティ領域（旗の前など）と絶対不可侵障害物（壁・机・バケツなど）を分離。
4. **Multi-Source BFS インフレーション**:
   - 不可侵障害物から中心点距離が `inflation.robot_radius` 以下: コスト 254 (絶対侵入不可)
   - `inflation.robot_radius` から `robot_radius + inflation.margin` まで: 距離に応じて線形減衰コストを付与
5. **スケーリング & 配信**: OccupancyGrid (0〜100) にスケーリングし、`publish_rate` [Hz] で定期パブリッシュ。

### パラメータ調整項目 (`config/costmap_params.yaml`)
| パラメータ名 | 既定値 | 役割と調整方法 |
|---|---|---|
| `publish_rate` | 1.0 Hz | コストマップの配信周波数。 |
| `resolution` | 0.05 m | グリッドの解像度。小さくすると分解能向上、大きくすると計算負荷軽減。 |
| `map_bounds.*` | -6.0〜6.0, -6.5〜6.5 | フィールド全体の物理範囲。 |
| `robot_clearance.z_min / z_max` | 0.00 / 1.40 m | 機体の接触する高さ範囲。これ以外の高さの障害物は無視されます。 |
| `inflation.robot_radius` | 0.45 m | 機体中心から不可侵とする半径。壁際へ限界まで寄せたい場合は縮小（接触に注意）。 |
| `inflation.margin` | 0.50 m | コストが波及するマージン幅。大きくすると壁から大きく離れた経路を選びます。 |

---

## 3. 経路計画 (`path_planner_flow.pu`)

### 処理の流れ
1. **自己位置取得 & スナップ**:
   - ロボット位置および目標位置がインフレーション領域等で障害物セル (cost >= 100) にある場合、`a_star.snap_distance` の同心円探索を行い、最寄りの通行可能セルに自動スナップ。
2. **8方向 A* グリッド探索**:
   - ステップ移動コストに `costmap_val * a_star.planning_cost` を加算して最短経路を探索。
3. **動的適応型 B-spline 平滑化**:
   - グリッドの折れ線経路を、障害物に接触しない最大の平滑化度合い（内部制御点寄せ率 `bspline.smoothing_strength`、サンプル数 `bspline.samples_per_segment`）で滑らかな連続曲線へ補間。
4. **姿勢(Yaw角)および旋回計画**:
   - 実機外形多角形 `footprint.polygon` を回転させながら、全角度で `rotation_planning.turn_max_cost` 以下となる最も安全な旋回地点を選定。
   - `rotation_planning.turn_distance` の区間でSmoothstep (S字) 補間により目標Yawへ滑らかに遷移。

### パラメータ調整項目 (`config/path_planner_params.yaml`)
| パラメータ名 | 既定値 | 役割と調整方法 |
|---|---|---|
| `a_star.planning_cost` | 1.5 | コストマップ値に対する重み。大きくすると高コストセル（障害物近傍）を大きく回避。小さくすると最短幾何直線に近い経路。 |
| `a_star.snap_distance` | 0.40 m | 壁際など障害物内セルから通行可能セルへスナップする最大探索半径。壁際での計画失敗時はこの値を広げます。 |
| `bspline.smoothing_strength` | 1.0 | 平滑化強度 (0.0=格子そのまま, 1.0=最大平滑化)。角を丸くして高速走行したい場合は大きく設定。 |
| `bspline.samples_per_segment` | 10 | B-spline 補間のサンプル密度。 |
| `footprint.polygon` | 機体外形座標列 | ロボットの実寸8頂点外形多角形。 |
| `rotation_planning.turn_max_cost` | 20 | 旋回を許可する最大コスト。狭い場所で旋回させたい場合は引き上げ、広い安全地帯で旋回させたい場合は引き下げます。 |
| `rotation_planning.angle_step` | 0.1 rad | 旋回干渉チェックの角度刻み幅。 |
| `rotation_planning.turn_distance` | 0.6 m | 旋回に要する移動距離。短くすると急激な旋回、長くすると緩やかな旋回。 |

---

## 4. 経路追従 (`path_follower_flow.pu`)

### 処理の流れ
1. **経路前処理 (onPlannedPath)**:
   - 受信したパスを `path_interpolation_step` 間隔で高密度に補間し、各点の符号付き曲率 $\kappa$ および接線ベクトルを算出。
   - `curvature_window` を用いて局所曲率を計算し、`max_lateral_acceleration` から許容通過速度を算出。
   - 旋回角速度要求 `|dθ/ds|` に応じて並進速度を `turn_yaw_rate_ratio` および `min_turn_speed` で制限。
   - ジャーク制限による減速立ち上がり遅れを打ち消すため、カーブ進入手前 `decel_lead_in_distance` から滑らかに減速を開始（減速前倒し）。
   - `decel_effective_ratio` による実効減速度を用いたバックワード走査とフォワード加速走査（`makeSCurve`）によりS字速度プロファイルを生成。
2. **周期制御ループ (controlLoop)**:
   - `control_rate` [Hz] で実行。自己位置の遅延 (`max_pose_age`) を判定。
   - 経路への直交射影により横偏差を計算。`max_path_error` 超過時は非常停止。
   - 先読み距離 `lookahead = clamp(min_lookahead + lookahead_gain * speed, min, max)` を計算し目標点を取得。
   - **向心加速度フィードフォワード (Centripetal FF)**:
     - 推定遅延時間先のパス点の曲率 $\kappa$ と接線ベクトルから、慣性遠心力に対抗する向心加速度 $\mathbf{a}_{ff} = k_{ff} \cdot v^2 \cdot \kappa \cdot \mathbf{n}_{left}$ を算出。
     - Pure Pursuitのフィードバック加速度と合算し、Jerk制限（躍度制限）を適用して車輪駆動指令へ統合。
   - 目標位置および姿勢に対するゴール判定 (`goal_tolerance`, `goal_yaw_tolerance`)。
   - 目標Yaw角に対するPID制御（LPF係数 `heading_d_filter_alpha` による微分ノイズ低減）。
   - 推定通信・アクチュエータ遅延 `control_comm_delay` と姿勢角から接地瞬間の予測姿勢を計算し、旋回中のドリフトをキャンセルした局所速度指令を出力。

### パラメータ調整項目 (`config/path_follower_params.yaml`)
| パラメータ名 | 既定値 | 役割と調整方法 |
|---|---|---|
| `control_rate` | 60.0 Hz | 制御周期。 |
| `max_speed` | 2.0 m/s | 最大並進速度。 |
| `max_acceleration` | 3.0 m/s^2 | 最大並進加速度。 |
| `max_deceleration` | 3.0 m/s^2 | 最大並進減速度。 |
| `max_jerk` | 40.0 m/s^3 | 最大躍度（加速度の変化率）。走行の滑らかさ、車輪スリップ防止に影響。 |
| `max_lateral_acceleration` | 2.20 m/s^2 | カーブ通過時の遠心力制限および向心加速度FFの上限値。 |
| `curvature_window` | 0.10 m | カーブ曲率計算の前後窓幅。 |
| `centripetal_ff_gain` | 1.0 | 向心加速度フィードフォワードゲイン。カーブ進入時の外膨らみ（アンダーステア）を抑える場合は 1.0 前後、内側に巻き込みすぎる場合は 0.7〜0.8 に調整。 |
| `decel_lead_in_distance` | 0.15 m | カーブ手前での減速開始前倒し距離マージン。急カーブ手前で減速が間に合わず突っ込む場合は大きくします。 |
| `decel_effective_ratio` | 0.85 | ジャーク制限下での実効減速度比率。S字減速時の平均減速度の低下を見越して手前から減速させます。 |
| `min_lookahead / max_lookahead` | 0.15 / 0.40 m | Pure Pursuitの最小・最大先読み距離。小さすぎると蛇行、大きすぎると内回り・角落ち。 |
| `lookahead_gain` | 0.20 s | 速度に応じた先読み距離の比例ゲイン。 |
| `goal_tolerance` | 0.01 m | ゴール到達とみなす位置許容誤差。 |
| `goal_yaw_tolerance` | 0.01 rad | ゴール到達とみなす角度許容誤差。 |
| `goal_approach_speed` | 0.60 m/s | ゴール直前の進入速度上限。 |
| `goal_position_gain` | 10.0 1/s | ゴール位置誤差に対する比例引き込みゲイン。 |
| `path_interpolation_step` | 0.05 m | 追従用経路のリサンプリング間隔。 |
| `heading_kp / ki / kd` | 1.2 / 0.1 / 0.0 | 方位(Yaw)制御のPIDゲイン。 |
| `heading_integral_max` | 0.5 rad | PID積分項のアンチワインドアップ上限。 |
| `heading_d_filter_alpha` | 0.4 | 方位PID微分の1次ローパスフィルタ係数 (0.0〜1.0)。ノイズ振動を抑える場合は小さくします。 |
| `turn_yaw_rate_ratio` | 0.75 | 旋回時の許容角速度上限比率。PIDフィードバック余裕を残すため1.0未満に設定。 |
| `min_turn_speed` | 0.3 m/s | 旋回区間での並進速度の下限値。 |
| `control_comm_delay` | 0.025 s | マイコン通信やアクチュエータ応答の推定遅延時間。旋回中の直進方向のドリフトを補正。 |
| `max_delay_compensation` | 0.20 s | 遅延補償の上限時間。 |

---

## 5. 動的パラメータ変更とログ出力

すべてのノードは `rclcpp` の動的パラメータ変更コールバック (`onSetParameters`) を備えています。
CLI または rqt_reconfigure 等からパラメータを変更すると、即座に検証・反映され、変更結果がログに出力されます。

```bash
# 例: 走行速度と先読みゲインの変更
ros2 param set /path_follower_node max_speed 1.5
ros2 param set /path_follower_node lookahead_gain 0.18

# ログ出力例:
# [path_follower_node]: Parameter 'max_speed' updated to: 1.5
# [path_follower_node]: Parameter 'lookahead_gain' updated to: 0.18
```
