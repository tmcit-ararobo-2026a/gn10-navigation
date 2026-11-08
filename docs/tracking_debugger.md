# 追従状況および速度比較デバッグツール

ロボットの経路追従状況および以下の3種類の速度を同期・比較可視化し、制御遅れや車輪スリップ、旋回挙動を解析するためのGUIデバッグ環境です。

1. **速度指令値 ($v_{\mathrm{cmd}}$)**: ナビゲーションノードが出力する機体ローカル速度指令
2. **ホイール順運動学速度 ($v_{\mathrm{wheel}}$)**: 車輪エンコーダ（M3508）フィードバックから順運動学により逆算した機体速度
3. **TF実効速度 ($v_{\mathrm{tf}}$)**: 自己位置推定（`map -> base_link`）の時間微分による実移動速度

---

## 1. 起動方法

### オンライン実行（実機またはシミュレータ稼働時）

```bash
# 方法1: ROS 2ノードとして起動
ros2 run gn10_navigation tracking_debugger_gui.py

# 方法2: Launchファイルで起動
ros2 launch gn10_navigation tracking_debugger.launch.py
```

画面上部の **「● Live ROS 2 Start」** ボタンを押すと、リアルタイム購読が開始されます。

### オフライン実行（rosbagの再生なしで直接解析）

画面上部の **「📂 Open Rosbag (.db3)」** を押し、記録済みの rosbag2 ディレクトリ（または `.db3` ファイル）を選択します。
ファイル全体が読み込まれ、シークバーで各時刻の状態を再生・確認できます。

---

## 2. 運動学モデルと速度算出式

### (1) 3輪オムニ順運動学 ($v_{\mathrm{wheel}}$)

メイン基板の運動学定義に基づき、各車輪の配置およびパラメータを以下のように設定しています：

- **Front (前輪, index 0)**: 左右方向の駆動
- **Back Left (左後輪, index 1)**: $120^\circ$ 配置
- **Back Right (右後輪, index 2)**: $-120^\circ$ 配置
- **M3508 ギア比**: $G = 19.0$
- **ホイール半径**: $r = 0.065\ \mathrm{m}$ (直径 130 mm)
- **機体中心半径**: $R = 0.40\ \mathrm{m}$

モータ軸角速度 $\omega_f, \omega_{bl}, \omega_{br}$ [rad/s] から各車輪の周速度 $v$ [m/s] を算出：

$$v_f = \frac{\omega_f}{G} \cdot r, \quad v_{bl} = \frac{\omega_{bl}}{G} \cdot r, \quad v_{br} = \frac{\omega_{br}}{G} \cdot r$$

機体ローカル座標系 (`base_link`) における速度への順運動学変換：

$$v_{\mathrm{wheel}, x} = \frac{v_{br} - v_{bl}}{\sqrt{3}}$$

$$v_{\mathrm{wheel}, y} = \frac{-2 v_f + v_{bl} + v_{br}}{3}$$

$$\omega_{\mathrm{wheel}, z} = -\frac{v_f + v_{bl} + v_{br}}{3 R}$$

$$v_{\mathrm{wheel}, \mathrm{norm}} = \sqrt{v_{\mathrm{wheel}, x}^2 + v_{\mathrm{wheel}, y}^2}$$

### (2) TF実効速度 ($v_{\mathrm{tf}}$)

`map -> base_link` の位置 $(X, Y)$ およびヨー角 $\theta$ の時間微分を機体ローカル座標系へ射影し、ローパスフィルタ（LPF）により平滑化：

$$v_{\mathrm{tf}, x} = \cos(\theta) \dot{X} + \sin(\theta) \dot{Y}$$

$$v_{\mathrm{tf}, y} = -\sin(\theta) \dot{X} + \cos(\theta) \dot{Y}$$

$$\omega_{\mathrm{tf}, z} = \dot{\theta}$$

$$v_{\mathrm{tf}, \mathrm{norm}} = \sqrt{v_{\mathrm{tf}, x}^2 + v_{\mathrm{tf}, y}^2}$$

---

## 3. 画面構成とデバッグ指標

### 2D 追従マップ (左側)
- **計画経路 (/planned_path)**: 青破線
- **実機走行軌跡 (TF)**: 灰色実線
- **ロボット姿勢・中心**: 黒丸および向き線
- **速度ベクトル矢印 (現在位置から重畳描画)**:
  - **青矢印**: 速度指令値 ($v_{\mathrm{cmd}}$)
  - **緑矢印**: ホイール順運動学速度 ($v_{\mathrm{wheel}}$)
  - **赤矢印**: TF実移動速度 ($v_{\mathrm{tf}}$)

#### 診断のポイント
- **青（指令）と緑（ホイール）の差異**: モータ速度フィードバック制御の応答遅れ、電流リミット、トルク飽和
- **緑（ホイール）と赤（実移動）の差異**: オムニホイールの車輪空転、スリップ、カーブ旋回時の遠心力による外流れ
- **青・緑と赤の向きの差**: 機体のスリップアングル（進行方向のズレ）

### 時系列グラフ (右側)
1. **並進合成速度 $v_{\mathrm{norm}}$ [m/s]**: 指令値、ホイール順運動学速度、TF実効速度の比較
2. **前後速度 $v_x$ および 横速度 $v_y$ [m/s]**: 機体軸ごとの追従
3. **ヨー角速度 $\omega_z$ [rad/s]**: 旋回角速度の応答
4. **追従横偏差 [m]**: 計画経路に対する直交誤差

### CSVエクスポート
画面上部の **「💾 Export CSV」** ボタンで、各時刻の速度、誤差、位置をCSVファイルとして保存できます。
