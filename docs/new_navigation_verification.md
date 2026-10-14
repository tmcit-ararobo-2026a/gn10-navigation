# 新経路APIの検証（T06〜T08）

実施日: 2026-10-14（Asia/Tokyo）。作業ディレクトリ: `/home/lambda/ros2_ws/src`。
基準HEAD: `cd6a53445a9791e280b1a62078a58c738250e877`、ブランチ `feat/10-path-service-action`。
T01 `8e1c126`、T06 `3a79ba0`、T07 `83c03f6` とT08の差分を対象に検証。

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/stm32-ros2/log-nav-t08-final build --base-paths gn10-navigation   --build-base /tmp/stm32-ros2/build --install-base /tmp/stm32-ros2/install   --packages-select gn10_navigation
source /tmp/stm32-ros2/install/setup.bash
ctest --test-dir /tmp/stm32-ros2/build/gn10_navigation --output-on-failure
colcon test-result --test-result-base /tmp/stm32-ros2/build/gn10_navigation/test_results --verbose
```

- Humble・GCC11・Python3.10でビルド: 成功／終了コード0。
- grid_safety_test 4ケース: 成功。高コスト重複順序、両方向角抜け、格子境界両隣、占有端点、短い横断、非有限・範囲外座標を検証。
- ComputePath実DDS結合: 成功。合成mapで地図不足・不正Pose/frame/quaternion・占有start/goal・無経路・端点一致・同一セル短距離・空Path通知・最新不正mapによる旧map失効。
- FollowPath実DDS結合: 成功。有効leaseなし拒否・不正Path拒否・同時goal拒否・取消・旧世代拒否・lease断・TF断・復旧時非再開・重複lease・point goal最終Yaw整列・完了ゼロ出力を検証。
- CTest3実行対象すべて成功／終了コード0。colcon結果6ケース、失敗0・skip0。
- `ros2 interface show gn10_navigation/srv/ComputePath`: 成功／終了コード0。
- ROS_LOCALHOST_ONLY=1・試験専用domain。速度はROS Topicだけで、UDP・アクチュエータ接続なし。sandbox外でlocalhost DDSを利用。
- CIへテスト実行を追加。CIの最終結果は別途確認。

実装は取消callbackでゼロ出力し、次のcontrol tickで取消結果を通知する。
結合試験は取消結果とゼロ出力を確認するが、実機の停止時間・物理停止を測定したものではない。
Goal位置・最終Yaw閾値、lease期限は初期設計値。実機調整・負荷・HIL・全体受入: 未実施。
Action実行中のgoal差替えは拒否し、取消完了後の新世代で要求する契約。
下位パッケージはmanager/robot_portに依存しない。
