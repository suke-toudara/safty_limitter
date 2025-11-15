# Safety Limiter

ROS2パッケージ：ロボットが障害物に衝突しないように安全監視を行います。

## 機能

- `cmd_vel`と点群データ（PointCloud2）を受信
- TFから自己位置を取得
- 速度と自己位置から将来のロボット位置を予測
- PCLのKD-tree近傍探索で障害物を検出
- **半径内に点群**があれば**停止**
- **半径+減速範囲内に点群**があれば**徐々に減速**（0~1）
- **各時刻の半径を円で可視化**（RVizに赤=停止、黄=減速）
- **トピックタイムアウト監視**で一定時間未受信なら停止

## トピック

### Subscribed Topics

- `cmd_vel_in` (geometry_msgs/Twist): 入力速度コマンド
- `cloud` (sensor_msgs/PointCloud2): 点群データ
- `footprint` (geometry_msgs/PolygonStamped): ロボットフットプリント（半径自動計算）

### Published Topics

- `cmd_vel_out` (geometry_msgs/Twist): 安全制限を適用した速度コマンド
- `safety_markers` (visualization_msgs/MarkerArray): 各時刻の監視半径（赤=停止、黄=減速）

## パラメータ

- `robot_frame` (string, default: "base_link"): ロボットフレーム
- `map_frame` (string, default: "odom"): マップフレーム
- `publish_rate` (double, default: 10.0): パブリッシュレート [Hz]
- `prediction_time` (double, default: 2.0): 予測時間 [秒]
- `prediction_step` (double, default: 0.1): 予測ステップ [秒]
- `slowdown_margin` (double, default: 0.2): 減速範囲 [m]
- `min_velocity_scale` (double, default: 0.0): 最小速度スケール (0=停止, 1=フル)
- `enable_visualization` (bool, default: true): 可視化ON/OFF
- `topic_timeout` (double, default: 1.0): トピックタイムアウト [秒]

**注**: ロボット半径は`/footprint`トピックから自動計算
