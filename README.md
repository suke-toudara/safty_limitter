# Safety Limiter

ROS2パッケージ：ロボットが障害物に衝突しないように安全監視を行います。

## 機能

- `cmd_vel`と点群データ（PointCloud2）を受信
- TFから自己位置を取得
- 速度と自己位置から将来のロボット位置を予測
- PCLの近傍探索（KD-tree）を使用して、フットプリント内の障害物を検出
- フットプリント内に点群がある場合は停止
- フットプリントを広げた範囲に点群がある場合は徐々に減速（0~1のスケール）
- **各時間での監視範囲を可視化**（RVizでマーカー表示）
- **トピックタイムアウト監視**：一定時間トピックが受信されない場合、ロボットを停止

## トピック

### Subscribed Topics

- `cmd_vel_in` (geometry_msgs/Twist): 入力速度コマンド（デフォルト: `/cmd_vel_raw`）
- `cloud` (sensor_msgs/PointCloud2): 点群データ（デフォルト: `/cloud`）

### Published Topics

- `cmd_vel_out` (geometry_msgs/Twist): 安全制限を適用した速度コマンド（デフォルト: `/cmd_vel`）
- `safety_markers` (visualization_msgs/MarkerArray): 監視範囲の可視化マーカー（赤：停止ゾーン、黄：減速ゾーン）

## パラメータ

- `robot_frame` (string, default: "base_link"): ロボットのフレームID
- `map_frame` (string, default: "odom"): マップのフレームID
- `publish_rate` (double, default: 10.0): パブリッシュレート [Hz]
- `prediction_time` (double, default: 2.0): 予測時間 [秒]
- `prediction_step` (double, default: 0.1): 予測のタイムステップ [秒]
- `footprint_x_front` (double, default: 0.5): ロボット前方の距離 [m]
- `footprint_x_rear` (double, default: 0.5): ロボット後方の距離 [m]
- `footprint_y` (double, default: 0.4): ロボットの半幅 [m]
- `safety_margin` (double, default: 0.0): 安全マージン（即座停止ゾーン）[m]
- `slowdown_margin` (double, default: 0.2): 減速マージン [m]
- `min_velocity_scale` (double, default: 0.0): 最小速度スケール（0.0=完全停止、1.0=フルスピード）
- `enable_visualization` (bool, default: true): 可視化マーカーの有効/無効
- `cmd_vel_timeout` (double, default: 0.5): cmd_velトピックのタイムアウト [秒]
- `cloud_timeout` (double, default: 1.0): 点群トピックのタイムアウト [秒]

## ビルド方法

```bash
cd <your_workspace>
colcon build --packages-select safety_limiter
source install/setup.bash
```

## 実行方法

```bash
ros2 launch safety_limiter safety_limiter.launch.py
```

カスタムパラメータを使用する場合：

```bash
ros2 launch safety_limiter safety_limiter.launch.py config_file:=/path/to/your/params.yaml
```

## 仕組み

1. **トピックタイムアウト監視**
   - `cmd_vel_in`と`cloud`の最終受信時刻を記録
   - 設定時間を超えた場合は速度を0にして停止
2. `cmd_vel_in`から速度コマンドを受信
3. `cloud`から点群データを受信し、PCL KD-treeを構築
4. TFから現在のロボット位置を取得
5. 速度コマンドに基づいて将来のロボット軌跡を予測
6. 各予測位置でのフットプリント内に障害物があるかチェック（KD-tree近傍探索）
   - フットプリント内に点群がある場合：速度スケール = `min_velocity_scale`（停止）
   - フットプリント+減速マージン内に点群がある場合：距離に応じて速度を減速（0~1）
7. スケールを適用した速度コマンドを`cmd_vel_out`にパブリッシュ
8. **可視化マーカーをパブリッシュ**（`enable_visualization`がtrueの場合）
   - 赤色のマーカー：停止ゾーン（フットプリント + safety_margin）
   - 黄色のマーカー：減速ゾーン（停止ゾーン + slowdown_margin）

## RVizでの可視化

可視化マーカーをRVizで表示するには：

1. RVizを起動
2. 「Add」→「MarkerArray」を選択
3. Topicを`/safety_markers`に設定
4. Fixed Frameを`odom`（またはmap_frameパラメータに合わせたフレーム）に設定

赤色の矩形が停止ゾーン、黄色の矩形が減速ゾーンを示します。
