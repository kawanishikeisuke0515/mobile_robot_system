# locomotion_closed_loop

ZED odomの実測速度を使う3軸P制御ノード。車体基準へのTF変換、サンプル数指定の移動平均、4輪配分、比例縮小による出力制限を備える。既存 `locomotion_core` のコードは変更しない。

## ビルド・起動

```bash
cd mobile_robot_system
colcon build --packages-select locomotion_closed_loop
source install/setup.bash
ros2 launch locomotion_closed_loop closed_loop_velocity.launch.py
```

任意の設定ファイルを使用する場合：

```bash
ros2 launch locomotion_closed_loop closed_loop_velocity.launch.py params_file:=/absolute/path/closed_loop_velocity.yaml
```

launchは制御ノードのみを起動する。ZED_WRAPPERとモータードライバーは別に起動する。既存の `rover_velocity` は停止し、`/rov/motors` の発行元を本ノードだけにする。

## 実機に合わせる設定

`config/closed_loop_velocity.yaml` の `body_frame` と、odomの `child_frame_id` から車体へのTFを確認する。両フレームが一致するときは変換しない。異なる場合は計測時刻でTFを参照し、回転と取付位置による速度補正を行う。TFがない場合は出力ゼロになる。

`motor_command_limit: 0.0` は駆動抑止。実機のRoboteq設定で確認した指令上限を正の値で設定する。これはRPM上限と断定できない。既存の固定±15クリップは使わない。4輪の最大絶対値が設定上限を超えたら全輪を同じ比率で縮小する。

暫定値はKp=0.1（各軸）、30 Hz、移動平均5サンプル、目標・odomタイムアウト0.5秒。実機で調整する。出力ゲイン1000/1000/30と配分係数375/63.5は既存値を踏襲し、単位変換としては扱わない。I項・D項はない。

`velocity_filter_enabled: false` または `velocity_filter_window_size: 1` で平滑化なし。全パラメーターは起動時設定であり、変更後は再起動する。

## 停止と復帰

起動時、指令途絶、odom途絶・古い計測・未来時刻・非有限値・TF異常ではゼロ出力。odom異常時は履歴を破棄する。異常からは有効な新規odomと有効期限内の目標指令がそろえば自動復帰する。停止指令は3軸すべてが厳密にゼロの場合で、P補正を掛けず出力ゼロにする。

本ノードは `deadman=True` を発行しない。既存ドライバーのdeadman管理は別系統のまま使用する。ノード強制終了、プロセス障害、通信断に対する停止は、本ノードからのゼロ送信では保証できない。実機側またはドライバー側の受信ウォッチドッグは別途確認・用意する。既存 `cmd_roboteq` に受信タイムアウト処理を追加する変更は含まない。

ZEDの追跡状態トピックによる停止判定は未実装。現在はodomの鮮度・数値・TFを検証する。新しい時刻で配信されるが追跡品質の悪い速度を、その検証だけでは識別できない。

## 診断とテスト

`/closed_loop_velocity/diagnostics`（`std_msgs/msg/String`、JSON）に目標、変換後の生速度、平滑化速度、誤差、P補正、制限前後の出力、縮小比率、サンプル数、履歴の時間幅、計測経過時間、停止理由を出す。

```bash
ros2 topic echo /closed_loop_velocity/diagnostics
PYTHONPATH=src/locomotion_closed_loop:$PYTHONPATH ROS_DOMAIN_ID=173 python3 -m pytest -q src/locomotion_closed_loop/test
```

ROS_DOMAIN_ID=173はテスト用の分離例。テストはドライバーを起動しない。
