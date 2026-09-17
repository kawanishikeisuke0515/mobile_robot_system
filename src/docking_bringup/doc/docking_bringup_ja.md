# UWB・ZED・Vision ドッキングBringup

## 起動する構成

`docking.launch.py` は以下をまとめて起動する。起動時は `IDLE` で準備を待ち、両Controllerの応答と有効なUWB位置・ZED姿勢が揃うと自動で移動を開始する。対象マーカーの検出はUWB移動開始の条件に含めない。

| 機能 | 起動対象 |
| --- | --- |
| ZED画像・CameraInfo・磁気データ | `zed_wrapper_data_hub/zed_data_hub.launch.py` → 公式ZED wrapper |
| UWB距離取得・位置推定 | `uwb_distance_publisher`, `uwb_position_publisher` |
| ZED姿勢推定 | `zed_heading_publisher` |
| マーカー検出 | `aruco_distance_publisher` |
| UWB位置姿勢制御 | `uwb_position_zed_pose_ctrl`（managed mode） |
| Visionドッキング制御 | `vision_distance_controller`（managed mode） |
| モード・速度選択 | `mode_manager` |
| 駆動系（選択式） | `rover_velocity`, `cmd_roboteq` |

カメラを起動するのはdata hubだけ。Controllerは既存の `managed_docking.launch.py` をincludeして起動し、`/rov_cmd_vel` はManagerのみがpublishする。

## ビルドと起動

ZED SDK、公式 `zed_wrapper` / `zed_description` および各ノードの実行依存が利用できる環境で実行する。

```bash
cd mobile_robot_system
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to docking_bringup
source install/setup.bash

ros2 launch docking_bringup docking.launch.py \
  handoff_x:=0.0 handoff_y:=1.0 handoff_yaw:=0.0 target_marker_id:=7
```

数値は例。引き渡し位置x/y [m]、姿勢 [rad]、対象マーカーIDは必須引数であり、環境に合わせて指定する。ZEDのnamespaceは `/zed2i/zed_node` に合わせている。

駆動ノードも標準で起動する（`start_locomotion:=true`）。駆動系なしで確認する場合は `start_locomotion:=false` を指定する。Manager強制終了時に備える駆動側の速度指令watchdogは別途対応が必要であり、本bringupでは追加していない。

```bash
ros2 topic echo /mode_manager/state
ros2 service call /mode_manager/stop std_srvs/srv/Trigger '{}'
```

動作はUWBで引き渡しposeへ移動→安定検出→Visionドッキング。検出喪失時はUWBで引き渡しposeへ復帰し、再試行する。詳しくは [Mode Manager仕様](../../mode_manager/doc/mode_manager_spec_ja.md) を参照する。

## 設定ファイル

既存のUWBアンカー位置、シリアル設定、ZED磁気校正、UWB制御ゲインをYAMLから読み込む。コピーしたYAMLを引数で差し替えられる。

| 引数 | 初期設定 |
| --- | --- |
| `uwb_distance_config` | `uwb_position_publisher/config/uwb_distance_publisher.yaml`。シリアルポート等 |
| `uwb_position_config` | `uwb_position_publisher/config/uwb_position_publisher.yaml`。アンカー座標等 |
| `heading_config` | `zed_heading_publisher/config/zed_heading_publisher.yaml`。磁気校正、yaw基準等 |
| `zed_config` | `zed_wrapper_data_hub/config/zed2i_data_hub.yaml`。ZED wrapper設定 |
| `manager_config` | `mode_manager/config/mode_manager.yaml`。timeout等 |
| `uwb_controller_config` | `uwb_position_zed_pose_ctrl/config/uwb_position_zed_pose_ctrl.yaml`。ゲイン・許容誤差等 |
| `vision_config` | 本packageの `config/vision.yaml`。Vision制御設定 |
| `aruco_config` | 本packageの `config/aruco.yaml`。画像topic、マーカー寸法等 |

```bash
ros2 launch docking_bringup docking.launch.py \
  handoff_x:=0.0 handoff_y:=1.0 handoff_yaw:=0.0 target_marker_id:=7 \
  heading_config:=/absolute/path/heading.yaml \
  uwb_distance_config:=/absolute/path/uwb_serial.yaml
```

YAMLよりlaunch引数の引き渡しpose・対象ID・`vision_target_z`（初期値1.3 m）・`docking_distance`（1.0 m）を優先する。両Controllerのmanaged modeはlaunchでtrueに固定する。対象IDはManagerとVisionに同じ値を渡す。

## 部分起動

| 引数 | 初期値 | falseの場合 |
| --- | --- | --- |
| `start_zed` | true | カメラを起動しない。外部の画像・CameraInfo・磁気topicを利用 |
| `start_uwb` | true | UWB距離取得と位置推定を起動しない。外部の `/uwb/position` を利用 |
| `start_heading` | true | headingを起動しない。外部の `/zed/heading` を利用 |
| `start_locomotion` | true | 駆動系を起動しない |

`zed_serial_number`（初期値0）で使用するZEDを指定できる。外部ノードを使う場合は対応する起動オプションをfalseにし、同じノードを重複起動しない。従来の単体制御用 `uwb_zed_docking.launch.py` / `vision_docking.launch.py` は本launchと併用しない。

このlaunchは実センサの時刻系を前提とする。ROS bagのsim time利用を含む全ノード共通の時刻切り替えは未対応。

## 検証範囲

パッケージビルド、launch引数の展開、センサ・駆動を無効にした4ノードの初期化・起動を確認済み。検証環境のUDP制限により、別プロセス間のROS通信はこの起動確認では未検証。また、この環境では公式 `zed_wrapper` / `zed_description` が利用できず、カメラを含む全体起動は未検証。実カメラ、UWBシリアル、モーター接続を伴う動作確認は別途実施する。

## 自動開始と停止

全体launchとmanaged launchは `auto_start:=true` が初期値。準備待ち中は速度ゼロで待機し、起動時の応答遅延だけではFAULTにしない。開始時に実行IDを更新し、リセット後の新しい制御指令を待つ。開始後の通信timeout監視は従来どおり。

stop serviceは準備待ち中の自動開始も取り消す。停止・完了・FAULT後に自動で再開せず、再開する場合は `/mode_manager/start` を呼ぶ。手動開始にしたい場合はlaunchに `auto_start:=false` を指定する。Manager単体起動時のパラメータ初期値はfalse。


## 実験ログ

`docking_logger` を標準で起動し、速度・モーター指令、UWB位置、ZED heading、OptiTrack、Vision、制御状態を記録する。

```bash
ros2 launch docking_bringup docking.launch.py \
  handoff_x:=0.0 handoff_y:=1.0 handoff_yaw:=0.0 target_marker_id:=7 \
  log_dir:=$HOME/docking_logs experiment_name:=trial_01
```

Excelで比較するファイルは、保存先の実験ディレクトリ内の **`timeline.csv`**。
20 Hzで各トピックの最新値を同じ行へまとめる。`elapsed_sec` をグラフの横軸に使う。
計測自体は同時ではなく、各列の `age_sec` / `stale` / `received` で鮮度を確認する。
受信ごとの個別CSVと設定スナップショット `metadata.yaml` も保存する。

- `enable_logging:=false`：ロガーを起動しない。
- `logger_config:=/absolute/path/logger.yaml`：記録周期、トピック、QoSなどを変更。
- `experiment_note:="実験メモ"`：metadataへメモを保存。
- OptiTrack初期値：`/vrpn_mocap/RigidBody_1/pose`。
- 起動時の保存先通知を確認する。記録失敗時もロボットの制御は継続する。
- 初回指令から確実に記録したい場合は `auto_start:=false` で起動し、ロガー準備後にstart serviceを呼ぶ。

ロガーだけ途中から起動する場合（bringup側のロガーとの重複起動を避ける）：

```bash
ros2 launch docking_logger docking_logger.launch.py \
  target_marker_id:=7 log_dir:=$HOME/docking_logs experiment_name:=trial_01
```

詳細は [Docking Logger仕様書](../../docking_logger/doc/docking_logger_spec_ja.md) を参照。

## ロボット中心姿勢の制御入力

UWB位置とheadingは `uwb_robot_pose_publisher` で中心姿勢 `/uwb/robot_pose` に変換し、UWB制御へ渡す。目標x/yはロボット中心の座標で、`handoff_yaw=0` は従来どおりWorldの+Y方向。`robot_pose_config` で中心補正ノードのYAMLを指定する。managed-only launchも中心補正ノードを起動するため、外部で同名ノードを重複起動しない。
