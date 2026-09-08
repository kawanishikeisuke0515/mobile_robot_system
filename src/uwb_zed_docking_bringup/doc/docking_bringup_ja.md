# UWB・ZED・Vision ドッキングBringup

## 起動する構成

`docking.launch.py` は以下をまとめて起動する。起動時のManagerは `IDLE` であり、移動はstart serviceで開始する。

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
colcon build --packages-up-to uwb_zed_docking_bringup
source install/setup.bash

ros2 launch uwb_zed_docking_bringup docking.launch.py \
  handoff_x:=0.0 handoff_y:=1.0 handoff_yaw:=0.0 target_marker_id:=7
```

数値は例。引き渡し位置x/y [m]、姿勢 [rad]、対象マーカーIDは必須引数であり、環境に合わせて指定する。ZEDのnamespaceは `/zed2i/zed_node` に合わせている。

駆動ノードも起動する場合は `start_locomotion:=true` を追加する。初期値はfalse。Manager強制終了時に備える駆動側の速度指令watchdogは別途対応が必要であり、本bringupでは追加していない。

```bash
ros2 topic echo /mode_manager/state
ros2 service call /mode_manager/start std_srvs/srv/Trigger '{}'
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
ros2 launch uwb_zed_docking_bringup docking.launch.py \
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
| `start_locomotion` | false | 駆動系を起動しない |

`zed_serial_number`（初期値0）で使用するZEDを指定できる。外部ノードを使う場合は対応する起動オプションをfalseにし、同じノードを重複起動しない。従来の単体制御用 `uwb_zed_docking.launch.py` / `vision_docking.launch.py` は本launchと併用しない。

このlaunchは実センサの時刻系を前提とする。ROS bagのsim time利用を含む全ノード共通の時刻切り替えは未対応。

## 検証範囲

パッケージビルド、launch引数の展開、センサ・駆動を無効にした4ノードの初期化・起動を確認済み。検証環境のUDP制限により、別プロセス間のROS通信はこの起動確認では未検証。また、この環境では公式 `zed_wrapper` / `zed_description` が利用できず、カメラを含む全体起動は未検証。実カメラ、UWBシリアル、モーター接続を伴う動作確認は別途実施する。
