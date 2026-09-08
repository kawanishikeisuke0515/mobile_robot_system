# Mode Manager 仕様書

Version: 0.2
Date: 2026-09-08
Status: 初版実装済み・模擬入力で検証、実機未検証

## 1. 目的

`mode_manager` はUWB・ZEDによる位置姿勢制御とVisionによるドッキング制御を切り替え、機体へ渡す速度指令を一元管理する。

UWBでマーカーを正面に捉えるための引き渡し位置・姿勢まで移動し、対象マーカーの安定検出を確認してVisionへ切り替える。Vision中に対象マーカーを見失った場合は一旦停止してUWBへ戻し、同じ引き渡し位置・姿勢へ復帰してからVisionを最初から再試行する。

初版ではManager内に状態機械と速度選択を実装する。状態機械はROS非依存の `mode_manager/core.py`、ROS接続は `mode_manager/node.py`、Controller側の共通接続処理は `mode_manager/controller_link.py` に分ける。

## 2. 構成と責務

```text
UWB位置・ZED姿勢 → UWB Controller ─ /uwb/control_output ─────┐
                                                           ├─ Mode Manager → /rov_cmd_vel → 駆動系
ArUco検出 → Vision Controller ─ /vision/control_output ──────┘
                     ↑                          │
                     └── 各 /control_request ──┘
```

| 要素 | 責務 |
| --- | --- |
| UWB Controller | 引き渡しposeへの速度指令、入力有効性、到達判定 |
| Vision Controller | 指定IDの追跡、ドッキング制御、完了判定 |
| Mode Manager | 状態遷移、Controllerの有効化・リセット、速度指令の選択 |
| 駆動系 | `/rov_cmd_vel` を受けて駆動する |

統合時は両Controllerを `managed_mode=true` とし、`/rov_cmd_vel` のpublisherをManagerだけにする。非選択中も観測と状態報告を続ける。Visionの制御状態は非選択中に進めない。

初期案の個別Twist topicとstatus topicは、実装では `ControllerOutput` に統合した。状態・速度・実行IDを一緒に届けることで、状態と速度の取り違えや切り替え前の指令の再利用を防ぐ。

## 3. 状態遷移

| 状態 | 出力・動作 | 遷移 |
| --- | --- | --- |
| `IDLE` | ゼロ速度。起動時の状態 | start serviceで `UWB_APPROACH` |
| `UWB_APPROACH` | 有効なUWB指令 | 引き渡しposeに到達したら `VISION_WAIT` |
| `VISION_WAIT` | ゼロ速度 | 到達条件と安定検出が揃ったら `VISION_DOCKING` |
| `VISION_DOCKING` | 有効なVision指令 | 検出喪失で `UWB_RECOVERY`、完了で `DONE` |
| `UWB_RECOVERY` | 一旦停止後、同じ引き渡しposeへUWBで復帰 | 再到達したら `VISION_WAIT` |
| `DONE` | ゼロ速度を維持 | start serviceで新しい実行 |
| `FAULT` | ゼロ速度を維持 | 原因解消後、start serviceで新しい実行 |

```text
IDLE ─ start → UWB_APPROACH ─ 到達 → VISION_WAIT ─ 安定検出 → VISION_DOCKING ─ 完了 → DONE
                                    ↑                          │
                                    └── 到達 ─ UWB_RECOVERY ←──┘ 検出喪失
```

全状態でstop serviceを受け付け、`IDLE` に戻してゼロ速度を出力する。実行中のstartは拒否する。動作中の必要なController出力の途絶、非有限な速度指令、時刻の巻き戻りは `FAULT` とする。

### 3.1 引き渡し条件

UWB Controllerが、最新かつ有効なUWB位置・ZED姿勢で、目標x/y/yawの許容誤差をすべて満たした場合に `target_reached=true` とする。Managerは速度ゼロから到達を推測しない。

`VISION_WAIT` でもUWBの到達状態を再確認する。位置・姿勢が有効で到達条件を外れた場合は `UWB_RECOVERY` へ戻す。入力が無効なら停止して待機し、古い到達結果ではVisionを開始しない。

### 3.2 安定検出

対象IDの `tracking_valid=true` が `stable_detection_time` 以上継続し、その区間で `min_detections` 以上の新しい検出を確認する。状態の定期publishだけでは検出回数は増えない。

判定は `VISION_WAIT` に入ってから開始する。false、通信timeout、検出カウンタの巻き戻りで継続判定をリセットする。`tracking_valid` は検出timeoutまでの短い欠落を許容するため、「全画像で連続検出」を意味しない。

### 3.3 マーカー喪失と復帰

対象マーカーの最後の有効な受信からVisionの `detection_timeout` を超えると `tracking_valid=false` となる。Managerは一旦ゼロ速度を出し、`UWB_RECOVERY` へ切り替える。

復帰途中で対象マーカーが見えても直接Visionへ切り替えない。引き渡しposeに再到達してから安定検出を確認する。UWB位置や姿勢が無効な間は復帰移動を行わない。

Visionの通信途絶は明示的な検出喪失と区別し、`FAULT` で停止する。同じ出力で完了と検出喪失が報告された場合は、喪失を優先する。`DONE` に入った後の未検出では復帰を開始しない。

### 3.4 待機・再試行

`VISION_WAIT` が `vision_wait_timeout` を超えたら `FAULT` とする。初版では自動復帰の回数上限は設けない。UWB入力が無効な場合は停止待機を継続する。異常後に勝手に再開せず、startを必要とする。

## 4. インターフェース

### 4.1 Topic

| Topic | 型 | 方向・用途 |
| --- | --- | --- |
| `/uwb/control_request` | `mode_manager_interfaces/msg/ControlRequest` | Manager → UWB。有効化と実行ID |
| `/vision/control_request` | 同上 | Manager → Vision。有効化と実行ID |
| `/uwb/control_output` | `mode_manager_interfaces/msg/ControllerOutput` | UWB → Manager。状態と速度 |
| `/vision/control_output` | 同上 | Vision → Manager。追跡・完了状態と速度 |
| `/rov_cmd_vel` | `geometry_msgs/msg/Twist` | Manager → 駆動系。選択指令またはゼロ速度 |
| `/mode_manager/state` | `std_msgs/msg/String` | 現在の状態名を定周期publish |

QoSはreliable・volatile・depth 1。必要ならROSのremappingでtopicを変更する。

### 4.2 メッセージ

`ControlRequest`:

| Field | 型 | 意味 |
| --- | --- | --- |
| `stamp` | `builtin_interfaces/Time` | 生成時刻 |
| `session_id` | `string` | Managerが状態遷移ごとに発行するUUID |
| `enabled` | `bool` | このControllerを制御用に有効化するか |

`ControllerOutput`:

| Field | 型 | 意味 |
| --- | --- | --- |
| `stamp` | `builtin_interfaces/Time` | 状態と速度の生成時刻 |
| `session_id` | `string` | Controllerが受け付けた実行ID |
| `active` | `bool` | 有効化され、Managerの定期要求がtimeoutしていない |
| `inputs_valid` | `bool` | 制御用入力が有効かつ最新 |
| `target_reached` | `bool` | UWBの引き渡しpose到達判定 |
| `tracking_valid` | `bool` | Visionの対象IDの検出がtimeoutしていない |
| `docking_complete` | `bool` | 現在のVision実行で完了条件を満たした |
| `target_marker_id` | `int32` | Visionの対象マーカーID。UWBでは-1 |
| `detection_sequence` | `uint64` | 対象IDの有効な検出受信ごとに増加するカウンタ |
| `command` | `geometry_msgs/Twist` | 今回生成した速度。非選択・入力無効ならゼロ |

`command` の利用成分は `linear.x`, `linear.y`, `angular.z`。Managerはその他の成分をゼロにする。

Visionの `inputs_valid` と `tracking_valid` は別に扱う。非選択中やリセット直後でも観測の追跡状態は報告できるが、制御用平均に新しいサンプルが揃うまでは速度を採用しない。

### 4.3 Service

| Service | 型 | 動作 |
| --- | --- | --- |
| `/mode_manager/start` | `std_srvs/srv/Trigger` | `IDLE/DONE/FAULT` から新しいUWB移動を開始 |
| `/mode_manager/stop` | `std_srvs/srv/Trigger` | 全状態から停止し `IDLE` へ移行 |

## 5. 切り替えとtimeout

1. 状態遷移時に新しい実行IDを発行し、保存していた出力を破棄する。
2. 遷移周期ではゼロ速度をpublishする。
3. 各Controllerへ新しい実行IDと有効化状態を定期送信する。
4. Controllerは新しい実行IDまたは有効化状態の変化を受け取ると制御状態をリセットする。
5. Managerは現在の実行IDに対応する、最新かつ有効な選択側出力だけを採用する。

UWBのリセットでは到達状態と入力受信時刻をクリアし、新しい位置・姿勢入力を待つ。Visionのリセットでは `PRE_DOCKING` に戻し、完了状態と制御用位置平均をクリアする。観測用の最終検出時刻と検出カウンタは保持する。

出力と要求の時刻が未来・期限切れ・重複・逆順の場合は採用しない。受信後の鮮度監視には単調時計を使用し、出力については生成から受信までの経過時間も差し引く。ノード間で同じROS時刻系を使うこと。

Managerは状態遷移後に最大 `output_timeout` の応答猶予を設け、その間は新しい指令がなければゼロ速度とする。その後、必要な出力がなければ `FAULT` にする。UWB移動中はUWB、Vision中はVision、Vision待機中は両方の出力を監視する。

ControllerはManagerからの要求が `manager_timeout` を超えたら無効化し、制御をリセットする。ゼロ速度の送信は実際の機体停止確認ではない。

## 6. パラメータ

| Manager parameter | 初期値 | 制約・用途 |
| --- | --- | --- |
| `target_marker_id` | 0 | 0以上。Vision側と一致させる |
| `control_rate` | 20.0 Hz | 有限・正。指令・状態・制御要求の周期 |
| `output_timeout` | 0.5 s | 有限・正。Controller出力の有効期間と応答猶予 |
| `stable_detection_time` | 0.5 s | 有限・正。安定検出期間 |
| `min_detections` | 3 | 2以上。期間中に必要な新規検出回数 |
| `vision_wait_timeout` | 30.0 s | 有限・正。Vision待機上限 |

両Controllerに `managed_mode`（初期値false）と `manager_timeout`（0.5秒、有限・正）を追加した。Visionには `target_marker_id`（単体時の初期値-1で全ID、管理時は0以上必須）と `cmd_vel_topic`（単体時の初期値 `/rov_cmd_vel`）を追加した。

Visionの既存 `detection_timeout` 初期値は0.5秒。対象外ID、非有限値、zが正でない検出は採用せず、検出時刻・検出カウンタも更新しない。

数値は初版の設定であり、実機で確定した値ではない。出力周期と通信遅延に対してtimeoutに余裕を持たせる。

## 7. ビルドと起動

ZEDカメラ・UWB・headingまでまとめて起動する場合は、`docking_bringup/docking.launch.py` を使う。[全体bringupの起動手順](../../docking_bringup/doc/docking_bringup_ja.md) に設定ファイルと部分起動オプションを記載する。以下の `managed_docking.launch.py` はController・Manager・検出器のみを起動する構成。

```bash
cd mobile_robot_system
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to docking_bringup
source install/setup.bash
```

統合launchはManager、両Controller、ArUco検出ノードを起動する。引き渡し位置・姿勢とIDは必須引数。以下の数値は例であり、使用する環境に合わせて変更する。

```bash
ros2 launch docking_bringup managed_docking.launch.py \
  handoff_x:=0.0 handoff_y:=1.0 handoff_yaw:=0.0 target_marker_id:=7
```

UWB位置、ZED heading、画像とCameraInfoを供給するノード、および駆動系は別途起動する。既存の単体Controllerを起動するlaunchと同時に使わず、最終速度topicのpublisherを一つにする。

```bash
ros2 service call /mode_manager/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mode_manager/state
ros2 service call /mode_manager/stop std_srvs/srv/Trigger '{}'
```

Managerだけを起動する場合:

```bash
ros2 launch mode_manager mode_manager.launch.py
```

このlaunchは `params_file` でYAMLを指定できる。単独ではControllerを起動しないため、別途対応するControllerが必要。

## 8. 検証

ROS非依存の状態機械テストと、実際のManager・両Controllerに模擬センサを入力するROSテストを用意した。カメラ・駆動ノードは起動しない。

```bash
python -m pytest -q src/mode_manager/test
```

ROS統合テストはビルド済みworkspaceをsourceして実行する。インターフェースやControllerが使えない環境では該当テストをskipする。

| 確認項目 | 期待動作 |
| --- | --- |
| 通常の到達と安定検出 | UWBからVisionへ切り替わる |
| 状態heartbeatのみ、対象外ID | 安定検出条件を満たさない |
| Visionの検出喪失 | ゼロ速度を出し、UWB復帰する |
| 復帰途中の再検出 | 到達まではUWBを継続する |
| UWB入力無効 | 復帰中も停止する |
| 完了後の未検出 | `DONE` を維持する |
| 選択外・旧実行IDの指令 | 採用しない |
| 出力timeout、非有限指令 | `FAULT` で停止する |
| 待機中のpose逸脱、待機timeout | UWB復帰、または `FAULT` |
| 統合構成の最終速度topic | Managerのみがpublishする |

## 9. 制限と今後の確認

- 実機の復帰経路、制御ゲイン、引き渡しpose、timeout設定は未検証。UWB復帰は既存の位置姿勢制御であり、経路計画や障害物回避を追加していない。
- ArUcoメッセージに画像取得時刻がないため、Visionの検出鮮度は受信時刻ベース。画像の遅延と新しい画像を区別する拡張は未実装。
- 未検出とカメラ入力の途絶は、どちらも検出timeoutとして扱う。Vision Controllerそのものの通信途絶は別に `FAULT` とする。
- Managerプロセスが強制終了した場合、Controllerの無効化だけでは駆動系に最終ゼロ指令が届く保証はない。駆動側の速度指令watchdogは本変更に含めておらず、実機運用前に対応・確認が必要。
- 自動復帰回数上限、機体の停止確認、構造化した異常理由topicは今後の検討事項。

関連する単体制御仕様は [UWB仕様](../../uwb_position_zed_pose_ctrl/doc/uwb_position_zed_pose_ctrl_requirements_ja.md) と [Vision仕様](../../vision_dist_ctrl/docs/requirements_ja.md) を参照する。
