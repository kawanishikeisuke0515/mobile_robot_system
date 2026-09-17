# Docking Logger 仕様書

## 1. 目的と状態

`docking_bringup` 実行中のモーター指令、ZED heading、UWB位置、OptiTrack姿勢、Vision位置情報を記録し、指令と実際の動き、制御モードの切り替えを後から比較できるようにする。

本書はスキーマバージョン3の実装仕様。専用の `docking_logger` パッケージ・ノードで記録する。シリアル送信内容は記録対象外とし、OptiTrackの初期トピックは既存の `uwb_optitrack_logger` に合わせる。以下の初期値・詳細設計を初版の実装基準とする。

## 2. 配置と責務

```text
mobile_robot_system/src/docking_logger/
├── docking_logger/
│   ├── __init__.py
│   └── docking_logger.py
├── config/docking_logger.yaml
├── launch/docking_logger.launch.py
├── doc/docking_logger_spec_ja.md
├── resource/docking_logger
├── package.xml
├── setup.py
└── setup.cfg
```

- ノード名・実行ファイル名は `docking_logger`。
- `docking_bringup` は起動の取りまとめ、ロガーは購読・保存を担当する。
- ロガー単体で起動し、既に動作しているシステムの記録を途中から開始できる。
- 既存の制御計算・モータードライバの処理は変更しない。UWB制御ノードには誤差のpublishを追加する。
- 速度指令のpublishや開始・停止serviceの呼び出しを行わない。
- ファイル書き込みはロガー内の専用スレッドで処理する。

## 3. 記録対象

トピック名はすべてパラメータで変更可能とする。

| 対象 | 初期トピック | 型 | 記録する内容 |
| --- | --- | --- | --- |
| 最終速度指令 | `/rov_cmd_vel` | `geometry_msgs/msg/Twist` | linear x/y/z、angular x/y/z |
| モーター指令 | `/rov/motors` | `std_msgs/msg/Float32MultiArray` | 4要素の指令値、受信要素数 |
| 駆動許可 | `/deadman` | `std_msgs/msg/Bool` | data |
| UWBロボット中心位置・姿勢 | `/uwb/robot_pose` | `geometry_msgs/msg/PoseStamped` | position x/y/z、quaternion x/y/z/w、header |
| UWB位置 | `/uwb/position` | `uwb_interfaces/msg/UwbPosition` | x_m、y_m、valid、device_time_ms、header |
| UWB制御誤差 | `/uwb/control_error` | `uwb_interfaces/msg/UwbControlError` | 制御計算で使用した位置・姿勢誤差、距離、session_id、active、inputs_valid、header |
| ZED heading | `/zed/heading` | `zed_interfaces/msg/ZedHeading` | raw_x/z、corrected_x/z、magnetic_heading_deg、robot_yaw_deg/rad、valid、header |
| OptiTrack | `/vrpn_mocap/RigidBody_1/pose` | `geometry_msgs/msg/PoseStamped` | position x/y/z、quaternion x/y/z/w、header |
| Vision | `/aruco/distance` | `aruco_interfaces/msg/ArucoDistance` | id、x/y/z、distance、theta、yaw、center_u/v、normalized_center_error |
| Manager状態 | `/mode_manager/state` | `std_msgs/msg/String` | 状態文字列 |
| UWB制御出力 | `/uwb/control_output` | `mode_manager_interfaces/msg/ControllerOutput` | 全フィールド |
| Vision制御出力 | `/vision/control_output` | `mode_manager_interfaces/msg/ControllerOutput` | 全フィールド |

`ControllerOutput` は stamp、session_id、active、inputs_valid、target_reached、tracking_valid、docking_complete、target_marker_id、detection_sequence、command（Twist全要素）を保存する。

### 3.1 モーター指令の意味

現在の指令経路は次のとおり。

```text
/rov_cmd_vel → rover_velocity → /rov/motors → cmd_roboteq → モーターコントローラ
```

`/rov/motors` はドライバ入力値であり、ドライバ内の整数化・符号反転・停止処理より前の値である。シリアル送信成功や実回転速度を示すものではない。単位を実測RPMと断定しない。

配列は `motor_0` ～ `motor_3` として保存する。現在のドライバでは0/1が前側コントローラのチャンネル1/2、2/3が後側のチャンネル1/2に対応する。左右の車輪名は推測で付けない。要素数が4以外の場合も受信を記録し、`shape_valid=false` とする。元の全配列を `data_json` 列に残す。

### 3.2 Visionの扱い

- 全マーカーの受信を `vision.csv` に保存する。
- 比較用一覧には設定された `target_marker_id` の最新値だけを採用する。
- 別IDの受信では対象マーカーの鮮度を更新しない。
- 対象マーカーが見えなくなった場合は最終値の経過時間で判定する。ゼロ位置に置き換えない。

### 3.3 UWBのP制御誤差

- Controllerの同じ制御計算結果を専用トピックに毎制御周期publishし、ロガーは再計算せず保存する。単体・managed両モードで利用できる。
- `raw_error_world_x/y`：目標位置 − UWB位置［m］。許容範囲を適用する前の符号付き誤差。
- `distance_error_m`：上記2成分のノルム（目標までの平面距離［m］）。
- `error_world_x/y`：各軸の許容範囲内を0にした世界座標誤差［m］。
- `error_body_x/y`：世界座標誤差を現在yawで機体座標に変換した、`kp_x/y` を掛ける直前の誤差［m］。現制御の変換は `body_x = -sin(yaw)*world_x + cos(yaw)*world_y`、`body_y = cos(yaw)*world_x + sin(yaw)*world_y`。
- `yaw_error`：目標yaw − 現在yawを `[-π, π)` に折り返した値［rad］。
- 速度制限、最低速度適用の前の誤差を記録するため、誤差が非ゼロでも指令が0の場合がある。
- 入力未受信・無効・timeout時もpublishし、`inputs_valid=false`、全誤差・距離を `nan` とする。timelineの `value_valid` はinputs_validと全数値の有限性で判定する。
- 制御が非activeでも入力が有効なら誤差を記録する。`active` と `session_id` を併記し、動作中かどうかを区別する。
- header.stampは制御計算時刻でありセンサ計測時刻ではない。世界座標・機体座標が混在するためheader.frame_idは空欄。制御出力トピックとの厳密な同時受信は保証しない。
- Controller側のトピックパラメータは `control_error_topic`、ロガー側は `uwb_control_error_topic`。両方の初期値は `/uwb/control_error`。
- 追加CSVは `uwb_control_error.csv`。timelineでは `uwb_control_error_` 接頭辞で全列と鮮度を保存する。metadataの `schema_version` は3。
- 利用前に `uwb_interfaces`、UWB Controller、`docking_logger` を再ビルドし、同じ新しいinterface環境をsourceする。

### 3.4 UWBロボット中心位置・姿勢

`uwb_robot_pose_topic`（初期値 `/uwb/robot_pose`、`geometry_msgs/msg/PoseStamped`）を購読し、タグ取り付けオフセット補正後のロボット中心位置・姿勢を記録する。変換は `uwb_robot_pose_publisher` が行い、ロガーでは再計算しない。

`uwb_robot_pose.csv` に position_x/y/z［m］、qx/qy/qz/qw、元のタイムスタンプと frame_id、受信時刻を保存する。`timeline.csv` にも `uwb_robot_pose_` 接頭辞で値と received/age_sec/stale/value_valid を保存する。

value_validは全位置・姿勢成分が有限でQuaternionが全ゼロでないことを表す。PoseStampedにvalidフラグはないため、未受信と受信停止はreceivedとstaleで確認する。timelineは各ストリームの最新値であり、厳密な時刻同期は行わない。

## 4. 保存形式

```text
<log_dir>/<UTC開始日時>_<experiment_name>_<一意な識別子>/
├── metadata.yaml
├── cmd_vel.csv
├── motor_commands.csv
├── deadman.csv
├── uwb.csv
├── uwb_robot_pose.csv
├── uwb_control_error.csv
├── zed_heading.csv
├── optitrack.csv
├── vision.csv
├── control_state.csv
├── uwb_control_output.csv
├── vision_control_output.csv
└── timeline.csv
```

- 起動ごとに保存ディレクトリを作成し、既存ログを上書きしない。
- UTF-8、ヘッダー付きCSVとする。文字列は標準CSVの引用規則に従う。
- boolは `true` / `false`、未受信・該当しない項目は空欄、非有限値は `nan` / `inf` / `-inf` として保存する。
- 時刻のナノ秒値は整数で保存する。浮動小数点値は不要な桁丸めを行わない。

### 4.1 個別CSV

各購読callbackで受信イベントを作り、受信ごとに1行記録する。一定周期で間引かない。ただしROS通信や書き込みキューで発生した欠落まで完全保存を保証するものではない。

共通列は以下とする。

| 列 | 意味 |
| --- | --- |
| `sample_seq` | ロガーが対象ストリームに付ける受信連番 |
| `recv_ros_time_ns` | callbackで取得したROS受信時刻 |
| `elapsed_sec` | 単調時計によるロガー開始からの経過時間 |
| `source_stamp_ns` | header.stampまたはstamp。時刻フィールドがなければ空欄 |
| `frame_id` | 元メッセージのheader.frame_id。なければ空欄 |

この共通列に、3章の対象ごとの列を追加する。元のvalidフラグや時刻は加工せず保存する。

### 4.2 比較用 `timeline.csv`

- 初期値20 Hzで、その時点の各ストリームの最新値を1行にまとめる。
- 行に `row_seq`、`row_ros_time_ns`、`elapsed_sec` を持たせる。
- 各ストリームに接頭辞を付け、個別CSVの値と共通列を展開する。
- 各ストリームに `received`、`age_sec`、`stale`、`value_valid` を追加する。
- `age_sec` は単調時計で測った最終受信からの時間。初回未受信時は空欄。
- `stale` は `age_sec > stale_timeout_sec` でtrue。未受信時は空欄。
- `value_valid` はUWB/ZEDでは元のvalidと必要な数値の有限性、モーターでは要素数と有限性、OptiTrackでは有限値と非ゼロquaternion、Visionでは位置・角度の有限性で判定する。
- 有効性が定義されない状態文字列等では `value_valid` を空欄にする。制御出力の各フラグは独立して保存する。
- 古い値も残すが、鮮度列で識別可能とする。未受信の値は空欄。
- 厳密な時刻同期、補間、座標変換は行わない。同じ行でも各センサの計測時刻は異なる。

### 4.3 実験設定 `metadata.yaml`

次を保存する。

- スキーマバージョン、記録ID、実験名、実験メモ、UTC開始日時。
- ロガーパラメータ、実際の購読トピック・型・QoS、時刻の扱い。
- 目標handoff x/y/yaw、target_marker_id、vision_target_z、docking_distance。
- bringupが使用した設定ファイルのパスと内容のスナップショット、launch引数による上書き値。
- 座標系・単位と、判明している取り付け位置や変換情報。
- 正常終了時に終了日時、各CSVの保存行数、キュー破棄数、記録エラー、終了状態を追記する。

bringup起動時は設定ファイルと上書き値をロガーへ渡す。単体起動では外部ノードの実効設定を自動取得したと見なさず、提供されない設定は不明として記録する。設定値の動的変更の追跡は初版の対象外。

初期状態は `recording` とし、正常終了時に `completed` に更新する。異常終了で更新されなかった場合に正常完了と誤認しない構成にする。metadata更新は一時ファイルからの置換で行う。

## 5. 時刻と座標系

- ROS受信時刻、メッセージ内時刻、単調時計による経過時間を区別する。
- 現在のTwist、モーター指令、deadman、Vision、Manager状態には計測時刻がない。受信時刻を計測時刻として表記しない。
- source stampが0の場合も元の0を保存する。別マシンの時計の同期はロガーが保証しない。
- 初版は現在のbringupと同じ実時間運用を対象とする。sim time／bag再生での運用は対象外。
- UWBはアンカー定義に基づくx/y［m］、ZED headingは設定されたyaw基準を維持する。
- OptiTrackは元のposition［m］とquaternion、frame_idを保存する。
- Visionはカメラから見たマーカーの相対位置［m］。OpenCVの右x・下y・前zであり、ロボットの世界座標位置ではない。theta/yawは現実装のradを保存する。
- UWB・OptiTrack・Vision間の座標合わせや、センサ取り付け位置の補正、センサ間の位置誤差の算出は後処理で行う。変換未設定の座標を直接差し引かない。

## 6. 起動・終了とパラメータ

### 6.1 主な初期値

| パラメータ | 初期値 | 意味 |
| --- | --- | --- |
| `enable_logging` | `true` | bringup側のロガー起動可否 |
| `log_dir` | `~/docking_logs` | 保存先。ホームを展開してmetadataに絶対パスを記録 |
| `experiment_name` | `docking` | 保存先に付ける実験名 |
| `experiment_note` | 空文字 | 実験メモ |
| `log_rate` | `20.0` | timeline記録周期［Hz］ |
| `flush_interval_sec` | `1.0` | 書き込みスレッドのflush間隔 |
| `stale_timeout_sec` | `1.0` | 鮮度判定の共通初期値。トピック別に上書き可能 |
| `queue_capacity` | `10000` | 書き込み待ちイベント数の上限 |
| `target_marker_id` | 必須指定 | timelineに採用するVisionマーカーID |
| `optitrack_pose_topic` | `/vrpn_mocap/RigidBody_1/pose` | 既存ロガーと同じ初期値 |

他のトピックは3章の値を初期値とする。鮮度判定はログ表示のための設定であり、Controllerのtimeout設定とは独立する。周期・容量は正値、名前は空でないことを検証する。実験名にパス区切りや親ディレクトリ参照を許さない。

### 6.2 起動例

```bash
ros2 launch docking_bringup docking.launch.py \
  handoff_x:=0.0 handoff_y:=1.0 handoff_yaw:=0.0 target_marker_id:=7 \
  enable_logging:=true log_dir:=$HOME/docking_logs experiment_name:=trial_01
```

```bash
ros2 launch docking_logger docking_logger.launch.py \
  target_marker_id:=7 log_dir:=$HOME/docking_logs experiment_name:=trial_01
```

数値は例。ロガー単体launchではセンサや制御ノードを起動しない。

### 6.3 記録期間

- ロガー起動直後から、準備待ちも含めて記録する。
- IDLE、DONE、FAULTになっても終了せず、ロガー終了まで記録する。
- 同じプロセス中の制御再スタートは同一フォルダに残す。ControllerOutputのsession_idをそれぞれ保存し、実行を識別する。
- Manager状態にはsession_idがないため、別トピック間の厳密な同時性は保証しない。
- 終了時は新規イベント受付を止め、キューを排出してflush・closeする。強制終了時の末尾保存は保証しない。
- ロガー起動順だけでは最初の指令の記録を保証できない。初回指令から必要な実験では `auto_start:=false` で起動し、保存開始の通知を確認してから既存のstart serviceを呼ぶ。

## 7. 書き込み・通信・異常系

### 7.1 書き込み

- 購読callbackでは時刻取得、最新値保持、書き込みキューへの投入のみを行う。
- timelineイベントも同じ書き込みスレッドで処理する。共有データの排他制御を行い、行の途中で値が変わらないようにする。
- キュー満杯時は新規イベントを破棄し、破棄数をストリーム別に数える。警告を間引いて表示する。無制限にメモリを増やさない。
- flushは定期実行するが、電源断に対する永続化保証を意味しない。

### 7.2 QoS

- 初版の購読depthは各100とする。
- OptiTrackはbest effort / volatile、その他は現在のpublisherに合わせてreliable / volatileを初期値とする。
- ストリームごとにreliabilityを設定可能とする。外部publisher利用時は互換性を確認する。
- 受信連番はロガー内で付ける番号であり、通信途中の全欠落数を示すものではない。

### 7.3 異常時の動作

| 状況 | 動作 |
| --- | --- |
| 保存先作成・初期ファイル作成失敗 | 明確なエラーとともにロガー起動失敗 |
| 一部トピック未受信 | 他の記録を継続、一覧は空欄・received=false |
| センサ値無効・非有限値 | 元の値・フラグを保存し、一覧の有効性に反映 |
| 対象マーカー喪失 | 最終受信時刻とstaleで識別 |
| キュー満杯 | 新規イベント破棄、警告と件数記録 |
| ディスク満杯・書き込み例外 | 記録失敗を明示し、ロガーを異常終了。可能な範囲でmetadata更新 |
| ロガー異常終了 | 制御ノードは継続。ロガーからロボット停止は行わない |

通常の起動ログには保存先を表示する。高頻度データを毎回端末へ出力しない。ロガーの起動成功をドッキング開始条件に追加する機能は初版には含めない。

## 8. 初版の対象外

- モーターコントローラへのシリアル送信文字列・送信結果、実回転速度。
- カメラ画像・動画、ROS bag、UWB生距離の記録。
- Visionメッセージへの撮影時刻追加。
- 厳密なセンサ時刻同期、座標変換、誤差グラフの自動生成。
- 他ノードの端末ログの集約、Managerの未公開の状態遷移理由。
- ロガーからの制御操作、動作中の記録開始・停止service。

## 9. 受け入れ条件

1. 各対象トピックに既知データを送信すると、個別CSVに値と受信時刻が記録される。
2. 通常負荷で、受信した短時間の指令変化が個別CSVに残る。
3. timelineが設定周期を目標に生成され、各個別CSVのsample_seqに対応する最新値を参照できる。
4. UWB・ZED・OptiTrack未接続でも起動・記録でき、未受信と無効値と古い値を区別できる。
5. 複数のマーカーIDが来ても、timelineは指定IDのみを採用する。
6. センサ途絶時にageが増加し、設定閾値を超えるとstaleになる。
7. IDLE・DONE・FAULT後も記録を継続し、制御再スタート時のsession_idを保存できる。
8. 正常終了時にキューを保存し、CSVとmetadataを閉じる。繰り返し起動しても上書きしない。
9. キュー超過・書き込み失敗を検出して通知し、ロボットへ指令を送らない。
10. bringup起動とロガー単体起動の両方が利用でき、enable_logging=falseでロガーを起動しない。
11. metadataに実験条件と提供された設定・上書き値が残り、未知の設定を取得済みとして扱わない。
12. UWBのP制御に使用した機体座標誤差と目標までの距離が個別CSV・timelineに残り、入力無効時はnanとinputs_valid=falseで識別できる。
13. 実機でログ有効時の制御周期・CPU負荷・キュー破棄数を確認し、実験に必要な周期で継続記録できることを確認する。


## 10. 実装・検証メモ

- パラメータは起動時に確定し、ロガーの設定を動作中に変更しない。
- QoS深さは `qos_depth`、ストリーム別設定は `<stream>_reliability` と `<stream>_stale_timeout_sec`。
  stream名は個別CSVの拡張子を除いた名前を使う。
- トピック設定は `<stream>_topic`。OptiTrackのみ `optitrack_pose_topic`。
- 単体起動で外部設定を添付する場合は `config_paths_json` に「設定名→ファイルパス」のJSONオブジェクト、
  `launch_overrides_json` に起動時の上書き値のJSONオブジェクトを指定する。
- `motor_commands.csv` の `data_json` では非有限値を文字列として保存し、有効なJSONを維持する。
- Excelで整数ナノ秒時刻を数値として開くと桁が丸められる場合がある。グラフは `elapsed_sec` を利用し、
  元のナノ秒整数の全桁を保持して読み込む場合は、その列を文字列に指定する。
- センサ未接続でも全CSVにヘッダーを作成する。既存値を保持するtimelineは同時刻計測を保証しない。
- 自動テストで未受信、鮮度、対象ID選択、異常値、終了時排出、キュー超過、書き込み失敗を確認。
  ROSメッセージとタイマーを使った直接callback試験を含む。
- 実機の別ノード間通信、長時間の記録負荷、制御周期への影響は実機確認が必要。
