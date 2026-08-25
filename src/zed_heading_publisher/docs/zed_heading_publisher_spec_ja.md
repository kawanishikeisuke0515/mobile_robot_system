# ZED Heading Publisher 設計仕様書

Version: 0.1 draft
Date: 2026-08-21
Status: draft

## 1. 概要

`zed_heading_publisher` は、ZED2i の magnetometer から磁場 raw data を取得し、hard-iron 補正とゼロ点補正を行ったうえで、ロボット yaw 角を ROS 2 topic に publish するノードである。

本ノードは方位角の算出と publish を担当する。速度指令、自己位置推定、センサフュージョン、地図座標系への変換は本設計の対象外とし、後段ノードで実施する。

## 2. 設計方針

- ZED2i センサ読み取り、磁場補正、yaw 算出、publish を 1 ノードに閉じる。
- hard-iron 補正値とゼロ点補正値は ROS parameter として外部から変更できるようにする。
- 制御で使う角度は `[-pi, pi)` の rad として publish する。
- 実験時の確認と rosbag 解析のため、raw 磁場値と補正後磁場値も同じ message に含める。
- 左旋回時に yaw が増加する向きを正方向とする。
- ZED2i から sensor data を取得できない周期では、古い値を再 publish しない。

## 3. パッケージ構成

### 3.1 実行パッケージ

```text
zed_heading_publisher
```

想定する主要ファイル:

```text
zed_heading_publisher/
  package.xml
  setup.py
  setup.cfg
  resource/zed_heading_publisher
  zed_heading_publisher/
    __init__.py
    zed_heading_publisher.py
  launch/
    zed_heading_publisher.launch.py
  config/
    zed_heading_publisher.yaml
  docs/
    zed_heading_publisher_spec_ja.md
```

### 3.2 メッセージパッケージ

カスタムメッセージは別パッケージに分離する。

```text
zed_interfaces
```

想定する主要ファイル:

```text
zed_interfaces/
  package.xml
  CMakeLists.txt
  msg/
    ZedHeading.msg
```

## 4. ノード設計

| 項目 | 内容 |
| --- | --- |
| Package | `zed_heading_publisher` |
| Node | `zed_heading_publisher` |
| Executable | `zed_heading_publisher` |
| Main file | `zed_heading_publisher/zed_heading_publisher.py` |

### 4.1 Responsibilities

`zed_heading_publisher` は以下を担当する。

1. ROS parameter の宣言、取得、validation
2. ZED2i camera の open
3. ZED2i sensor data の周期取得
4. magnetometer raw field の取得
5. X-Z 平面の hard-iron 補正
6. 磁気角 `magnetic_heading_deg` の算出
7. ロボット yaw `robot_yaw_deg`, `robot_yaw_rad` の算出
8. `zed_interfaces/msg/ZedHeading` の生成
9. `/zed/heading` への publish
10. 終了時の ZED2i close

## 5. Subscribe Topic

なし。

本ノードは ROS topic から sensor data を subscribe しない。ZED SDK の `pyzed.sl.Camera` から直接 sensor data を取得する。

## 6. Publish Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/zed/heading` | `zed_interfaces/msg/ZedHeading` | raw 磁場値、補正後磁場値、磁気角、ロボット yaw を publish する |

### 6.1 Message Fields

```text
std_msgs/Header header

float32 raw_x
float32 raw_z

float32 corrected_x
float32 corrected_z

float32 magnetic_heading_deg
float32 robot_yaw_deg
float32 robot_yaw_rad

bool valid
```

| Field | Type | Requirement |
| --- | --- | --- |
| `header` | `std_msgs/Header` | publish 時刻と `frame_id` を格納すること。 |
| `raw_x` | `float32` | ZED2i magnetometer の未補正 X 軸磁場値を格納すること。 |
| `raw_z` | `float32` | ZED2i magnetometer の未補正 Z 軸磁場値を格納すること。 |
| `corrected_x` | `float32` | `raw_x - center_x` を格納すること。 |
| `corrected_z` | `float32` | `raw_z - center_z` を格納すること。 |
| `magnetic_heading_deg` | `float32` | 補正後 X-Z 平面から算出した磁気角[deg]を格納すること。範囲は `[-180, 180)` とする。 |
| `robot_yaw_deg` | `float32` | ゼロ点補正後のロボット yaw[deg]を格納すること。範囲は `[-180, 180)` とする。 |
| `robot_yaw_rad` | `float32` | `robot_yaw_deg` を rad に変換した値を格納すること。範囲は `[-pi, pi)` とする。 |
| `valid` | `bool` | センサ取得と角度算出が成功した場合 `true` とすること。 |

## 7. Parameters

| Parameter | Type | Default | Range / Constraint | 用途 |
| --- | --- | --- | --- | --- |
| `center_x` | `double` | `-2.5354` | finite value | 360 度回転データから求めた磁場中心 X |
| `center_z` | `double` | `-10.3439` | finite value | 360 度回転データから求めた磁場中心 Z |
| `zero_heading_deg` | `double` | `40.0` | finite value | ロボット実 yaw = 0 deg のときの磁気角 |
| `publish_rate_hz` | `double` | `20.0` | `> 0.0` | sensor data 取得と publish の周期 |
| `frame_id` | `string` | `zed2i_mag` | non-empty | message header の frame id |
| `invert_yaw` | `bool` | `false` | `true` or `false` | yaw 符号反転が必要な場合に使用する |

### 7.1 Parameter Requirements

| ID | Requirement |
| --- | --- |
| PR-001 | `center_x`, `center_z`, `zero_heading_deg` は source code 固定値ではなく ROS parameter から設定できること。 |
| PR-002 | `publish_rate_hz <= 0.0` の場合、起動時に validation error とすること。 |
| PR-003 | `frame_id` が空文字の場合、起動時に validation error とすること。 |
| PR-004 | `center_x`, `center_z`, `zero_heading_deg` が finite value でない場合、起動時に validation error とすること。 |
| PR-005 | 起動時 log に `center_x`, `center_z`, `zero_heading_deg`, `publish_rate_hz`, `frame_id`, `invert_yaw` を出力すること。 |
| PR-006 | parameter は launch 引数または YAML から設定できること。 |

### 7.2 YAML Example

```yaml
zed_heading_publisher:
  ros__parameters:
    center_x: -2.5354
    center_z: -10.3439
    zero_heading_deg: 40.0
    publish_rate_hz: 20.0
    frame_id: zed2i_mag
    invert_yaw: false
```

## 8. 角度定義

### 8.1 Coordinate Selection

ZED2i magnetometer の X-Z 平面を使用する。

```text
raw_x = magnetic_field_uncalibrated[0]
raw_z = magnetic_field_uncalibrated[2]
```

Y 軸は本ノードの heading 算出には使用しない。

### 8.2 Hard-Iron Correction

360 度回転データから求めた磁場中心を引く。

```text
corrected_x = raw_x - center_x
corrected_z = raw_z - center_z
```

### 8.3 Magnetic Heading

補正後の X-Z 平面から磁気角を算出する。

```text
magnetic_heading_deg = atan2(corrected_x, corrected_z) [deg]
magnetic_heading_deg = normalize_to_180(magnetic_heading_deg)
```

正規化範囲:

```text
[-180, 180)
```

### 8.4 Robot Yaw

ロボット実 yaw = 0 deg のときに計測した磁気角 `zero_heading_deg` を引く。

```text
robot_yaw_deg = magnetic_heading_deg - zero_heading_deg
robot_yaw_deg = normalize_to_180(robot_yaw_deg)
```

`invert_yaw == true` の場合は、正規化前に符号を反転する。

```text
robot_yaw_deg = -robot_yaw_deg
robot_yaw_deg = normalize_to_180(robot_yaw_deg)
```

rad 値は以下で算出する。

```text
robot_yaw_rad = radians(robot_yaw_deg)
```

### 8.5 Positive Direction

ロボットが左旋回したときに `robot_yaw_deg` と `robot_yaw_rad` が増加する向きを正方向とする。

現時点の実機確認では、ZED2i magnetometer の `atan2(corrected_x, corrected_z)` に基づく角度は左旋回で増加する。そのため `invert_yaw` の default は `false` とする。

## 9. Angle Normalization

角度正規化関数は以下の仕様とする。

```text
normalize_to_180(angle_deg) = (angle_deg + 180.0) % 360.0 - 180.0
```

期待値:

| Input [deg] | Output [deg] |
| --- | --- |
| `0.0` | `0.0` |
| `180.0` | `-180.0` |
| `181.0` | `-179.0` |
| `359.0` | `-1.0` |
| `-181.0` | `179.0` |
| `-360.0` | `0.0` |

## 10. Control Flow

本ノードは明示的な state machine を持たない。timer callback により周期的に sensor data 取得、補正、publish を行う。

```text
STARTUP
  -> declare parameters
  -> validate parameters
  -> open ZED2i
  -> create publisher
  -> create timer
  -> TIMER_LOOP

TIMER_LOOP
  -> get_sensors_data
  -> get_magnetometer_data
  -> get_magnetic_field_uncalibrated
  -> extract raw_x and raw_z
  -> apply hard-iron correction
  -> calculate magnetic_heading_deg
  -> calculate robot_yaw_deg
  -> calculate robot_yaw_rad
  -> publish /zed/heading

SHUTDOWN
  -> close ZED2i
```

### 10.1 Startup Requirements

| ID | Requirement |
| --- | --- |
| CF-001 | node 起動時に全 parameter を宣言、取得、validation すること。 |
| CF-002 | ZED2i は `pyzed.sl.Camera` で open すること。 |
| CF-003 | `camera_resolution` は初期実装では `HD720` とすること。 |
| CF-004 | `camera_fps` は初期実装では `30` とすること。 |
| CF-005 | `depth_mode` は `NONE` とすること。 |
| CF-006 | ZED2i を open できない場合は起動失敗とすること。 |

### 10.2 Timer Loop Requirements

| ID | Requirement |
| --- | --- |
| CF-010 | timer period は `1.0 / publish_rate_hz` とすること。 |
| CF-011 | sensor data は `sl.TIME_REFERENCE.CURRENT` で取得すること。 |
| CF-012 | `get_sensors_data` が成功した周期のみ publish すること。 |
| CF-013 | publish message の `header.stamp` は publish 時の ROS clock とすること。 |
| CF-014 | publish message の `header.frame_id` は `frame_id` parameter とすること。 |
| CF-015 | publish 成功時の `valid` は `true` とすること。 |

## 11. QoS

初期実装では以下を使用する。

```text
history: keep last
depth: 10
reliability: reliable
```

高頻度 publish で最新値のみが重要になった場合は、`depth: 1` または `best effort` への変更を検討する。

## 12. Failure Cases

| ID | Failure Case | Required Behavior |
| --- | --- | --- |
| FC-001 | parameter validation error | 起動失敗とする |
| FC-002 | ZED SDK import error | error log を出し、起動失敗とする |
| FC-003 | ZED2i open failure | error log を出し、起動失敗とする |
| FC-004 | `get_sensors_data` failure | warn log を出し、その周期では publish しない |
| FC-005 | magnetometer data 取得 failure | warn log を出し、その周期では publish しない |
| FC-006 | raw 磁場値が finite value でない | warn log を出し、その周期では publish しない |

warn log は必要に応じて throttle し、センサ取得失敗が連続した場合でも log が過剰に増えないようにする。

## 13. Safety Requirements

| ID | Requirement |
| --- | --- |
| SR-001 | sensor data 取得失敗時に古い yaw を再 publish しないこと。 |
| SR-002 | raw 磁場値が不正な場合に推定値を捏造して publish しないこと。 |
| SR-003 | 本ノードは速度指令や motor command を publish しないこと。 |
| SR-004 | `center_x`, `center_z`, `zero_heading_deg` の使用値を起動 log で追跡できること。 |
| SR-005 | yaw 正方向は左旋回で増加する向きとし、後段の `angular.z` 正方向と整合させること。 |

## 14. 実機確認項目

| ID | Test | Expected Result |
| --- | --- | --- |
| TT-001 | ロボット実 yaw = 0 deg 方向に置いて起動する | `robot_yaw_deg` が `0 deg` 付近になる |
| TT-002 | ロボットを左にゆっくり旋回する | `robot_yaw_deg` が増加する |
| TT-003 | ロボットを右にゆっくり旋回する | `robot_yaw_deg` が減少する |
| TT-004 | `+179 deg` 付近からさらに左旋回する | `robot_yaw_deg` が `-180 deg` 付近へ連続的に折り返す |
| TT-005 | `/zed/heading` を rosbag 記録する | raw, corrected, yaw が同一 timestamp の message として記録される |
| TT-006 | `center_x`, `center_z`, `zero_heading_deg` を YAML で変更して起動する | 起動 log と publish 値に parameter 変更が反映される |

## 15. Future Extensions

初期実装では 1 ノード構成とする。将来、補正アルゴリズムやセンサフュージョンを分離したくなった場合は、以下の 2 ノード構成への移行を検討する。

```text
zed_magnetometer_publisher
  -> raw magnetic field を publish

heading_estimator
  -> raw magnetic field を subscribe
  -> 補正、yaw 算出、publish
```

ただし、初期実装では topic 数、launch 設定、同期処理の複雑化を避けるため、`zed_heading_publisher` 1 ノードに集約する。
