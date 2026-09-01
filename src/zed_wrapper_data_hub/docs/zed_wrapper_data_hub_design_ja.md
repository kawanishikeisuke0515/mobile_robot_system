# ZED Wrapper Data Hub 設計仕様書

Version: 0.1 draft
Date: 2026-09-01
Status: draft

## 1. 概要

本仕様書は、ZED2/ZED2i 由来の複数種類のデータを同時利用するための hub 設計方針を定義する。

`src/zed_wrapper_data_hub` は、公式 `zed_wrapper` を ZED data hub として起動するための薄い ROS 2 package とする。下流ノードをまとめて起動する bringup package は後続で別途作成する。

今後、既存の Vision と Magnetic heading に加えて VIO を利用する予定である。ZED camera device への直接アクセスを複数ノードに分散させると、camera open の競合、grab 周期の不整合、CPU/GPU/電力消費の増加が起こりやすい。

そのため、本設計では Stereolabs 公式 `zed_wrapper` を ZED data hub として扱い、ZED device access を 1 箇所に集約する。下流ノードは公式 wrapper が publish する ROS topic を subscribe し、既存の計算処理と publish topic を維持する。

## 2. 目的

- ZED device open/grab を 1 系統に集約する。
- Image、Magnetic、VIO を同時に利用できる構成にする。
- 不要な機能を config で無効化し、消費電力と計算負荷を抑える。
- 既存の `/aruco/distance` と `/zed/heading` の topic interface をできるだけ維持する。
- 将来の sensor fusion や controller 変更に備え、データ取得とデータ処理の責務を分離する。
- bringup 実装に入る前に、ZED data hub と下流処理ノードの責務境界を明確にする。
- ハブ単体で公式 `zed_wrapper` を起動し、必要な ZED topic を確認できるようにする。

## 3. 設計方針

### 3.1 ZED Data Hub

本システムにおける `zed_data_hub` は、まずは公式 `zed_wrapper` が担う役割として定義する。

```text
zed_wrapper
  -> ZED2/ZED2i を open
  -> image / sensor / odometry / pose を publish
```

公式 wrapper で要求を満たせない場合に限り、薄い relay node または自作 `zed_data_hub` node を追加する。

### 3.2 Downstream Processing

ZED 由来データを使う処理ノードは、ZED camera device を直接 open しない。

```text
aruco_distance_publisher
  -> image topic を subscribe
  -> ArUco marker pose を計算
  -> /aruco/distance を publish

zed_heading_publisher
  -> magnetometer または sensor topic を subscribe
  -> hard-iron 補正と yaw 算出
  -> /zed/heading を publish

controller / fusion node
  -> /aruco/distance, /zed/heading, VIO odometry/pose を subscribe
  -> velocity command または fused state を publish
```

### 3.3 Optional Enable

ZED wrapper 側の config で必要な module と publish topic のみを有効化する。

```text
Vision docking:
  image: on
  magnetic: optional
  vio: optional
  depth: off unless required
  point cloud: off
  object detection: off
  mapping: off

UWB + ZED heading control:
  image: off
  magnetic: on
  vio: optional
  depth: off unless required

VIO evaluation:
  image: optional
  magnetic: optional
  vio: on
  depth: off unless required by selected tracking mode
```

## 4. Target Architecture

```text
                     +----------------+
                     |  ZED2 / ZED2i  |
                     +--------+-------+
                              |
                              v
                     +----------------+
                     |  zed_wrapper   |
                     |  data hub role |
                     +---+--------+---+
                         |        |
        +----------------+        +----------------+
        |                                  |       |
        v                                  v       v
+------------------------+      +----------------+ +----------------+
| aruco_distance_publisher|      | zed_heading_  | | VIO consumer   |
| SUB image               |      | publisher      | | or controller  |
| PUB /aruco/distance     |      | SUB mag/sensor | | SUB odom/pose  |
+------------------------+      | PUB /zed/heading| +----------------+
                                +----------------+
```

## 5. Node Responsibilities

### 5.1 `zed_wrapper`

`zed_wrapper` は以下を担当する。

1. ZED camera device の open/close
2. camera grab loop
3. image topic の publish
4. sensor topic の publish
5. odometry/pose topic の publish
6. TF publish
7. depth、mapping、object detection など追加機能の optional enable

`zed_wrapper` は `/aruco/distance` や `/zed/heading` を publish しない。

### 5.2 `aruco_distance_publisher`

`aruco_distance_publisher` は以下を担当する。

1. ZED wrapper の image topic を subscribe
2. 必要に応じて左右 image の選択または topic 選択
3. camera calibration の読み込み
4. ArUco marker detection
5. marker pose、distance、theta、yaw、center error の算出
6. 既存 topic `/aruco/distance` の publish

移行後、本ノードは OpenCV `VideoCapture(0)` で camera device を直接 open しない。

### 5.3 `zed_heading_publisher`

`zed_heading_publisher` は以下を担当する。

1. ZED wrapper の magnetometer または sensor topic を subscribe
2. raw magnetic field の抽出
3. hard-iron 補正
4. zero heading 補正
5. robot yaw の算出
6. 既存 topic `/zed/heading` の publish

移行後、本ノードは `pyzed.sl.Camera` で ZED camera device を直接 open しない。

### 5.4 VIO Consumer

VIO を利用する controller または fusion node は、公式 wrapper の odometry/pose topic を subscribe する。

初期段階では VIO の再計算や独自推定は行わず、公式 wrapper の出力をそのまま利用する。

## 6. Topic Design

実際の topic 名は公式 `zed_wrapper` の namespace と config に依存するため、本 package の launch/config と後続の bringup/config で明示する。

| Data | Source | Consumer | Purpose |
| --- | --- | --- | --- |
| Image | `zed_wrapper` image topic | `aruco_distance_publisher` | ArUco detection |
| Magnetic | `zed_wrapper` mag/sensor topic | `zed_heading_publisher` | Magnetic yaw estimation |
| VIO odometry | `zed_wrapper` odometry topic | controller/fusion node | relative motion estimate |
| VIO pose | `zed_wrapper` pose topic | controller/fusion node | map/world pose estimate |
| ArUco distance | `/aruco/distance` | controller/logger | marker-relative docking state |
| ZED heading | `/zed/heading` | controller/logger | corrected robot yaw |

## 7. Config Policy

ZED wrapper config は運用 mode ごとに分ける方針とする。

初期実装では、ハブ単体起動用の `zed2i_data_hub.yaml` を本 package に配置する。運用 mode ごとの細分化は、実機確認後に追加する。

```text
config/zed2i_data_hub.yaml
zed2i_docking.yaml
zed2i_vio_only.yaml
zed2i_vision_only.yaml
zed2i_heading_only.yaml
```

初期の docking config では以下を基本方針とする。

```yaml
video:
  publish_rgb: true
  publish_left_right: false
  publish_raw: false
  publish_gray: false
  publish_stereo: false

sensors:
  publish_imu: true
  publish_imu_raw: false
  publish_mag: true
  publish_baro: false
  publish_temp: false

depth:
  depth_mode: 'NONE'
  publish_depth_map: false
  publish_point_cloud: false

pos_tracking:
  pos_tracking_enabled: true
  imu_fusion: true
  publish_odom_pose: true
  publish_cam_path: false
  two_d_mode: true

mapping:
  mapping_enabled: false

object_detection:
  od_enabled: false
```

ただし、選択した positional tracking mode が depth を必要とする場合は、`depth.depth_mode` を軽量な mode に変更する。

## 8. Launch Policy

本 package は ZED wrapper 単体を data hub として起動する launch file を提供する。

```text
zed_data_hub.launch.py
  -> zed_wrapper
```

下流ノードを含む全体起動は、後続の bringup package で同一 launch file から起動できるようにする。

```text
future zed_docking_bringup.launch.py
  -> zed_wrapper
  -> aruco_distance_publisher
  -> zed_heading_publisher
  -> controller
  -> locomotion nodes
```

`zed_data_hub.launch.py` では、以下を切り替えられるようにする。

| Argument | Type | Default | Purpose |
| --- | --- | --- | --- |
| `camera_model` | string | `zed2i` | ZED camera model |
| `camera_name` | string | `zed2i` | ZED wrapper camera name |
| `namespace` | string | empty | Optional namespace |
| `node_name` | string | `zed_node` | ZED wrapper node name |
| `serial_number` | string | `0` | Camera serial number |
| `camera_id` | string | `-1` | Camera device ID |
| `publish_tf` | bool | `true` | odom to camera_link TF |
| `publish_map_tf` | bool | `true` | map to odom TF |
| `publish_imu_tf` | bool | `false` | IMU TF |
| `enable_ipc` | bool | `true` | ZED wrapper IPC |
| `ros_params_override_path` | string | `config/zed2i_data_hub.yaml` | ZED wrapper override config |

## 9. Migration Plan

### Phase 1: Hub Package

- 本仕様書を追加する。
- `zed_wrapper_data_hub` package を追加する。
- 公式 `zed_wrapper` の data hub 用 config 方針を決める。
- `zed_data_hub.launch.py` で公式 `zed_wrapper` を起動する。
- topic 名と frame_id を実機環境で確認する。

### Phase 1.5: Bringup Package Design

- ZED wrapper と下流ノードを起動する bringup package を別途設計する。
- config YAML の配置先を決める。
- launch argument と mode 切り替えを設計する。

### Phase 2: ArUco Input Migration

- `aruco_distance_publisher` に `image_topic` parameter を追加する。
- `sensor_msgs/msg/Image` を subscribe する。
- `cv_bridge` で OpenCV image に変換する。
- 既存の ArUco 計算処理と `/aruco/distance` publish は維持する。
- 旧 `VideoCapture(0)` mode は移行期間だけ optional fallback として残すか、別 branch で削除する。

### Phase 3: Heading Input Migration

- `zed_heading_publisher` に `mag_topic` または `sensor_topic` parameter を追加する。
- ZED wrapper の magnetometer/sensor topic を subscribe する。
- 既存の `calculate_heading` と `/zed/heading` publish は維持する。
- `pyzed.sl.Camera` direct open は廃止する。

### Phase 4: VIO Integration

- controller または fusion node に VIO odometry/pose topic を subscribe させる。
- まずは logging/evaluation のみに利用する。
- 制御に使う前に frame alignment、timestamp、drift、dropout behavior を実機確認する。

## 10. Failure Cases

| ID | Failure Case | Required Behavior |
| --- | --- | --- |
| FC-001 | `zed_wrapper` が起動しない | 下流ノードは入力 timeout を log し、古い値を再 publish しない |
| FC-002 | image topic が届かない | `aruco_distance_publisher` は `/aruco/distance` を publish しない |
| FC-003 | magnetic topic が届かない | `zed_heading_publisher` は `/zed/heading` を publish しない |
| FC-004 | VIO odometry/pose が invalid | controller は VIO 値を制御に使わない |
| FC-005 | timestamp が大きくずれる | fusion/controller 側で stale data として扱う |
| FC-006 | frame_id が想定と異なる | 起動 log または validation で検出し、設定を修正できるようにする |

## 11. Safety Requirements

| ID | Requirement |
| --- | --- |
| SR-001 | ZED camera device を複数ノードが直接 open しないこと。 |
| SR-002 | sensor input が途切れた場合、古い検出値や古い yaw を再 publish しないこと。 |
| SR-003 | controller は stale な ArUco、heading、VIO data を制御に使わないこと。 |
| SR-004 | depth、point cloud、mapping、object detection は必要時のみ有効化すること。 |
| SR-005 | VIO を制御に使う前に、frame alignment と yaw 正方向を実機で確認すること。 |

## 12. 実機確認項目

| ID | Test | Expected Result |
| --- | --- | --- |
| TT-001 | `zed_wrapper` のみ起動する | image、sensor、odom/pose の必要 topic が確認できる |
| TT-002 | docking config で起動する | depth/point cloud/object detection/mapping が不要に publish されない |
| TT-003 | `aruco_distance_publisher` を image subscribe mode で起動する | marker 検出時に `/aruco/distance` が publish される |
| TT-004 | `zed_heading_publisher` を sensor subscribe mode で起動する | `/zed/heading` が publish され、左旋回で yaw が増加する |
| TT-005 | VIO odometry/pose を rosbag 記録する | 移動方向と frame_id が期待と一致する |
| TT-006 | ZED input を一時的に止める | 下流ノードが古い値を再 publish しない |

## 13. Open Questions

| ID | Question | Note |
| --- | --- | --- |
| OQ-001 | ArUco に使う image topic は RGB rectified と left/right raw のどちらにするか | calibration file との整合が必要 |
| OQ-002 | `zed_heading_publisher` が subscribe すべき公式 topic 型は何か | 実機の `ros2 topic list -t` で確認する |
| OQ-003 | VIO は odometry と pose のどちらを controller に渡すか | drift と loop closure の扱いで決める |
| OQ-004 | VIO の frame を robot base frame に変換する責務をどの node が持つか | TF 設計が必要 |
| OQ-005 | `depth_mode: NONE` で目的の VIO が成立するか | selected tracking mode に依存する |

## 14. Initial Decision

初期方針は以下とする。

1. 公式 `zed_wrapper` を `zed_data_hub` として採用する。
2. 自作 `zed_data_hub` node は初期実装では作らない。
3. `aruco_distance_publisher` と `zed_heading_publisher` は subscribe 型へ移行する。
4. VIO はまず公式 wrapper の odometry/pose を logging/evaluation で使う。
5. 制御への VIO 組み込みは、topic、frame、timestamp、drift の実機確認後に行う。
