# aruco_docking_sim 要求仕様書

Version: 0.1 draft
Date: 2026-06-29
Status: draft

## 1. 目的

本要求仕様書は、ArUcoドッキング制御を実機なしで検証するためのシミュレーション基盤 `aruco_docking_sim` の要求を定義する。

本シミュレーション基盤は、短期的には軽量なROS 2閉ループシミュレーションとして使い、将来的にはNVIDIA Omniverse / Isaac Sim上のシミュレーション環境へ接続できる土台として扱う。

既存の `aruco_dist_ctrl/aruco_distance_controller` は、原則として変更しない。シミュレーション環境は、実機と同じROS 2 topic interfaceを提供することでcontrollerに接続する。

## 2. 対象範囲

### 2.1 対象

| 項目 | 内容 |
| --- | --- |
| Simulation package | `aruco_docking_sim` |
| Controller package | `aruco_dist_ctrl` |
| Controller node | `aruco_distance_controller` |
| Input to controller | `/aruco/distance` |
| Output from controller | `/rov_cmd_vel` |
| Future simulator | Omniverse / Isaac Sim |

### 2.2 対象外

初版では以下を対象外とする。

- Omniverse / Isaac Sim sceneの作成
- USD asset、material、lighting、physics設定の作成
- 実カメラ画像レンダリングによるArUco検出
- ローバー全体の詳細な車輪接地物理
- 実機モータドライバ `cmd_roboteq` の起動

ただし、将来これらを追加しても既存controllerとの接続topicを変えないことを前提とする。

## 3. 基本方針

シミュレーション基盤は、controllerから見て実機と同じ入出力を提供する。

```text
Simulation World
  - lightweight ROS simulation
  - Omniverse / Isaac Sim
  - real robot

ROS Interface Adapter
  - /aruco/distance
  - /rov_cmd_vel
  - optional /tf, /odom, /joint_states

Controller
  - aruco_dist_ctrl/aruco_distance_controller
```

短期的には `/aruco/distance` を直接publishする軽量simを作れる構成とする。将来的にはOmniverse上の仮想カメラ画像から `aruco_distance_publisher` を通して `/aruco/distance` を生成する構成も許容する。

## 4. シミュレーションモード

### 4.1 軽量ArUcoDistanceシミュレーション

仮想ロボット状態と仮想マーカー位置から `aruco_interfaces/msg/ArucoDistance` を計算してpublishする。

目的:

- controllerの状態遷移を高速に確認する
- parameter tuningの初期値を探す
- ノイズ、検出ロスト、初期位置ずれへの挙動を確認する
- CIやローカル環境で実行しやすい検証手段にする

### 4.2 Omniverse連携シミュレーション

Omniverse / Isaac Simが仮想ロボット、カメラ、ArUcoマーカー、ドッキング環境を表現し、ROS 2 bridgeまたはadapterを通してcontrollerと接続する。

接続経路は2種類を許容する。

```text
Path A: synthetic observation
Omniverse adapter -> /aruco/distance -> aruco_distance_controller

Path B: camera pipeline
Omniverse camera -> image topic -> aruco_distance_publisher -> /aruco/distance -> aruco_distance_controller
```

初期導入ではPath Aを優先する。Path Bは画像処理込みのend-to-end検証が必要になった段階で追加する。

## 5. ROS Interface Requirements

| ID | Requirement |
| --- | --- |
| SIM-IF-001 | シミュレーション環境は `/aruco/distance` をpublishできること。 |
| SIM-IF-002 | シミュレーション環境は `/rov_cmd_vel` をsubscribeできること。 |
| SIM-IF-003 | `/aruco/distance` のmessage typeは `aruco_interfaces/msg/ArucoDistance` とすること。 |
| SIM-IF-004 | `/rov_cmd_vel` のmessage typeは `geometry_msgs/msg/Twist` とすること。 |
| SIM-IF-005 | `aruco_distance_controller` のtopic名、message type、parameter名は、シミュレーションのために変更しないこと。 |
| SIM-IF-006 | シミュレーション用launchは実機モータドライバ `cmd_roboteq` を起動しないこと。 |
| SIM-IF-007 | シミュレーション用launchは必要に応じて `aruco_distance_controller` を起動できること。 |
| SIM-IF-008 | シミュレーション用launchはcontrollerの主要parameterを実機launchと同じ名前で指定できること。 |
| SIM-IF-009 | 将来 `/tf`, `/odom`, `/joint_states` を追加しても、controllerの必須interfaceは `/aruco/distance` と `/rov_cmd_vel` のまま維持すること。 |

## 6. 軽量シミュレータ要求

| ID | Requirement |
| --- | --- |
| SIM-LITE-001 | 軽量シミュレータは仮想ロボット状態 `x`, `z`, `yaw` を保持すること。 |
| SIM-LITE-002 | 初期 `x`, `z`, `yaw` はROS parameterで指定できること。 |
| SIM-LITE-003 | 仮想マーカーまたはドッキング壁の位置とyawをROS parameterで指定できること。 |
| SIM-LITE-004 | `/rov_cmd_vel.linear.x` に基づいて仮想ロボットの前後位置を更新すること。 |
| SIM-LITE-005 | `/rov_cmd_vel.linear.y` に基づいて仮想ロボットの横位置を更新すること。 |
| SIM-LITE-006 | `/rov_cmd_vel.angular.z` に基づいて仮想ロボットのyawを更新すること。 |
| SIM-LITE-007 | 仮想ロボット状態から `ArucoDistance.z` を計算すること。 |
| SIM-LITE-008 | 仮想ロボット状態から `ArucoDistance.yaw` を計算すること。 |
| SIM-LITE-009 | 仮想ロボット状態から `ArucoDistance.normalized_center_error` を計算すること。 |
| SIM-LITE-010 | `ArucoDistance.x`, `distance`, `theta`, `center_u`, `center_v` も可能な範囲で整合した値をpublishすること。 |
| SIM-LITE-011 | update rateはROS parameterで指定できること。 |
| SIM-LITE-012 | `z`, `yaw`, `normalized_center_error` に独立したノイズを追加できること。 |
| SIM-LITE-013 | ArUco検出ロストを確率、距離条件、視野条件、または手動parameterで再現できること。 |
| SIM-LITE-014 | 検出ロスト中は `/aruco/distance` のpublishを停止できること。 |
| SIM-LITE-015 | 仮想ロボット状態、疑似観測値、速度指令をログ出力できること。 |

## 7. Omniverse連携要求

| ID | Requirement |
| --- | --- |
| SIM-OMNI-001 | Omniverse連携では、controllerとの接続にROS 2 topicを使用すること。 |
| SIM-OMNI-002 | Omniverse側の座標系とROS側の座標系変換を明示的に定義すること。 |
| SIM-OMNI-003 | Omniverse側の速度入力adapterは `/rov_cmd_vel` のbody-frame速度を受け取れること。 |
| SIM-OMNI-004 | Omniverse側の観測adapterは `/aruco/distance` を生成できること。 |
| SIM-OMNI-005 | 仮想カメラ画像を使う場合、既存 `aruco_distance_publisher` に接続できるimage topic構成を定義すること。 |
| SIM-OMNI-006 | Omniverse adapterはsimulation timeを使う場合、ROS 2の `/clock` と `use_sim_time` の扱いを文書化すること。 |
| SIM-OMNI-007 | Omniverse sceneの変更によりcontroller topic contractを破壊しないこと。 |

## 8. Parameter Requirements

### 8.1 共通parameter

| Parameter | Type | Default | 用途 |
| --- | --- | --- | --- |
| `publish_rate` | `float` | `20.0` | 疑似観測publish周期 |
| `initial_x` | `float` | `0.2` | 仮想ロボット初期横位置 |
| `initial_z` | `float` | `2.0` | 仮想ロボット初期前方距離 |
| `initial_yaw` | `float` | `0.0` | 仮想ロボット初期yaw |
| `marker_x` | `float` | `0.0` | 仮想マーカー横位置 |
| `marker_z` | `float` | `0.0` | 仮想マーカー位置 |
| `marker_yaw` | `float` | `0.0` | 仮想マーカーまたは壁のyaw |
| `image_width` | `float` | `1280.0` | `center_u` と `normalized_center_error` の計算 |
| `image_height` | `float` | `720.0` | `center_v` の計算 |
| `camera_horizontal_fov` | `float` | `1.2` | marker center errorの簡易計算 |
| `z_noise_stddev` | `float` | `0.0` | `z` ノイズ標準偏差 |
| `yaw_noise_stddev` | `float` | `0.0` | `yaw` ノイズ標準偏差 |
| `center_error_noise_stddev` | `float` | `0.0` | `normalized_center_error` ノイズ標準偏差 |
| `dropout_probability` | `float` | `0.0` | 疑似検出ロスト確率 |

### 8.2 parameter validation

| ID | Requirement |
| --- | --- |
| SIM-PARAM-001 | `publish_rate > 0.0` であること。 |
| SIM-PARAM-002 | `image_width > 0.0` であること。 |
| SIM-PARAM-003 | `image_height > 0.0` であること。 |
| SIM-PARAM-004 | `camera_horizontal_fov > 0.0` であること。 |
| SIM-PARAM-005 | noise stddevは `>= 0.0` であること。 |
| SIM-PARAM-006 | `dropout_probability` は `0.0 <= dropout_probability <= 1.0` であること。 |

## 9. 検証要求

| ID | Requirement |
| --- | --- |
| SIM-TEST-001 | 軽量simとcontrollerをlaunchし、`/aruco/distance` がpublishされることを確認できること。 |
| SIM-TEST-002 | controllerが `/rov_cmd_vel` をpublishすることを確認できること。 |
| SIM-TEST-003 | 初期位置がtargetから外れている場合、controllerがtarget方向の速度指令を出すことを確認できること。 |
| SIM-TEST-004 | 条件が揃った場合、controllerが `PRE_DOCKING` から `FINAL_DOCKING` へ遷移することを確認できること。 |
| SIM-TEST-005 | `docking_distance` 到達後に `/rov_cmd_vel` がzero velocityになることを確認できること。 |
| SIM-TEST-006 | 検出ロスト時にcontrollerがzero velocityをpublishすることを確認できること。 |
| SIM-TEST-007 | ノイズ付与時に移動平均、tolerance、deadbandの影響を確認できること。 |

## 10. Safety Requirements

| ID | Requirement |
| --- | --- |
| SIM-SAFE-001 | シミュレーションlaunchから実機モータドライバを起動しないこと。 |
| SIM-SAFE-002 | シミュレーションlaunchで実機serial deviceへ接続しないこと。 |
| SIM-SAFE-003 | 実機launchとシミュレーションlaunchを明確に分離すること。 |
| SIM-SAFE-004 | シミュレーション専用node名、package名、launch名を使い、実機起動と混同しにくくすること。 |
| SIM-SAFE-005 | 将来Omniverse連携時も、実機モータ出力とsim出力が同時に有効にならない構成にすること。 |

## 11. 初期実装方針

初版では、以下を優先して実装する。

1. `sim_interface_contract.md` にROS topic、message、座標系、符号規約を定義する。
2. 軽量sim nodeを追加し、`/rov_cmd_vel` と `/aruco/distance` の閉ループを作る。
3. 実機ドライバを含まないsimulation launchを追加する。
4. ログまたはtopic echoで状態遷移と停止条件を確認する。

Omniverse / Isaac Simのscene作成は、上記のtopic contractが固まった後に行う。
