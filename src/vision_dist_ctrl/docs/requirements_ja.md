# vision_dist_ctrl 要求仕様書

Version: 0.1 draft
Date: 2026-06-16
Status: draft

## 1. 目的

本要求仕様書は、`vision_dist_ctrl/vision_distance_controller` において、ArUcoマーカーを基準にロボットをターゲット位置へドッキングさせるための要求を定義する。

既存のROS 2 topic、message、launch構成、状態遷移の基本仕様はできるだけ変更しない。

本要求は、実装前にAIへ設計仕様整理を依頼するための「人間が定義した要求仕様」として扱う。

## 2. 対象ノード

| 項目 | 内容 |
| --- | --- |
| Package | `vision_dist_ctrl` |
| Node | `vision_distance_controller` |
| Main file | `public/mobile_robot_system/src/vision_dist_ctrl/vision_dist_ctrl/vision_distance_controller.py` |
| Launch | `public/mobile_robot_system/src/vision_docking_bringup/launch/vision_docking.launch.py` |

## 3. ノードの目的

対象ノードは、ArUcoマーカーの検出結果をもとに、ロボットを指定されたターゲット位置へドッキングさせるための速度指令を生成する。

本要求で整理する目的は、以下を満たすことである。

1. ArUcoマーカーを基準に、前後位置、横位置、画像中心を制御する。
2. `target_z`、`target_x`、`docking_distance` で定義されたターゲット位置へ到達する。
3. `estimated_x` と `estimated_z` の観測ノイズを移動平均で平滑化する。
4. 既存の2状態ドッキング制御を維持する。
5. ゲイン、最小速度、移動平均windowを実機チューニングできる状態を維持する。

## 4. Subscribe Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/aruco/distance` | `aruco_interfaces/msg/ArucoDistance` | ArUcoマーカーの距離、yaw、画像中心誤差を受け取る |

### 使用フィールド

| Field | Type | 用途 |
| --- | --- | --- |
| `z` | `float` | マーカーまでの前方距離 |
| `yaw` | `float` | 壁相対位置推定に使用するマーカーyaw |
| `normalized_center_error` | `float` | 画像中心合わせ用のyaw制御に使用 |

## 5. Publish Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/rov_cmd_vel` | `geometry_msgs/msg/Twist` | ローバー速度指令をpublishする |

### 使用フィールド

| Field | 用途 |
| --- | --- |
| `linear.x` | 前後方向速度指令 |
| `linear.y` | 横方向速度指令 |
| `angular.z` | yaw角速度指令 |

## 6. Parameters

既存parameter名は原則として維持する。新規parameterを追加する場合も、既存launchから未指定で起動できるdefault値を必ず持たせる。

### 6.1 parameter

| Parameter | Type | Default | Range / Constraint | 用途 |
| --- | --- | --- | --- | --- |
| `target_z` | `float` | `1.3` | `docking_distance <= target_z` | PRE_DOCKINGでの前後目標距離 |
| `kp_z` | `float` | `2.0` | `>= 0.0` | 前後方向Pゲイン |
| `min_forward_speed` | `float` | `0.30` | `0.0 <= min_forward_speed <= max_forward_speed` | 前後方向の最小速度指令 |
| `max_forward_speed` | `float` | `0.95` | `>= 0.0` | 前後方向の最大速度指令 |
| `z_tolerance` | `float` | `0.01` | `>= 0.0` | PRE_DOCKINGの前後停止許容誤差 |
| `docking_distance` | `float` | `1.0` | `0.0 <= docking_distance <= target_z` | FINAL_DOCKINGの最終停止距離 |
| `target_x` | `float` | `0.0` | 制約なし | 横方向目標位置 |
| `kp_x` | `float` | `0.4` | `>= 0.0` | 横方向Pゲイン |
| `min_lateral_speed` | `float` | `0.30` | `0.0 <= min_lateral_speed <= max_lateral_speed` | 横方向の最小速度指令 |
| `max_lateral_speed` | `float` | `0.95` | `>= 0.0` | 横方向の最大速度指令 |
| `x_tolerance` | `float` | `0.01` | `>= 0.0` | 横方向停止許容誤差 |
| `target_yaw` | `float` | `0.0` | rad | 壁相対位置推定の基準yaw |
| `kp_center` | `float` | `0.3` | `>= 0.0` | 画像中心合わせ用yaw Pゲイン |
| `center_deadband` | `float` | `0.05` | `>= 0.0` | 画像中心合わせ用deadband |
| `max_angular_speed` | `float` | `0.5` | `>= 0.0` | yaw角速度の最大値 |
| `position_average_window_size` | `int` | `5` | `>= 1` | `estimated_x` と `estimated_z` の移動平均window数 |
| `detection_timeout` | `float` | `0.5` | `> 0.0` | ArUco検出timeout |
| `control_rate` | `float` | `20.0` | `> 0.0` | 制御周期 |

### 6.2 チューニング要求

| ID | Requirement |
| --- | --- |
| TR-001 | `min_forward_speed` は、実機が確実に動き出す最小値として調整できること。 |
| TR-002 | `kp_z` と `min_forward_speed` の組み合わせにより、ターゲット付近の速度指令が過大にならないこと。 |
| TR-003 | `z_tolerance` は、ArUco距離推定ノイズより小さすぎない値に調整できること。 |
| TR-004 | `kp_x`、`min_lateral_speed`、`x_tolerance` も前後方向と同じ考え方で調整できること。 |
| TR-005 | `position_average_window_size` は、ターゲット付近のノイズ低減と制御遅れのバランスを実機で調整できること。 |
| TR-006 | parameter変更はlaunch引数またはROS parameterで行えること。 |

## 7. State Machine / Control Flow

```text
PRE_DOCKING
  -> FINAL_DOCKING
```

### 7.1 PRE_DOCKING

目的:

- 初期位置からpre-docking poseへ移動する。
- 前後位置、横位置、画像中心合わせを行う。

制御入力:

```text
raw_estimated_x = aruco_z * sin(wrap_pi(aruco_yaw - target_yaw))
raw_estimated_z = aruco_z * cos(wrap_pi(aruco_yaw - target_yaw))

estimated_x = moving_average(raw_estimated_x)
estimated_z = moving_average(raw_estimated_z)

forward_error = estimated_z - target_z
lateral_error = estimated_x - target_x
center_error = normalized_center_error
```

`estimated_x` と `estimated_z` は、最新のArUco検出から計算した壁相対位置に対して移動平均を適用した値とする。`normalized_center_error` は画像内の即時的なmarker位置を表すため、本要求では移動平均の対象外とする。

### 7.1.1 Moving Average Requirements

| ID | Requirement |
| --- | --- |
| MA-001 | `/aruco/distance` 受信時に `raw_estimated_x` と `raw_estimated_z` を計算すること。 |
| MA-002 | `raw_estimated_x` と `raw_estimated_z` は、それぞれ独立した固定長bufferで保持すること。 |
| MA-003 | buffer長は `position_average_window_size` により指定すること。 |
| MA-004 | `position_average_window_size == 1` に明示設定した場合、移動平均なしの既存挙動と同等になること。 |
| MA-005 | buffer内のsample数が `position_average_window_size` 未満の場合、保持しているsampleのみで平均すること。 |
| MA-006 | `estimated_x` と `estimated_z` を使う前後・横方向制御、状態遷移判定、FINAL_DOCKING停止判定は、移動平均後の値を使用すること。 |
| MA-007 | `normalized_center_error` に基づくyaw制御は、移動平均後の `estimated_x/z` ではなく、受信した最新の `normalized_center_error` を使用すること。 |
| MA-008 | ArUco検出timeout時は、位置平均bufferをclearすること。 |
| MA-009 | timeout後に検出が復帰した場合、古い検出値を含まないbufferから移動平均を再開すること。 |

要求:

| ID | Requirement |
| --- | --- |
| CF-001 | `abs(forward_error) < z_tolerance` の場合、`linear.x = 0.0` とすること。 |
| CF-002 | `abs(lateral_error) < x_tolerance` の場合、`linear.y = 0.0` とすること。 |
| CF-003 | `abs(center_error) < center_deadband` の場合、`angular.z = 0.0` とすること。 |
| CF-004 | tolerance外では、P制御により目標方向へ速度指令を出すこと。 |
| CF-005 | 最小速度補償を行う場合でも、速度指令の符号は目標方向と一致すること。 |
| CF-006 | ターゲット付近で `linear.x` が正負に反転し続ける状態を、チューニングまたは速度整形で抑制できること。 |

### 7.2 PRE_DOCKING to FINAL_DOCKING

遷移条件:

```text
recent_detection == true
abs(estimated_z - target_z) < z_tolerance
abs(estimated_x - target_x) < x_tolerance
abs(normalized_center_error) < center_deadband
```

要求:

| ID | Requirement |
| --- | --- |
| CF-010 | 状態遷移条件は既存仕様から変更しないこと。 |
| CF-011 | 状態遷移時に新しいtopicや外部入力を要求しないこと。 |
| CF-012 | `estimated_z` と `estimated_x` は移動平均後の値を使用すること。 |

### 7.3 FINAL_DOCKING

目的:

- pre-docking poseから最終停止距離まで前進する。
- 必要に応じて横方向と画像中心合わせを継続する。

停止条件:

```text
estimated_z <= docking_distance
abs(estimated_x - target_x) < x_tolerance
abs(normalized_center_error) < center_deadband
```

要求:

| ID | Requirement |
| --- | --- |
| CF-020 | `estimated_z <= docking_distance` の場合、`linear.x = 0.0`, `linear.y = 0.0`, `angular.z = 0.0` とすること。 |
| CF-021 | `estimated_z > docking_distance` の場合、`estimated_z - docking_distance` に基づき前進速度を計算すること。 |
| CF-022 | FINAL_DOCKING中の前進速度も `kp_z`、`min_forward_speed`、`max_forward_speed` の影響を受けること。 |
| CF-023 | 最終停止距離直前で過大な最小速度指令により突っ込みすぎないように、parameter調整または内部速度整形で抑制できること。 |
| CF-024 | FINAL_DOCKING停止条件の `estimated_z` は移動平均後の値を使用すること。 |

## 8. Failure Cases

| ID | Failure Case | Required Behavior |
| --- | --- | --- |
| FC-001 | `/aruco/distance` が未受信 | `/rov_cmd_vel` にzero velocityをpublishする |
| FC-002 | `detection_timeout` を超えてArUco検出が途切れる | zero velocityをpublishし、必要に応じて `PRE_DOCKING` に戻る |
| FC-003 | `min_forward_speed > max_forward_speed` | 起動時にparameter validation errorとする |
| FC-004 | `docking_distance > target_z` | 起動時にparameter validation errorとする |
| FC-005 | `control_rate <= 0.0` | 起動時にparameter validation errorとする |
| FC-006 | `position_average_window_size < 1` | 起動時にparameter validation errorとする |
| FC-007 | ArUco距離推定値がターゲット付近でノイズにより揺れる | 移動平均、tolerance/deadband、速度調整により速度指令のチャタリングを抑制できること |

## 9. Safety Requirements

| ID | Requirement |
| --- | --- |
| SR-001 | ArUco検出timeout時は必ずzero velocityをpublishすること。 |
| SR-002 | `linear.x` は `[-max_forward_speed, max_forward_speed]` に制限すること。 |
| SR-003 | `linear.y` は `[-max_lateral_speed, max_lateral_speed]` に制限すること。 |
| SR-004 | `angular.z` は `[-max_angular_speed, max_angular_speed]` に制限すること。 |
| SR-005 | parameter validationに失敗した状態で制御を開始しないこと。 |
| SR-006 | 実機試験では外部停止手段を用意した状態で実施すること。 |
| SR-007 | timeout前の古い位置推定値を、検出復帰後の移動平均に混ぜないこと。 |

## 10. 実機確認観点

### 10.1 確認するtopic

| Topic | 確認内容 |
| --- | --- |
| `/aruco/distance` | `z`, `yaw`, `normalized_center_error` のノイズ量とターゲット付近の揺れ |
| `/rov_cmd_vel` | `linear.x`, `linear.y`, `angular.z` の符号、飽和、min速度張り付き |
| `rov/motors` | 速度指令に対するmotor commandの変化 |

### 10.2 記録する値

| Value | 目的 |
| --- | --- |
| `aruco_z` | 前後距離誤差の確認 |
| `aruco_z_cos_yaw` | 壁相対の前後推定値確認 |
| `aruco_z_sin_yaw` | 壁相対の横位置推定値確認 |
| `estimated_z_average` | 移動平均後の前後推定値確認 |
| `estimated_x_average` | 移動平均後の横位置推定値確認 |
| `cmd_linear_x` | 前後速度指令の振動確認 |
| `cmd_linear_y` | 横速度指令の振動確認 |
| `cmd_angular_z` | 画像中心合わせ制御の確認 |

### 10.3 実機テスト条件

| ID | Test | Pass Condition |
| --- | --- | --- |
| VT-001 | `position_average_window_size:=1` で制御する | 移動平均なしの既存挙動と同等に動作する |
| VT-002 | `position_average_window_size` を増やしてターゲット付近の `/rov_cmd_vel` を記録する | `cmd_linear_x` と `cmd_linear_y` の短周期チャタリングが減る |
| VT-003 | markerを一時的に隠してから再検出させる | timeout前の古い位置推定値が復帰後の平均に混ざらない |
| VT-004 | FINAL_DOCKING停止直前の挙動を確認する | 移動平均による遅れで `docking_distance` を大きく越えて突っ込みすぎない |


## 11. 設計仕様書作成時の注意

AIが設計仕様書を作成する場合、以下を守ること。

1. 既存topic、message、node責務を勝手に変更しない。
2. `vision_distance_controller` の責務は速度指令生成までとする。
3. motor commandの不感帯補償は本nodeで直接扱わない。
4. 追加parameterが必要な場合は、既存挙動と互換になるdefault値を定義する。
5. `position_average_window_size` のdefaultは、実機チューニング初期値として `5` とする。
6. 移動平均の対象は `estimated_x` と `estimated_z` に限定し、yaw制御用の `normalized_center_error` は対象外とする。
7. 実装案が複数ある場合は、parameter調整のみで対応する案、移動平均を追加する案、内部速度整形を追加する案を分けて提示する。

## 12. AIへの依頼文

要求仕様一覧:

この要求仕様に従って、ROS2ノード `vision_distance_controller` の仕様を整理してください。

レビュー観点:

1. 要求仕様との一致
2. Topic / Parameter の整合性
3. State machine / control flow の明確さ
4. Failure case / Safety 対応
5. 実機テスト観点
6. Node責務の分離

注意:

AIによる勝手な仕様追加を避け、既存仕様をできるだけ維持してください。
