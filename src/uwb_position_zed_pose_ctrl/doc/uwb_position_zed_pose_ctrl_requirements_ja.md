# UWB + ZED Pose Controller 要求仕様書

Version: 0.1 draft
Date: 2026-08-25
Status: draft

## 1. 目的

本要求仕様書は、UWB 位置推定結果と ZED2i heading をもとに、ロボットを指定された目標位置および目標姿勢へ移動させる P 制御ノードの要求を定義する。

基本の制御方針は `vision_dist_ctrl/vision_distance_controller` と同じく、ROS 2 topic を subscribe し、`geometry_msgs/msg/Twist` の速度指令を publish する構成とする。

本要求では、ターゲット位置とターゲット角度に許容マージンを持たせ、目標近傍で速度指令が細かく反転し続ける状態を抑制できることを追加要求とする。

## 2. 対象ノード

ノード名は `uwb_position_zed_pose_ctrl` とする。

| 項目 | 内容 |
| --- | --- |
| Package | `uwb_position_zed_pose_ctrl` |
| Node | `uwb_position_zed_pose_ctrl` |
| Main file | `uwb_position_zed_pose_ctrl/uwb_position_zed_pose_ctrl.py` |
| Launch | `launch/uwb_position_zed_pose_ctrl.launch.py` |

## 3. ノードの目的

対象ノードは、UWB で推定した 2D 位置と ZED2i magnetometer 由来の yaw 角を使用し、ロボットを任意の目標 pose へ移動させるための速度指令を生成する。

本要求で整理する目的は、以下を満たすことである。

1. `/uwb/position` から現在位置 `x_m`, `y_m` を取得する。
2. `/zed/heading` から現在 yaw `robot_yaw_rad` を取得する。
3. `target_x`, `target_y`, `target_yaw` で定義された目標 pose へ到達する。
4. 位置誤差と yaw 誤差に対して P 制御を行う。
5. 目標位置と目標 yaw には、それぞれ許容マージンを設定できる。
6. ゲイン、最大速度、最小速度、tolerance を実機チューニングできる。
7. センサ timeout または invalid data 時は安全に停止する。

## 4. Subscribe Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/uwb/position` | `uwb_interfaces/msg/UwbPosition` | UWB によるロボット 2D 位置を受け取る |
| `/zed/heading` | `zed_interfaces/msg/ZedHeading` | ZED2i によるロボット yaw を受け取る |

### 4.1 `/uwb/position` 使用フィールド

| Field | Type | 用途 |
| --- | --- | --- |
| `x_m` | `float32` | ロボット現在位置 x [m] |
| `y_m` | `float32` | ロボット現在位置 y [m] |
| `valid` | `bool` | UWB 位置推定が有効か判定する |

### 4.2 `/zed/heading` 使用フィールド

| Field | Type | 用途 |
| --- | --- | --- |
| `robot_yaw_rad` | `float32` | ロボット現在 yaw [rad] |
| `valid` | `bool` | heading が有効か判定する |

## 5. Publish Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/rov_cmd_vel` | `geometry_msgs/msg/Twist` | ローバー速度指令を publish する |

### 5.1 使用フィールド

| Field | 用途 |
| --- | --- |
| `linear.x` | ロボット前後方向速度指令 |
| `linear.y` | ロボット横方向速度指令 |
| `angular.z` | yaw 角速度指令 |

`linear.z`, `angular.x`, `angular.y` は使用せず、常に `0.0` とする。

## 6. Parameters

parameter は launch 引数または YAML から変更できること。未指定で起動できる default 値を持たせる。

| Parameter | Type | Default | Range / Constraint | 用途 |
| --- | --- | --- | --- | --- |
| `uwb_position_topic` | `string` | `/uwb/position` | non-empty | subscribe する UWB 位置 topic |
| `zed_heading_topic` | `string` | `/zed/heading` | non-empty | subscribe する ZED heading topic |
| `cmd_vel_topic` | `string` | `/rov_cmd_vel` | non-empty | publish する速度指令 topic |
| `target_x` | `float` | `0.0` | finite | 目標 x 位置 [m] |
| `target_y` | `float` | `0.0` | finite | 目標 y 位置 [m] |
| `target_yaw` | `float` | `0.0` | finite rad | 目標 yaw [rad] |
| `x_tolerance` | `float` | `0.05` | `>= 0.0` | x 方向の停止許容誤差 [m] |
| `y_tolerance` | `float` | `0.05` | `>= 0.0` | y 方向の停止許容誤差 [m] |
| `yaw_tolerance` | `float` | `0.05` | `>= 0.0` | yaw の停止許容誤差 [rad] |
| `kp_x` | `float` | `0.4` | `>= 0.0` | x 方向 P ゲイン |
| `kp_y` | `float` | `0.4` | `>= 0.0` | y 方向 P ゲイン |
| `kp_yaw` | `float` | `0.8` | `>= 0.0` | yaw P ゲイン |
| `min_linear_speed` | `float` | `0.0` | `0.0 <= min_linear_speed <= max_linear_speed` | 並進方向の最小速度指令 |
| `max_linear_speed` | `float` | `0.5` | `>= 0.0` | 並進方向の最大速度指令 |
| `min_angular_speed` | `float` | `0.0` | `0.0 <= min_angular_speed <= max_angular_speed` | yaw の最小角速度指令 |
| `max_angular_speed` | `float` | `0.5` | `>= 0.0` | yaw 角速度の最大値 |
| `position_timeout` | `float` | `0.5` | `> 0.0` | UWB 位置の timeout [s] |
| `heading_timeout` | `float` | `0.5` | `> 0.0` | ZED heading の timeout [s] |
| `control_rate` | `float` | `20.0` | `> 0.0` | 制御周期 [Hz] |

### 6.1 許容マージン要求

| ID | Requirement |
| --- | --- |
| MR-001 | `abs(target_x - current_x) <= x_tolerance` の場合、x 方向の速度指令を `0.0` とすること。 |
| MR-002 | `abs(target_y - current_y) <= y_tolerance` の場合、y 方向の速度指令を `0.0` とすること。 |
| MR-003 | `abs(wrap_pi(target_yaw - current_yaw)) <= yaw_tolerance` の場合、yaw 角速度指令を `0.0` とすること。 |
| MR-004 | `x_tolerance`, `y_tolerance`, `yaw_tolerance` は実機の UWB / heading ノイズより小さすぎない値に調整できること。 |
| MR-005 | 目標 pose 到達判定は、位置と yaw の全 tolerance を同時に満たした場合のみ true とすること。 |

## 7. Control Strategy

### 7.1 座標と誤差定義

UWB の `x_m`, `y_m` は world / anchor 座標系上のロボット位置として扱う。速度指令はロボット座標系の `linear.x`, `linear.y` として publish するため、world 座標系の位置誤差を現在 yaw でロボット座標系へ変換して制御する。

yaw は ZED heading publisher と同じく、左旋回時に増加する向きを正方向とする。`angular.z > 0.0` は反時計回り、`angular.z < 0.0` は時計回りの回転指令として扱う。

角度誤差は `wrap_pi(target_yaw - current_yaw)` で計算し、正規化範囲は `[-pi, pi)` とする。これにより、目標 yaw と現在 yaw の差がちょうど 180 deg の場合は `-pi` として扱い、時計回りに回転する。

```text
raw_error_world_x = target_x - current_x
raw_error_world_y = target_y - current_y
yaw_error = wrap_pi(target_yaw - current_yaw)

error_world_x = 0.0 if abs(raw_error_world_x) <= x_tolerance else raw_error_world_x
error_world_y = 0.0 if abs(raw_error_world_y) <= y_tolerance else raw_error_world_y

error_body_x =  cos(current_yaw) * error_world_x + sin(current_yaw) * error_world_y
error_body_y = -sin(current_yaw) * error_world_x + cos(current_yaw) * error_world_y
```

`error_body_x` はロボット前方を正、`error_body_y` はロボット左方向または右方向の定義を実装前に `locomotion_core/rover_velocity` と合わせること。

### 7.2 P 制御

位置と yaw は独立した P 制御とする。

```text
cmd.linear.x = kp_x * error_body_x
cmd.linear.y = kp_y * error_body_y
cmd.angular.z = kp_yaw * yaw_error
```

yaw 制御では、`yaw_error > 0.0` の場合に反時計回り、`yaw_error < 0.0` の場合に時計回りへ回転する。

tolerance 内の world 座標系誤差は個別に `0.0` としてから、body 座標系へ変換する。

```text
if abs(error_world_x) <= x_tolerance:
    error_world_x = 0.0

if abs(error_world_y) <= y_tolerance:
    error_world_y = 0.0

if abs(yaw_error) <= yaw_tolerance:
    cmd.angular.z = 0.0
```

位置到達判定と tolerance 適用は world 座標系の `x/y` 誤差を使用し、速度指令生成は tolerance 適用後に body 座標系へ変換した誤差を使用する。

### 7.3 速度制限

P 制御で計算した速度指令は、最大速度で飽和させる。

```text
cmd.linear.x = clamp(cmd.linear.x, -max_linear_speed, max_linear_speed)
cmd.linear.y = clamp(cmd.linear.y, -max_linear_speed, max_linear_speed)
cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed, max_angular_speed)
```

最小速度補償を有効にする場合、速度指令の符号は必ず目標方向と一致させる。

```text
if abs(cmd.linear.x) > 0.0:
    cmd.linear.x = sign(cmd.linear.x) * max(abs(cmd.linear.x), min_linear_speed)
```

`linear.y` と `angular.z` も同じ考え方とする。

## 8. State Machine / Control Flow

初期仕様では単一状態の pose tracking とする。

```text
TRACKING
```

### 8.1 TRACKING

目的:

- 現在 pose から目標 pose へ移動する。
- 位置と yaw の各誤差が tolerance 内に入ったら停止する。

制御入力:

```text
current_x = latest_uwb_position.x_m
current_y = latest_uwb_position.y_m
current_yaw = latest_zed_heading.robot_yaw_rad
```

到達条件:

```text
recent_position == true
recent_heading == true
abs(target_x - current_x) <= x_tolerance
abs(target_y - current_y) <= y_tolerance
abs(wrap_pi(target_yaw - current_yaw)) <= yaw_tolerance
```

要求:

| ID | Requirement |
| --- | --- |
| CF-001 | UWB 位置と ZED heading の両方が recent かつ valid の場合のみ制御を行うこと。 |
| CF-002 | 到達条件を満たした場合、`/rov_cmd_vel` に zero velocity を publish し続けること。 |
| CF-003 | tolerance 外では、P 制御により目標方向へ速度指令を出すこと。 |
| CF-004 | 位置制御と yaw 制御は独立に計算し、同時に指令できること。 |
| CF-005 | 目標近傍で速度指令が反転し続ける状態は、tolerance、最小速度、最大速度、ゲイン調整で抑制できること。 |
| CF-006 | `target_yaw == 0 deg`, `current_yaw == 180 deg` のように yaw 誤差がちょうど 180 deg の場合は、`yaw_error = -pi` として時計回りに回転すること。 |

## 9. Failure Cases

| ID | Failure Case | Required Behavior |
| --- | --- | --- |
| FC-001 | `/uwb/position` が未受信 | zero velocity を publish する |
| FC-002 | `/zed/heading` が未受信 | zero velocity を publish する |
| FC-003 | `/uwb/position` の `valid == false` | zero velocity を publish する |
| FC-004 | `/zed/heading` の `valid == false` | zero velocity を publish する |
| FC-005 | `position_timeout` を超えて UWB 位置が更新されない | zero velocity を publish する |
| FC-006 | `heading_timeout` を超えて ZED heading が更新されない | zero velocity を publish する |
| FC-007 | parameter validation に失敗する | 起動時に error とし、制御を開始しない |
| FC-008 | UWB 位置または yaw が finite value でない | 該当サンプルを invalid として扱い、zero velocity を publish する |

## 10. Safety Requirements

| ID | Requirement |
| --- | --- |
| SR-001 | sensor timeout または invalid data 時は必ず zero velocity を publish すること。 |
| SR-002 | `linear.x` と `linear.y` は `[-max_linear_speed, max_linear_speed]` に制限すること。 |
| SR-003 | `angular.z` は `[-max_angular_speed, max_angular_speed]` に制限すること。 |
| SR-004 | parameter validation に失敗した状態で制御を開始しないこと。 |
| SR-005 | 実機試験では外部停止手段を用意した状態で実施すること。 |

## 11. 実機確認観点

### 11.1 確認する topic

| Topic | 確認内容 |
| --- | --- |
| `/uwb/position` | `x_m`, `y_m`, `valid` の安定性と目標近傍の揺れ |
| `/zed/heading` | `robot_yaw_rad`, `valid` の安定性と yaw drift |
| `/rov_cmd_vel` | `linear.x`, `linear.y`, `angular.z` の符号、飽和、目標近傍の反転 |
| `rov/motors` | 速度指令に対する motor command の変化 |

### 11.2 記録する値

| Value | 目的 |
| --- | --- |
| `current_x` | UWB x 位置の確認 |
| `current_y` | UWB y 位置の確認 |
| `current_yaw` | ZED yaw の確認 |
| `error_world_x` | x 方向誤差の確認 |
| `error_world_y` | y 方向誤差の確認 |
| `yaw_error` | yaw 誤差の確認 |
| `cmd_linear_x` | 前後速度指令の確認 |
| `cmd_linear_y` | 横速度指令の確認 |
| `cmd_angular_z` | yaw 角速度指令の確認 |

### 11.3 実機テスト条件

| ID | Test | Pass Condition |
| --- | --- | --- |
| VT-001 | 目標位置から離れた状態で起動する | 目標方向へ `linear.x/y` が出る |
| VT-002 | 目標 yaw と異なる向きで起動する | 目標 yaw へ近づく向きに `angular.z` が出る |
| VT-003 | 位置が `x_tolerance/y_tolerance` 内に入る | 並進速度指令が `0.0` になる |
| VT-004 | yaw が `yaw_tolerance` 内に入る | yaw 角速度指令が `0.0` になる |
| VT-005 | UWB position を一時的に止める、または invalid にする | zero velocity を publish する |
| VT-006 | ZED heading を一時的に止める、または invalid にする | zero velocity を publish する |
| VT-007 | tolerance を広げて目標近傍の挙動を確認する | `/rov_cmd_vel` の短周期反転が減る |

## 12. 未決事項

| ID | Item | Memo |
| --- | --- | --- |
| TBD-002 | `linear.y` の正方向 | `locomotion_core/rover_velocity` の実機座標定義と合わせる |
| TBD-003 | UWB 座標系と ZED yaw のゼロ方向 | anchor 座標系の x/y と `robot_yaw_rad == 0` の方向を実機で合わせる |
| TBD-004 | UWB 位置の平滑化 | 初期仕様では controller 内平滑化なし。必要なら移動平均 parameter を追加する |
