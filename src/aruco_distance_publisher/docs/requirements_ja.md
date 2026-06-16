# aruco_distance_publisher 要求仕様書

Version: 0.1 draft
Date: 2026-06-16
Status: draft

## 1. 目的

本要求仕様書は、`aruco_distance_publisher/aruco_distance_publisher` において、カメラ画像からArUcoマーカーを検出し、ドッキング制御に必要なマーカー位置・距離・姿勢・画像中心誤差をpublishするための要求を定義する。

本要求は、実装前にAIへ設計仕様整理を依頼するための「人間が定義した要求仕様」として扱う。

## 2. 対象ノード

| 項目 | 内容 |
| --- | --- |
| Package | `aruco_distance_publisher` |
| Node | `aruco_distance_publisher` |
| Main file | `public/mobile_robot_system/src/aruco_distance_publisher/aruco_distance_publisher/distance_publisher/aruco_distance_publisher_node.py` |
| Related message | `public/mobile_robot_system/src/aruco_interfaces/msg/ArucoDistance.msg` |

## 3. ノードの目的

対象ノードは、ZED2カメラ画像からArUcoマーカーを検出し、OpenCV camera frameにおけるマーカーの相対位置と、画像内の中心誤差を `/aruco/distance` にpublishする。

本nodeは速度指令を生成しない。ドッキング制御、速度制限、状態遷移、motor command変換は別nodeの責務とする。

## 4. Subscribe Topic

なし。

本nodeはROS topicから画像をsubscribeしない。OpenCV `VideoCapture(0)` によりカメラデバイスから画像を取得する。

## 5. Publish Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/aruco/distance` | `aruco_interfaces/msg/ArucoDistance` | 検出したArUcoマーカーの相対位置、距離、姿勢、画像中心誤差をpublishする |

### 5.1 Message Fields

| Field | Type | Requirement |
| `id` | `int` | 検出したArUco marker IDを格納すること。 |
| `x` | `float` | OpenCV camera frameの右方向位置[m]を格納すること。 |
| `y` | `float` | OpenCV camera frameの下方向位置[m]を格納すること。 |
| `z` | `float` | OpenCV camera frameの前方向位置[m]を格納すること。 |
| `distance` | `float` | `sqrt(x^2 + y^2 + z^2)` によるマーカーまでの距離[m]を格納すること。 |
| `theta` | `float` | `atan2(x, z)` による水平bearing angle[rad]を格納すること。 |
| `yaw` | `float` | marker rvecから推定したmarker yaw[rad]を格納すること。 |
| `center_u` | `float` | 検出cornerから計算したmarker中心u座標[pixel]を格納すること。 |
| `center_v` | `float` | 検出cornerから計算したmarker中心v座標[pixel]を格納すること。 |
| `normalized_center_error` | `float` | 画像中心からの水平誤差を、画像半幅で正規化して格納すること。 |

## 6. Parameters

| Parameter | Type | Default | Range / Constraint | 用途 |
| `camera_side` | `string` | `left` | `left` または `right` | ZED2の左右どちらの画像を使うか選択する |
| `marker_length` | `float` | `0.168` | `> 0.0` | ArUcoマーカー一辺の長さ[m] |

### 6.1 Parameter Requirements

| ID | Requirement |
| PR-001 | `camera_side` が `left` または `right` 以外の場合、起動時にvalidation errorとすること。 |
| PR-002 | `marker_length <= 0.0` の場合、起動時にvalidation errorとすること。 |
| PR-003 | `camera_side` に対応するcamera calibration fileを読み込むこと。 |
| PR-004 | parameterはlaunch引数またはROS parameterで設定できること。 |

## 7. State Machine / Control Flow

本nodeは明示的なstate machineを持たない。timer callbackにより周期的に画像取得、検出、publishを行う。

```text
STARTUP
  -> validate parameters
  -> load camera calibration
  -> open camera device
  -> TIMER_LOOP

TIMER_LOOP
  -> read frame
  -> split selected camera image
  -> convert to grayscale
  -> detect ArUco markers
  -> estimate marker pose
  -> calculate message fields
  -> publish /aruco/distance for each detected marker
```

### 7.1 Startup Requirements

| ID | Requirement |
| CF-001 | node起動時に `camera_side` と `marker_length` をvalidateすること。 |
| CF-002 | `camera_side` に対応する calibration fileをsource treeまたはinstall treeから探索すること。 |
| CF-003 | calibration fileから `cameraMatrix` と `distCoeffs` を読み込むこと。 |
| CF-004 | camera device 0をOpenCVで開くこと。 |
| CF-005 | cameraを開けない場合は起動失敗とすること。 |

### 7.2 Detection Loop Requirements

| ID | Requirement |
| CF-010 | cameraからframeを取得できない場合、その周期ではpublishしないこと。 |
| CF-011 | ZED2のside-by-side画像から `camera_side` に対応する半分の画像を使用すること。 |
| CF-012 | ArUco dictionaryは `DICT_5X5_50` を使用すること。 |
| CF-013 | marker pose推定には `marker_length`、`cameraMatrix`、`distCoeffs` を使用すること。 |
| CF-014 | markerが検出されない場合、その周期では `/aruco/distance` をpublishしないこと。 |
| CF-015 | 複数markerが検出された場合、検出された各markerについて `ArucoDistance` をpublishすること。 |

### 7.3 Coordinate Requirements

| ID | Requirement |
| --- | --- |
| CR-001 | published poseはOpenCV camera frameに従うこと。 |
| CR-002 | `x` はcamera右方向を正とすること。 |
| CR-003 | `y` はcamera下方向を正とすること。 |
| CR-004 | `z` はcamera前方向を正とすること。 |
| CR-005 | `normalized_center_error` はmarker中心が画像中心のとき `0.0` とすること。 |
| CR-006 | `normalized_center_error` はmarker中心が画像右側のとき正、左側のとき負とすること。 |

## 8. Failure Cases

| ID | Failure Case | Required Behavior |
| FC-001 | `camera_side` が不正 | 起動時にvalidation errorとする |
| FC-002 | `marker_length <= 0.0` | 起動時にvalidation errorとする |
| FC-003 | calibration fileが見つからない | 起動失敗とする |
| FC-004 | calibration fileを読み込めない | 起動失敗とする |
| FC-005 | camera device 0を開けない | 起動失敗とする |
| FC-006 | frame取得に失敗 | error logを出し、その周期ではpublishしない |
| FC-007 | marker未検出 | publishしない |
| FC-008 | 複数marker検出 | 各markerについてpublishする |

## 9. Safety Requirements

| ID | Requirement |
| SR-001 | marker未検出時に古い検出値を再publishしないこと。 |
| SR-002 | frame取得失敗時に推定値を捏造してpublishしないこと。 |
| SR-003 | calibration file読み込み失敗時に無補正または不明なcamera parameterでpose推定しないこと。 |
| SR-004 | 本nodeは速度指令やmotor commandをpublishしないこと。 |


## 10. 設計仕様書作成時の注意

AIが設計仕様書を作成する場合、以下を守ること。

1. 本nodeの責務を「ArUco検出結果のpublish」に限定する。
2. 後段controllerの速度制御仕様を本nodeに混ぜない。
3. `/aruco/distance` のmessage fieldと座標系を明確にする。
4. marker未検出時に古い値をpublishしない仕様を維持する。
5. calibration fileとcamera_sideの関係を明記する。

## 11. AIへの依頼文

要求仕様一覧:

この要求仕様に従って、ROS2ノード `aruco_distance_publisher` の仕様を整理してください。

レビュー観点:

1. 要求仕様との一致
2. Topic / Parameter の整合性
3. Control flow の明確さ
4. Failure case / Safety 対応
5. 実機テスト観点
6. Node責務の分離

注意:

AIによる勝手な仕様追加を避け、`aruco_distance_publisher` の責務をArUco検出結果のpublishに限定してください。
