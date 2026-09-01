# aruco_distance_publisher 要求仕様書

Version: 0.2 draft
Date: 2026-09-01
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

対象ノードは、ZED data hub がpublishするZED画像topicをsubscribeし、ArUcoマーカーを検出し、OpenCV camera frameにおけるマーカーの相対位置と、画像内の中心誤差を `/aruco/distance` にpublishする。

本nodeは速度指令を生成しない。ドッキング制御、速度制限、状態遷移、motor command変換は別nodeの責務とする。

移行後、本nodeはZED camera deviceを直接openしない。ZED device accessは `zed_wrapper_data_hub` が起動する公式 `zed_wrapper` に集約する。

## 4. Subscribe Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/zed2i/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` | ArUco検出に使用するrectified RGB image |
| `/zed2i/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/msg/CameraInfo` | camera intrinsic parameter取得候補 |

topic名はROS parameterで変更可能とする。初期defaultは、`zed_wrapper_data_hub` の実機確認済みtopicに合わせる。

`CameraInfo` のsubscribeは初期実装ではdefault有効とする。ZED wrapperのrectified image topicを使うため、対応するCameraInfoを使ってpose推定する。

## 5. Publish Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/aruco/distance` | `aruco_interfaces/msg/ArucoDistance` | 検出したArUcoマーカーの相対位置、距離、姿勢、画像中心誤差をpublishする |

### 5.1 Message Fields

| Field | Type | Requirement |
| --- | --- | --- |
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
| --- | --- | --- | --- | --- |
| `image_topic` | `string` | `/zed2i/zed_node/rgb/color/rect/image` | non-empty | ArUco検出に使用するimage topic |
| `camera_info_topic` | `string` | `/zed2i/zed_node/rgb/color/rect/camera_info` | empty or topic name | CameraInfoをsubscribeする場合のtopic |
| `use_camera_info` | `bool` | `true` | `true` or `false` | calibration fileではなくCameraInfoを使用する |
| `camera_side` | `string` | `left` | `left` または `right` | calibration file選択用。side-by-side画像分割には使用しない |
| `marker_length` | `float` | `0.168` | `> 0.0` | ArUcoマーカー一辺の長さ[m] |

### 6.1 Parameter Requirements

| ID | Requirement |
| --- | --- |
| PR-001 | `image_topic` が空文字の場合、起動時にvalidation errorとすること。 |
| PR-002 | `camera_info_topic` は空文字またはtopic名を許容すること。 |
| PR-003 | `use_camera_info == false` の場合、`camera_side` が `left` または `right` 以外なら起動時にvalidation errorとすること。 |
| PR-004 | `marker_length <= 0.0` の場合、起動時にvalidation errorとすること。 |
| PR-005 | `use_camera_info == false` の場合、`camera_side` に対応するcamera calibration fileを読み込むこと。 |
| PR-006 | `use_camera_info == true` の場合、CameraInfoを受信するまでpose推定を行わないこと。 |
| PR-007 | parameterはlaunch引数またはROS parameterで設定できること。 |

## 7. State Machine / Control Flow

本nodeは明示的なstate machineを持たない。image callbackにより画像受信、検出、publishを行う。

```text
STARTUP
  -> validate parameters
  -> load camera calibration or wait for CameraInfo
  -> create image subscription
  -> optional create CameraInfo subscription
  -> IMAGE_CALLBACK

IMAGE_CALLBACK
  -> receive image message
  -> convert ROS image to OpenCV image
  -> convert to grayscale
  -> detect ArUco markers
  -> estimate marker pose
  -> calculate message fields
  -> publish /aruco/distance for each detected marker
```

### 7.1 Startup Requirements

| ID | Requirement |
| --- | --- |
| CF-001 | node起動時に `image_topic`, `camera_info_topic`, `use_camera_info`, `camera_side`, `marker_length` をvalidateすること。 |
| CF-002 | `use_camera_info == false` の場合、`camera_side` に対応する calibration fileをsource treeまたはinstall treeから探索すること。 |
| CF-003 | `use_camera_info == false` の場合、calibration fileから `cameraMatrix` と `distCoeffs` を読み込むこと。 |
| CF-004 | `use_camera_info == true` の場合、CameraInfo messageからcamera matrixとdistortion coefficientsを取得すること。 |
| CF-005 | `image_topic` の `sensor_msgs/msg/Image` subscriptionを作成すること。 |
| CF-006 | image subscription は sensor data QoS を使用すること。 |
| CF-007 | `camera_info_topic` が空文字でない場合、`sensor_msgs/msg/CameraInfo` subscriptionを作成できること。 |
| CF-008 | CameraInfo subscription は sensor data QoS を使用すること。 |
| CF-009 | ZED camera deviceをOpenCV `VideoCapture(0)` で直接openしないこと。 |

### 7.2 Detection Loop Requirements

| ID | Requirement |
| --- | --- |
| CF-010 | image messageを受信した場合のみ検出処理を行うこと。 |
| CF-011 | ROS image messageをOpenCV imageへ変換すること。 |
| CF-012 | ZED wrapperのrectified single image topicを入力とし、side-by-side画像分割は行わないこと。 |
| CF-013 | ArUco dictionaryは `DICT_5X5_50` を使用すること。 |
| CF-014 | marker pose推定には `marker_length`、`cameraMatrix`、`distCoeffs` を使用すること。 |
| CF-015 | calibrationが未確定の場合、そのimageではpose推定とpublishを行わないこと。 |
| CF-016 | markerが検出されない場合、そのimageでは `/aruco/distance` をpublishしないこと。 |
| CF-017 | 複数markerが検出された場合、検出された各markerについて `ArucoDistance` をpublishすること。 |

### 7.3 Coordinate Requirements

| ID | Requirement |
| --- | --- |
| CR-001 | published poseはOpenCV camera frameに従うこと。 |
| CR-002 | `x` はcamera右方向を正とすること。 |
| CR-003 | `y` はcamera下方向を正とすること。 |
| CR-004 | `z` はcamera前方向を正とすること。 |
| CR-005 | `normalized_center_error` はmarker中心が画像中心のとき `0.0` とすること。 |
| CR-006 | `normalized_center_error` はmarker中心が画像右側のとき正、左側のとき負とすること。 |
| CR-007 | 入力画像がrectified imageの場合、対応するrectified camera calibrationを使用すること。 |

## 8. Failure Cases

| ID | Failure Case | Required Behavior |
| --- | --- | --- |
| FC-001 | `image_topic` が空文字 | 起動時にvalidation errorとする |
| FC-002 | `camera_side` が不正 | `use_camera_info == false` の場合は起動時にvalidation errorとする |
| FC-003 | `marker_length <= 0.0` | 起動時にvalidation errorとする |
| FC-004 | calibration fileが見つからない | `use_camera_info == false` の場合は起動失敗とする |
| FC-005 | calibration fileを読み込めない | `use_camera_info == false` の場合は起動失敗とする |
| FC-006 | CameraInfoが未受信 | `use_camera_info == true` の場合はpose推定とpublishを行わない |
| FC-007 | image message変換に失敗 | warn logを出し、そのimageではpublishしない |
| FC-008 | marker未検出 | publishしない |
| FC-009 | 複数marker検出 | 各markerについてpublishする |
| FC-010 | image topicが途切れる | 古い検出値を再publishしない |

## 9. Safety Requirements

| ID | Requirement |
| --- | --- |
| SR-001 | marker未検出時に古い検出値を再publishしないこと。 |
| SR-002 | image未受信時またはimage変換失敗時に推定値を捏造してpublishしないこと。 |
| SR-003 | calibration未確定時に無補正または不明なcamera parameterでpose推定しないこと。 |
| SR-004 | 本nodeは速度指令やmotor commandをpublishしないこと。 |
| SR-005 | 本nodeはZED camera deviceを直接openしないこと。 |


## 10. 設計仕様書作成時の注意

AIが設計仕様書を作成する場合、以下を守ること。

1. 本nodeの責務を「ArUco検出結果のpublish」に限定する。
2. 後段controllerの速度制御仕様を本nodeに混ぜない。
3. `/aruco/distance` のmessage fieldと座標系を明確にする。
4. marker未検出時に古い値をpublishしない仕様を維持する。
5. 入力画像topicとcalibration sourceの関係を明記する。
6. ZED camera device accessは `zed_wrapper_data_hub` 側の責務とし、本nodeには含めない。

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
