# ArUco 距離ゲート式ドッキングコントローラ仕様

## 目的

この文書は、ArUcoドッキングシステム用の実験的なドッキングコントローラを定義する。

現在のスコープでは、ロボットはすでにカメラがArUcoマーカーを検出できる範囲にいるものとする。長距離ナビゲーション、UWBナビゲーション、障害物回避、マーカー探索、カメラ省電力制御は、将来のシステム上位フェーズとして扱う。このコントローラは、ArUco観測が得られた後の安定したドッキングだけに集中する。

従来のコントローラは、前後移動、横移動、yaw回転を同時に指令できた。実機テストでは、yawと並進を混ぜると不安定化したり、フリーズすることがあった。そのため、このコントローラではマーカーとの距離に応じて使う運動軸を切り替える。

```text
遠い:     前進 + 弱いマーカー中心合わせyaw
近い:     横移動とyaw補正を許可
視野端:   並進を止め、yawだけで視野復帰
最終距離: 停止
```

最初の実装はシンプルに保つ。遠い間は完全な姿勢合わせを狙わない。マーカーを見失わずにその場回転できる程度まで近づいたら、yawと横移動を使い始める。

## スコープ

対象:

```text
- ArUcoマーカーがすでに検出できる位置からドッキングする。
- ドッキング中にマーカーを見失わない。
- ドッキングステーションとの衝突を避ける。
- yaw + 前進の不安定挙動を避ける。
- マーカータイムアウトや復帰できない視野リスク時に安全停止する。
- 水平方向の視野リスクから、前進せずに復帰する。
```

対象外:

```text
- ドッキングエリアまでのUWBナビゲーション。
- 障害物検知と回避。
- 見えていないマーカーの探索。
- 省電力のためのカメラノードON/OFF。
- 衝突境界内からの自動復帰。
```

想定する上位システムの流れ:

```text
UWB / navigation / obstacle フェーズでロボットをドッキングステーション付近まで移動する。
ArUcoマーカーが見える。
このドッキングコントローラが引き継ぐ。
```

## 目標距離

```text
target_distance = 1.0 m
align_distance  = 1.2 m
minimum_safe_z = 1.0 m
```

用語:

```text
aruco_distance = msg.distance = sqrt(x^2 + y^2 + z^2)
aruco_z        = msg.z        = ロボット前方軸の距離
```

`target_distance` は最終ドッキングのユークリッド距離である。`align_distance` は、yaw補正と横移動を許可し始めるユークリッド距離である。`minimum_safe_z` は、前方軸に対する強制安全境界である。

このドッキング手順では、`minimum_safe_z` を `align_distance` と同じ値にしてはいけない。`align_distance` は通常の遠距離接近を止めて姿勢合わせを始める距離であり、`minimum_safe_z` は前進が危険になる距離である。両方を1.2mにすると、1.2m地点から1.0mのドッキング距離まで最終前進できなくなる。

alignとtargetの判定には、ArUcoメッセージの `distance` フィールドを使う。

```text
distance = sqrt(x^2 + y^2 + z^2)
```

実際の `align_distance` は実機テストで調整する。マーカー検出が安定する程度に近く、かつドッキング前にyawと横ズレを補正する余裕が残る距離にする。

衝突安全には `aruco_distance` ではなく `aruco_z` を使う。最初の実装では、`aruco_distance` と `aruco_z` の両方をログに残す。

## 衝突ガード

ロボットが最終ドッキング距離付近にいる場合、ドッキングステーションに衝突する可能性がある。この境界内では前進指令を絶対に出さない。

基本ルール:

```text
if aruco_z <= minimum_safe_z:
    linear.x = 0.0
```

安全境界には `aruco_distance` ではなく `aruco_z` を使う。衝突リスクは主にロボットの前方軸に沿って発生するためである。横ズレや高さズレが大きい場合、ユークリッド距離は衝突安全の判定として紛らわしい。

## ドッキング完了条件

`DOCKED` に入るのは、前方距離、横ズレ、マーカーyawがすべて許容範囲内のときだけにする。

```text
error_z = aruco_z - target_z
error_x = aruco_x - target_x
error_yaw = wrap_pi(aruco_yaw - target_yaw)

abs(error_z) < z_tolerance
and abs(error_x) < x_tolerance
and abs(error_yaw) < yaw_tolerance -> DOCKED
```

最終ドッキング完了条件にユークリッド距離 `aruco_distance` だけを使ってはいけない。前方軸や横方向のズレを隠してしまう可能性がある。

最初の実装では、この境界に入ったとき、姿勢誤差が許容範囲内なら `DOCKED`、許容範囲外なら `HOLD` に遷移する。

```text
if docked condition is true:
    transition to DOCKED

if aruco_z <= minimum_safe_z
and pose errors are not acceptable:
    transition to HOLD
```

この境界から後退するには、明示的な復帰モードまたは手動操作が必要である。最初の実装では自動後退しない。

任意の復帰候補:

```text
BACK_OUT_RECOVERY
```

この復帰は、基本ドッキングが安定した後に追加できる。ロボットがドッキングステーションに近すぎ、かつ姿勢誤差が許容範囲外の場合だけ有効化する。復帰時はyawや横移動を使わず、十分に再整列できる距離まで低速で後退する。

後退が安全であることを上位システムが確認できる場合、またはテストオペレータが明示的に許可した場合以外、この復帰を有効化してはいけない。

## 座標前提

このコントローラは以下をsubscribeする。

```text
/aruco/distance
type: aruco_interfaces/msg/ArucoDistance
```

使用するフィールド:

```text
x        カメラ座標でマーカーが右方向にある量 [m]
y        カメラ座標でマーカーが下方向にある量 [m]
z        カメラ座標でマーカーが前方にある量 [m]
distance カメラからマーカーまでのユークリッド距離 [m]
theta    水平方向のbearing、atan2(x, z) [rad]
yaw      rvecから推定したマーカー面のyaw [rad]
```

ロボット指令の対応:

```text
Twist.linear.x   前後移動
Twist.linear.y   横移動
Twist.angular.z  yaw回転
```

## 視野ガードと復帰

ドッキング中はマーカーを見失ってはいけない。コントローラは以下を計算する。

```text
theta_x = atan2(aruco_x, aruco_z)
theta_y = atan2(aruco_y, aruco_z)
```

`theta_x` は画像上の水平方向ズレの近似である。`theta_y` は垂直方向ズレの近似である。このローバーは垂直方向の誤差を直接補正できないが、`theta_y` はマーカーが視野外へ出そうか判断する警告として有用である。

推奨するガード:

```text
if abs(theta_x) > theta_x_stop_limit:
    RECOVER_VISIBILITY に遷移

if abs(theta_y) > theta_y_stop_limit:
    HOLD に遷移

if abs(theta_x) > theta_x_slow_limit or abs(theta_y) > theta_y_slow_limit:
    前進速度を落とす
```

水平方向のズレは、ロボットをyaw回転させることでマーカーを画像中心へ戻せる可能性がある。垂直方向のズレはこのローバーでは直接補正できないため、`theta_y` のstop limit違反は停止条件として扱う。

`RECOVER_VISIBILITY` では前進や横移動を指令してはいけない。`theta_x` が安全な範囲に戻るまで、その場回転だけを使う。

## ステート

### WAIT_FOR_MARKER

最近のArUco検出が得られるまで停止する。

このステートは長距離マーカー探索を担当しない。上位システムまたはセンサパイプラインから有効なマーカー検出が届くのを待つだけである。

指令:

```text
linear.x = 0.0
linear.y = 0.0
angular.z = 0.0
```

遷移:

```text
recent marker detection and aruco_z <= minimum_safe_z -> HOLD or DOCKED
recent marker detection and aruco_distance <= align_distance -> NEAR_ALIGN
recent marker detection and aruco_distance > align_distance -> FAR_GUIDED_APPROACH
```

この初期振り分けは重要である。ロボットがすでに `align_distance` 内からドッキングを開始する可能性がある。その場合は `FAR_GUIDED_APPROACH` を通らず、直接 `NEAR_ALIGN` に入る。ただし、すでに前方軸の安全境界内にいる場合は除く。

### FAR_GUIDED_APPROACH

マーカーから遠い間は、前進しながら弱いマーカー中心合わせyawだけを使う。この範囲では横移動やマーカー面yaw合わせをしない。

長距離ではyaw推定がノイズを持ちやすく、強い回転でマーカーが視野外へ出る可能性がある。このステートのyaw指令は、マーカー `yaw` ではなく `theta_x` を使う。

`theta_x` はカメラをマーカー中心へ向ける量である。マーカー `yaw` はマーカー面の向きを推定する値なので、ロボットが近づくまで使わない。

指令:

```text
linear.x = far_approach_speed
linear.y = 0.0
angular.z = clamp(kp_far_center * theta_x,
                  -max_far_center_speed,
                  max_far_center_speed)
```

遷移:

```text
aruco_z <= minimum_safe_z -> HOLD or DOCKED
aruco_distance <= align_distance -> NEAR_ALIGN
abs(theta_x) > theta_x_stop_limit -> RECOVER_VISIBILITY
abs(theta_y) > theta_y_stop_limit -> HOLD
marker lost -> WAIT_FOR_MARKER
```

任意の減速:

```text
if visibility slow guard violated:
    linear.x = reduced_far_approach_speed
```

### NEAR_ALIGN

マーカーを見失わずに局所補正できる程度まで近づいたら、横ズレとマーカーyawを合わせる。

このステートでは横移動とyawを指令できる。ただし、yaw + 前進で不安定化したため、最初の実装では `linear.x = 0.0` を維持する。

`NEAR_ALIGN` に入った直後は、マーカー中心を向く動作を優先する。`theta_x` が `theta_x_slow_limit` を超えている間は、横移動とマーカー面yaw補正を止め、`theta_x` によるその場回転だけを行う。

指令:

```text
lateral_error = aruco_x - target_x
yaw_error = wrap_pi(aruco_yaw - target_yaw)

linear.x = 0.0

if abs(theta_x) > theta_x_slow_limit:
    linear.y = 0.0
    angular.z = clamp_with_min(-kp_visibility_recovery * theta_x,
                               min_visibility_recovery_speed,
                               max_visibility_recovery_speed)
    return

linear.y = clamp(-kp_lateral * lateral_error,
                 -max_lateral_align_speed,
                 max_lateral_align_speed)
angular.z = clamp(kp_yaw * yaw_error,
                  -max_yaw_align_speed,
                  max_yaw_align_speed)
```

遷移:

```text
aruco_z <= minimum_safe_z
and docked condition is true -> DOCKED

aruco_z <= minimum_safe_z
and docked condition is not true -> HOLD

abs(lateral_error) < x_tolerance
and abs(yaw_error) < yaw_tolerance -> FINAL_APPROACH

aruco_distance > align_distance + align_hysteresis -> FAR_GUIDED_APPROACH
abs(theta_x) > theta_x_stop_limit -> RECOVER_VISIBILITY
abs(theta_y) > theta_y_stop_limit -> HOLD
marker lost -> WAIT_FOR_MARKER
```

もし横移動とyawの同時指令でもフリーズする場合は、このステートを以下に分ける。

```text
NEAR_LATERAL_ALIGN
NEAR_YAW_ALIGN
```

### FINAL_APPROACH

`align_distance` から `target_z` へ向かって前進する。

このステートに入る前に、ロボットは横方向とマーカー面yawがだいたい合っている想定である。前進速度は `FAR_GUIDED_APPROACH` より低くする。

最初の実装では前進だけを使う。ログで安全が確認できたら、将来、小さなyaw保持や横方向保持を追加できる。

指令:

```text
linear.x = final_approach_speed
linear.y = 0.0
angular.z = 0.0
```

遷移:

```text
docked condition is true -> DOCKED

aruco_z <= minimum_safe_z
and docked condition is not true -> HOLD

if abs(lateral_error) > final_x_realign_threshold
or abs(yaw_error) > final_yaw_realign_threshold:
    return to NEAR_ALIGN

abs(theta_x) > theta_x_stop_limit -> RECOVER_VISIBILITY
abs(theta_y) > theta_y_stop_limit -> HOLD
marker lost -> WAIT_FOR_MARKER
```

### RECOVER_VISIBILITY

マーカーがカメラ視野の水平方向端に近づいたときに復帰する。

水平視野リスクで永久停止してしまうのは保守的すぎる。マーカーがまだ検出できているなら、その場回転でマーカーを画像中心へ戻せる可能性がある。このステートでは前進も横移動もしない。

指令:

```text
linear.x = 0.0
linear.y = 0.0
angular.z = clamp_with_min(-kp_visibility_recovery * theta_x,
                           min_visibility_recovery_speed,
                           max_visibility_recovery_speed)
```

実機テストでは、このローバーの指令系では符号が逆だった。そのため復帰指令では意図的に `theta_x` を反転する。

遷移:

```text
aruco_z <= minimum_safe_z
and docked condition is true -> DOCKED

aruco_z <= minimum_safe_z
and docked condition is not true -> HOLD

abs(theta_y) > theta_y_stop_limit -> HOLD

abs(theta_x) < theta_x_slow_limit
and aruco_distance <= align_distance -> NEAR_ALIGN

abs(theta_x) < theta_x_slow_limit
and aruco_distance > align_distance -> FAR_GUIDED_APPROACH

marker lost -> WAIT_FOR_MARKER
```

`RECOVER_VISIBILITY` にはヒステリシスを持たせる。`theta_x_stop_limit` で入り、`theta_x_slow_limit` 未満になってから抜ける。これによりstop閾値付近での細かい状態切り替えを避ける。

### HOLD

現在のステートを続けるべきでない時に短時間停止する。`HOLD` は永久停止ではなく、一時停止である。最初の実装では一定時間だけ停止し、その後、マーカーデータが正常なら適切なステートへ復帰し、正常でなければ `WAIT_FOR_MARKER` に戻る。

指令:

```text
linear.x = 0.0
linear.y = 0.0
angular.z = 0.0
```

遷移:

```text
hold elapsed < hold_duration -> remain in HOLD
hold elapsed >= hold_duration and marker healthy and docked condition is true -> DOCKED
hold elapsed >= hold_duration and marker healthy and aruco_z <= minimum_safe_z -> HOLD
hold elapsed >= hold_duration and marker healthy and aruco_distance <= align_distance -> NEAR_ALIGN
hold elapsed >= hold_duration and marker healthy and aruco_distance > align_distance -> FAR_GUIDED_APPROACH
hold elapsed >= hold_duration and marker not healthy -> WAIT_FOR_MARKER
```

### BACK_OUT_RECOVERY

将来テスト用の任意ステート。最初の自動実装には含めない。

ロボットが `minimum_safe_z` 内にいて、ドッキング完了と呼べるほど姿勢が良くなく、かつオペレータまたは上位supervisorが後退安全を確認した場合に使う。

指令:

```text
linear.x = -back_out_speed
linear.y = 0.0
angular.z = 0.0
```

遷移:

```text
aruco_distance >= back_out_target_distance -> NEAR_ALIGN
marker lost -> WAIT_FOR_MARKER
operator disables recovery -> HOLD
```

安全制約:

```text
back_out_speed must be low
do not command yaw while backing out
do not command lateral motion while backing out
stop immediately on marker timeout or upstream obstacle warning
```

### DOCKED

ロボットを停止する。

指令:

```text
linear.x = 0.0
linear.y = 0.0
angular.z = 0.0
```

## 初期パラメータ

実機テスト開始用の値であり、最終調整値ではない。

```yaml
target_distance: 1.0
align_distance: 1.2
minimum_safe_z: 1.0
align_hysteresis: 0.10
back_out_target_distance: 1.2

target_x: 0.0
target_z: 1.0
target_yaw: 0.0

x_tolerance: 0.05
z_tolerance: 0.03
yaw_tolerance: 0.06

final_x_realign_threshold: 0.08
final_yaw_realign_threshold: 0.10

theta_x_slow_limit: 0.15
theta_x_stop_limit: 0.25
theta_y_slow_limit: 0.15
theta_y_stop_limit: 0.25

kp_lateral: 0.4
kp_yaw: 0.6
kp_far_center: 0.3
kp_visibility_recovery: 0.3

far_approach_speed: 0.3
reduced_far_approach_speed: 0.3
final_approach_speed: 0.3
back_out_speed: 0.05
min_far_center_speed: 0.3
max_far_center_speed: 0.95
min_visibility_recovery_speed: 0.3
max_visibility_recovery_speed: 0.95
min_lateral_align_speed: 0.3
max_lateral_align_speed: 0.95
min_yaw_align_speed: 0.3
max_yaw_align_speed: 0.95

detection_timeout: 0.5
hold_duration: 0.8
control_rate: 20.0
```

## 安全要件

- ArUco検出がタイムアウトしたら停止する。
- `aruco_z <= minimum_safe_z` の時は前進指令を出さない。
- 最初の実装では、`minimum_safe_z` 内から自動後退しない。
- 将来 `BACK_OUT_RECOVERY` を追加する場合は、オペレータまたは上位supervisorの許可がある場合だけ有効化する。
- `FAR_GUIDED_APPROACH` では弱い `theta_x` 中心合わせyawだけを使い、遠距離ではマーカー `yaw` を使わない。
- `FINAL_APPROACH` の速度は `FAR_GUIDED_APPROACH` より低くする。
- 最初の実装では `NEAR_ALIGN` 中に前進しない。
- 視野ガード違反は姿勢合わせより優先する。
- 水平方向の視野リスクでは、yawだけの `RECOVER_VISIBILITY` を使う。
- 垂直方向の視野リスクでは `HOLD` で停止する。
- `RECOVER_VISIBILITY` 中は前進も横移動も指令しない。
- `HOLD` は有限時間にする。`hold_duration` 後、適切なステートに復帰するか `WAIT_FOR_MARKER` に戻る。
- 各実験で `/aruco/distance` と `/rov_cmd_vel` を記録する。

## 検証計画

ログ記録:

```bash
ros2 run aruco_dist_ctrl aruco_cmd_logger --ros-args \
  -p output_dir:=/tmp/aruco_docking_logs \
  -p log_rate:=20.0
```

最小実験:

```text
1. FAR_GUIDED_APPROACHのみ
2. FAR_GUIDED_APPROACH + NEAR_ALIGN。NEAR_ALIGN中はlinear.x無効
3. FINAL_APPROACHを追加して1.0mまで進む
4. 水平方向の視野端ケースを作り、RECOVER_VISIBILITYを確認する
5. NEAR_ALIGNでフリーズする場合、横移動専用とyaw専用のサブステートに分ける
```

各実験で記録すること:

```text
- マーカーを見失わなかったか
- yaw指令でフリーズしたか
- lateral + yaw同時指令でフリーズしたか
- wheel commandが飽和したか
- 停止時のaruco_z
- 停止時のaruco_distance
- 停止時のaruco_xとaruco_y
- 停止時のaruco_yaw
```

## 今後の作業

- UWBナビゲーション、障害物処理、ArUco取得、ArUcoドッキングを切り替える上位supervisorを追加する。
- ドッキング挙動が安定した後に、省電力用のカメラactive/inactive制御を追加する。
- 画像余白で視野判定できるよう、`ArucoDistance` にマーカー中心pixelフィールドを追加する。
- より滑らかな連続ドッキングのため、硬いステート遷移を距離依存ブレンドに置き換える。
- 安全がログで確認できたら、`FINAL_APPROACH` 中に小さな保持補正を追加する。
- 後方安全を確認できるようになったら、ガード付き `BACK_OUT_RECOVERY` を追加する。
- `/rov_cmd_vel` にslew rate limitを追加する。
- `rover_velocity` で個別モータ制限前にwheel commandを正規化する。
