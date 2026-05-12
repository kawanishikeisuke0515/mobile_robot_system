# Camera Calibration Tools

Utilities for collecting ZED2 chessboard images and generating OpenCV calibration parameters.

## Capture Images
```bash
python3 tools/calibration/chess_capture.py
```

Capture right-camera images:

```bash
python3 tools/calibration/chess_capture.py --eye right
```

Use a different chessboard inner-corner pattern:

```bash
python3 tools/calibration/chess_capture.py --pattern-cols 9 --pattern-rows 6
```

Use a different capture resolution:

```bash
python3 tools/calibration/chess_capture.py --width 4416 --height 1242
```

Captured images are saved to:

```text
calibration_data/sample_images/
```

## Run Calibration
```bash
python3 tools/calibration/calibrate_camera.py
```

Run right-camera calibration:

```bash
python3 tools/calibration/calibrate_camera.py --eye right
```

Use a different chessboard pattern or square size:

```bash
python3 tools/calibration/calibrate_camera.py --pattern-cols 9 --pattern-rows 6 --square-size 0.025
```

`pattern-cols` and `pattern-rows` are the number of inner chessboard corners. `square-size` is the physical square side length in meters.

The calibration result is saved to both the calibration data directory and the ROS2 node runtime directory:

```text
calibration_data/calib_result_left.npz
calibration_data/calib_result_right.npz
src/aruco_distance_publisher/aruco_distance_publisher/distance_publisher/calib_result_left.npz
src/aruco_distance_publisher/aruco_distance_publisher/distance_publisher/calib_result_right.npz
```
