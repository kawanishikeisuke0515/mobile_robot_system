import argparse
import cv2
import os
import time
import numpy as np
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture ZED2 chessboard images for camera calibration."
    )
    parser.add_argument(
        "--eye",
        choices=("left", "right"),
        default="left",
        help="ZED2 camera side to capture.",
    )
    parser.add_argument(
        "--pattern-cols",
        type=int,
        default=9,
        help="Number of inner chessboard corners along columns.",
    )
    parser.add_argument(
        "--pattern-rows",
        type=int,
        default=6,
        help="Number of inner chessboard corners along rows.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=4416,
        help="Requested capture frame width.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1242,
        help="Requested capture frame height.",
    )
    return parser.parse_args()


args = parse_args()
if args.width <= 0 or args.height <= 0:
    raise ValueError("capture width and height must be greater than 0")

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"requested frame size = {args.width}x{args.height}")
print(f"actual frame size = {actual_width}x{actual_height}")

pattern_size = (args.pattern_cols, args.pattern_rows)
if pattern_size[0] <= 0 or pattern_size[1] <= 0:
    raise ValueError("pattern size must be greater than 0")
img_count = 0
max_images = 25

flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)

repo_root = Path(__file__).resolve().parents[2]
save_dir = repo_root / "calibration_data" / "sample_images"
os.makedirs(save_dir, exist_ok=True)

# Minimum interval between saved frames
min_save_interval_sec = 2.0

# Thresholds to avoid saving nearly identical frames
position_change_threshold = 25.0
area_change_threshold = 0.015

last_save_time = 0.0
last_saved_metrics = None


def order_quad_points(pts):
    """
    Sort 4 corner points into:
    [top-left, top-right, bottom-right, bottom-left]
    """
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).reshape(-1)

    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(s)]
    ordered[2] = pts[np.argmax(s)]
    ordered[1] = pts[np.argmin(d)]
    ordered[3] = pts[np.argmax(d)]
    return ordered


def polygon_area(pts):
    """Compute polygon area using shoelace formula"""
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def quad_edge_ratio(quad):
    """
    Compute ratio between longest and shortest edge.
    Used to estimate perspective distortion.
    """
    edges = []
    for i in range(4):
        p1 = quad[i]
        p2 = quad[(i + 1) % 4]
        edges.append(np.linalg.norm(p1 - p2))
    edges = np.array(edges)

    if np.min(edges) < 1e-6:
        return 999.0
    return float(np.max(edges) / np.min(edges))


def min_border_distance(pts, w, h):
    """Distance from chessboard corners to image border"""
    xs = pts[:, 0]
    ys = pts[:, 1]
    return float(min(xs.min(), ys.min(), w - xs.max(), h - ys.max()))


def classify(score):
    """Assign quality label"""
    if score >= 75:
        return "good"
    elif score >= 50:
        return "maybe"
    else:
        return "bad"


def evaluate_frame(gray, corners, pattern_size):
    """
    Evaluate chessboard image quality after detection.
    Returns:
        score, label, metrics(dict)
    """
    h, w = gray.shape
    sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
    brightness = gray.mean()

    corners_refined = cv2.cornerSubPix(
        gray,
        corners.astype(np.float32),
        (11, 11),
        (-1, -1),
        criteria
    )

    corners_2d = corners_refined.reshape(-1, 2)

    nx, ny = pattern_size

    quad = np.array([
        corners_2d[0],
        corners_2d[nx - 1],
        corners_2d[-1],
        corners_2d[-nx]
    ], dtype=np.float32)

    quad = order_quad_points(quad)

    board_area = polygon_area(quad)
    board_area_ratio = board_area / float(w * h)

    border_margin = min_border_distance(corners_2d, w, h)
    perspective_ratio = quad_edge_ratio(quad)

    center_x = float(np.mean(corners_2d[:, 0]))
    center_y = float(np.mean(corners_2d[:, 1]))

    score = 100.0
    reason = []

    # Blur penalty
    if sharpness < 30:
        score -= 35
        reason.append("very_blurry")
    elif sharpness < 60:
        score -= 20
        reason.append("blurry")
    elif sharpness < 100:
        score -= 8
        reason.append("slightly_soft")

    # Brightness penalty
    if brightness < 40 or brightness > 220:
        score -= 20
        reason.append("bad_brightness")
    elif brightness < 60 or brightness > 200:
        score -= 8
        reason.append("suboptimal_brightness")

    # Board size penalty
    if board_area_ratio < 0.05:
        score -= 30
        reason.append("board_too_small")
    elif board_area_ratio < 0.10:
        score -= 15
        reason.append("board_small")
    elif board_area_ratio > 0.75:
        score -= 10
        reason.append("board_too_large")

    # Border proximity penalty
    if border_margin < 10:
        score -= 30
        reason.append("too_close_to_border")
    elif border_margin < 25:
        score -= 15
        reason.append("close_to_border")

    # Perspective penalty
    if perspective_ratio > 8.0:
        score -= 30
        reason.append("extreme_perspective")
    elif perspective_ratio > 5.0:
        score -= 15
        reason.append("strong_perspective")

    score = max(0.0, min(100.0, score))
    label = classify(score)

    metrics = {
        "sharpness": float(sharpness),
        "brightness": float(brightness),
        "board_area_ratio": float(board_area_ratio),
        "border_margin": float(border_margin),
        "perspective_ratio": float(perspective_ratio),
        "center_x": float(center_x),
        "center_y": float(center_y),
        "reason": ",".join(reason),
    }

    return score, label, metrics


def is_sufficiently_different(metrics, last_saved_metrics):
    """
    Avoid saving nearly identical frames.
    Compare board center movement and board area change.
    """
    if last_saved_metrics is None:
        return True

    dx = metrics["center_x"] - last_saved_metrics["center_x"]
    dy = metrics["center_y"] - last_saved_metrics["center_y"]
    dist = (dx * dx + dy * dy) ** 0.5

    area_diff = abs(
        metrics["board_area_ratio"] - last_saved_metrics["board_area_ratio"]
    )

    if dist >= position_change_threshold:
        return True

    if area_diff >= area_change_threshold:
        return True

    return False


while img_count < max_images:
    ret, frame = cap.read()
    if not ret:
        print("failed to read frame")
        time.sleep(1)
        continue

    height, width = frame.shape[:2]
    half_width = width // 2

    if args.eye == "left":
        mono_frame = frame[:, :half_width]
        eye_label = "left"
    else:
        mono_frame = frame[:, half_width:]
        eye_label = "right"

    gray = cv2.cvtColor(mono_frame, cv2.COLOR_BGR2GRAY)
    ret_cb, corners = cv2.findChessboardCorners(gray, pattern_size, flags)

    if not ret_cb:
        print(f"[NG] chessboard not found in {eye_label} image")
        time.sleep(0.5)
        continue

    score, label, metrics = evaluate_frame(gray, corners, pattern_size)

    print(
        f"[INFO] label={label} score={score:.1f} "
        f"sharp={metrics['sharpness']:.1f} "
        f"area={metrics['board_area_ratio']:.3f} "
        f"margin={metrics['border_margin']:.1f} "
        f"persp={metrics['perspective_ratio']:.2f} "
        f"reason={metrics['reason']}"
        f"raw frame shape = {frame.shape}"
        f"mono frame shape = {mono_frame.shape}"
        )

    if label != "good":
        time.sleep(0.5)
        continue

    now = time.time()

    if (now - last_save_time) < min_save_interval_sec:
        time.sleep(0.2)
        continue

    if not is_sufficiently_different(metrics, last_saved_metrics):
        print("[SKIP] too similar to previous saved image")
        time.sleep(0.5)
        continue

    raw_name = os.path.join(save_dir, f"calib_{eye_label}_{img_count:02d}.png")
    cv2.imwrite(raw_name, mono_frame)

    print(f"[OK] saved: {raw_name}")
    img_count += 1
    last_save_time = now
    last_saved_metrics = metrics.copy()

    time.sleep(1.0)

cap.release()
print("done")
