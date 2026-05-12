import argparse
import glob
import os
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate OpenCV calibration parameters from chessboard images."
    )
    parser.add_argument(
        "--eye",
        choices=("left", "right"),
        default="left",
        help="ZED2 camera side to calibrate.",
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
        "--square-size",
        type=float,
        default=0.025,
        help="Chessboard square size in meters.",
    )
    return parser.parse_args()


# =========================
# Configuration
# =========================
args = parse_args()
repo_root = Path(__file__).resolve().parents[2]
image_dir = repo_root / "calibration_data" / "sample_images"
image_pattern = f"calib_{args.eye}_*.png"
pattern_size = (args.pattern_cols, args.pattern_rows)
if pattern_size[0] <= 0 or pattern_size[1] <= 0:
    raise ValueError("pattern size must be greater than 0")
square_size = args.square_size
if square_size <= 0.0:
    raise ValueError("square size must be greater than 0")
out_name = f"calib_result_{args.eye}.npz"
out_paths = [
    repo_root / "calibration_data" / out_name,
    repo_root / "src" / "aruco_distance_publisher" / "aruco_distance_publisher" / "distance_publisher" / out_name,
]

criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)

# Prepare 3D object points
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []
used_files = []
img_size = None

image_paths = sorted(glob.glob(str(image_dir / image_pattern)))

if not image_paths:
    print(f"No {args.eye} calibration images found.")
    exit()

for fname in image_paths:
    img = cv2.imread(fname)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_size = gray.shape[::-1]

    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)
    if not found:
        continue

    # Sub-pixel refinement
    corners = cv2.cornerSubPix(
        gray,
        corners.astype(np.float32),
        (11, 11),
        (-1, -1),
        criteria
    )

    objpoints.append(objp)
    imgpoints.append(corners)
    used_files.append(fname)

if len(objpoints) < 5:
    print("Not enough valid images.")
    exit()

# Run calibration
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    img_size,
    None,
    None
)

print("=== Calibration Result ===")
print("RMS reprojection error =", ret)
print("\ncameraMatrix =")
print(cameraMatrix)
print("\ndistCoeffs =")
print(distCoeffs)

# Per-image reprojection error
print("\n=== Per-image error ===")
for i in range(len(objpoints)):
    projected, _ = cv2.projectPoints(
        objpoints[i],
        rvecs[i],
        tvecs[i],
        cameraMatrix,
        distCoeffs
    )

    err = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
    print(f"{os.path.basename(used_files[i])} : {err:.4f}")

# Save result for both calibration records and ROS2 runtime use.
print("\n=== Saved calibration files ===")
for out_path in out_paths:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        out_path,
        cameraMatrix=cameraMatrix,
        distCoeffs=distCoeffs,
        rms=ret
    )
    print(out_path)
