import math

import pytest

from zed_heading_publisher.calibrate_zed_heading import estimate_soft_iron_calibration
from zed_heading_publisher.zed_heading_publisher import calculate_heading


def _variance_ratio(samples):
    mean_x = sum(x for x, _ in samples) / len(samples)
    mean_z = sum(z for _, z in samples) / len(samples)
    var_x = sum((x - mean_x) ** 2 for x, _ in samples) / len(samples)
    var_z = sum((z - mean_z) ** 2 for _, z in samples) / len(samples)
    return var_x / var_z


def test_axis_aligned_soft_iron_estimate_equalizes_variance():
    raw_samples = [
        (10.0 + 4.0 * math.sin(angle), -3.0 + 2.0 * math.cos(angle))
        for angle in [2.0 * math.pi * index / 180 for index in range(180)]
    ]

    calibration = estimate_soft_iron_calibration(raw_samples)

    corrected_samples = [
        (
            calibration.soft_iron_matrix_00 * (raw_x - calibration.center_x)
            + calibration.soft_iron_matrix_01 * (raw_z - calibration.center_z),
            calibration.soft_iron_matrix_10 * (raw_x - calibration.center_x)
            + calibration.soft_iron_matrix_11 * (raw_z - calibration.center_z),
        )
        for raw_x, raw_z in raw_samples
    ]
    assert calibration.center_x == pytest.approx(10.0)
    assert calibration.center_z == pytest.approx(-3.0)
    assert _variance_ratio(corrected_samples) == pytest.approx(1.0, abs=1e-6)


def test_calculate_heading_applies_soft_iron_matrix_before_heading():
    sample = calculate_heading(
        raw_x=3.0,
        raw_z=4.0,
        center_x=1.0,
        center_z=1.0,
        zero_heading_deg=0.0,
        soft_iron_matrix_00=2.0,
        soft_iron_matrix_01=0.0,
        soft_iron_matrix_10=0.0,
        soft_iron_matrix_11=1.0,
    )

    assert sample.corrected_x == 4.0
    assert sample.corrected_z == 3.0
    assert sample.magnetic_heading_deg == pytest.approx(
        math.degrees(math.atan2(4.0, 3.0))
    )
