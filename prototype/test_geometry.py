"""Synthetic geometry checks for the panorama prototype.

These exercise the projection/stabilization math with known ground truth — no
bag required — so a sign error or convention regression fails loudly. Run with
the prototype venv:

    .venv/bin/python prototype/test_geometry.py     # plain runner
    .venv/bin/python -m pytest prototype/test_geometry.py   # if pytest installed
"""
from __future__ import annotations

import numpy as np

import bag_source as bs
import panorama as pano


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX (yaw*pitch*roll) rotation from radians."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _cyl_v(d: np.ndarray, scale: float = 77.0) -> float:
    """Cylinder vertical coord for a direction in warper-world coords."""
    return scale * d[1] / np.hypot(d[0], d[2])


def test_quat_to_matrix() -> None:
    np.testing.assert_allclose(bs.quat_to_matrix(0, 0, 0, 1), np.eye(3), atol=1e-12)
    # 90 deg about Z: x-axis -> y-axis
    R = bs.quat_to_matrix(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    np.testing.assert_allclose(R @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-9)
    # non-unit input is normalized
    R2 = bs.quat_to_matrix(0, 0, 2 * np.sin(np.pi / 4), 2 * np.cos(np.pi / 4))
    np.testing.assert_allclose(R, R2, atol=1e-12)


def test_level_base_rotation_removes_yaw() -> None:
    """level<-base must keep the boat's roll & pitch but drop heading (yaw)."""
    for roll, pitch, yaw in [(0.2, -0.1, 1.0), (-0.35, 0.15, -2.2), (0.0, 0.3, 0.7)]:
        R_nu_base = _rpy(roll, pitch, yaw)   # base -> north_up
        R_nu_level = _rpy(0.0, 0.0, yaw)     # level -> north_up (heading only)
        R_level_base = bs.level_base_rotation(R_nu_level, R_nu_base)
        # decompose result (ZYX): yaw ~ 0, pitch/roll preserved
        out_yaw = np.arctan2(R_level_base[1, 0], R_level_base[0, 0])
        out_pitch = np.arctan2(-R_level_base[2, 0],
                               np.hypot(R_level_base[2, 1], R_level_base[2, 2]))
        out_roll = np.arctan2(R_level_base[2, 1], R_level_base[2, 2])
        assert abs(out_yaw) < 1e-9, f"yaw not removed: {out_yaw}"
        np.testing.assert_allclose([out_roll, out_pitch], [roll, pitch], atol=1e-9)


def test_horizon_stays_level_under_motion() -> None:
    """A true-horizon ray projects to v~0 for any boat orientation.

    This pins the warper rotation convention (R_warp maps optical->world) and
    the AXIS_FIX that aligns the cylinder axis with gravity. If either were
    transposed/wrong, stabilization would tilt the horizon and this fails.
    """
    rng = np.random.default_rng(0)
    # arbitrary fixed camera mounting (optical -> base), a proper rotation
    R_base_optical = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])
    for _ in range(20):
        roll = rng.uniform(-0.6, 0.6)
        pitch = rng.uniform(-0.3, 0.3)
        # stabilization reference = level: R_level_base holds roll+pitch
        R_level_base = _rpy(roll, pitch, 0.0).T   # base->level removes them
        R_ref_optical = R_level_base @ R_base_optical
        R_warp = pano.AXIS_FIX @ R_ref_optical
        for heading in np.linspace(-np.pi, np.pi, 8):     # horizon ray at any bearing
            d_level = np.array([np.cos(heading), np.sin(heading), 0.0])  # elevation 0 (Z up)
            d_opt = R_ref_optical.T @ d_level             # ray as seen by the camera
            d_warp = R_warp @ d_opt
            assert abs(_cyl_v(d_warp)) < 1e-6, f"horizon not level: v={_cyl_v(d_warp)}"


def _main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
        except Exception as e:  # regression that errors instead of asserting
            failed += 1
            print(f"FAIL {t.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_main())
