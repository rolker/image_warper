"""Cylindrical-panorama stitching + roll stabilization (Phase 1 prototype).

Each camera image is undistorted to a pinhole model, projected onto a shared
cylinder with OpenCV's rotation warper (same math as the C++ `detail` warpers
the original ROS 1 node used), and feather-blended onto a panorama canvas.

The rotation handed to the warper maps the camera optical frame into a chosen
*stabilization reference* frame:

    R_warp = AXIS_FIX @ R_reference_optical(t)

`R_reference_optical` comes from TF (`bag_source.orientation` @ the static
camera-mounting chain). `AXIS_FIX` converts the ROS reference convention
(X fwd, Y left, Z up) into the warper's world convention (cylinder axis = Y,
pointing down) — the same 90°-about-X fix the original node applied via its
`scalar` quaternion. The stabilization reference choice (none / level / north_up)
is what removes boat roll & pitch from the horizon.
"""
from __future__ import annotations

import cv2
import numpy as np

import bag_source as bs

# 90 deg about X: ROS (X fwd, Y left, Z up) -> warper world (Y down = cyl axis).
AXIS_FIX = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
])

# Rotate the whole panorama about the cylinder axis so the +/-180 deg wrap seam
# lands in a gap between cameras (cameras sit at 0/90/180/270; 45 deg puts the
# seam mid-gap) instead of splitting one camera across both edges.
SEAM_YAW_DEG = 45.0


def _rot_cyl_axis(deg: float) -> np.ndarray:
    """Rotation about the warper-world cylinder axis (Y)."""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

# Stabilization reference frames, by CLI keyword.
REFERENCES = {
    "none": bs.F_BASE,        # raw boat frame: horizon tilts with the boat
    "roll_pitch": bs.F_LEVEL, # remove roll AND pitch (leveled, heading-following)
    "north_up": bs.F_NORTH_UP,# fully world-referenced, north-up
}


def _roll_only(R_ref_base: np.ndarray) -> np.ndarray:
    """Keep only the roll (X-axis, in-plane) component of a level->base rotation.

    Decomposes ZYX euler and zeroes yaw & pitch, so the horizon is leveled for
    roll but pitch is left in the image (the issue's "roll-only, pitch optional").
    """
    roll = np.arctan2(R_ref_base[2, 1], R_ref_base[2, 2])
    c, s = np.cos(roll), np.sin(roll)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def reference_rotation(src: bs.BagSource, cam: str, t_ns: int, mode: str) -> np.ndarray:
    """R_reference_optical(t) for `cam` at `t_ns` under stabilization `mode`."""
    R_base_optical = src.static_rotation(bs.F_BASE, f"{bs.NS}{cam}_optical")
    if mode == "roll":
        R_ref_base = _roll_only(src.orientation(t_ns, reference=bs.F_LEVEL))
    else:
        R_ref_base = src.orientation(t_ns, reference=REFERENCES[mode])
    return R_ref_base @ R_base_optical


def undistort(img: np.ndarray, ci: bs.CameraInfo) -> tuple[np.ndarray, np.ndarray]:
    """Undistort to a pinhole model (same K). Returns (rectified_bgr, valid_mask)."""
    K = ci.K.astype(np.float32)
    rect = cv2.undistort(img, K, ci.D.astype(np.float32))
    mask = cv2.undistort(np.full(img.shape[:2], 255, np.uint8), K, ci.D.astype(np.float32))
    return rect, mask


def stitch(
    src: bs.BagSource,
    images: dict[str, tuple[int, np.ndarray]],
    t_ns: int,
    mode: str = "roll",
    scale: float | None = None,
    canvas: tuple[int, int, int, int] | None = None,
    infos: dict[str, bs.CameraInfo] | None = None,
) -> np.ndarray:
    """Stitch per-camera images into one feather-blended cylindrical panorama (BGR).

    `canvas` = (x, y, w, h) result ROI in cylinder pixel coords. Pass a fixed
    canvas for video (constant output size); leave None for stills (the ROI is
    the tight bounding box of the warped tiles).
    `infos` overrides the per-camera intrinsics (e.g. full-res for the HEVC RGB
    path); defaults to the bag's segmentation CameraInfo.
    """
    def info(cam: str) -> bs.CameraInfo:
        return infos[cam] if infos else src.camera_info(cam)

    if scale is None:
        scale = float(np.median([info(c).K[0, 0] for c in images]))
    warper = cv2.PyRotationWarper("cylindrical", scale)
    seam = _rot_cyl_axis(SEAM_YAW_DEG)

    corners, warped_imgs, warped_masks = [], [], []
    for cam, (_ts, img) in images.items():
        ci = info(cam)
        rect, vmask = undistort(img, ci)
        K = ci.K.astype(np.float32)
        R = (seam @ AXIS_FIX @ reference_rotation(src, cam, t_ns, mode)).astype(np.float32)
        corner, wimg = warper.warp(rect, K, R, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
        _corner, wmask = warper.warp(vmask, K, R, cv2.INTER_NEAREST, cv2.BORDER_CONSTANT)
        corners.append(corner)
        warped_imgs.append(wimg)
        warped_masks.append(wmask)

    # Always blend over the tight bounding box: the FeatherBlender writes out of
    # bounds (heap corruption) if a fed tile isn't fully inside the prepared ROI.
    sizes = [(w.shape[1], w.shape[0]) for w in warped_imgs]
    roi = cv2.detail.resultRoi(corners, sizes)  # (x, y, w, h)
    blender = cv2.detail.Blender_createDefault(cv2.detail.Blender_FEATHER)
    blender.prepare(roi)
    for wimg, wmask, corner in zip(warped_imgs, warped_masks, corners):
        blender.feed(wimg.astype(np.int16), wmask, corner)
    result, result_mask = blender.blend(None, None)
    result = np.clip(result, 0, 255).astype(np.uint8)
    if canvas is None:
        return result
    return _paste_to_canvas(result, result_mask, (roi[0], roi[1]), canvas)


def _paste_to_canvas(result, result_mask, corner, canvas) -> np.ndarray:
    """Paste a blended strip into a fixed-size canvas, wrapping in x (cylinder seam)."""
    cx, cy, cw, ch = canvas
    out = np.zeros((ch, cw, 3), np.uint8)
    rh, rw = result.shape[:2]
    ys = np.arange(rh) + (corner[1] - cy)
    xs = (np.arange(rw) + (corner[0] - cx)) % cw  # wrap around the cylinder
    yv = (ys >= 0) & (ys < ch)
    if not yv.any():
        return out
    ys, result, result_mask = ys[yv], result[yv], result_mask[yv]
    m = result_mask.astype(bool)
    yy = np.broadcast_to(ys[:, None], (ys.size, rw))[m]
    xx = np.broadcast_to(xs[None, :], (ys.size, rw))[m]
    out[yy, xx] = result[m]
    return out


def full_canvas(scale: float, height: int | None = None, v_center: int = 0) -> tuple[int, int, int, int]:
    """Fixed full-360 cylinder ROI (x, y, w, h) for constant-size video frames.

    Height defaults to ~1.8x the cylinder radius (covers the cameras' vertical
    FOV plus their ~5 deg downtilt) so it scales with `scale` for both the
    segmentation (scale~77) and full-res RGB (scale~860) paths.
    """
    width = int(round(2 * np.pi * scale))
    if height is None:
        height = int(round(1.8 * scale))
    return (-width // 2, v_center - height // 2, width, height)
