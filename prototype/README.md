# image_warper — Phase 1 prototype

Offline Python prototype for [issue #1](https://github.com/rolker/image_warper/issues/1)
(port to ROS 2 / jazzy with a 4-camera stabilized panorama). It reads a BizzyBoat
`*_ffmpeg_seg` bag and renders a roll-stabilized **cylindrical 360° panorama** from
the four OAK cameras, as a single PNG or an mp4.

Its purpose is to settle the projection geometry and stabilization approach
**before** the Phase 2 rclcpp node — not to be production code. The ROS 1 source in
the repo root is untouched.

## Setup

Per [ADR-0009](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0009-python-package-management-policy.md),
install into a project-local venv (never bare `pip` / `--break-system-packages`):

```bash
python3 -m venv .venv                 # from the image_warper repo root
.venv/bin/python -m pip install -r prototype/requirements.txt
```

Requires `ffmpeg`/libav on the system (PyAV uses it to decode HEVC).

## Usage

```bash
cd prototype
BAG=~/data/logs/gabby/logs/bizzy_images/bag_2026-04-29T19.35.07_ffmpeg_seg   # dir or .mcap

# single frame -> PNG (segmentation imagery, fast)
../.venv/bin/python make_panorama.py --bag "$BAG" --time 60 --mode roll_pitch -o out/frame.png

# full-res RGB still (decodes HEVC up to the requested time)
../.venv/bin/python make_panorama.py --bag "$BAG" --time 60 --source rgb -o out/frame.png

# video over a time range -> mp4
../.venv/bin/python make_panorama.py --bag "$BAG" --video --start 180 --end 215 \
    --mode roll_pitch --source rgb -o out/pano.mp4
```

**`--mode`** (stabilization reference frame, from `/tf`):

| mode | reference | effect |
|------|-----------|--------|
| `none` | `bizzy/base_link` | raw boat frame — horizon tilts/rocks with the boat |
| `roll` | (level, roll only) | level the horizon for roll; pitch left in the image |
| `roll_pitch` | `bizzy/base_link_level` | fully leveled, heading-following *(default)* |
| `north_up` | `bizzy/base_link_north_up` | world-referenced, north-up (panorama scrolls as the boat turns) |

**`--source`**: `seg` = `segmentation/compressed` (128×96, fast geometry iteration);
`rgb` = full-res HEVC `image_raw/ffmpeg` (1920×1080, operator-quality).

**`--stamp-offset SECONDS`** (default 0): subtracted from every image header stamp
before the TF orientation lookup. The `image_raw/ffmpeg` stamps trail actual
capture by **~0.6 s** (≈3 frame periods @ 5 fps — encoder pipeline latency baked
into `EncodedFrame::getTimestamp()`; measured by cross-correlating content motion
against the correctly-stamped segmentation stream and `/tf`, see
[unh_marine_perception#41](https://github.com/rolker/unh_marine_perception/issues/41)).
So for `--source rgb` pass `--stamp-offset 0.6`; the `seg` stream is
capture-accurate (+4 ms) and needs none. Each tile is warped with the
orientation at its **own** (offset-corrected) stamp, not the anchor frame's.

## Tests

```bash
../.venv/bin/python test_geometry.py        # or: -m pytest test_geometry.py
```

`test_geometry.py` verifies the math with known ground truth (no bag): quaternion→matrix,
that the level reference removes heading while keeping roll/pitch, and — the key check —
that a true-horizon ray projects to a constant cylinder height under any boat
orientation (this pins the warper rotation convention and the gravity-aligned cylinder axis).

## How it works

1. **`bag_source.py`** — reads the `.mcap` with `mcap` + `mcap-ros2-support` (message
   schemas are embedded in the bag, so no ROS install is needed). Provides per-camera
   `CameraInfo`, decoded images (`segmentation/compressed` via OpenCV; `image_raw/ffmpeg`
   HEVC via PyAV), and TF rotations — the static camera-mounting chain
   `bizzy/base_link → oak_X → oak_X_optical` and the time-varying boat orientation
   (`bizzy/base_link_north_up → base_link` / `→ base_link_level`).
2. **`panorama.py`** — per camera: undistort to a pinhole model, then warp onto a shared
   cylinder (`cv2.PyRotationWarper`) with rotation `AXIS_FIX · R_reference_optical(t)`,
   and feather-blend the tiles. `AXIS_FIX` aligns the ROS reference frame (Z up) with the
   warper's gravity-down cylinder axis; the reference-frame choice is what removes roll/pitch.
3. **`make_panorama.py`** — CLI; still and video, segmentation or RGB.

## Findings & assumptions (feed into Phase 2)

- **Stabilization is geometrically correct** — verified analytically (`test_geometry.py`)
  and visually on RGB: the horizon stays level and continuous across the four cameras as
  the boat rolls (confirmed at a −21° roll moment).
- **Inter-camera gaps.** Segmentation imagery rectifies to ~79° HFOV at 90° spacing, so
  there are ~11° wedge gaps. Full-res RGB rectifies to ~96°, nearly closing them. The
  cameras' shared ~5° downtilt leaves the nadir uncovered (expected for a horizon panorama).
- **Full-res intrinsics are reconstructed, not measured.** The bag only carries the 128×96
  segmentation `CameraInfo`. `fullres_camera_info()` assumes the segmentation is a centred
  4:3 crop of the 16:9 sensor (1920→1440) resized to 128×96 — `1440/128 == 1080/96 == 11.25`
  is what makes the crop hypothesis hold. **Verify against the OAK factory calibration in
  Phase 2** before trusting RGB intrinsics for measurement.
- **Parallax ignored.** The panorama assumes a common optical centre (rotation only; TF
  translation dropped). Near-field objects show seam mismatch between adjacent cameras;
  the horizon is unaffected.
- **HEVC random access.** Decoding starts on a keyframe, so RGB stills decode forward from
  the bag start to the requested time. Phase 2 should index keyframes or decode in the node.
