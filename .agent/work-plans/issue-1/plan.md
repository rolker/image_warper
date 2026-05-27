# Plan: Port to ROS 2 / jazzy with 4-camera stabilized panorama — Phase 1

## Issue

https://github.com/rolker/image_warper/issues/1

This plan scopes **Phase 1 only** — an offline Python prototype to validate
cylindrical-projection geometry and roll-only stabilization from recorded bags.
Phases 2 (rclcpp node) and 3 (workspace/manifest integration) remain as
described in the issue body and get their own plans once Phase 1 settles the
geometry.

## Context

The existing code (`noetic`/`jazzy` branches, identical) is Deepak Narayan's ROS 1
`imageStabilize_360VR`: per-camera undistort → tf2-derived rotation → OpenCV
`SphericalWarper` → `FeatherBlender` composite, hardcoded for 6 `pano_N` cameras
with baked-in intrinsics. It does not build under colcon (`find_package(catkin)`
fails) and is not in any `.repos` manifest.

Phase 1 does **not** touch that ROS 1 code. It builds a standalone, throwaway-grade
Python prototype that reads BizzyBoat bags and emits a stitched, roll-stabilized
cylindrical panorama — first as a single static frame (geometry check), then as an
mp4 (time-varying orientation). Its purpose is to settle the projection math and
stabilization approach *before* committing to a C++ node in Phase 2.

Confirmed inputs in `~/data/logs/bizzy_images/*_ffmpeg_seg` (e.g.
`bag_2026-04-29T19.35.07_ffmpeg_seg`), mcap format:
- `/bizzy/sensors/cameras/oak_{forward,aft,port,starboard}/image_raw/ffmpeg` — HEVC `FFMPEGPacket` (raw RGB camera)
- `/bizzy/sensors/cameras/oak_{...}/segmentation/compressed` — `CompressedImage` (segmentation overlay, trivially decodable)
- `/bizzy/sensors/cameras/oak_{...}/segmentation/camera_info` — `CameraInfo` (K + distortion)
- `/tf`, `/tf_static` — `TFMessage`

## Approach

1. **Bag access layer** (`prototype/bag_source.py`) — read the mcap with `mcap` +
   `mcap-ros2-support` (schemas are embedded in the file, so `FFMPEGPacket` /
   `CameraInfo` / `TFMessage` deserialize without a ROS install). Expose: per-camera
   image iterators, a one-shot `camera_info` fetch, and a small TF buffer
   (`tf2`-free) that returns the `base_link`→`oak_*_optical` static transforms and the
   time-varying world→`base_link` orientation by nearest-timestamp lookup.
2. **Confirm the orientation source** — enumerate `/tf` + `/tf_static` frames in the
   bag to find the level/north-referenced frame the original used
   (`north_up_base_link`) or its equivalent. If the dynamic boat orientation isn't in
   this bag's `/tf`, join by timestamp with the parallel `~/data/logs/bizzyboat/<run>`
   bag (SBG/mavros). Resolve before video stage; static stage can use identity.
3. **Image decode** — `CompressedImage` via `cv2.imdecode` (segmentation, for fast
   geometry iteration); `FFMPEGPacket` HEVC via PyAV (`av`) decoder context for the
   true RGB camera image. Stage the decode: segmentation first, HEVC once geometry holds.
4. **Cylindrical projection + composite** (`prototype/panorama.py`) — per camera:
   undistort with its `CameraInfo`, then project onto a shared 360° cylinder using the
   camera's static mounting rotation (from step 1) and intrinsics. Use
   `cv2.PyRotationWarper("cylindrical", scale)` (same math as the C++ `detail`
   warpers). Composite the 4 warped images by feathered alpha over the cylinder canvas.
   **Validate with a single static frame first** (the issue's gate).
5. **Roll-only stabilization** (in `panorama.py`) — extract roll from the world→`base_link`
   orientation and fold it into each camera's rotation matrix as an in-plane correction
   so the horizon stays level. Pitch left as an optional knob (off by default). Keep a
   per-camera time-delay offset to handle tf-vs-image skew (the original's concern).
6. **CLI** (`prototype/make_panorama.py`) — `--bag <path> --frame <t>` → PNG (static);
   `--bag <path> [--start --end]` → mp4 (video, `cv2.VideoWriter` / imageio-ffmpeg).
7. **Geometry sanity check** (`prototype/test_geometry.py`) — a light synthetic test:
   two overlapping synthetic pinhole views of a known pattern should stitch with aligned
   features on the cylinder. Not a full suite — see Open Questions.

## Files to Change

| File | Change |
|------|--------|
| `prototype/requirements.txt` | New: `numpy`, `opencv-python`, `av` (PyAV), `mcap`, `mcap-ros2-support`, `imageio[ffmpeg]` |
| `prototype/README.md` | New: `.venv` setup, how to run static + video modes, input-bag expectations |
| `prototype/bag_source.py` | New: mcap reader, decoders, minimal TF lookup |
| `prototype/panorama.py` | New: undistort + cylindrical warp + roll stabilization + composite |
| `prototype/make_panorama.py` | New: CLI entry (static PNG / video mp4) |
| `prototype/test_geometry.py` | New: synthetic cylindrical-projection sanity check |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Prototype is intentionally minimal (3 modules + CLI + req); no node, no manifest entry, no blender-machinery port until Phase 1 proves the approach. |
| Improve incrementally | Static frame → video; segmentation imagery → HEVC RGB. Each stage independently reviewable. |
| Capture decisions | Projection (cylindrical) and stabilization (roll-only) choices recorded in the issue; Phase 1 findings will feed the Phase 2 plan/ADR-style note. |
| A change includes its consequences | Phase 1 adds no consumers and touches no built code, so no downstream refs to update; the ROS 1 colcon-build failure is tracked separately (see Open Questions). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0009 (Python package mgmt) | Yes | Prototype deps go in a project-local `.venv` via `prototype/requirements.txt`; never bare `pip` / `--break-system-packages`. |
| ADR-0008 (ROS 2 conventions) | Partial | Phase 1 is standalone Python, not a ROS node; full convention compliance lands in Phase 2. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `prototype/` Python deps | `.gitignore` for `.venv`, `__pycache__`, output artifacts | Yes |
| Nothing in the built ROS 1 code | (no dependents) | N/A — Phase 1 is additive |
| `image_warper` still breaks `make build` | Decide whether to `COLCON_IGNORE` until Phase 2 ports it | Open Question |

## Open Questions

- **Sub-issue vs. direct branch.** The issue body says Phase 1 should be a sub-issue.
  Work directly on `feature/issue-1`, or file a Phase 1 sub-issue + stacked PR?
- **Imagery source for validation.** OK to validate geometry on `segmentation/compressed`
  (easy, but it's the segmentation overlay) and add HEVC RGB after, or must the prototype
  decode `image_raw/ffmpeg` (true RGB) from the start?
- **Orientation source.** If this bag's `/tf` lacks a level/world-referenced frame, is
  joining the parallel `~/data/logs/bizzyboat/<run>` bag by timestamp acceptable for the prototype?
- **Build breakage.** Should this PR also add a `COLCON_IGNORE` so the unported catkin
  package stops failing `make build` until Phase 2, or leave that out of Phase 1 scope?
- **Test rigor.** One synthetic geometry check for a throwaway prototype — enough, or
  do you want more before Phase 2?

## Estimated Scope

Single PR for Phase 1 (the prototype), assuming work proceeds on `feature/issue-1`.
Phases 2 and 3 are separate, larger efforts with their own plans.
