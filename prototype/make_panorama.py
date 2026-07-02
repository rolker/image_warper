#!/usr/bin/env python3
"""Render a stabilized cylindrical panorama from a BizzyBoat ffmpeg_seg bag.

Phase 1 prototype CLI. Two modes:

  Single frame (PNG):
      make_panorama.py --bag BAG --time 450 --mode roll_pitch -o frame.png

  Video (mp4) over a time range:
      make_panorama.py --bag BAG --video --start 60 --end 120 --mode roll_pitch -o pano.mp4

Stabilization modes:
  none        raw boat frame (horizon tilts with the boat)
  roll        level the horizon for roll only (pitch left in image)
  roll_pitch  fully leveled, heading-following  [default]
  north_up    world-referenced, north-up (panorama scrolls as the boat turns)

Imagery is the segmentation/compressed stream (Phase 1, segmentation-first).
"""
from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

import bag_source as bs
import panorama as pano

# Composited tiles older than this relative to the frame being emitted (or the
# requested still time) are suspect — a camera stream that stalled or ended.
STALE_NS = 1_500_000_000


def _resolve_bag(path: str) -> str:
    import glob
    import os
    if os.path.isdir(path):
        hits = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not hits:
            sys.exit(f"no .mcap under {path}")
        if len(hits) > 1:
            # A split rosbag2 recording; reading only the first file would
            # silently drop the rest of the run.
            listing = "\n  ".join(hits)
            sys.exit(f"{path} holds {len(hits)} .mcap files (split recording — "
                     f"not supported yet); pass one explicitly:\n  {listing}")
        return hits[0]
    return path


def _infos(src: bs.BagSource, source: str, img: np.ndarray) -> dict | None:
    """Per-camera intrinsics for the chosen imagery source (None = segmentation).

    Assumes all OAK cameras share the full-res size (1920x1080 on BizzyBoat), so
    one decoded frame's shape sizes the intrinsics for every camera.
    """
    if source != "rgb":
        return None
    h, w = img.shape[:2]
    return {c: src.fullres_camera_info(c, w, h) for c in src.available_cameras()}


def render_still(src: bs.BagSource, t_ns: int, mode: str, out: str, source: str,
                 stamp_offset_ns: int = 0) -> None:
    if source == "rgb":
        # decode forward from bag start, keeping the latest frame per camera up to t
        latest: dict[str, tuple[int, np.ndarray]] = {}
        for ts, cam, img in src.iter_rgb(src.start_ns, t_ns):
            latest[cam] = (ts, img)
        images = latest
        for cam, (ts, _img) in images.items():
            if t_ns - ts > STALE_NS:
                print(f"warning: {cam} nearest frame is {(t_ns - ts) / 1e9:.1f}s before "
                      "--time (stream stalled or ended); compositing a stale tile",
                      file=sys.stderr)
        infos = _infos(src, source, next(iter(images.values()))[1]) if images else None
    else:
        images = src.images_at(t_ns)
        infos = None
    if len(images) < 2:
        sys.exit(f"only {len(images)} camera(s) near t={t_ns}; need >=2 to stitch")
    pan = pano.stitch(src, images, mode=mode, infos=infos, stamp_offset_ns=stamp_offset_ns)
    if not cv2.imwrite(out, pan):
        sys.exit(f"could not write {out} (missing directory or bad extension?)")
    print(f"wrote {out}  ({pan.shape[1]}x{pan.shape[0]}, {len(images)} cameras, "
          f"mode={mode}, source={source})")


def render_video(src: bs.BagSource, start_ns: int, end_ns: int, mode: str, fps: float,
                 out: str, source: str, stamp_offset_ns: int = 0) -> None:
    stream = src.iter_rgb if source == "rgb" else src.iter_images
    latest: dict[str, tuple[int, np.ndarray]] = {}
    infos = None
    scale = canvas = writer = None
    anchor = None   # first camera seen sets the output cadence (all cameras ~5 Hz)
    warned_stale: set[str] = set()
    last_seen_ns = last_emit_ns = None
    n = 0
    try:
        for t_ns, cam, img in stream(start_ns, end_ns):
            latest[cam] = (t_ns, img)
            last_seen_ns = t_ns
            if anchor is None:
                anchor = cam
            if writer is None:                   # lazily size output from the first frame
                infos = _infos(src, source, img)
                scale = float(np.median([
                    (infos[c].K[0, 0] if infos else src.camera_info(c).K[0, 0])
                    for c in src.available_cameras()
                ]))
                canvas = pano.full_canvas(scale)
                writer = cv2.VideoWriter(
                    out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (canvas[2], canvas[3]))
                if not writer.isOpened():
                    sys.exit(f"could not open VideoWriter for {out}")
            # Emit one output frame per anchor-camera frame, once >=2 cameras have data.
            if cam == anchor and len(latest) >= 2:
                for c, (ts, _img) in latest.items():
                    if t_ns - ts > STALE_NS and c not in warned_stale:
                        warned_stale.add(c)
                        print(f"warning: {c} tile is {(t_ns - ts) / 1e9:.1f}s stale at "
                              f"t={(t_ns - src.start_ns) / 1e9:.1f}s (camera stalled?); "
                              "compositing its last frame. Warned once per camera.",
                              file=sys.stderr)
                pan = pano.stitch(src, dict(latest), mode=mode,
                                  scale=scale, canvas=canvas, infos=infos,
                                  stamp_offset_ns=stamp_offset_ns)
                writer.write(pan)
                last_emit_ns = t_ns
                n += 1
    finally:
        if writer is not None:
            writer.release()
    if n == 0:
        sys.exit("no frames written (no images in range, or <2 cameras)")
    # Emission is keyed to the anchor camera, so an anchor stall ends the video
    # early while other cameras keep streaming — the in-loop stale warnings
    # can't fire for that case (they only run on anchor emits).
    if last_seen_ns - last_emit_ns > STALE_NS:
        print(f"warning: anchor camera {anchor} went quiet "
              f"{(last_seen_ns - last_emit_ns) / 1e9:.1f}s before the range end; "
              "video truncated early", file=sys.stderr)
    print(f"wrote {out}  ({canvas[2]}x{canvas[3]}, {n} frames @ {fps} fps, "
          f"mode={mode}, source={source})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="path to .mcap (or a bag directory)")
    ap.add_argument("--mode", default="roll_pitch", choices=list(pano.REFERENCES) + ["roll"])
    ap.add_argument("--source", default="seg", choices=["seg", "rgb"],
                    help="imagery: 'seg' = segmentation/compressed (fast), 'rgb' = full-res HEVC")
    ap.add_argument("-o", "--out", help="output path (default frame.png / pano.mp4)")
    ap.add_argument("--video", action="store_true", help="render an mp4 over a time range")
    ap.add_argument("--time", type=float, help="still: seconds from bag start (default: midpoint)")
    ap.add_argument("--start", type=float, help="video: start seconds from bag start (default: 0)")
    ap.add_argument("--end", type=float,
                    help="video: end seconds from bag start (default: bag end)")
    ap.add_argument("--fps", type=float, help="video: output frame rate (default: 5)")
    ap.add_argument("--stamp-offset", type=float, default=0.0, metavar="SECONDS",
                    help="subtract from image stamps before the TF orientation lookup; "
                         "use ~0.6 with --source rgb (image_raw/ffmpeg stamps trail "
                         "capture by ~0.6 s — unh_marine_perception#41)")
    args = ap.parse_args()

    # Mode/argument interactions — reject silently-ignored combinations.
    if args.video and args.time is not None:
        ap.error("--time selects a still; use --start/--end with --video")
    if not args.video and any(v is not None for v in (args.start, args.end, args.fps)):
        ap.error("--start/--end/--fps require --video")
    if args.video and args.end is not None and args.end <= (args.start or 0.0):
        ap.error(f"--end ({args.end}) must be greater than --start ({args.start or 0.0})")
    if args.fps is not None and args.fps <= 0:
        ap.error(f"--fps ({args.fps}) must be greater than 0")

    try:
        src = bs.BagSource(_resolve_bag(args.bag))
    except ValueError as e:                       # e.g. statistics-less truncated bag
        sys.exit(f"error: {e}")
    offset_ns = int(args.stamp_offset * 1e9)

    try:
        if args.video:
            start_ns = src.start_ns + int((args.start or 0.0) * 1e9)
            end_ns = src.end_ns if args.end is None else src.start_ns + int(args.end * 1e9)
            render_video(src, start_ns, end_ns, args.mode,
                         5.0 if args.fps is None else args.fps,
                         args.out or "pano.mp4", args.source, stamp_offset_ns=offset_ns)
        else:
            t_ns = ((src.start_ns + src.end_ns) // 2 if args.time is None
                    else src.start_ns + int(args.time * 1e9))
            render_still(src, t_ns, args.mode, args.out or "frame.png", args.source,
                         stamp_offset_ns=offset_ns)
    except KeyError as e:                         # e.g. missing TF edge/chain in the bag
        sys.exit(f"error: bag is missing required data: {e}")


if __name__ == "__main__":
    main()
