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


def _resolve_bag(path: str) -> str:
    import glob
    import os
    if os.path.isdir(path):
        hits = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not hits:
            sys.exit(f"no .mcap under {path}")
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
    return {c: src.fullres_camera_info(c, w, h) for c in bs.CAMERAS}


def render_still(src: bs.BagSource, t_ns: int, mode: str, out: str, source: str,
                 stamp_offset_ns: int = 0) -> None:
    if source == "rgb":
        # decode forward from bag start, keeping the latest frame per camera up to t
        latest: dict[str, tuple[int, np.ndarray]] = {}
        for ts, cam, img in src.iter_rgb(src.start_ns, t_ns):
            latest[cam] = (ts, img)
        images = latest
        infos = _infos(src, source, next(iter(images.values()))[1]) if images else None
    else:
        images = src.images_at(t_ns)
        infos = None
    if len(images) < 2:
        sys.exit(f"only {len(images)} camera(s) near t={t_ns}; need >=2 to stitch")
    pan = pano.stitch(src, images, mode=mode, infos=infos, stamp_offset_ns=stamp_offset_ns)
    cv2.imwrite(out, pan)
    print(f"wrote {out}  ({pan.shape[1]}x{pan.shape[0]}, {len(images)} cameras, "
          f"mode={mode}, source={source})")


def render_video(src: bs.BagSource, start_ns: int, end_ns: int, mode: str, fps: float,
                 out: str, source: str, stamp_offset_ns: int = 0) -> None:
    stream = src.iter_rgb if source == "rgb" else src.iter_images
    latest: dict[str, tuple[int, np.ndarray]] = {}
    infos = None
    scale = canvas = writer = None
    anchor = None   # first camera seen sets the output cadence (all cameras ~5 Hz)
    n = 0
    try:
        for t_ns, cam, img in stream(start_ns, end_ns):
            latest[cam] = (t_ns, img)
            if anchor is None:
                anchor = cam
            if writer is None:                   # lazily size output from the first frame
                infos = _infos(src, source, img)
                scale = float(np.median([
                    (infos[c].K[0, 0] if infos else src.camera_info(c).K[0, 0]) for c in bs.CAMERAS
                ]))
                canvas = pano.full_canvas(scale)
                writer = cv2.VideoWriter(
                    out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (canvas[2], canvas[3]))
                if not writer.isOpened():
                    sys.exit(f"could not open VideoWriter for {out}")
            # Emit one output frame per anchor-camera frame, once >=2 cameras have data.
            if cam == anchor and len(latest) >= 2:
                pan = pano.stitch(src, dict(latest), mode=mode,
                                  scale=scale, canvas=canvas, infos=infos,
                                  stamp_offset_ns=stamp_offset_ns)
                writer.write(pan)
                n += 1
    finally:
        if writer is not None:
            writer.release()
    if n == 0:
        sys.exit("no frames written (no images in range, or <2 cameras)")
    print(f"wrote {out}  ({canvas[2]}x{canvas[3]}, {n} frames @ {fps} fps, "
          f"mode={mode}, source={source})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="path to .mcap (or a bag directory)")
    ap.add_argument("--mode", default="roll_pitch", choices=list(pano.REFERENCES) + ["roll"])
    ap.add_argument("--source", default="seg", choices=["seg", "rgb"],
                    help="imagery: 'seg' = segmentation/compressed (fast), 'rgb' = full-res HEVC")
    ap.add_argument("-o", "--out", help="output path (default frame.png / pano.mp4)")
    ap.add_argument("--video", action="store_true", help="render an mp4 over a time range")
    ap.add_argument("--time", type=float, help="still: seconds from bag start (default: midpoint)")
    ap.add_argument("--start", type=float, default=0.0, help="video: start seconds from bag start")
    ap.add_argument("--end", type=float, help="video: end seconds from bag start (default: bag end)")
    ap.add_argument("--fps", type=float, default=5.0, help="video: output frame rate")
    ap.add_argument("--stamp-offset", type=float, default=0.0, metavar="SECONDS",
                    help="subtract from image stamps before the TF orientation lookup; "
                         "use ~0.6 with --source rgb (image_raw/ffmpeg stamps trail "
                         "capture by ~0.6 s — unh_marine_perception#41)")
    args = ap.parse_args()

    src = bs.BagSource(_resolve_bag(args.bag))
    offset_ns = int(args.stamp_offset * 1e9)

    if args.video:
        start_ns = src.start_ns + int(args.start * 1e9)
        end_ns = src.end_ns if args.end is None else src.start_ns + int(args.end * 1e9)
        render_video(src, start_ns, end_ns, args.mode, args.fps, args.out or "pano.mp4",
                     args.source, stamp_offset_ns=offset_ns)
    else:
        t_ns = (src.start_ns + src.end_ns) // 2 if args.time is None else src.start_ns + int(args.time * 1e9)
        render_still(src, t_ns, args.mode, args.out or "frame.png", args.source,
                     stamp_offset_ns=offset_ns)


if __name__ == "__main__":
    main()
