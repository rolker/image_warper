"""Standalone access to BizzyBoat ffmpeg_seg bags for the panorama prototype.

Reads an .mcap bag with the `mcap` + `mcap-ros2-support` libraries (message
definitions are embedded in the bag, so no ROS install is needed), and exposes
the pieces the cylindrical-panorama prototype needs:

  * per-camera CameraInfo (intrinsics K + distortion d),
  * decoded images at (or nearest to) a target time,
  * TF rotations, both the static camera-mounting chain and the time-varying
    boat orientation used for horizon stabilization.

Phase 1 ignores TF translation — the panorama assumes a common optical centre,
so only rotations matter (parallax between the four cameras is a known
limitation, noted in README.md).
"""
from __future__ import annotations

import bisect
import sys
from dataclasses import dataclass

import av
import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

CAMERAS = ("oak_forward", "oak_aft", "oak_port", "oak_starboard")

# Frame names (all under the `bizzy/` namespace in these bags).
NS = "bizzy/"
F_NORTH_UP = NS + "base_link_north_up"   # gravity-aligned, north-locked reference
F_LEVEL = NS + "base_link_level"         # gravity-aligned, heading-following
F_BASE = NS + "base_link"                # full boat orientation

# Dynamic-TF lookups farther than this from the nearest sample are suspect
# (clamped past the bag edge, or spanning a mid-bag TF dropout at ~10 Hz).
MAX_TF_GAP_NS = 1_000_000_000


def _topic(cam: str, suffix: str) -> str:
    return f"/{NS}sensors/cameras/{cam}/{suffix}"


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def level_base_rotation(R_nu_level: np.ndarray, R_nu_base: np.ndarray) -> np.ndarray:
    """R mapping bizzy/base_link -> bizzy/base_link_level.

    Given the two north-up-referenced rotations from /tf (north_up->level and
    north_up->base, each mapping its child's coords into north_up), compose to
    base->level = (north_up->level)^-1 . (north_up->base). The result holds the
    boat's roll+pitch with heading (yaw) removed.
    """
    return R_nu_level.T @ R_nu_base


@dataclass
class CameraInfo:
    width: int
    height: int
    K: np.ndarray          # 3x3
    D: np.ndarray          # distortion coefficients
    distortion_model: str


class BagSource:
    """Lazy, multi-pass reader over a single .mcap file.

    Construction does one cheap pass for static data (CameraInfo, /tf_static and
    the dynamic-orientation timeline). Image reads are separate time-windowed
    passes so a single-frame render never decodes the whole bag.
    """

    def __init__(self, path: str):
        self.path = path
        self._camera_info: dict[str, CameraInfo] = {}
        self._static_rot: dict[str, tuple[str, np.ndarray]] = {}  # child -> (parent, R)
        # dynamic orientation edges we care about: parent -> sorted [(t_ns, R)]
        self._dyn: dict[tuple[str, str], list[tuple[int, np.ndarray]]] = {}
        self._dyn_times: dict[tuple[str, str], list[int]] = {}  # lookup index for _dyn
        self._gap_warned: set[tuple[str, str]] = set()
        self.start_ns: int = 0
        self.end_ns: int = 0
        self._scan_static()

    # -- internal -----------------------------------------------------------
    def _reader(self, fh):
        return make_reader(fh, decoder_factories=[DecoderFactory()])

    def _scan_static(self) -> None:
        info_topics = {_topic(c, "segmentation/camera_info"): c for c in CAMERAS}
        dyn_edges = {(F_NORTH_UP, F_BASE), (F_NORTH_UP, F_LEVEL)}
        with open(self.path, "rb") as fh:
            reader = self._reader(fh)
            summary = reader.get_summary()
            if not (summary and summary.statistics):
                # Without statistics every relative-time computation (--time,
                # --start, midpoint default) would silently anchor at 0 (1970).
                raise ValueError(
                    f"{self.path}: no mcap summary statistics — truncated recording? "
                    "Try 'mcap recover' on the file first.")
            self.start_ns = summary.statistics.message_start_time
            self.end_ns = summary.statistics.message_end_time
            for _schema, channel, _msg, ros_msg in reader.iter_decoded_messages(
                topics=["/tf_static", "/tf", *info_topics],
            ):
                t = channel.topic
                if t in info_topics and info_topics[t] not in self._camera_info:
                    self._camera_info[info_topics[t]] = CameraInfo(
                        width=ros_msg.width,
                        height=ros_msg.height,
                        K=np.array(ros_msg.k, dtype=np.float64).reshape(3, 3),
                        D=np.array(ros_msg.d, dtype=np.float64),
                        distortion_model=ros_msg.distortion_model,
                    )
                elif t == "/tf_static":
                    for tr in ros_msg.transforms:
                        q = tr.transform.rotation
                        self._static_rot[tr.child_frame_id] = (
                            tr.header.frame_id,
                            quat_to_matrix(q.x, q.y, q.z, q.w),
                        )
                elif t == "/tf":
                    for tr in ros_msg.transforms:
                        edge = (tr.header.frame_id, tr.child_frame_id)
                        if edge in dyn_edges:
                            q = tr.transform.rotation
                            self._dyn.setdefault(edge, []).append(
                                (_stamp_ns(tr.header), quat_to_matrix(q.x, q.y, q.z, q.w))
                            )
        for edge, seq in self._dyn.items():
            seq.sort(key=lambda kv: kv[0])
            self._dyn_times[edge] = [kv[0] for kv in seq]

    # -- public: intrinsics -------------------------------------------------
    def camera_info(self, cam: str) -> CameraInfo:
        return self._camera_info[cam]

    def available_cameras(self) -> tuple[str, ...]:
        """Cameras that actually published segmentation CameraInfo in this bag.

        A camera can be down for a whole run; consumers should iterate this
        instead of CAMERAS so a 3-camera bag degrades instead of KeyError-ing.
        """
        return tuple(c for c in CAMERAS if c in self._camera_info)

    # -- public: TF ---------------------------------------------------------
    def static_rotation(self, target: str, source: str) -> np.ndarray:
        """R that maps a vector in `source` frame into `target`, via static TF only.

        Walks `source` up its static-parent chain to `target`. Raises if the
        chain doesn't reach `target`.
        """
        R = np.eye(3)
        frame = source
        chain = []
        while frame != target:
            if frame not in self._static_rot:
                raise KeyError(f"no static TF chain from {source} to {target} (stuck at {frame})")
            parent, R_pc = self._static_rot[frame]  # parent <- frame (R_pc maps frame->parent)
            R = R_pc @ R
            frame = parent
            chain.append(frame)
            if len(chain) > 32:
                raise RuntimeError(f"TF chain too deep / cyclic from {source}: {chain}")
        return R

    def _nearest_dynamic(self, edge: tuple[str, str], t_ns: int) -> np.ndarray:
        seq = self._dyn.get(edge)
        if not seq:
            raise KeyError(f"no dynamic TF samples for edge {edge}")
        times = self._dyn_times[edge]
        i = bisect.bisect_left(times, t_ns)
        if i == 0:
            t_best, best = seq[0]
        elif i >= len(seq):
            t_best, best = seq[-1]
        else:
            before, after = seq[i - 1], seq[i]
            t_best, best = before if (t_ns - before[0]) <= (after[0] - t_ns) else after
        gap = abs(t_best - t_ns)
        if gap > MAX_TF_GAP_NS and edge not in self._gap_warned:
            self._gap_warned.add(edge)
            print(f"warning: TF lookup {gap / 1e9:.2f}s from the nearest {edge[1]} sample "
                  "(clamped past bag edge or TF dropout); orientation may be stale. "
                  "Further gaps on this edge are not reported.", file=sys.stderr)
        return best

    def orientation(self, t_ns: int, reference: str = F_LEVEL) -> np.ndarray:
        """R mapping `reference` frame -> bizzy/base_link at time `t_ns`.

        reference=F_LEVEL  -> roll+pitch of the boat relative to a leveled,
                              heading-following frame (use to remove roll/pitch).
        reference=F_NORTH_UP -> full orientation relative to north-up level.
        reference=F_BASE   -> identity (no stabilization).
        """
        if reference == F_BASE:
            return np.eye(3)
        R_nu_base = self._nearest_dynamic((F_NORTH_UP, F_BASE), t_ns)
        if reference == F_NORTH_UP:
            return R_nu_base
        if reference == F_LEVEL:
            R_nu_level = self._nearest_dynamic((F_NORTH_UP, F_LEVEL), t_ns)
            return level_base_rotation(R_nu_level, R_nu_base)
        raise ValueError(f"unknown reference frame {reference}")

    # -- public: images -----------------------------------------------------
    def images_at(self, t_ns: int, window_ns: int = 300_000_000) -> dict[str, tuple[int, np.ndarray]]:
        """Nearest segmentation/compressed frame per camera to `t_ns`.

        Returns cam -> (frame_time_ns, BGR image). Searches a +/- window so a
        single-frame render does not scan the whole bag.
        """
        topics = {_topic(c, "segmentation/compressed"): c for c in CAMERAS}
        best: dict[str, tuple[int, bytes]] = {}
        lo, hi = max(self.start_ns, t_ns - window_ns), t_ns + window_ns
        with open(self.path, "rb") as fh:
            for _schema, channel, _msg, ros_msg in self._reader(fh).iter_decoded_messages(
                topics=list(topics), start_time=lo, end_time=hi,
            ):
                cam = topics[channel.topic]
                ts = _stamp_ns(ros_msg.header)
                if cam not in best or abs(ts - t_ns) < abs(best[cam][0] - t_ns):
                    best[cam] = (ts, bytes(ros_msg.data))
        out: dict[str, tuple[int, np.ndarray]] = {}
        for cam, (ts, data) in best.items():
            img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                out[cam] = (ts, img)
        return out

    def fullres_camera_info(self, cam: str, width: int, height: int) -> CameraInfo:
        """Reconstruct intrinsics for the full-res HEVC frame from the segmentation
        CameraInfo.

        ASSUMPTION (Phase 1): the segmentation image is a centered crop of the
        full frame to the segmentation aspect ratio, then uniformly resized.
        For these bags that is a 4:3 centre crop of the 16:9 sensor (1920->1440,
        240 px/side) resized to 128x96 — 1440/128 == 1080/96 == 11.25, which is
        what makes the crop hypothesis hold. Verify against the OAK factory
        calibration in Phase 2 before trusting this for measurement.
        """
        seg = self.camera_info(cam)
        # centred crop of `full` to the segmentation aspect ratio
        if width * seg.height >= height * seg.width:   # full is wider -> crop width
            crop_h = height
            crop_w = height * seg.width / seg.height
        else:                                          # full is taller -> crop height
            crop_w = width
            crop_h = width * seg.height / seg.width
        crop_x0 = (width - crop_w) / 2.0
        crop_y0 = (height - crop_h) / 2.0
        sx = crop_w / seg.width
        sy = crop_h / seg.height
        K = seg.K.copy()
        K[0, 0] *= sx                       # fx
        K[1, 1] *= sy                       # fy
        K[0, 2] = seg.K[0, 2] * sx + crop_x0  # cx
        K[1, 2] = seg.K[1, 2] * sy + crop_y0  # cy
        return CameraInfo(width=width, height=height, K=K, D=seg.D.copy(),
                          distortion_model=seg.distortion_model)

    def iter_rgb(self, start_ns: int | None = None, end_ns: int | None = None):
        """Stream (t_ns, cam, full-res BGR) by decoding the HEVC image_raw/ffmpeg
        streams. One interleaved pass; each camera has its own decoder.

        HEVC needs a keyframe before the first frame decodes, so a range that
        does not begin on a keyframe yields nothing until the next one.

        Each decoded frame is stamped with the header time of the packet that
        completed it. These camera encoders emit one frame per packet with no
        B-frame reordering, so packet stamp == frame stamp; a codec with
        reordering/delay would shift stamps by whole frame periods here.
        """
        topics = {_topic(c, "image_raw/ffmpeg"): c for c in CAMERAS}
        decoders = {c: av.CodecContext.create("hevc", "r") for c in CAMERAS}
        last_ts: dict[str, int] = {}
        with open(self.path, "rb") as fh:
            for _schema, channel, _msg, ros_msg in self._reader(fh).iter_decoded_messages(
                topics=list(topics), start_time=start_ns, end_time=end_ns,
            ):
                cam = topics[channel.topic]
                t_ns = _stamp_ns(ros_msg.header)
                last_ts[cam] = t_ns
                try:
                    frames = decoders[cam].decode(av.Packet(bytes(ros_msg.data)))
                except av.FFmpegError:
                    continue  # skip undecodable packet (e.g. before first keyframe)
                for frame in frames:
                    yield t_ns, cam, frame.to_ndarray(format="bgr24")
            # Flush frames still buffered in libav at end-of-range (stamped with
            # that camera's last packet time — exact for these no-reorder streams).
            for cam, dec in decoders.items():
                if cam not in last_ts:
                    continue
                try:
                    frames = dec.decode(None)
                except av.FFmpegError:
                    continue
                for frame in frames:
                    yield last_ts[cam], cam, frame.to_ndarray(format="bgr24")

    def iter_images(self, start_ns: int | None = None, end_ns: int | None = None):
        """Stream (t_ns, cam, BGR) for segmentation/compressed in time order.

        One pass over the bag — used by video rendering, which keeps the latest
        frame per camera rather than re-scanning a window per output frame.
        """
        topics = {_topic(c, "segmentation/compressed"): c for c in CAMERAS}
        with open(self.path, "rb") as fh:
            for _schema, channel, _msg, ros_msg in self._reader(fh).iter_decoded_messages(
                topics=list(topics), start_time=start_ns, end_time=end_ns,
            ):
                img = cv2.imdecode(np.frombuffer(bytes(ros_msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    yield _stamp_ns(ros_msg.header), topics[channel.topic], img
