---
issue: 1
---

# Issue #1 — Port to ROS 2 / jazzy with 4-camera stabilized panorama

## Plan Authored
**Status**: complete
**When**: 2026-05-27 13:10 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-1/plan.md` at `0dffa9a` (Phase 1 scope only)
**PR**: https://github.com/rolker/image_warper/pull/2 (`[PLAN]` prefix, base `jazzy`, `Part of #1` — does not close umbrella)
**Phases**: single PR for Phase 1; Phases 2–3 deferred to their own plans

### Open questions
- [x] Sub-issue vs. direct branch — Phase 1 on `feature/issue-1`, or file a Phase 1 sub-issue + stacked PR (issue body suggested a sub-issue)?
- [x] Imagery source — validate geometry on `segmentation/compressed` then add HEVC RGB, or decode `image_raw/ffmpeg` (true RGB) from the start?
- [x] Orientation source — if this bag's `/tf` lacks a level/world frame, OK to join the parallel `~/data/logs/bizzyboat/<run>` bag by timestamp?
- [x] Build breakage — add `COLCON_IGNORE` to stop `make build` failing on the unported catkin package in this PR, or out of Phase 1 scope?
- [x] Test rigor — one synthetic geometry check for a throwaway prototype, or more before Phase 2?

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-02 17:20 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: changes-requested

**Branch**: feature/issue-1 at `dc9a0cc`
**Mode**: pre-push
**Depth**: Deep (reason: 969 changed lines ≥ 200)
**Must-fix**: 3 | **Suggestions**: 15
**Round**: 1 | **Ship**: continue — 3 mechanical must-fixes (silent-failure paths on plausible field data); one fix pass then re-verify

### Findings
- [x] (must-fix) cv2.imwrite return unchecked — silent failure + false "wrote" message — `prototype/make_panorama.py:70`
- [x] (must-fix) missing mcap statistics leaves start_ns/end_ns=0 silently (truncated field bags) — `prototype/bag_source.py:105`
- [x] (must-fix) KeyError when a camera lacks camera_info; degrade to present cameras — `prototype/make_panorama.py:52,91`
- [x] (suggestion) plan sync: roll_pitch default vs plan's roll-only, --time vs --frame, seam choice, #41 stamp discovery, full-res intrinsics assumption — `.agent/work-plans/issue-1/plan.md`
- [x] (suggestion) tick resolved open-question checkboxes + add Implementation entry — `.agent/work-plans/issue-1/progress.md`
- [x] (suggestion) fix broken ADR-0009 relative link (use absolute GitHub URL) — `prototype/README.md:14`
- [x] (suggestion) scope *.png/*.mp4 gitignore patterns to prototype/ — `.gitignore`
- [x] (suggestion) warn on TF lookup gap (clamp beyond threshold) — `prototype/bag_source.py:169`
- [x] (suggestion) flush HEVC decoders at end-of-range; note packet-vs-frame stamp caveat — `prototype/bag_source.py:258`
- [x] (suggestion) staleness bound / anchor-death warning in video loop — `prototype/make_panorama.py:81-99`
- [x] (suggestion) staleness bound in rgb still path — `prototype/make_panorama.py:59`
- [x] (suggestion) warn or handle multi-file (split) bag directories — `prototype/make_panorama.py:36`
- [x] (suggestion) precompute per-edge TF times array (O(N·M) rebuild per lookup) — `prototype/bag_source.py:167`
- [x] (suggestion) drop unused imageio[ffmpeg] dep or implement fallback — `prototype/requirements.txt:8`
- [x] (suggestion) validate CLI arg interactions (--time with --video, --end<=--start) — `prototype/make_panorama.py:122`
- [x] (suggestion) test runner: count non-AssertionError exceptions as failures — `prototype/test_geometry.py:89`
- [x] (suggestion) orientation() docstring direction inverted — `prototype/bag_source.py:177`
- [x] (suggestion) 7x E501 line length >99 — various

## Implementation
**Status**: complete
**When**: 2026-07-02 18:05 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Branch**: feature/issue-1 at `aa780b8`
**Addressed**: Local Review (Pre-Push) 2026-07-02 17:20 at `dc9a0cc` (3 must-fix, 15 suggestions)
**Commits**: e812691, b84fdbf, 07ad422, 3c314d9, 7f1b9fc, b44e5cd, faca6cc, ca7a58d, 9a987fe, 57197c7, b24f107, aa780b8

### Actions
- [x] imwrite failure now exits with a message — `prototype/make_panorama.py` (e812691)
- [x] bags without summary statistics rejected with recovery hint — `prototype/bag_source.py` (b84fdbf)
- [x] degrade to present cameras (available_cameras()) — `prototype/make_panorama.py`, `bag_source.py` (07ad422)
- [x] plan synced: roll_pitch default, --time, seam-270, #41 stamp discovery, intrinsics assumption — `plan.md` (b24f107)
- [x] plan-authored open-question checkboxes ticked; this entry records implementation — `progress.md`
- [x] README ADR-0009 link now absolute GitHub URL — `prototype/README.md` (57197c7)
- [x] gitignore *.png/*.mp4 scoped to prototype/ — `.gitignore` (57197c7)
- [x] TF lookup gap warning (once per edge, >1s) — `prototype/bag_source.py` (3c314d9)
- [x] HEVC decoders flushed at end-of-range; packet-stamp caveat documented — `prototype/bag_source.py` (7f1b9fc)
- [x] stale-tile warnings in video loop (once per camera) — `prototype/make_panorama.py` (b44e5cd)
- [x] stale-frame warning in rgb still path — `prototype/make_panorama.py` (b44e5cd)
- [x] split multi-.mcap bag dirs rejected with explicit listing — `prototype/make_panorama.py` (faca6cc)
- [x] per-edge TF time index precomputed (was O(N*M) rebuild) — `prototype/bag_source.py` (3c314d9)
- [x] unused imageio[ffmpeg] dropped from requirements + plan table — `prototype/requirements.txt` (ca7a58d)
- [x] CLI arg-combination validation (--time/--video, --end<=--start, still+video flags) — `prototype/make_panorama.py` (faca6cc)
- [x] test runner counts non-assertion exceptions as failures — `prototype/test_geometry.py` (9a987fe)
- [x] orientation() docstring direction corrected — `prototype/bag_source.py` (57197c7)
- [x] 7x E501 wrapped; flake8 (ament profile) clean — all prototype files (aa780b8)

Verified after fixes: test_geometry 3/3; seg + rgb (--stamp-offset 0.6) stills and a
10 s seg video render correctly; all four new CLI/IO error paths exercised and fail
loudly with the intended messages.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-02 18:35 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-1 at `514d40c`
**Mode**: pre-push
**Depth**: Deep (reason: 969 changed lines >= 200; round-2 re-review)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 2 | **Ship**: recommended — all 18 round-1 findings verified fixed under cold re-derivation; new items are degradation-path polish only

### Findings
- [ ] (suggestion) anchor-camera stall silently truncates video; stale warning gated on anchor emits — `prototype/make_panorama.py:117`
- [ ] (suggestion) image iterators use CAMERAS not available_cameras(); imagery-without-camera_info still KeyErrors — `prototype/bag_source.py:231`
- [ ] (suggestion) validate --fps > 0 (explicit 0 silently becomes 5.0; negative reaches VideoWriter) — `prototype/make_panorama.py:173`
- [ ] (suggestion) wrap BagSource construction so ValueError/KeyError exit cleanly like other CLI errors — `prototype/make_panorama.py:167`
