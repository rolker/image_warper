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
- [ ] Sub-issue vs. direct branch — Phase 1 on `feature/issue-1`, or file a Phase 1 sub-issue + stacked PR (issue body suggested a sub-issue)?
- [ ] Imagery source — validate geometry on `segmentation/compressed` then add HEVC RGB, or decode `image_raw/ffmpeg` (true RGB) from the start?
- [ ] Orientation source — if this bag's `/tf` lacks a level/world frame, OK to join the parallel `~/data/logs/bizzyboat/<run>` bag by timestamp?
- [ ] Build breakage — add `COLCON_IGNORE` to stop `make build` failing on the unported catkin package in this PR, or out of Phase 1 scope?
- [ ] Test rigor — one synthetic geometry check for a throwaway prototype, or more before Phase 2?

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
- [ ] (must-fix) cv2.imwrite return unchecked — silent failure + false "wrote" message — `prototype/make_panorama.py:70`
- [ ] (must-fix) missing mcap statistics leaves start_ns/end_ns=0 silently (truncated field bags) — `prototype/bag_source.py:105`
- [ ] (must-fix) KeyError when a camera lacks camera_info; degrade to present cameras — `prototype/make_panorama.py:52,91`
- [ ] (suggestion) plan sync: roll_pitch default vs plan's roll-only, --time vs --frame, seam choice, #41 stamp discovery, full-res intrinsics assumption — `.agent/work-plans/issue-1/plan.md`
- [ ] (suggestion) tick resolved open-question checkboxes + add Implementation entry — `.agent/work-plans/issue-1/progress.md`
- [ ] (suggestion) fix broken ADR-0009 relative link (use absolute GitHub URL) — `prototype/README.md:14`
- [ ] (suggestion) scope *.png/*.mp4 gitignore patterns to prototype/ — `.gitignore`
- [ ] (suggestion) warn on TF lookup gap (clamp beyond threshold) — `prototype/bag_source.py:169`
- [ ] (suggestion) flush HEVC decoders at end-of-range; note packet-vs-frame stamp caveat — `prototype/bag_source.py:258`
- [ ] (suggestion) staleness bound / anchor-death warning in video loop — `prototype/make_panorama.py:81-99`
- [ ] (suggestion) staleness bound in rgb still path — `prototype/make_panorama.py:59`
- [ ] (suggestion) warn or handle multi-file (split) bag directories — `prototype/make_panorama.py:36`
- [ ] (suggestion) precompute per-edge TF times array (O(N·M) rebuild per lookup) — `prototype/bag_source.py:167`
- [ ] (suggestion) drop unused imageio[ffmpeg] dep or implement fallback — `prototype/requirements.txt:8`
- [ ] (suggestion) validate CLI arg interactions (--time with --video, --end<=--start) — `prototype/make_panorama.py:122`
- [ ] (suggestion) test runner: count non-AssertionError exceptions as failures — `prototype/test_geometry.py:89`
- [ ] (suggestion) orientation() docstring direction inverted — `prototype/bag_source.py:177`
- [ ] (suggestion) 7x E501 line length >99 — various
