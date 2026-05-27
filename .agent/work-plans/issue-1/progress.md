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
