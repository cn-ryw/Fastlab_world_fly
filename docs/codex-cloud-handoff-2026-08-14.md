# Codex Cloud handoff — 2026-08-14

This is a sanitized continuation record for a future Codex Cloud chat. It summarizes repository state and verification evidence; it is not a raw export of local chat history.

## Repository and branch

- Repository: `cn-ryw/MindCloud_World_Fly` (public GitHub repository)
- Working branch: `feat/da360-metric-depth-v2`
- Remote branch baseline when this handoff was captured: `73eb65215c33d75e611b8dafd5f95a48929e1512`
- Source-work HEAD captured by this handoff: `aa1066b`
- Before the handoff metadata commit was created and pushed, the local branch was 8 commits ahead of that baseline: 68 files changed, 9,927 insertions, and 1,498 deletions.
- Do not push this branch to `upstream`; the intended publication target is `origin` only.

The eight local commits, oldest first, are:

1. `dba37be` — optimize the YOPO real-time path and terminal control
2. `a477ef3` — optimize the real-time flight path and remove the unauthorized SHA mechanism
3. `5f9903d` — add T8L serial control and the YOPO rolling goal
4. `71be31d` — preserve the Chrome perception pipeline and SO3 control baseline
5. `3f1e7dd` — checkpoint raw RGBA planning and the 50 m rolling goal
6. `22aad2c` — checkpoint the `demo30` panorama and control baseline
7. `bf0ac4b` — preserve panorama scheduling diagnostics
8. `aa1066b` — preserve deterministic panorama scheduling

## System objective and current evidence boundary

The project is a browser-based Google Photorealistic 3D Tiles UAV simulator. Its intended navigation path is Cesium cubemap rendering to 384×192 ERP RGB, DA360 metric depth estimation, YOPO Poly5 trajectory generation, and browser-side SO3 tracking.

The active engineering objective remains a defensible YOPO_360 sim-to-sim migration. Do not equate implemented code or unit coverage with live acceptance. In particular:

- DA360 metric scale-only calibration was manually accepted for sim-to-sim use, but its automatic LOLO accuracy gates did not pass.
- Current code does not yet have accepted dense Cesium truth depth.
- Current code does not yet have accepted Firefox/GPU 15 Hz closed-loop flight evidence.
- Live model, calibration, experiment data, and GPU/browser state are local dependencies and are not expected to exist in a fresh cloud container.

Read these sources before changing behavior:

- `README.md`
- `docs/implementation-status-v2.md`
- `docs/da360-metric-scale-diagnosis.md`
- `docs/yopo-strategy-selection.md`
- `dependencies.versions.json`

## Uncommitted local work — not available to cloud from Git yet

The local worktree contains a separate, unstaged patch touching:

- `index.html`
- `launch-chrome-gpu.sh`
- `launch-firefox-gpu.sh`
- `src/cesium-world.js`
- `src/demo-performance.js`
- `src/main.js`

Patch size at handoff: 328 insertions and 25 deletions.

The patch adds a fail-closed fixed-goal collision-corridor preload, route-local coverage sampling and gap diagnostics, preload time budgeting, request-scheduler tuning during preload, and larger `demo30` preload defaults. It blocks fixed-goal navigation when collision coverage is incomplete. This patch has not been committed because its verification is not green and its ownership/scope must be confirmed before publication.

Do not recreate this patch from the summary alone. Either continue from the local machine through Codex Remote, or first make an explicit, reviewed commit containing exactly the six files above.

## Verification performed at handoff

### JavaScript tests

The test loop stopped at the first failure:

- `tests/test_collision_sweep_cache.js`: 5 passed, 0 failed.
- `tests/test_controller_config_migration.js`: 35 passed, 1 failed.
- Failing assertion: `damaged arm binding fails safe to unassigned`.
- Remaining JavaScript test files were not run after that failure.

### Python gates

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_fit_da360_metric.py \
  tests/test_serve_security.py \
  tests/test_evaluation_gates.py
```

Result: 129 passed, 1 failed.

The failing contract says the closed-loop evaluator URL allowlist no longer matches the flight logger. `src/flight-logger.js` allows `perfProfile`, `dynamicSse`, and `tileRequestsPerServer`, while `scripts/evaluate_closed_loop.py` does not.

Do not report the branch or the uncommitted patch as test-passing until these failures are understood and the complete lightweight test sweep is rerun.

## Safety and synchronization exclusions

The following local runtime artifacts were intentionally neither read nor selected for upload:

- `.chrome-mindcloud-flight/`
- `.chrome-mindcloud-profile/`
- `.dev-chrome.log`

They may contain browser state, login information, browsing data, or machine-specific logs. Keep them out of Git and out of cloud prompts.

A JWT-like Cesium access token already exists in `src/cesium-world.js` on the remote branch. No new credential-like material was detected in added lines across the eight local commits, but future publication must still treat the repository as public and avoid adding secrets, private logs, raw experiment data, model weights, or local browser profiles.

## Suggested first cloud prompt

> Continue the `feat/da360-metric-depth-v2` work using this handoff as the evidence boundary. First inspect the current branch, `README.md`, and `docs/implementation-status-v2.md`. Do not assume the uncommitted local six-file corridor-preload patch exists. Reproduce the two recorded lightweight test failures before proposing fixes. Preserve fail-closed planning authorization and do not claim live Firefox/GPU, metric-accuracy, or 15 Hz acceptance without new raw evidence.
