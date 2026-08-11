# DA360 metric-scale diagnosis and simulator A/B protocol

## Corrected conclusion

The 12 audited captures reject one fixed cross-scene mapping

```text
inverse_depth = a * pred_disp + b
```

for the current low-resolution input pipeline. They do **not** prove that the
captures were unusable, that DA360 cannot support an online scale cue, or that
panoramic metric depth is impossible.

DA360's documented output is *scale-invariant disparity*: the image-dependent
shift is removed, but an image-dependent scale remains. Its official evaluator
recovers one scale per image from ground truth, and its example inference code
normalizes each predicted image independently. A single permanent `a` therefore
exceeds the model's published contract.

Primary sources:

- [DA360 paper](https://arxiv.org/html/2512.22819)
- [DA360 official evaluation](https://github.com/Insta360-Research-Team/DA360/blob/main/evaluate.py)
- [DA360 official inference example](https://github.com/Insta360-Research-Team/DA360/blob/main/test.py)
- [Depth Any Panoramas metric model](https://arxiv.org/html/2512.16913)
- [Depth Any Panoramas code and weights](https://github.com/Insta360-Research-Team/DAP)

## What the current evidence establishes

| Evidence | Result | Interpretation boundary |
|---|---:|---|
| Atomic bundles | 12 unique frames/captures; hashes and identities agree | No obvious file mixing |
| Vehicle speed | maximum about `1.98e-5 m/s` | Effectively stationary |
| Ready ray anchors | 1,140 / 1,536 (74.22%) | Coverage passed; not an independent proof of ray geometry |
| Anchors in 0.5–20 m | 833 | Enough to reject the current fit |
| Current global LOLO median AbsRel | 37.6% | Fails the 15% gate |
| Current global LOLO p90 AbsRel | 150.7% | Fails the 30% gate |
| Per-frame scale max/min | 4.6324× | The earlier approximate 4.45× statement was imprecise |
| Robust scale p90/p10 | 1.904× | The 4.63× extreme is not representative of every frame |
| Per-frame scale-only oracle | 11/12 median gates, only 4/12 p90 gates pass | Scale explains much, not the unsafe tail |

Within-site max/min is 1.176× (A), 3.309× (B), 1.025× (C), and 1.102×
(D). Site B contains the dominant frame anomaly. Scale is also strongly
confounded with distance: near and far sites do not have balanced range
distributions. The present data cannot uniquely separate scene semantics,
distance non-linearity, low source resolution, and a residual geometry/model
error.

Every accepted RGB input was only 134×67 JPEG and was enlarged by the service
to 476×238. The prior 1036×518 pilot enlarged the same 134×67 source; it was a
model-grid experiment, not a native-source-resolution test.

The quantitative figure in the experiment evidence directory shows the raw
frame scales, anchor-distance association, and LOLO tail errors. It deliberately
states association rather than assigning one cause.

## User-visible, falsifiable simulator experiment

First implement exact pose export/restore so every arm sees the same position,
yaw, tiles, and capture revision. Do not rely on manually returning to a similar
view.

Run two fixed poses as a pilot before collecting another 12-frame dataset:

| Arm | ERP source | Upload to DA360 | DA360 model grid | Purpose |
|---|---:|---:|---:|---|
| A | 384×192, face 96 | 134×67 JPEG74 | 476×238 | Current baseline |
| B | 476×238, face about 192 | 476×238 near-lossless | 476×238 | Isolate lost source detail |
| C | 1036×518, face about 384 | 1036×518 near-lossless | 1036×518 | True native end-to-end test |

Each pose should contain balanced anchors at 1–2, 3–5, 8–12, and 15–20 m,
using a 32×16 grid and a separately reported depth-edge mask. For the exact
same frozen RGB SHA, repeat raw inference 30 times to distinguish model/input
effects from nondeterminism.

For each arm, report all of the following instead of a single fit number:

1. no alignment;
2. global scale-only (`b=0`);
3. per-location scale-only;
4. per-frame oracle scale-only, as a non-deployable upper bound;
5. per-frame affine, as a shape diagnostic only;
6. horizontal ERP roll tests at 22.5°, 45°, and 90°, rolled back before comparison.

Then compare four closed-loop sources on the same route and initial state:

- Cesium metric truth + YOPO (control/coordinate baseline);
- DA360 + per-frame oracle scale (upper bound only);
- DA360 + a deployable online sparse metric cue and temporal filter;
- a purpose-trained panoramic metric model such as DAP (offline A/B first).

The operator's simulator observations are part of acceptance, but planning
authorization remains fail-closed until saved metrics and closed-loop logs pass
the existing accuracy, collision, latency, and effective-replanning gates.

## Direction convention used by the depth top-down view

- local `+X` is east, `+Y` is up, and `+Z` is north;
- simulator yaw 0 faces body forward toward local `-Z` (south);
- compass bearing is `(sim_yaw + 180°) mod 360`;
- the visualization is north-up, with the capture yaw rotating the ERP scan;
- metric/truth endpoints are translated from capture position to the current
  drone centre; relative depth is explicitly marked capture-centric.

The current reset orientation is deterministic, not random: yaw 0, facing
south. The north-up compass supplies the world reference without silently
changing the established dynamics convention. A future decision to spawn
physically facing north would require yaw 180° plus controller/config migration
tests, rather than only relabelling the compass.
