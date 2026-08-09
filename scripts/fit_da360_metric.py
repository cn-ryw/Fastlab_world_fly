#!/usr/bin/env python3
"""
Offline inverse-depth metric fitting: 1/z = a * pred_disp + b.

Takes DA360 raw pred_disp (.npz from /depth/raw) and Cesium sparse anchors
(.json), fits scale-only and scale+shift models via robust regression, and
outputs metric-depth arrays together with a validation report.

Usage:
    python scripts/fit_da360_metric.py \\
        --raw depth_raw.npz \\
        --anchors cesium_anchors.json \\
        --output experiment_data/metric_fit_sample/

Repeat the paired --raw/--anchors flags for multi-capture fitting. Anchor
metadata must include locationId for leave-one-location-out validation; a
runtime depth_calibration.json is emitted only after all acceptance gates pass.
"""
import argparse
import json
import os
import sys
import warnings
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from PIL import Image

warnings.filterwarnings("ignore", category=RuntimeWarning)

EPS = np.float32(1e-6)
SEED = 42
DEFAULT_HUBER_DELTA = 1.345  # standard 95%-efficiency Huber constant


# ---------------------------------------------------------------------------
# Depth colour map (matches da360_server.py for visual comparison)
# ---------------------------------------------------------------------------
def depth_to_color(depth, sample_limit=65536):
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        return np.zeros((*depth.shape, 3), dtype=np.uint8)
    vals = depth[valid]
    if vals.size > sample_limit:
        vals = vals[::max(1, int(np.ceil(vals.size / sample_limit)))]
    near, far = np.percentile(vals, 2.0), np.percentile(vals, 98.0)
    t = np.zeros(depth.shape, dtype=np.float32)
    t[valid] = 1.0 - (np.clip(depth[valid], near, far) - near) / max(far - near, 1e-6)
    t = np.clip(t, 0, 1)
    stops = np.array([[4, 3, 30], [20, 25, 210], [0, 210, 255], [92, 255, 120],
                      [255, 238, 67], [255, 64, 43], [210, 38, 255]], dtype=np.float32)
    scaled = t * (len(stops) - 1)
    lo = np.floor(scaled).astype(np.int32)
    hi = np.clip(lo + 1, 0, len(stops) - 1)
    frac = (scaled - lo)[..., None]
    color = stops[lo] * (1 - frac) + stops[hi] * frac
    color[~valid] = 0
    return color.astype(np.uint8)


# ---------------------------------------------------------------------------
# Fitting
# ---------------------------------------------------------------------------
def _validate_fit_inputs(x, y, min_points):
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    y = np.asarray(y, dtype=np.float64).reshape(-1)
    if x.size != y.size:
        raise ValueError("x and y must have the same length")
    if x.size < min_points:
        raise ValueError(f"need at least {min_points} finite samples")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("fit samples must be finite")
    return x, y


def _linear_residuals(params, x, y):
    """Raw inverse-depth residuals; robustification is applied once by SciPy."""
    a, b = params
    return y - (a * x + b)


def fit_scale_shift(x, y, delta=DEFAULT_HUBER_DELTA):
    """Huber-robust fit: y = a*x + b  with a > 0 constraint."""
    x, y = _validate_fit_inputs(x, y, min_points=2)
    if np.ptp(x) <= 1e-12:
        raise ValueError("scale+shift fit requires non-constant disparity samples")
    if not np.isfinite(delta) or delta <= 0:
        raise ValueError("Huber delta must be positive and finite")
    # Initial guess via Theil-Sen (median of pairwise slopes)
    n = len(x)
    if n >= 2:
        idx = np.random.default_rng(SEED).choice(n, size=min(n, 200), replace=False)
        xs, ys = x[idx], y[idx]
        slopes = []
        for i in range(len(xs)):
            for j in range(i + 1, len(xs)):
                if abs(xs[j] - xs[i]) > 1e-9:
                    slopes.append((ys[j] - ys[i]) / (xs[j] - xs[i]))
        a0 = float(np.median(slopes)) if slopes else 1.0
    else:
        a0 = 1.0
    a0 = max(1e-9, a0)
    b0 = float(np.median(y - a0 * x))

    result = least_squares(
        lambda p: _linear_residuals(p, x, y),
        [a0, b0],
        bounds=([1e-12, -np.inf], [np.inf, np.inf]),
        max_nfev=2000,
        loss='huber',
        f_scale=delta,
    )
    a, b = float(result.x[0]), float(result.x[1])
    residuals = y - (a * x + b)
    return a, b, residuals


def fit_scale_only(x, y, delta=DEFAULT_HUBER_DELTA):
    """Huber-robust fit: y = a*x  (no intercept)."""
    x, y = _validate_fit_inputs(x, y, min_points=1)
    if not np.isfinite(delta) or delta <= 0:
        raise ValueError("Huber delta must be positive and finite")
    nonzero = np.abs(x) > 1e-12
    if not np.any(nonzero):
        raise ValueError("scale-only fit requires non-zero disparity samples")
    a0 = max(1e-9, float(np.median(y[nonzero] / x[nonzero])))
    result = least_squares(
        lambda a: y - a[0] * x,
        [a0],
        bounds=([1e-12], [np.inf]),
        max_nfev=2000,
        loss='huber',
        f_scale=delta,
    )
    a = float(result.x[0])
    residuals = y - a * x
    return a, residuals


def fit_ransac(x, y, n_trials=100, min_samples=4, inlier_thresh=0.5):
    """Simple RANSAC wrapper: returns best (a, b, inlier_mask)."""
    x, y = _validate_fit_inputs(x, y, min_points=2)
    if len(x) < min_samples:
        raise ValueError(f"RANSAC needs at least {min_samples} samples")
    best_inliers = 0
    best_a, best_b = 1.0, 0.0
    best_mask = np.zeros(len(x), dtype=bool)
    rng = np.random.default_rng(SEED)

    for _ in range(n_trials):
        idx = rng.choice(len(x), size=min(len(x), min_samples), replace=False)
        try:
            a, b, _ = fit_scale_shift(x[idx], y[idx])
        except Exception:
            continue
        residuals = np.abs(y - (a * x + b))
        mask = residuals < inlier_thresh
        nin = mask.sum()
        if nin > best_inliers:
            best_inliers = nin
            best_a, best_b = a, b
            best_mask = mask

    if best_inliers >= min_samples:
        a, b, _ = fit_scale_shift(x[best_mask], y[best_mask])
        return a, b, best_mask
    return best_a, best_b, best_mask


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def compute_metrics(y_true, y_pred, valid=None):
    """Standard metrics in metres; non-positive/non-finite depths are invalid."""
    y_true = np.asarray(y_true, dtype=np.float64)
    y_pred = np.asarray(y_pred, dtype=np.float64)
    if y_true.shape != y_pred.shape:
        raise ValueError("metric arrays must have the same shape")
    if valid is None:
        valid = np.ones_like(y_true, dtype=bool)
    else:
        valid = np.asarray(valid, dtype=bool)
        if valid.shape != y_true.shape:
            raise ValueError("metric validity mask must match depth shape")
    valid = valid & np.isfinite(y_true) & np.isfinite(y_pred) & (y_true > 0) & (y_pred > 0)
    yt, yp = y_true[valid], y_pred[valid]
    if len(yt) < 2:
        return {
            "median_abs_rel": None,
            "p90_abs_rel": None,
            "rmse_m": None,
            "median_error_m": None,
            "p90_error_m": None,
            "n_valid": int(valid.sum()),
        }
    abs_rel = np.median(np.abs(yt - yp) / np.maximum(yt, EPS))
    p90_val = np.percentile(np.abs(yt - yp) / np.maximum(yt, EPS), 90)
    rmse = np.sqrt(np.mean((yt - yp) ** 2))
    abs_err = np.abs(yt - yp)
    return {
        "median_abs_rel": float(abs_rel),
        "p90_abs_rel": float(p90_val),
        "rmse_m": float(rmse),
        "median_error_m": float(np.median(abs_err)),
        "p90_error_m": float(np.percentile(abs_err, 90)),
        "n_valid": int(valid.sum()),
    }


def inverse_depth_to_metric_depth(inverse_depth, base_valid=None):
    """Convert positive inverse depth (1/m) to metres and return its validity."""
    inverse_depth = np.asarray(inverse_depth)
    valid = np.isfinite(inverse_depth) & (inverse_depth > 0)
    if base_valid is not None:
        base_valid = np.asarray(base_valid, dtype=bool)
        if base_valid.shape != inverse_depth.shape:
            raise ValueError("base validity mask must match inverse-depth shape")
        valid &= base_valid
    depth = np.full(inverse_depth.shape, np.nan, dtype=np.float32)
    depth[valid] = (1.0 / inverse_depth[valid]).astype(np.float32)
    return depth, valid


def map_pixel_center(coordinate, source_size, target_size):
    """Map an integer-centre pixel coordinate between equal-FOV resolutions."""
    if not all(np.isfinite(v) for v in (coordinate, source_size, target_size)):
        raise ValueError("pixel mapping values must be finite")
    if source_size <= 0 or target_size <= 0:
        raise ValueError("image dimensions must be positive")
    return (float(coordinate) + 0.5) * float(target_size) / float(source_size) - 0.5


def sample_wrapped_bilinear(array, valid_mask, u, v):
    """Bilinear ERP sample with horizontal seam wrap and vertical edge clamp."""
    array = np.asarray(array)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    if array.ndim != 2 or valid_mask.shape != array.shape:
        raise ValueError("bilinear sampler requires matching 2-D array and mask")
    if not np.isfinite(u) or not np.isfinite(v):
        return None
    height, width = array.shape
    if height == 0 or width == 0 or v < -0.5 or v > height - 0.5:
        return None

    wrapped_u = float(u) % width
    clamped_v = min(max(float(v), 0.0), height - 1.0)
    x0_raw = int(np.floor(wrapped_u))
    y0 = int(np.floor(clamped_v))
    x1_raw = x0_raw + 1
    y1 = min(y0 + 1, height - 1)
    fx = wrapped_u - x0_raw
    fy = clamped_v - y0
    x0, x1 = x0_raw % width, x1_raw % width

    weighted = (
        (x0, y0, (1.0 - fx) * (1.0 - fy)),
        (x1, y0, fx * (1.0 - fy)),
        (x0, y1, (1.0 - fx) * fy),
        (x1, y1, fx * fy),
    )
    value = 0.0
    for x, y, weight in weighted:
        if weight <= 1e-15:
            continue
        if not valid_mask[y, x] or not np.isfinite(array[y, x]):
            return None
        value += float(array[y, x]) * weight
    return value


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def load_data(raw_path, anchors_path):
    """Load raw pred_disp .npz and Cesium anchors .json."""
    with np.load(raw_path, allow_pickle=False) as raw:
        missing = {"pred_disp", "valid_mask", "metadata_json"} - set(raw.files)
        if missing:
            raise ValueError(f"raw NPZ missing arrays: {', '.join(sorted(missing))}")
        pred_disp = np.asarray(raw["pred_disp"], dtype=np.float32).copy()
        valid_mask = np.asarray(raw["valid_mask"], dtype=bool).copy()
        md = json.loads(str(raw["metadata_json"]))
    if pred_disp.ndim != 2 or valid_mask.shape != pred_disp.shape:
        raise ValueError("pred_disp and valid_mask must be matching 2-D arrays")
    H, W = pred_disp.shape

    with open(anchors_path) as f:
        anchor_data = json.load(f)

    anchors = anchor_data.get("anchors", [])
    failures = anchor_data.get("failures", [])
    meta = anchor_data.get("metadata", {})
    source_w = float(meta.get("imageWidth", W))
    source_h = float(meta.get("imageHeight", H))
    if not np.isfinite(source_w) or not np.isfinite(source_h) or source_w <= 0 or source_h <= 0:
        raise ValueError("anchor metadata imageWidth/imageHeight must be positive")

    # Map integer-centre ERP coordinates to raw output coordinates.  Width
    # wraps across the ERP seam; height uses edge clamping.
    x_vals, inv_depth_vals, location_ids, sampled_anchors = [], [], [], []
    location_value = meta.get("locationId", meta.get("location_id", ""))
    default_location = "" if location_value is None else str(location_value)
    anchor_frame_id = meta.get("frameId", meta.get("frame_id"))
    raw_frame_id = md.get("frame_id")
    if anchor_frame_id is not None and raw_frame_id is not None and str(anchor_frame_id) != str(raw_frame_id):
        raise ValueError(
            f"anchor/raw frame mismatch: {anchor_frame_id!r} != {raw_frame_id!r}"
        )
    for a in anchors:
        try:
            source_u = float(a["u"])
            source_v = float(a["v"])
            distance_m = float(a["distance"])
        except (KeyError, TypeError, ValueError):
            continue
        if not np.isfinite(distance_m) or distance_m <= 0:
            continue
        raw_u = map_pixel_center(source_u, source_w, W)
        raw_v = map_pixel_center(source_v, source_h, H)
        disparity = sample_wrapped_bilinear(pred_disp, valid_mask, raw_u, raw_v)
        if disparity is None or not np.isfinite(disparity):
            continue
        x_vals.append(disparity)
        inv_depth_vals.append(1.0 / distance_m)
        anchor_location = a.get("locationId", a.get("location_id", default_location))
        location_ids.append("" if anchor_location is None else str(anchor_location))
        sampled_anchors.append({
            "u": source_u,
            "v": source_v,
            "raw_u": raw_u,
            "raw_v": raw_v,
            "distance_m": distance_m,
            "pred_disp": disparity,
        })

    x_arr = np.array(x_vals, dtype=np.float64)
    inverse_depth_arr = np.array(inv_depth_vals, dtype=np.float64)
    try:
        n_anchor_candidates = int(meta.get("totalCells", len(anchors) + len(failures)))
    except (TypeError, ValueError):
        n_anchor_candidates = len(anchors) + len(failures)
    n_anchor_candidates = max(len(anchors), n_anchor_candidates)

    return {
        "pred_disp": pred_disp,
        "valid_mask": valid_mask,
        "H": H, "W": W,
        "x": x_arr, "z": inverse_depth_arr,
        "location_ids": np.asarray(location_ids, dtype=str),
        "sampled_anchors": sampled_anchors,
        "metadata": md,
        "anchor_metadata": meta,
        "n_anchors": n_anchor_candidates,
        "n_hit_anchors": len(anchors),
        "n_valid_anchors": len(x_arr),
        "valid_anchor_fraction": len(x_arr) / n_anchor_candidates if n_anchor_candidates else 0.0,
        "_anchors_path": str(anchors_path),
        "sample_id": str(meta.get("captureId") or Path(raw_path).stem),
    }


def combine_datasets(datasets):
    """Combine captures for one calibration while preserving location groups."""
    if not datasets:
        raise ValueError("at least one raw/anchor capture pair is required")
    reference = datasets[0]
    reference_metadata = reference.get("metadata", {})
    compatibility_keys = (
        "model", "width", "height", "input_scale", "resample", "checkpoint_sha256"
    )
    reference_anchor_metadata = reference.get("anchor_metadata", {})
    for dataset in datasets[1:]:
        metadata = dataset.get("metadata", {})
        mismatches = [
            key for key in compatibility_keys
            if key in reference_metadata and key in metadata
            and reference_metadata[key] != metadata[key]
        ]
        if mismatches:
            raise ValueError(
                "capture metadata mismatch for calibration: " + ", ".join(mismatches)
            )
        anchor_metadata = dataset.get("anchor_metadata", {})
        if (
            "verticalFovDeg" in reference_anchor_metadata
            and "verticalFovDeg" in anchor_metadata
            and reference_anchor_metadata["verticalFovDeg"] != anchor_metadata["verticalFovDeg"]
        ):
            raise ValueError("capture metadata mismatch for calibration: verticalFovDeg")

    combined = dict(reference)
    combined["x"] = np.concatenate([dataset["x"] for dataset in datasets])
    combined["z"] = np.concatenate([dataset["z"] for dataset in datasets])
    combined["location_ids"] = np.concatenate([
        np.asarray(dataset.get("location_ids", []), dtype=str) for dataset in datasets
    ])
    combined["n_anchors"] = sum(dataset["n_anchors"] for dataset in datasets)
    combined["n_hit_anchors"] = sum(dataset.get("n_hit_anchors", 0) for dataset in datasets)
    combined["n_valid_anchors"] = len(combined["x"])
    combined["valid_anchor_fraction"] = (
        combined["n_valid_anchors"] / combined["n_anchors"]
        if combined["n_anchors"] else 0.0
    )
    combined["n_samples"] = len(datasets)
    combined["sample_ids"] = [dataset["sample_id"] for dataset in datasets]
    # Preview artifacts intentionally use the first sample only; the fitted
    # parameters and validation use every capture above.
    combined["preview_sample_id"] = reference["sample_id"]
    return combined


def _validation_folds(location_ids, n):
    """Prefer leave-one-location-out; otherwise make a deterministic holdout."""
    location_ids = np.asarray(location_ids, dtype=str).reshape(-1)
    if location_ids.size == n and np.all(location_ids != ""):
        unique_locations = sorted(set(location_ids.tolist()))
        folds = []
        for location in unique_locations:
            val_idx = np.flatnonzero(location_ids == location)
            train_idx = np.flatnonzero(location_ids != location)
            if train_idx.size >= 4 and val_idx.size >= 2:
                folds.append((train_idx, val_idx, location))
        if len(folds) >= 2:
            return "leave-one-location-out", folds

    rng = np.random.default_rng(SEED)
    idx = rng.permutation(n)
    n_train = min(n - 2, max(4, int(np.floor(n * 0.7))))
    return "deterministic-random-holdout", [(idx[:n_train], idx[n_train:], "random-holdout")]


def _fit_scale_shift_model(x, inverse_depth, use_ransac):
    if use_ransac:
        a, b, _ = fit_ransac(x, inverse_depth)
        return a, b
    a, b, _ = fit_scale_shift(x, inverse_depth)
    return a, b


def _depth_metrics_from_inverse(true_inverse, predicted_inverse, max_true_depth_m=None):
    true_depth, true_valid = inverse_depth_to_metric_depth(true_inverse)
    predicted_depth, predicted_valid = inverse_depth_to_metric_depth(predicted_inverse)
    valid = true_valid & predicted_valid
    if max_true_depth_m is not None:
        valid &= true_depth <= max_true_depth_m
    return compute_metrics(true_depth, predicted_depth, valid)


def _cross_validate(x, inverse_depth, folds, use_ransac):
    scale_predictions = np.full(len(x), np.nan, dtype=np.float64)
    shift_predictions = np.full(len(x), np.nan, dtype=np.float64)
    fold_reports = []
    for train_idx, val_idx, label in folds:
        a_scale, _ = fit_scale_only(x[train_idx], inverse_depth[train_idx])
        a_shift, b_shift = _fit_scale_shift_model(
            x[train_idx], inverse_depth[train_idx], use_ransac
        )
        scale_predictions[val_idx] = a_scale * x[val_idx]
        shift_predictions[val_idx] = a_shift * x[val_idx] + b_shift
        fold_reports.append({
            "held_out": label,
            "n_train": int(len(train_idx)),
            "n_val": int(len(val_idx)),
            "scale_only": _depth_metrics_from_inverse(
                inverse_depth[val_idx], scale_predictions[val_idx]
            ),
            "scale_shift": _depth_metrics_from_inverse(
                inverse_depth[val_idx], shift_predictions[val_idx]
            ),
        })

    scale_valid = np.isfinite(scale_predictions)
    shift_valid = np.isfinite(shift_predictions)
    return {
        "scale_only": _depth_metrics_from_inverse(
            inverse_depth[scale_valid], scale_predictions[scale_valid]
        ),
        "scale_shift": _depth_metrics_from_inverse(
            inverse_depth[shift_valid], shift_predictions[shift_valid]
        ),
        "scale_only_near_10m": _depth_metrics_from_inverse(
            inverse_depth[scale_valid], scale_predictions[scale_valid], max_true_depth_m=10.0
        ),
        "scale_shift_near_10m": _depth_metrics_from_inverse(
            inverse_depth[shift_valid], shift_predictions[shift_valid], max_true_depth_m=10.0
        ),
        "folds": fold_reports,
    }


def _write_json_atomic(path, payload):
    path = Path(path)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    with open(temporary_path, "w", encoding="utf-8") as output:
        json.dump(payload, output, indent=2, allow_nan=False)
        output.write("\n")
    os.replace(temporary_path, path)


def _write_report(output_dir, report):
    _write_json_atomic(Path(output_dir) / "fit_report.json", report)


def run_fitting(data, output_dir, use_ransac=False):
    os.makedirs(output_dir, exist_ok=True)
    # Never leave a previously accepted calibration in a reused output
    # directory after a later run fails or is interrupted before validation.
    for stale_name in ("depth_calibration.json", "depth_calibration_candidate.json"):
        (Path(output_dir) / stale_name).unlink(missing_ok=True)

    x = np.asarray(data["x"], dtype=np.float64)
    inverse_depth = np.asarray(data["z"], dtype=np.float64)
    n = len(x)
    if n < 6:
        report = {
            "success": False,
            "error": f"insufficient valid anchors: {n} (need >=6 for a held-out validation)",
            "n_total_anchors": int(data.get("n_anchors", n)),
            "n_valid_anchors": n,
        }
        _write_report(output_dir, report)
        return report

    finite = np.isfinite(x) & np.isfinite(inverse_depth) & (inverse_depth > 0)
    if not np.all(finite):
        x = x[finite]
        inverse_depth = inverse_depth[finite]
        location_ids = np.asarray(data.get("location_ids", []), dtype=str)
        location_ids = location_ids[finite] if location_ids.size == n else np.array([], dtype=str)
    else:
        location_ids = np.asarray(data.get("location_ids", []), dtype=str)
    n = len(x)
    if n < 6:
        report = {
            "success": False,
            "error": f"insufficient finite positive anchors: {n} (need >=6)",
            "n_total_anchors": int(data.get("n_anchors", n)),
            "n_valid_anchors": n,
        }
        _write_report(output_dir, report)
        return report

    try:
        validation_strategy, folds = _validation_folds(location_ids, n)
        validation = _cross_validate(x, inverse_depth, folds, use_ransac)
        final_a_scale, _ = fit_scale_only(x, inverse_depth)
        final_a_shift, final_b_shift = _fit_scale_shift_model(x, inverse_depth, use_ransac)
    except ValueError as error:
        report = {
            "success": False,
            "error": str(error),
            "n_total_anchors": int(data.get("n_anchors", n)),
            "n_valid_anchors": n,
        }
        _write_report(output_dir, report)
        return report

    scale_metrics = validation["scale_only"]
    shift_metrics = validation["scale_shift"]
    scale_error = scale_metrics["median_abs_rel"]
    shift_error = shift_metrics["median_abs_rel"]
    selected_model = (
        "scale_only"
        if scale_error is not None and shift_error is not None and scale_error - shift_error < 0.01
        else "scale_shift"
    )
    selected_a = final_a_scale if selected_model == "scale_only" else final_a_shift
    selected_b = 0.0 if selected_model == "scale_only" else final_b_shift

    report = {
        "success": True,
        "relation": "inverse_depth_1_per_m = a * pred_disp + b",
        "output_units": "metres",
        "pixel_sampling": "integer-pixel-centres; resolution-scaled; horizontal-wrap bilinear",
        "robust_loss": "huber-once",
        "huber_delta_inverse_m": DEFAULT_HUBER_DELTA,
        "n_total_anchors": int(data.get("n_anchors", n)),
        "n_valid_anchors": n,
        "valid_anchor_fraction": float(n / max(1, data.get("n_anchors", n))),
        "n_samples": int(data.get("n_samples", 1)),
        "sample_ids": list(data.get("sample_ids", [data.get("sample_id", "unknown")])),
        "model": data.get("metadata", {}).get("model", "unknown"),
        "seed": SEED,
        "validation": {
            "strategy": validation_strategy,
            "folds": validation["folds"],
        },
        "scale_only": {
            "a": float(final_a_scale),
            "b": 0.0,
            **scale_metrics,
            "near_10m": validation["scale_only_near_10m"],
        },
        "scale_shift": {
            "a": float(final_a_shift),
            "b": float(final_b_shift),
            **shift_metrics,
            "near_10m": validation["scale_shift_near_10m"],
        },
        "selected_model": selected_model,
        "calibration": {
            "a": float(selected_a),
            "b": float(selected_b),
            "relation": "inverse_depth_1_per_m = a * pred_disp + b",
        },
    }

    # Convert inverse-depth predictions to actual metric depths.  A non-positive
    # fitted inverse depth is invalid; clipping it would fabricate a far range.
    pred_disp = np.asarray(data["pred_disp"], dtype=np.float32)
    raw_valid = np.asarray(data["valid_mask"], dtype=bool)
    inverse_scale = final_a_scale * pred_disp
    inverse_shift = final_a_shift * pred_disp + final_b_shift
    depth_scale, valid_scale = inverse_depth_to_metric_depth(inverse_scale, raw_valid)
    depth_shift, valid_shift = inverse_depth_to_metric_depth(inverse_shift, raw_valid)
    selected_valid = valid_scale if selected_model == "scale_only" else valid_shift

    np.save(os.path.join(output_dir, "metric_depth_scale.npy"), depth_scale)
    np.save(os.path.join(output_dir, "metric_depth_shift.npy"), depth_shift)
    np.save(os.path.join(output_dir, "metric_valid_mask_scale.npy"), valid_scale.astype(np.uint8))
    np.save(os.path.join(output_dir, "metric_valid_mask_shift.npy"), valid_shift.astype(np.uint8))
    np.save(os.path.join(output_dir, "metric_valid_mask.npy"), selected_valid.astype(np.uint8))

    Image.fromarray(depth_to_color(depth_scale)).save(
        os.path.join(output_dir, "metric_preview_scale.png")
    )
    Image.fromarray(depth_to_color(depth_shift)).save(
        os.path.join(output_dir, "metric_preview_shift.png")
    )

    anchor_viz = np.zeros((data["H"], data["W"], 3), dtype=np.uint8)
    for anchor in data.get("sampled_anchors", []):
        u = int(round(anchor["raw_u"])) % data["W"]
        v = min(data["H"] - 1, max(0, int(round(anchor["raw_v"]))))
        anchor_viz[v, u] = [0, 255, 0]
    Image.fromarray(anchor_viz).save(os.path.join(output_dir, "anchor_overlay.png"))

    selected_metrics = report[selected_model]
    selected_near_metrics = selected_metrics["near_10m"]
    nonempty_locations = sorted(set(location_ids[location_ids != ""].tolist()))
    inference_metadata = data.get("metadata", {})
    checkpoint_sha256 = str(inference_metadata.get("checkpoint_sha256", "") or "").lower()
    fingerprint_complete = (
        bool(inference_metadata.get("model"))
        and isinstance(inference_metadata.get("width"), (int, np.integer))
        and int(inference_metadata["width"]) > 0
        and isinstance(inference_metadata.get("height"), (int, np.integer))
        and int(inference_metadata["height"]) > 0
        and inference_metadata.get("resample") in {"bicubic", "bilinear"}
        and len(checkpoint_sha256) == 64
        and all(character in "0123456789abcdef" for character in checkpoint_sha256)
    )
    collection_requirements = {
        "locations": len(nonempty_locations),
        "locations_min": 4,
        "captures": report["n_samples"],
        "captures_min": 12,
        "leave_one_location_out": validation_strategy == "leave-one-location-out",
        "inference_fingerprint_complete": bool(fingerprint_complete),
    }
    passed_gate = (
        selected_metrics["median_abs_rel"] is not None
        and selected_metrics["p90_abs_rel"] is not None
        and selected_metrics["median_abs_rel"] <= 0.15
        and selected_metrics["p90_abs_rel"] <= 0.30
        and selected_near_metrics["p90_error_m"] is not None
        and selected_near_metrics["p90_error_m"] <= 1.0
        and report["valid_anchor_fraction"] >= 0.70
        and collection_requirements["locations"] >= collection_requirements["locations_min"]
        and collection_requirements["captures"] >= collection_requirements["captures_min"]
        and collection_requirements["leave_one_location_out"]
        and collection_requirements["inference_fingerprint_complete"]
    )
    report["collection"] = collection_requirements
    report["acceptance"] = {
        "passed": bool(passed_gate),
        "median_abs_rel_max": 0.15,
        "p90_abs_rel_max": 0.30,
        "near_10m_p90_error_m_max": 1.0,
        "valid_anchor_fraction_min": 0.70,
    }
    report["conclusion"] = (
        "metric calibration passed held-out gates"
        if passed_gate else "metric calibration did not pass held-out gates"
    )
    calibration_candidate = {
        "schema_version": 1,
        "accepted": bool(passed_gate),
        "a": float(selected_a),
        "b": float(selected_b),
        "depth_min_m": 0.5,
        "depth_max_m": 20.0,
        "model": inference_metadata.get("model"),
        "width": inference_metadata.get("width", data["W"]),
        "height": inference_metadata.get("height", data["H"]),
        "resample": inference_metadata.get("resample"),
        "checkpoint_sha256": checkpoint_sha256 or None,
        "selected_model": selected_model,
        "relation": "inverse_depth_1_per_m = a * pred_disp + b",
        "fit_report": "fit_report.json",
        "acceptance": report["acceptance"],
        "collection": report["collection"],
    }
    _write_json_atomic(Path(output_dir) / "depth_calibration_candidate.json", calibration_candidate)
    accepted_calibration = None
    if passed_gate:
        accepted_calibration = "depth_calibration.json"
        _write_json_atomic(Path(output_dir) / accepted_calibration, calibration_candidate)

    report["artifacts"] = {
        "scale_depth": "metric_depth_scale.npy",
        "scale_shift_depth": "metric_depth_shift.npy",
        "selected_valid_mask": "metric_valid_mask.npy",
        "anchor_overlay": "anchor_overlay.png",
        "preview_sample_id": data.get("preview_sample_id", data.get("sample_id", "unknown")),
        "calibration_candidate": "depth_calibration_candidate.json",
        "accepted_calibration": accepted_calibration,
    }

    # The report is written only after every conclusion and artifact field is
    # complete; atomic replace prevents consumers from reading partial JSON.
    _write_report(output_dir, report)
    print(json.dumps(report, indent=2, allow_nan=False))
    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Offline inverse-depth metric fitting")
    parser.add_argument(
        "--raw", required=True, action="append",
        help="DA360 raw .npz from /depth/raw; repeat with --anchors for multiple captures",
    )
    parser.add_argument(
        "--anchors", required=True, action="append",
        help="Cesium anchors .json; repeat in the same order as --raw",
    )
    parser.add_argument("--output", default="experiment_data/metric_fit_sample", help="Output directory")
    parser.add_argument("--ransac", action="store_true", help="Enable RANSAC outlier rejection")
    args = parser.parse_args()

    if len(args.raw) != len(args.anchors):
        parser.error("--raw and --anchors must be repeated the same number of times")
    captures = [load_data(raw, anchors) for raw, anchors in zip(args.raw, args.anchors)]
    data = combine_datasets(captures)
    report = run_fitting(data, args.output, use_ransac=args.ransac)

    if not report["success"]:
        print(f"Fitting failed: {report['error']}", file=sys.stderr)
        sys.exit(1)

    print(f"\nConclusion: {report['conclusion']}")


if __name__ == "__main__":
    main()
