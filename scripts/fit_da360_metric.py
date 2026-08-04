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
"""
import argparse, json, os, sys, warnings
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
    t = 1.0 - (np.clip(depth, near, far) - near) / max(far - near, 1e-6)
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
def _huber_residuals(params, x, y, delta):
    """Residuals scaled for Huber loss: ρ(r) = 0.5 r² for |r|≤δ, δ|r|-0.5δ² otherwise."""
    a, b = params
    residuals = y - (a * x + b)
    abs_r = np.abs(residuals)
    mask = abs_r <= delta
    scaled = np.empty_like(residuals)
    scaled[mask] = residuals[mask]
    scaled[~mask] = np.sign(residuals[~mask]) * np.sqrt(2 * delta * abs_r[~mask] - delta * delta)
    return scaled


def fit_scale_shift(x, y, delta=DEFAULT_HUBER_DELTA):
    """Huber-robust fit: y = a*x + b  with a > 0 constraint."""
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
        lambda p: _huber_residuals(p, x, y, delta),
        [a0, b0],
        bounds=([1e-12, -np.inf], [np.inf, np.inf]),
        max_nfev=2000,
        loss='soft_l1',
        f_scale=delta,
    )
    a, b = float(result.x[0]), float(result.x[1])
    residuals = y - (a * x + b)
    return a, b, residuals


def fit_scale_only(x, y, delta=DEFAULT_HUBER_DELTA):
    """Huber-robust fit: y = a*x  (no intercept)."""
    a0 = max(1e-9, float(np.median(y / np.maximum(x, 1e-9))))
    result = least_squares(
        lambda a: _huber_residuals([a[0], 0.0], x, y, delta),
        [a0],
        bounds=([1e-12], [np.inf]),
        max_nfev=2000,
        loss='soft_l1',
        f_scale=delta,
    )
    a = float(result.x[0])
    residuals = y - a * x
    return a, residuals


def fit_ransac(x, y, n_trials=100, min_samples=4, inlier_thresh=0.5):
    """Simple RANSAC wrapper: returns best (a, b, inlier_mask)."""
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
    """Standard depth metrics."""
    if valid is None:
        valid = np.ones_like(y_true, dtype=bool)
    yt, yp = y_true[valid], y_pred[valid]
    if len(yt) < 2:
        return {}
    abs_rel = np.median(np.abs(yt - yp) / np.maximum(yt, EPS))
    p90_val = np.percentile(np.abs(yt - yp) / np.maximum(yt, EPS), 90)
    rmse = np.sqrt(np.mean((yt - yp) ** 2))
    med_err = np.median(np.abs(yt - yp))
    return {
        "median_abs_rel": float(abs_rel),
        "p90_abs_rel": float(p90_val),
        "rmse": float(rmse),
        "median_error_m": float(med_err),
        "n_valid": int(valid.sum()),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def load_data(raw_path, anchors_path):
    """Load raw pred_disp .npz and Cesium anchors .json."""
    raw = np.load(raw_path, allow_pickle=True)
    pred_disp = raw["pred_disp"]
    valid_mask = raw["valid_mask"].astype(bool)
    md = json.loads(str(raw["metadata_json"]))
    H, W = pred_disp.shape

    with open(anchors_path) as f:
        anchor_data = json.load(f)

    anchors = anchor_data.get("anchors", [])
    meta = anchor_data.get("metadata", {})

    # Map anchor ERP pixels to pred_disp values (nearest-neighbour)
    x_vals, z_vals = [], []
    for a in anchors:
        u, v = int(round(a["u"])), int(round(a["v"]))
        d = a["distance"]
        if 0 <= u < W and 0 <= v < H and valid_mask[v, u] and d > 0:
            x_vals.append(float(pred_disp[v, u]))
            z_vals.append(1.0 / d)  # inverse metric depth

    x_arr = np.array(x_vals, dtype=np.float64)
    z_arr = np.array(z_vals, dtype=np.float64)

    return {
        "pred_disp": pred_disp,
        "valid_mask": valid_mask,
        "H": H, "W": W,
        "x": x_arr, "z": z_arr,
        "metadata": md,
        "anchor_metadata": meta,
        "n_anchors": len(anchors),
        "n_valid_anchors": len(x_arr),
    }


def run_fitting(data, output_dir, use_ransac=False):
    os.makedirs(output_dir, exist_ok=True)

    x, z = data["x"], data["z"]
    n = len(x)
    if n < 4:
        return {
            "success": False,
            "error": f"insufficient valid anchors: {n} (need >=4)",
            "n_valid_anchors": n,
        }

    # Train/held-out split
    rng = np.random.default_rng(SEED)
    idx = rng.permutation(n)
    n_train = max(4, int(n * 0.7))
    train_idx, val_idx = idx[:n_train], idx[n_train:]
    x_train, z_train = x[train_idx], z[train_idx]
    x_val, z_val = x[val_idx], z[val_idx]

    report = {
        "success": True,
        "n_total_anchors": data["n_anchors"],
        "n_valid_anchors": n,
        "n_train": int(n_train),
        "n_val": int(len(val_idx)),
        "model": data["metadata"]["model"],
        "seed": SEED,
    }

    # ── scale-only ──
    if use_ransac:
        a_s, _, inlier_mask = fit_ransac(x_train, z_train)
    else:
        a_s, _ = fit_scale_only(x_train, z_train)
    z_pred_scale = a_s * x_val
    report["scale_only"] = {
        "a": float(a_s),
        **compute_metrics(z_val, z_pred_scale),
    }

    # ── scale+shift ──
    if use_ransac:
        a_ss, b_ss, inlier_mask = fit_ransac(x_train, z_train)
    else:
        a_ss, b_ss, _ = fit_scale_shift(x_train, z_train)
    z_pred_shift = a_ss * x_val + b_ss
    report["scale_shift"] = {
        "a": float(a_ss),
        "b": float(b_ss),
        **compute_metrics(z_val, z_pred_shift),
    }

    # ── Metric depth images ──
    pred_disp = data["pred_disp"]
    valid_mask = data["valid_mask"]
    z_metric_scale = a_s * pred_disp
    z_metric_shift = a_ss * pred_disp + b_ss

    # Save metric depths
    np.save(os.path.join(output_dir, "metric_depth_scale.npy"), z_metric_scale)
    np.save(os.path.join(output_dir, "metric_depth_shift.npy"), z_metric_shift)
    np.save(os.path.join(output_dir, "metric_valid_mask.npy"), valid_mask.astype(np.uint8))

    # Preview images
    viz_scale = depth_to_color(z_metric_scale)
    viz_shift = depth_to_color(z_metric_shift)
    Image.fromarray(viz_scale).save(os.path.join(output_dir, "metric_preview_scale.png"))
    Image.fromarray(viz_shift).save(os.path.join(output_dir, "metric_preview_shift.png"))

    # Anchor overlay (mark sampled anchor positions on ERP grid)
    anchor_path = data.get("_anchors_path", "")
    if anchor_path and os.path.exists(anchor_path):
        anchor_viz = np.zeros((data["H"], data["W"], 3), dtype=np.uint8)
        with open(anchor_path) as af:
            anchor_json = json.load(af)
        for a_json in anchor_json.get("anchors", []):
            u, v = int(round(a_json["u"])), int(round(a_json["v"]))
            if 0 <= u < data["W"] and 0 <= v < data["H"]:
                anchor_viz[v, u] = [0, 255, 0]
        Image.fromarray(anchor_viz).save(os.path.join(output_dir, "anchor_overlay.png"))

    # Fit report
    with open(os.path.join(output_dir, "fit_report.json"), "w") as f:
        json.dump(report, f, indent=2)

    # Conclusion
    val = report["scale_shift"]["median_abs_rel"]
    if val < 0.3:
        conclusion = "metricization initially effective"
    elif val < 1.0:
        conclusion = "interface runs but metric error is moderate"
    else:
        conclusion = "anchors or tiles insufficient, unable to evaluate"

    report["conclusion"] = conclusion

    # Print summary
    print(json.dumps(report, indent=2))
    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Offline inverse-depth metric fitting")
    parser.add_argument("--raw", required=True, help="DA360 raw .npz from /depth/raw")
    parser.add_argument("--anchors", required=True, help="Cesium anchors .json")
    parser.add_argument("--output", default="experiment_data/metric_fit_sample", help="Output directory")
    parser.add_argument("--ransac", action="store_true", help="Enable RANSAC outlier rejection")
    args = parser.parse_args()

    data = load_data(args.raw, args.anchors)
    data["_anchors_path"] = args.anchors
    report = run_fitting(data, args.output, use_ransac=args.ransac)

    if not report["success"]:
        print(f"Fitting failed: {report['error']}", file=sys.stderr)
        sys.exit(1)

    print(f"\nConclusion: {report['conclusion']}")


if __name__ == "__main__":
    main()
