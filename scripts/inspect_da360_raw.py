#!/usr/bin/env python3
"""Inspect a DA360 raw .npz file: print metadata, statistics, and optional visualization.

Usage:
    python scripts/inspect_da360_raw.py <raw.npz> [--vis preview.png] [--hist hist.png]
"""
import argparse, json, sys
import numpy as np
from PIL import Image


def depth_to_color(depth, sample_limit=65536):
    """Reproduce the same pseudo-colour as da360_server.py for side-by-side comparison."""
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


def main():
    parser = argparse.ArgumentParser(description="Inspect DA360 raw .npz output")
    parser.add_argument("npz", help="Path to raw .npz file from /depth/raw")
    parser.add_argument("--vis", default=None, help="Save pseudo-colour preview PNG")
    parser.add_argument("--hist", default=None, help="Save histogram PNG (requires matplotlib)")
    args = parser.parse_args()

    data = np.load(args.npz, allow_pickle=True)
    md = json.loads(str(data["metadata_json"]))
    disp = data["pred_disp"]
    rel_depth = data["relative_depth"]
    mask = data["valid_mask"].astype(bool)

    print("=" * 50)
    print("DA360 Raw Depth Inspection")
    print("=" * 50)
    print(f"Model:        {md['model']}")
    print(f"Device:       {md['device']}")
    print(f"Input size:   {md['width']} x {md['height']}")
    print(f"AMP:          {md['amp']}")
    print()
    print(f"pred_disp:    dtype={disp.dtype}  shape={disp.shape}")
    print(f"              min={disp.min():.6f}  max={disp.max():.6f}  mean={disp[mask].mean():.6f}")
    print(f"rel_depth:    dtype={rel_depth.dtype}  shape={rel_depth.shape}")
    print(f"              min={rel_depth[mask].min():.6f}  max={rel_depth[mask].max():.6f}")
    print(f"valid_mask:   {mask.sum()} / {mask.size} pixels ({mask.mean() * 100:.1f}% valid)")
    print()

    if args.vis:
        coloured = depth_to_color(rel_depth)
        Image.fromarray(coloured).save(args.vis)
        print(f"Preview:      {args.vis}")

    if args.hist:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, axs = plt.subplots(1, 2, figsize=(10, 4))
            axs[0].hist(disp[mask].ravel(), bins=100, color="steelblue", edgecolor="none")
            axs[0].set_title("pred_disp (raw disparity)")
            axs[0].set_xlabel("disparity"); axs[0].set_ylabel("count")
            axs[1].hist(rel_depth[mask].ravel(), bins=100, color="darkorange", edgecolor="none")
            axs[1].set_title("relative_depth (1/disp)")
            axs[1].set_xlabel("depth"); axs[1].set_ylabel("count")
            plt.tight_layout(); fig.savefig(args.hist, dpi=100); plt.close()
            print(f"Histogram:    {args.hist}")
        except ImportError:
            print("Histogram skipped: matplotlib not available.", file=sys.stderr)


if __name__ == "__main__":
    main()
