#!/usr/bin/env python3
"""Compare ordered panorama configurations with the 6x144/SSE96 baseline."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np


GATES = {
    "depth_relative_median_max": 0.05,
    "depth_relative_p90_max": 0.15,
    "endpoint_position_median_m_max": 1.0,
    "endpoint_position_p90_m_max": 2.0,
}


def _percentile(values, quantile):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, quantile * 100.0)) if values.size else None


def evaluate_candidate(baseline, candidate):
    baseline_depth = np.asarray(baseline["metric_depth"], dtype=np.float64)
    candidate_depth = np.asarray(candidate["metric_depth"], dtype=np.float64)
    if baseline_depth.shape != candidate_depth.shape:
        raise ValueError("metric_depth shapes differ")
    valid = np.isfinite(baseline_depth) & np.isfinite(candidate_depth) & (baseline_depth > 0)
    if not np.any(valid):
        raise ValueError("no finite positive baseline depth pixels")
    relative = np.abs(candidate_depth[valid] - baseline_depth[valid]) / baseline_depth[valid]

    baseline_endstate = np.asarray(baseline["endstate"], dtype=np.float64)
    candidate_endstate = np.asarray(candidate["endstate"], dtype=np.float64)
    if baseline_endstate.shape != candidate_endstate.shape or baseline_endstate.shape[-1] != 9:
        raise ValueError("axis-major endstate arrays must have identical [...,9] shapes")
    position_delta = candidate_endstate[..., [0, 3, 6]] - baseline_endstate[..., [0, 3, 6]]
    endpoint_error = np.linalg.norm(position_delta, axis=-1)

    metrics = {
        "depth_relative_median": _percentile(relative, 0.5),
        "depth_relative_p90": _percentile(relative, 0.9),
        "endpoint_position_median_m": _percentile(endpoint_error, 0.5),
        "endpoint_position_p90_m": _percentile(endpoint_error, 0.9),
        "valid_depth_pixels": int(valid.sum()),
        "endpoint_samples": int(endpoint_error.size),
    }
    checks = {
        "depth_relative_median": metrics["depth_relative_median"] <= GATES["depth_relative_median_max"],
        "depth_relative_p90": metrics["depth_relative_p90"] <= GATES["depth_relative_p90_max"],
        "endpoint_position_median_m": metrics["endpoint_position_median_m"] <= GATES["endpoint_position_median_m_max"],
        "endpoint_position_p90_m": metrics["endpoint_position_p90_m"] <= GATES["endpoint_position_p90_m_max"],
    }
    return {"passed": all(checks.values()), "checks": checks, "metrics": metrics}


def _load_npz(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument(
        "--candidate", action="append", required=True, metavar="LABEL=PATH",
        help="repeat in highest-to-lowest quality order",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    baseline = _load_npz(args.baseline)
    reports = []
    selected = None
    for specification in args.candidate:
        if "=" not in specification:
            parser.error("--candidate must be LABEL=PATH")
        label, raw_path = specification.split("=", 1)
        report = evaluate_candidate(baseline, _load_npz(Path(raw_path)))
        report["label"] = label
        report["path"] = raw_path
        reports.append(report)
        if selected is None and report["passed"]:
            selected = label
    output = {"passed": selected is not None, "selected": selected, "gates": GATES, "candidates": reports}
    rendered = json.dumps(output, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if selected is not None else 2


if __name__ == "__main__":
    sys.exit(main())
