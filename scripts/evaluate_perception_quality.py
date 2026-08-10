#!/usr/bin/env python3
"""Compare paired panorama configurations with the 6x144/SSE96 baseline.

Each NPZ must contain ``metric_depth[C,H,W]``, ``valid_mask[C,H,W]``,
``endstate[C,9]``, ``case_ids[C]`` and ``case_provenance[C]``.  Case IDs and
provenance must each be unique and match the baseline exactly; NaN values may
only occur outside the declared valid mask and cannot be used to evade the
minimum coverage requirement.
"""

import argparse
import json
import sys
import zipfile
from pathlib import Path

import numpy as np


GATES = {
    "depth_relative_median_max": 0.05,
    "depth_relative_p90_max": 0.15,
    "endpoint_position_median_m_max": 1.0,
    "endpoint_position_p90_m_max": 2.0,
}

DEFAULT_MIN_CASES = 12
DEFAULT_MIN_DEPTH_SAMPLES = 10_000
DEFAULT_MIN_ENDPOINT_SAMPLES = 12
DEFAULT_MIN_CANDIDATE_COVERAGE = 0.95
REQUIRED_ARRAYS = {
    "metric_depth", "valid_mask", "endstate", "case_ids", "case_provenance"
}


def _percentile(values, quantile):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, quantile * 100.0)) if values.size else None


def _string_vector(value, field, expected_cases=None):
    array = np.asarray(value)
    if array.ndim != 1:
        raise ValueError(f"{field} must be a one-dimensional array")
    if expected_cases is not None and array.shape != (expected_cases,):
        raise ValueError(f"{field} must contain exactly one value per case")
    strings = []
    for raw in array.tolist():
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", errors="strict")
        text = str(raw)
        if not text or text != text.strip() or any(ord(char) < 32 for char in text):
            raise ValueError(f"{field} contains an empty or invalid value")
        strings.append(text)
    return np.asarray(strings, dtype=np.str_)


def _validate_archive(archive, label):
    missing = REQUIRED_ARRAYS - set(archive)
    if missing:
        raise ValueError(f"{label} missing arrays: {', '.join(sorted(missing))}")

    depth = np.asarray(archive["metric_depth"], dtype=np.float64)
    mask = np.asarray(archive["valid_mask"])
    endstate = np.asarray(archive["endstate"], dtype=np.float64)
    if depth.ndim != 3:
        raise ValueError(f"{label} metric_depth must have shape [cases,height,width]")
    if mask.shape != depth.shape:
        raise ValueError(f"{label} valid_mask shape differs from metric_depth")
    if endstate.shape != (depth.shape[0], 9):
        raise ValueError(f"{label} endstate must have shape [cases,9]")
    if not np.issubdtype(mask.dtype, np.bool_) and not np.all(
        np.isin(mask, (0, 1))
    ):
        raise ValueError(f"{label} valid_mask must contain only boolean/0/1 values")
    mask = mask.astype(bool, copy=False)
    if np.any(mask & (~np.isfinite(depth) | (depth <= 0))):
        raise ValueError(
            f"{label} valid_mask marks non-finite or non-positive depth as valid"
        )
    if not np.all(np.isfinite(endstate)):
        raise ValueError(f"{label} endstate contains non-finite values")

    case_count = depth.shape[0]
    case_ids = _string_vector(archive["case_ids"], f"{label} case_ids", case_count)
    provenance = _string_vector(
        archive["case_provenance"], f"{label} case_provenance", case_count
    )
    if len(set(case_ids.tolist())) != case_count:
        raise ValueError(f"{label} case_ids must be unique")
    if len(set(provenance.tolist())) != case_count:
        raise ValueError(f"{label} case_provenance must be unique")
    return {
        "depth": depth,
        "mask": mask,
        "endstate": endstate,
        "case_ids": case_ids,
        "case_provenance": provenance,
    }


def evaluate_candidate(
    baseline,
    candidate,
    *,
    min_cases=DEFAULT_MIN_CASES,
    min_depth_samples=DEFAULT_MIN_DEPTH_SAMPLES,
    min_endpoint_samples=DEFAULT_MIN_ENDPOINT_SAMPLES,
    min_candidate_coverage=DEFAULT_MIN_CANDIDATE_COVERAGE,
    gates=None,
):
    effective_gates = dict(GATES if gates is None else gates)
    base = _validate_archive(baseline, "baseline")
    cand = _validate_archive(candidate, "candidate")
    if base["depth"].shape != cand["depth"].shape:
        raise ValueError("metric_depth shapes differ")
    if not np.array_equal(base["case_ids"], cand["case_ids"]):
        raise ValueError("candidate case_ids do not exactly match baseline order")
    if not np.array_equal(base["case_provenance"], cand["case_provenance"]):
        raise ValueError("candidate case_provenance does not exactly match baseline")

    case_count = base["depth"].shape[0]
    baseline_counts = base["mask"].reshape(case_count, -1).sum(axis=1)
    common = base["mask"] & cand["mask"]
    common_counts = common.reshape(case_count, -1).sum(axis=1)
    if np.any(baseline_counts == 0):
        raise ValueError("every baseline case must contain valid depth samples")
    case_coverage = common_counts / baseline_counts
    baseline_valid_samples = int(baseline_counts.sum())
    valid_depth_samples = int(common_counts.sum())
    candidate_coverage = (
        valid_depth_samples / baseline_valid_samples if baseline_valid_samples else 0.0
    )

    relative = np.abs(cand["depth"][common] - base["depth"][common]) / base["depth"][common]
    if relative.size and not np.all(np.isfinite(relative)):
        raise ValueError("relative depth error contains non-finite values")

    position_delta = cand["endstate"][:, [0, 3, 6]] - base["endstate"][:, [0, 3, 6]]
    endpoint_error = np.linalg.norm(position_delta, axis=-1)
    finite_endpoint = np.isfinite(endpoint_error)
    endpoint_samples = int(finite_endpoint.sum())

    metrics = {
        "case_count": int(case_count),
        "depth_relative_median": _percentile(relative, 0.5),
        "depth_relative_p90": _percentile(relative, 0.9),
        "endpoint_position_median_m": _percentile(endpoint_error, 0.5),
        "endpoint_position_p90_m": _percentile(endpoint_error, 0.9),
        "baseline_valid_depth_pixels": baseline_valid_samples,
        "valid_depth_pixels": valid_depth_samples,
        "candidate_coverage": float(candidate_coverage),
        "candidate_min_case_coverage": float(case_coverage.min()),
        "endpoint_samples": endpoint_samples,
    }
    checks = {
        "minimum_cases": case_count >= int(min_cases),
        "minimum_depth_samples": valid_depth_samples >= int(min_depth_samples),
        "candidate_coverage": candidate_coverage >= float(min_candidate_coverage),
        "candidate_per_case_coverage": bool(np.all(
            case_coverage >= float(min_candidate_coverage)
        )),
        "minimum_endpoint_samples": endpoint_samples >= int(min_endpoint_samples),
        "all_endpoints_finite": endpoint_samples == case_count,
        "depth_relative_median": metrics["depth_relative_median"] is not None
        and metrics["depth_relative_median"]
        <= effective_gates["depth_relative_median_max"],
        "depth_relative_p90": metrics["depth_relative_p90"] is not None
        and metrics["depth_relative_p90"]
        <= effective_gates["depth_relative_p90_max"],
        "endpoint_position_median_m": metrics["endpoint_position_median_m"] is not None
        and metrics["endpoint_position_median_m"]
        <= effective_gates["endpoint_position_median_m_max"],
        "endpoint_position_p90_m": metrics["endpoint_position_p90_m"] is not None
        and metrics["endpoint_position_p90_m"]
        <= effective_gates["endpoint_position_p90_m_max"],
    }
    requirements = {
        "min_cases": int(min_cases),
        "min_depth_samples": int(min_depth_samples),
        "min_endpoint_samples": int(min_endpoint_samples),
        "min_candidate_coverage": float(min_candidate_coverage),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": metrics,
        "requirements": requirements,
    }


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
    parser.add_argument("--min-cases", type=int, default=DEFAULT_MIN_CASES)
    parser.add_argument(
        "--min-depth-samples", type=int, default=DEFAULT_MIN_DEPTH_SAMPLES
    )
    parser.add_argument(
        "--min-endpoint-samples", type=int, default=DEFAULT_MIN_ENDPOINT_SAMPLES
    )
    parser.add_argument(
        "--min-candidate-coverage", type=float,
        default=DEFAULT_MIN_CANDIDATE_COVERAGE,
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    if args.min_cases < 1 or args.min_depth_samples < 1 or args.min_endpoint_samples < 1:
        parser.error("minimum case/sample counts must be positive")
    if not 0 < args.min_candidate_coverage <= 1:
        parser.error("--min-candidate-coverage must be in (0,1]")

    baseline = _load_npz(args.baseline)
    reports = []
    selected = None
    for specification in args.candidate:
        if "=" not in specification:
            parser.error("--candidate must be LABEL=PATH")
        label, raw_path = specification.split("=", 1)
        try:
            report = evaluate_candidate(
                baseline,
                _load_npz(Path(raw_path)),
                min_cases=args.min_cases,
                min_depth_samples=args.min_depth_samples,
                min_endpoint_samples=args.min_endpoint_samples,
                min_candidate_coverage=args.min_candidate_coverage,
            )
        except (OSError, ValueError, EOFError, zipfile.BadZipFile) as exc:
            report = {"passed": False, "error": str(exc), "checks": {}, "metrics": {}}
        report["label"] = label
        report["path"] = raw_path
        reports.append(report)
        if selected is None and report["passed"]:
            selected = label
    output = {
        "passed": selected is not None,
        "selected": selected,
        "gates": GATES,
        "requirements": {
            "min_cases": args.min_cases,
            "min_depth_samples": args.min_depth_samples,
            "min_endpoint_samples": args.min_endpoint_samples,
            "min_candidate_coverage": args.min_candidate_coverage,
        },
        "candidates": reports,
    }
    rendered = json.dumps(output, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if selected is not None else 2


if __name__ == "__main__":
    sys.exit(main())
