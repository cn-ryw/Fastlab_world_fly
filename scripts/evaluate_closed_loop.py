#!/usr/bin/env python3
"""Evaluate one browser flight log against the trusted 15 Hz gate."""

import argparse
import json
import math
import sys
from pathlib import Path


GATES = {
    "mean_planning_hz_min": 15.0,
    "planning_interval_p95_ms_max": 100.0,
    "capture_to_apply_p95_ms_max": 150.0,
    "physics_update_interval_p95_ms_max": 33.3,
    "warm_server_p95_ms_max": 50.0,
}


def percentile(values, quantile):
    values = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not values:
        return None
    index = max(0, min(len(values) - 1, math.ceil(quantile * len(values)) - 1))
    return values[index]


def evaluate_log(log, warmup_samples=5):
    duration = float(log.get("duration_s") or 0)
    applied = [
        item for item in log.get("perception", [])
        if item.get("mode") == "planning" and item.get("outcome") == "applied"
        and item.get("planningAuthorized") is True
    ]
    unique = {}
    for item in applied:
        unique.setdefault(str(item.get("frameId")), item)
    ordered = sorted(unique.values(), key=lambda item: float(item.get("recordedAtMs", 0)))
    apply_times = [float(item["recordedAtMs"]) for item in ordered if "recordedAtMs" in item]
    planning_intervals = [current - previous for previous, current in zip(apply_times, apply_times[1:])]
    frame_times = [float(frame["t"]) * 1000.0 for frame in log.get("frames", []) if "t" in frame]
    physics_intervals = [current - previous for previous, current in zip(frame_times, frame_times[1:])]
    capture_to_apply = [item.get("captureToApplyMs") for item in ordered]
    server_values = [item.get("serverMs") for item in ordered]
    warm_server_values = server_values[max(0, int(warmup_samples)):]

    metrics = {
        "duration_s": duration,
        "unique_planning_frames": len(ordered),
        "mean_planning_hz": len(ordered) / duration if duration > 0 else 0.0,
        "planning_interval_p95_ms": percentile(planning_intervals, 0.95),
        "capture_to_apply_p95_ms": percentile(capture_to_apply, 0.95),
        "physics_update_interval_p95_ms": percentile(physics_intervals, 0.95),
        "warm_server_p95_ms": percentile(warm_server_values, 0.95),
        "warmup_samples_discarded": min(len(server_values), max(0, int(warmup_samples))),
    }
    checks = {
        "mean_planning_hz": metrics["mean_planning_hz"] >= GATES["mean_planning_hz_min"],
        "planning_interval_p95_ms": metrics["planning_interval_p95_ms"] is not None
        and metrics["planning_interval_p95_ms"] <= GATES["planning_interval_p95_ms_max"],
        "capture_to_apply_p95_ms": metrics["capture_to_apply_p95_ms"] is not None
        and metrics["capture_to_apply_p95_ms"] <= GATES["capture_to_apply_p95_ms_max"],
        "physics_update_interval_p95_ms": metrics["physics_update_interval_p95_ms"] is not None
        and metrics["physics_update_interval_p95_ms"] <= GATES["physics_update_interval_p95_ms_max"],
        "warm_server_p95_ms": metrics["warm_server_p95_ms"] is not None
        and metrics["warm_server_p95_ms"] <= GATES["warm_server_p95_ms_max"],
    }
    return {"passed": all(checks.values()), "gates": GATES, "checks": checks, "metrics": metrics}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="flight-log JSON produced by FlightLogger")
    parser.add_argument("--warmup-samples", type=int, default=5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    with args.log.open(encoding="utf-8") as stream:
        report = evaluate_log(json.load(stream), args.warmup_samples)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    sys.exit(main())
