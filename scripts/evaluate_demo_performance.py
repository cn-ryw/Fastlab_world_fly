#!/usr/bin/env python3
"""Evaluate repeated demo-flight logs against the 30 FPS / 10 Hz gates."""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path


LIMITS = {
    "mainMedianFps": (">=", 29.0),
    "mainFrameIntervalP95Ms": ("<=", 40.0),
    "mainFrameIntervalP99Ms": ("<=", 66.7),
    "mainLongFrame250MsCount": ("<=", 0.0),
    "uniquePlanningHz": (">=", 10.0),
    "planningIntervalP95Ms": ("<=", 100.0),
    "da360P95Ms": ("<=", 50.0),
    "yopoP95Ms": ("<=", 10.0),
    "captureToApplyP95Ms": ("<=", 150.0),
}
TILE_LIMITS = (18, 12, 8, 6)


def _finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def evaluate_log(log):
    perf = log.get("perf") or {}
    duration = _finite_number(log.get("duration_s")) or 0.0
    failures = []
    for key, (operator, threshold) in LIMITS.items():
        value = _finite_number(perf.get(key))
        if value is None:
            failures.append(f"{key}=missing")
            continue
        passed = value >= threshold if operator == ">=" else value <= threshold
        if not passed:
            failures.append(f"{key}={value:g} {operator} {threshold:g}")

    long_100 = _finite_number(perf.get("mainLongFrame100MsCount"))
    allowed_long_100 = max(1, math.ceil(duration / 60.0))
    if long_100 is None:
        failures.append("mainLongFrame100MsCount=missing")
    elif long_100 > allowed_long_100:
        failures.append(
            f"mainLongFrame100MsCount={long_100:g} <= {allowed_long_100}"
        )

    runtime = log.get("runtime") or {}
    return {
        "passed": not failures,
        "failures": failures,
        "tileRequestsPerServer": runtime.get("tileRequestsPerServer"),
        "performanceProfile": runtime.get("performanceProfile"),
        "yopoStrategy": runtime.get("yopoStrategy"),
        "userAgent": runtime.get("userAgent"),
    }


def select_tile_limit(results, minimum_runs=3):
    grouped = defaultdict(list)
    for result in results:
        limit = result.get("tileRequestsPerServer")
        if limit in TILE_LIMITS:
            grouped[limit].append(result)
    for limit in TILE_LIMITS:
        runs = grouped[limit]
        if len(runs) >= minimum_runs and all(item["passed"] for item in runs):
            return limit
    return None


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--min-runs", type=int, default=3)
    args = parser.parse_args(argv)

    results = []
    for path in args.logs:
        with path.open(encoding="utf-8") as stream:
            result = evaluate_log(json.load(stream))
        result["path"] = str(path)
        results.append(result)
        status = "PASS" if result["passed"] else "FAIL"
        print(f"{status} tiles={result['tileRequestsPerServer']} {path}")
        for failure in result["failures"]:
            print(f"  - {failure}")

    selected = select_tile_limit(results, max(1, args.min_runs))
    if selected is None:
        print("SELECTED none (no candidate has enough fully passing runs)")
        return 1
    print(f"SELECTED tileRequestsPerServer={selected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
