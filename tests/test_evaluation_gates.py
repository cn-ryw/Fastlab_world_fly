import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


closed_loop = load_script("evaluate_closed_loop.py")
quality = load_script("evaluate_perception_quality.py")


def synthetic_log(interval_ms=62.5, capture_to_apply=100, server_ms=40, physics_ms=16):
    count = 80
    perception = [
        {
            "frameId": index,
            "mode": "planning",
            "outcome": "applied",
            "planningAuthorized": True,
            "recordedAtMs": index * interval_ms,
            "captureToApplyMs": capture_to_apply,
            "serverMs": server_ms,
        }
        for index in range(count)
    ]
    frames = [{"t": index * physics_ms / 1000.0} for index in range(320)]
    return {"duration_s": count * interval_ms / 1000.0, "perception": perception, "frames": frames}


def test_closed_loop_passes_all_five_gates():
    report = closed_loop.evaluate_log(synthetic_log())
    assert report["passed"]
    assert all(report["checks"].values())


def test_closed_loop_fails_slow_planning_and_service():
    report = closed_loop.evaluate_log(
        synthetic_log(interval_ms=125, capture_to_apply=180, server_ms=70, physics_ms=40)
    )
    assert not report["passed"]
    assert not report["checks"]["mean_planning_hz"]
    assert not report["checks"]["warm_server_p95_ms"]


def test_closed_loop_never_counts_unauthorized_relative_preview_as_planning():
    log = synthetic_log()
    for item in log["perception"]:
        item["planningAuthorized"] = False
        item["dropReason"] = "da360-relative-is-preview-only"
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert report["metrics"]["unique_planning_frames"] == 0
    assert report["metrics"]["mean_planning_hz"] == 0


def test_perception_quality_selects_threshold_compliant_candidate():
    baseline = {
        "metric_depth": np.full((4, 8), 10.0, dtype=np.float32),
        "endstate": np.zeros((10, 9), dtype=np.float32),
    }
    passing = {
        "metric_depth": np.full((4, 8), 10.4, dtype=np.float32),
        "endstate": np.tile(np.array([0.5, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32), (10, 1)),
    }
    failing = {
        "metric_depth": np.full((4, 8), 13.0, dtype=np.float32),
        "endstate": np.tile(np.array([3.0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32), (10, 1)),
    }
    assert quality.evaluate_candidate(baseline, passing)["passed"]
    assert not quality.evaluate_candidate(baseline, failing)["passed"]
