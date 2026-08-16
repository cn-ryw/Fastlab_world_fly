import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "evaluate_demo_performance",
    ROOT / "scripts" / "evaluate_demo_performance.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def passing_log(tile_limit=8):
    return {
        "duration_s": 60,
        "runtime": {
            "tileRequestsPerServer": tile_limit,
            "performanceProfile": "demo30",
            "yopoStrategy": "d70_h30_epoch30",
            "userAgent": "Firefox",
        },
        "perf": {
            "mainMedianFps": 30,
            "mainFrameIntervalP95Ms": 38,
            "mainFrameIntervalP99Ms": 60,
            "mainLongFrame100MsCount": 1,
            "mainLongFrame250MsCount": 0,
            "uniquePlanningHz": 12,
            "planningIntervalP95Ms": 90,
            "da360P95Ms": 40,
            "yopoP95Ms": 5,
            "captureToApplyP95Ms": 130,
        },
    }


def test_demo_log_gate_accepts_joint_render_and_planning_target():
    assert MODULE.evaluate_log(passing_log())["passed"] is True


def test_demo_log_gate_rejects_smooth_video_with_slow_planning():
    log = passing_log()
    log["perf"]["uniquePlanningHz"] = 9.9
    result = MODULE.evaluate_log(log)
    assert result["passed"] is False
    assert any("uniquePlanningHz" in failure for failure in result["failures"])


def test_largest_fully_passing_concurrency_is_selected():
    results = []
    for limit in (12, 8):
        for _ in range(3):
            results.append(MODULE.evaluate_log(passing_log(limit)))
    results[0]["passed"] = False
    assert MODULE.select_tile_limit(results, minimum_runs=3) == 8
