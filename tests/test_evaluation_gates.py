import importlib.util
import json
import re
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


closed_loop = load_script("evaluate_closed_loop.py")
quality = load_script("evaluate_perception_quality.py")


def synthetic_log(
    *,
    measured_count=1000,
    warmup_count=30,
    interval_ms=62.5,
    capture_to_apply=100.0,
    server_ms=40.0,
    physics_ms=16.0,
    depth_mode="da360-metric",
    calibration_id="calibration-sha-1",
    service_fingerprint="service-sha-1",
):
    count = warmup_count + measured_count
    perception = [
        {
            "frameId": index,
            "goalId": "goal-1",
            "generation": 1,
            "mode": "planning",
            "outcome": "applied",
            "planningAuthorized": True,
            "trajectoryApplied": True,
            "trajectoryAppliedAtMs": index * interval_ms,
            "recordedAtMs": index * interval_ms + 0.1,
            "captureToApplyMs": capture_to_apply,
            "serverMs": server_ms,
            "depthMode": depth_mode,
            "calibrationId": calibration_id,
            "calibrationAccuracyAccepted": True,
            "serviceFingerprint": service_fingerprint,
            "rgbTilesReady": True,
            "rgbReadyFaces": 6,
            "rgbTotalFaces": 6,
        }
        for index in range(count)
    ]
    end_ms = perception[-1]["trajectoryAppliedAtMs"] + physics_ms
    frames = []
    timestamp = 0.0
    while timestamp <= end_ms:
        frames.append({"recordedAtMs": timestamp, "t": timestamp / 1000.0})
        timestamp += physics_ms
    return {
        "schemaVersion": 2,
        "monotonicStartMs": 0.0,
        "resolvedUrl": "http://127.0.0.1:8080/?panoProfile=flight&depthMs=20",
        "navigationSession": {"goalId": "goal-1", "generation": 1},
        "duration_s": end_ms / 1000.0,
        "perception": perception,
        "frames": frames,
    }


def quality_archive(cases=12, height=32, width=32, depth=10.0):
    return {
        "metric_depth": np.full((cases, height, width), depth, dtype=np.float32),
        "valid_mask": np.ones((cases, height, width), dtype=np.uint8),
        "endstate": np.zeros((cases, 9), dtype=np.float32),
        "case_ids": np.asarray([f"case-{index:02d}" for index in range(cases)]),
        "case_provenance": np.asarray(
            [f"scene-pose-state-goal-sha-{index:02d}" for index in range(cases)]
        ),
    }


def test_closed_loop_passes_strict_schema_after_unified_warmup():
    log = synthetic_log()
    # Warm-up-only values below the hard per-frame safety envelope must be
    # excluded from measured percentiles, not just the server percentile.
    for event in log["perception"][:30]:
        event["captureToApplyMs"] = 200
        event["serverMs"] = 5_000
    report = closed_loop.evaluate_log(log)
    assert report["passed"]
    assert all(report["checks"].values())
    assert report["metrics"]["warmup_frames_discarded"] == 30
    assert report["metrics"]["unique_planning_frames"] == 1000
    assert report["metrics"]["capture_to_apply_p95_ms"] == 100.0
    assert report["metrics"]["warm_server_p95_ms"] == 40.0
    assert report["metrics"]["rgb_tiles_ready_planning_fraction"] == 1.0


def test_closed_loop_reports_partial_rgb_tiles_without_blocking_user_test():
    log = synthetic_log()
    for event in log["perception"]:
        event["rgbTilesReady"] = False
        event["rgbReadyFaces"] = 3
        event["rgbTileError"] = True
        event["rgbReadinessReason"] = "tile-error"

    report = closed_loop.evaluate_log(log)

    assert report["passed"]
    assert report["metrics"]["rgb_tiles_ready_planning_frames"] == 0
    assert report["metrics"]["rgb_tiles_partial_planning_frames"] == len(
        log["perception"]
    )
    assert report["metrics"]["rgb_tiles_unknown_planning_frames"] == 0
    assert report["metrics"]["rgb_tile_error_planning_frames"] == len(
        log["perception"]
    )
    assert report["metrics"]["rgb_tiles_ready_planning_fraction"] == 0.0


def test_closed_loop_resolved_url_allowlist_matches_flight_logger_source():
    source = (ROOT / "src" / "flight-logger.js").read_text(encoding="utf-8")
    match = re.search(
        r"const SAFE_URL_QUERY_KEYS = new Set\(\[(.*?)\]\);",
        source,
        flags=re.DOTALL,
    )
    assert match is not None
    logger_keys = set(re.findall(r"'([^']+)'", match.group(1)))
    assert closed_loop.SAFE_RESOLVED_URL_QUERY_KEYS == logger_keys


def test_closed_loop_accepts_allowlisted_flight_url():
    log = synthetic_log()
    log["resolvedUrl"] = (
        "https://127.0.0.1:8080/index.html"
        "?panoProfile=flight&panoCaptureProfile=flight"
        "&panoFacesPerSlice=2&depthMs=20&yopoMaxFrameAgeMs=250"
        "&flightPreloadRadius=650&flightPreloadMinCoverage=0.9"
        "&flightPreloadViewTimeoutMs=1500&flightPreloadViewAttempts=3"
        "&flightPreloadStrict=1"
    )

    report = closed_loop.evaluate_log(log)

    assert report["passed"]
    assert report["checks"]["resolved_url_safe"]
    assert report["metrics"]["resolved_url_errors"] == []


def test_closed_loop_requires_resolved_url():
    missing = synthetic_log()
    missing.pop("resolvedUrl")
    report = closed_loop.evaluate_log(missing)
    assert not report["passed"]
    assert not report["checks"]["resolved_url_safe"]

    null = synthetic_log()
    null["resolvedUrl"] = None
    report = closed_loop.evaluate_log(null)
    assert not report["passed"]
    assert not report["checks"]["resolved_url_safe"]


@pytest.mark.parametrize("resolved_url", [
    "ftp://127.0.0.1/?panoProfile=flight",
    "/sim?panoProfile=flight",
    "https://evil.example/?panoProfile=flight",
    "http://127.0.0.1/private/token-DEMO_PATH_SECRET?panoProfile=flight",
    "http://127.0.0.1/%ZZ?panoProfile=flight",
    "http://%ZZ/?panoProfile=flight",
    "https://pilot:secret@127.0.0.1/?panoProfile=flight",
    "https://127.0.0.1/?panoProfile=flight#secret",
    "https://127.0.0.1/?panoProfile=flight&unknownSetting=1",
    "https://127.0.0.1/?panoProfile=flight&credential=secret",
    (
        "https://127.0.0.1/?panoProfile=flight"
        "&da360Url=https%3A%2F%2Fuser%3Apass%40api.example%2Fdepth"
        "%3Ftoken%3Dsecret"
    ),
    (
        "https://127.0.0.1/"
        "?panoProfile=https%3A%2F%2Fuser%3Apass%40api.example%2F"
        "%3Ftoken%3Dsecret"
    ),
    "https://127.0.0.1/?panoProfile=FLIGHT",
    "https://127.0.0.1/?panoProfile=flight&depthMs=fast",
    "https://127.0.0.1/?panoProfile=flight&depthMs=20&depthMs=20",
    (
        "https://127.0.0.1/"
        "?panoProfile=flight&panoProfile=calibration"
    ),
])
def test_closed_loop_rejects_unsafe_resolved_url(resolved_url):
    log = synthetic_log()
    log["resolvedUrl"] = resolved_url

    report = closed_loop.evaluate_log(log)

    assert not report["passed"]
    assert not report["checks"]["resolved_url_safe"]
    assert report["metrics"]["resolved_url_errors"]


@pytest.mark.parametrize("numeric_value", [
    "-1", ".5", "1.", "1e2", "0x10", "0b10", "0o10",
])
def test_closed_loop_resolved_url_matches_finite_javascript_number_values(
    numeric_value,
):
    log = synthetic_log()
    log["resolvedUrl"] = (
        "https://127.0.0.1/?panoProfile=flight&depthMs=" + numeric_value
    )

    report = closed_loop.evaluate_log(log)

    assert report["checks"]["resolved_url_safe"]


def test_closed_loop_short_six_frame_log_cannot_pass_default_gate():
    log = synthetic_log(measured_count=6, warmup_count=0, interval_ms=64.0)
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["minimum_measurement_duration"]
    assert not report["checks"]["minimum_planning_frames"]


def test_closed_loop_sparse_physics_timestamps_cannot_fake_p95_gate():
    log = synthetic_log()
    measurement_start = log["perception"][30]["trajectoryAppliedAtMs"]
    measurement_end = log["perception"][-1]["trajectoryAppliedAtMs"]
    log["frames"] = [
        {"recordedAtMs": measurement_start, "t": measurement_start / 1000.0},
        {"recordedAtMs": measurement_end, "t": measurement_end / 1000.0},
    ]
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["physics_measurement_coverage"]
    assert not report["checks"]["minimum_physics_frames"]
    assert report["metrics"]["physics_frames_in_measurement"] == 2
    assert report["metrics"]["physics_span_coverage"] == 1.0


def test_closed_loop_truncated_physics_window_cannot_fake_sample_count():
    log = synthetic_log(physics_ms=1.0)
    measurement_start = log["perception"][30]["trajectoryAppliedAtMs"]
    cutoff = measurement_start + 10_000.0
    log["frames"] = [
        frame for frame in log["frames"]
        if measurement_start <= frame["recordedAtMs"] <= cutoff
    ]
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["physics_measurement_coverage"]
    assert report["checks"]["minimum_physics_frames"]
    assert report["metrics"]["physics_measurement_coverage"] < 0.95


def test_closed_loop_burst_plus_endpoint_cannot_fake_physics_coverage():
    log = synthetic_log()
    measurement_start = log["perception"][30]["trajectoryAppliedAtMs"]
    measurement_end = log["perception"][-1]["trajectoryAppliedAtMs"]
    burst = [measurement_start + index for index in range(1900)]
    log["frames"] = [
        {"recordedAtMs": timestamp, "t": timestamp / 1000.0}
        for timestamp in [*burst, measurement_end]
    ]
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert report["checks"]["minimum_physics_frames"]
    assert report["checks"]["physics_update_interval_p95_ms"]
    assert not report["checks"]["physics_measurement_coverage"]
    assert report["metrics"]["physics_span_coverage"] == 1.0


def test_closed_loop_explicit_overrides_support_short_offline_smoke():
    log = synthetic_log(measured_count=6, warmup_count=0, interval_ms=64.0)
    report = closed_loop.evaluate_log(
        log,
        warmup_samples=0,
        min_duration_s=0.3,
        min_planning_frames=6,
    )
    assert report["passed"]
    assert report["requirements"]["min_measurement_duration_s"] == 0.3
    assert report["requirements"]["min_planning_frames"] == 6


def test_closed_loop_rejects_duplicate_exact_composite_identity():
    log = synthetic_log(measured_count=6, warmup_count=0, interval_ms=50.0)
    first = dict(log["perception"][1])
    exact_duplicate = dict(first, trajectoryAppliedAtMs=300.0)
    different_goal = dict(first, goalId="goal-2", trajectoryAppliedAtMs=350.0)
    different_generation = dict(first, generation=2, trajectoryAppliedAtMs=400.0)
    log["perception"].extend([exact_duplicate, different_goal, different_generation])
    report = closed_loop.evaluate_log(
        log,
        warmup_samples=0,
        min_duration_s=0,
        min_planning_frames=1,
    )
    assert not report["passed"]
    assert not report["checks"]["no_duplicate_planning_frames"]
    assert report["checks"]["planning_timestamps_strictly_increasing"]
    assert report["metrics"]["duplicate_planning_frames"] == 1
    assert report["metrics"]["unique_planning_frames"] == 8


@pytest.mark.parametrize("replacement", [62.5, 62.0])
def test_closed_loop_rejects_nonincreasing_planning_timestamps_in_log_order(
    replacement,
):
    log = synthetic_log()
    log["perception"][2]["trajectoryAppliedAtMs"] = replacement
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["planning_timestamps_strictly_increasing"]
    assert report["metrics"]["nonincreasing_planning_timestamps"] == 1


@pytest.mark.parametrize("replacement", [16.0, 15.0])
def test_closed_loop_rejects_nonincreasing_physics_timestamps_in_log_order(
    replacement,
):
    log = synthetic_log()
    log["frames"][2]["recordedAtMs"] = replacement
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["physics_timestamps_strictly_increasing"]
    assert report["metrics"]["nonincreasing_physics_timestamps"] == 1


def test_closed_loop_requires_actual_trajectory_install_acknowledgement():
    log = synthetic_log()
    for event in log["perception"]:
        event.pop("trajectoryApplied")
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["actual_trajectory_evidence"]
    assert report["metrics"]["unique_planning_frames"] == 0
    assert report["metrics"]["invalid_field_counts"]["trajectoryApplied"] > 0


def test_closed_loop_rejects_even_one_false_applied_claim():
    log = synthetic_log()
    log["perception"][40].pop("trajectoryApplied")
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["all_candidate_planning_events_valid"]


def test_closed_loop_rejects_one_overage_observation_hidden_below_p95():
    log = synthetic_log()
    log["perception"][100]["captureToApplyMs"] = 1000.0
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["planning_observation_hard_age"]
    assert report["metrics"]["overage_planning_frames"] == 1
    assert report["metrics"]["maximum_capture_to_apply_ms"] == 1000.0
    assert report["checks"]["capture_to_apply_p95_ms"]


def test_closed_loop_relative_mode_cannot_pass_even_if_authorized_flag_is_true():
    report = closed_loop.evaluate_log(synthetic_log(depth_mode="da360-relative"))
    assert not report["passed"]
    assert not report["checks"]["depth_modes_allowed"]
    assert report["metrics"]["unique_planning_frames"] == 0


def test_closed_loop_requires_complete_identity_and_stable_fingerprints():
    log = synthetic_log()
    log["perception"][40]["goalId"] = None
    log["perception"][50]["calibrationId"] = "calibration-sha-2"
    log["perception"][60]["serviceFingerprint"] = "service-sha-2"
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["required_identity_fields"]
    assert not report["checks"]["calibration_id_stable"]
    assert not report["checks"]["service_fingerprint_stable"]


def test_closed_loop_rejects_experimental_unaccepted_metric_calibration():
    log = synthetic_log()
    for event in log["perception"]:
        event["calibrationAccuracyAccepted"] = False
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["calibration_accuracy_accepted"]
    assert report["metrics"]["invalid_field_counts"][
        "calibrationAccuracyAccepted"
    ] == len(log["perception"])


def test_closed_loop_requires_one_declared_navigation_session():
    missing = synthetic_log()
    missing.pop("navigationSession")
    report = closed_loop.evaluate_log(missing)
    assert not report["passed"]
    assert not report["checks"]["navigation_session_metadata"]

    mixed = synthetic_log()
    midpoint = len(mixed["perception"]) // 2
    for event in mixed["perception"][midpoint:]:
        event["goalId"] = "goal-2"
        event["generation"] = 2
    report = closed_loop.evaluate_log(mixed)
    assert not report["passed"]
    assert not report["checks"]["planning_identity_matches_navigation_session"]
    assert not report["checks"]["goal_id_stable"]
    assert not report["checks"]["generation_stable"]


@pytest.mark.parametrize("outcome", ["stale", "blocked", "rejected"])
def test_closed_loop_rejects_cross_session_non_applied_planning_events(outcome):
    log = synthetic_log()
    log["perception"].append({
        "frameId": "foreign-frame",
        "goalId": "other-goal",
        "generation": 999,
        "mode": "planning",
        "outcome": outcome,
        "planningAuthorized": False,
        "trajectoryApplied": False,
        "dropReason": "foreign-session",
    })
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["all_planning_events_match_navigation_session"]
    assert report["metrics"]["foreign_or_malformed_planning_events"] == 1


@pytest.mark.parametrize("identity", [
    {},
    {"goalId": "goal-1", "generation": 1},
])
def test_closed_loop_rejects_malformed_non_applied_planning_identity(identity):
    log = synthetic_log()
    malformed = {
        "frameId": "malformed-frame",
        "mode": "planning",
        "outcome": "stale",
        "planningAuthorized": False,
        "trajectoryApplied": False,
    }
    malformed.update(identity)
    if identity:
        malformed.pop("frameId")
    log["perception"].append(malformed)
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["all_planning_events_match_navigation_session"]
    assert report["metrics"]["foreign_or_malformed_planning_events"] == 1


def test_closed_loop_rejects_duplicate_non_applied_planning_identity():
    log = synthetic_log()
    blocked = {
        "frameId": "blocked-frame",
        "goalId": "goal-1",
        "generation": 1,
        "mode": "planning",
        "outcome": "blocked",
        "planningAuthorized": False,
        "trajectoryApplied": False,
        "dropReason": "preview-only",
    }
    log["perception"].extend([dict(blocked), dict(blocked)])
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["all_planning_control_identities_unique"]
    assert report["metrics"]["duplicate_all_planning_identities"] == 1


def test_closed_loop_requires_schema_and_physics_timestamps():
    log = synthetic_log()
    log.pop("schemaVersion")
    log.pop("monotonicStartMs")
    for frame in log["frames"]:
        frame.pop("recordedAtMs")
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["schema_version"]
    assert not report["checks"]["physics_timestamps_present"]


@pytest.mark.parametrize("duration", [None, 0.0])
def test_closed_loop_requires_positive_session_duration(duration):
    log = synthetic_log()
    if duration is None:
        log.pop("duration_s")
    else:
        log["duration_s"] = duration
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["session_metadata_valid"]
    assert not report["checks"]["planning_events_within_session"]


def test_closed_loop_rejects_apply_timestamps_outside_declared_session():
    before_start = synthetic_log()
    before_start["perception"][0]["trajectoryAppliedAtMs"] = -1.0
    report = closed_loop.evaluate_log(before_start)
    assert not report["passed"]
    assert not report["checks"]["planning_events_within_session"]

    after_end = synthetic_log()
    after_end["perception"][-1]["trajectoryAppliedAtMs"] = (
        after_end["duration_s"] * 1000.0 + 10.0
    )
    report = closed_loop.evaluate_log(after_end)
    assert not report["passed"]
    assert not report["checks"]["planning_events_within_session"]


def test_closed_loop_rejects_planning_outage_at_declared_session_tail():
    log = synthetic_log()
    log["duration_s"] = 120.0

    report = closed_loop.evaluate_log(log)

    assert not report["passed"]
    assert report["checks"]["planning_events_within_session"]
    assert not report["checks"]["planning_reaches_session_end"]
    assert report["metrics"]["last_legal_planning_apply_ms"] == 64_312.5
    assert report["metrics"]["planning_session_tail_gap_ms"] == 55_687.5
    assert report["requirements"]["max_planning_session_tail_gap_ms"] == 255.1


def test_closed_loop_rejects_planning_outage_at_declared_session_head():
    log = synthetic_log()
    outage_ms = 55_000.0
    for event in log["perception"]:
        event["trajectoryAppliedAtMs"] += outage_ms
        event["recordedAtMs"] += outage_ms
    for frame in log["frames"]:
        frame["recordedAtMs"] += outage_ms
        frame["t"] += outage_ms / 1000.0
    log["duration_s"] += outage_ms / 1000.0

    report = closed_loop.evaluate_log(log)

    assert not report["passed"]
    assert report["checks"]["planning_events_within_session"]
    assert not report["checks"]["planning_starts_near_session_beginning"]
    assert report["checks"]["planning_reaches_session_end"]
    assert report["metrics"]["first_legal_planning_apply_ms"] == outage_ms
    assert report["metrics"]["planning_session_head_gap_ms"] == outage_ms
    assert report["requirements"]["max_planning_session_head_gap_ms"] == 255.1


@pytest.mark.parametrize("generation", [
    "not-an-integer", -1, 2**53, 1.5, True,
])
def test_closed_loop_rejects_generation_outside_api_contract(generation):
    log = synthetic_log()
    log["navigationSession"]["generation"] = generation
    for event in log["perception"]:
        event["generation"] = generation

    report = closed_loop.evaluate_log(log)

    assert not report["passed"]
    assert not report["checks"]["navigation_session_metadata"]
    assert not report["checks"]["all_planning_events_match_navigation_session"]
    assert not report["checks"]["required_identity_fields"]


@pytest.mark.parametrize("override", [
    {"outcome": "applied", "planningAuthorized": False, "trajectoryApplied": True},
    {"outcome": "blocked", "planningAuthorized": False, "trajectoryApplied": True},
    {"outcome": "rejected", "planningAuthorized": True, "trajectoryApplied": True},
])
def test_closed_loop_rejects_contradictory_planning_apply_claims(override):
    log = synthetic_log()
    contradictory = dict(log["perception"][-1])
    contradictory.update(override, frameId="contradictory-frame")
    log["perception"].append(contradictory)
    report = closed_loop.evaluate_log(log)
    assert not report["passed"]
    assert not report["checks"]["no_contradictory_planning_events"]
    assert report["metrics"]["contradictory_planning_events"] == 1


@pytest.mark.parametrize("field,value", [
    ("min_duration_s", float("nan")),
    ("min_physics_coverage", float("nan")),
    ("min_physics_coverage", 0.0),
    ("planning_observation_hard_age_ms", float("nan")),
    ("planning_observation_hard_age_ms", 0.0),
    ("planning_observation_hard_age_ms", 250.1),
    ("planning_observation_hard_age_ms", 1000.0),
])
def test_closed_loop_rejects_invalid_gate_overrides(field, value):
    with pytest.raises(ValueError):
        closed_loop.evaluate_log(synthetic_log(), **{field: value})


def test_closed_loop_fails_each_slow_runtime_gate_independently():
    report = closed_loop.evaluate_log(
        synthetic_log(
            interval_ms=125.0,
            capture_to_apply=180.0,
            server_ms=70.0,
            physics_ms=40.0,
        )
    )
    assert not report["passed"]
    assert not report["checks"]["mean_planning_hz"]
    assert not report["checks"]["planning_interval_p95_ms"]
    assert not report["checks"]["capture_to_apply_p95_ms"]
    assert not report["checks"]["physics_update_interval_p95_ms"]
    assert not report["checks"]["warm_server_p95_ms"]


def test_perception_quality_selects_fully_paired_candidate():
    baseline = quality_archive()
    candidate = quality_archive(depth=10.4)
    candidate["endstate"][:, 0] = 0.5
    report = quality.evaluate_candidate(baseline, candidate)
    assert report["passed"]
    assert all(report["checks"].values())
    assert report["metrics"]["valid_depth_pixels"] == 12 * 32 * 32
    assert report["metrics"]["endpoint_samples"] == 12


def test_perception_quality_rejects_accuracy_regression():
    baseline = quality_archive()
    candidate = quality_archive(depth=13.0)
    candidate["endstate"][:, 0] = 3.0
    report = quality.evaluate_candidate(baseline, candidate)
    assert not report["passed"]
    assert not report["checks"]["depth_relative_p90"]
    assert not report["checks"]["endpoint_position_p90_m"]


def test_perception_quality_one_finite_pixel_cannot_game_nan_filtering():
    baseline = quality_archive()
    candidate = quality_archive()
    candidate["metric_depth"][:] = np.nan
    candidate["valid_mask"][:] = 0
    candidate["metric_depth"][0, 0, 0] = 10.0
    candidate["valid_mask"][0, 0, 0] = 1
    report = quality.evaluate_candidate(baseline, candidate)
    assert not report["passed"]
    assert report["metrics"]["valid_depth_pixels"] == 1
    assert not report["checks"]["minimum_depth_samples"]
    assert not report["checks"]["candidate_coverage"]
    assert not report["checks"]["candidate_per_case_coverage"]


def test_perception_quality_rejects_nan_declared_valid_and_nan_endpoint():
    baseline = quality_archive()
    candidate = quality_archive()
    candidate["metric_depth"][0, 0, 0] = np.nan
    with pytest.raises(ValueError, match="marks non-finite"):
        quality.evaluate_candidate(baseline, candidate)

    candidate = quality_archive()
    candidate["endstate"][0, 0] = np.nan
    with pytest.raises(ValueError, match="endstate contains non-finite"):
        quality.evaluate_candidate(baseline, candidate)


@pytest.mark.parametrize("field", ["case_ids", "case_provenance"])
def test_perception_quality_requires_exact_case_and_provenance_pairing(field):
    baseline = quality_archive()
    candidate = quality_archive()
    candidate[field] = candidate[field].copy()
    candidate[field][0] = "mismatched-case"
    with pytest.raises(ValueError, match=field):
        quality.evaluate_candidate(baseline, candidate)


@pytest.mark.parametrize("archive_name", ["baseline", "candidate"])
def test_perception_quality_requires_unique_case_provenance(archive_name):
    baseline = quality_archive()
    candidate = quality_archive()
    archive = baseline if archive_name == "baseline" else candidate
    archive["case_provenance"][1] = archive["case_provenance"][0]
    with pytest.raises(
        ValueError, match=rf"{archive_name} case_provenance must be unique"
    ):
        quality.evaluate_candidate(baseline, candidate)


def test_perception_quality_enforces_minimum_cases_and_endpoint_count():
    baseline = quality_archive(cases=2, height=80, width=80)
    candidate = quality_archive(cases=2, height=80, width=80)
    report = quality.evaluate_candidate(baseline, candidate)
    assert not report["passed"]
    assert not report["checks"]["minimum_cases"]
    assert not report["checks"]["minimum_endpoint_samples"]
    assert report["metrics"]["endpoint_samples"] == 2


def test_perception_quality_cli_reports_corrupt_candidate_and_continues(
    tmp_path, capsys,
):
    baseline_path = tmp_path / "baseline.npz"
    good_path = tmp_path / "good.npz"
    corrupt_path = tmp_path / "corrupt.npz"
    np.savez(baseline_path, **quality_archive())
    np.savez(good_path, **quality_archive())
    corrupt_path.write_bytes(b"PK\x03\x04truncated")

    result = quality.main([
        "--baseline", str(baseline_path),
        "--candidate", f"broken={corrupt_path}",
        "--candidate", f"good={good_path}",
    ])
    output = json.loads(capsys.readouterr().out)
    assert result == 0
    assert output["selected"] == "good"
    assert output["candidates"][0]["passed"] is False
    assert output["candidates"][0]["error"]
