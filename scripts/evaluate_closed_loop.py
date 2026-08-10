#!/usr/bin/env python3
"""Evaluate one browser flight log against the trusted 15 Hz gate.

Schema v2 is deliberately fail closed.  A frame counts only when the browser
records a successful trajectory-install acknowledgement, an allowed metric
depth mode, complete frame identity, and stable calibration/service
fingerprints.  The first ``warmup_frames`` legal unique frames are excluded
from every timed perception metric.
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path
from urllib.parse import parse_qsl, urlsplit


GATES = {
    "mean_planning_hz_min": 15.0,
    "planning_interval_p95_ms_max": 100.0,
    "capture_to_apply_p95_ms_max": 150.0,
    "physics_update_interval_p95_ms_max": 33.3,
    "warm_server_p95_ms_max": 50.0,
}

DEFAULT_REQUIRED_SCHEMA_VERSION = 2
DEFAULT_WARMUP_FRAMES = 30
DEFAULT_MIN_DURATION_S = 60.0
DEFAULT_MIN_PLANNING_FRAMES = 900
DEFAULT_MIN_PHYSICS_COVERAGE = 0.95
DEFAULT_PLANNING_OBSERVATION_HARD_AGE_MS = 250.0
DEFAULT_ALLOWED_DEPTH_MODES = ("da360-metric", "cesium-truth")
SESSION_DURATION_ROUNDING_TOLERANCE_MS = 5.1
MAX_SAFE_GENERATION = 2**53 - 1

# Keep this exact allowlist synchronized with SAFE_URL_QUERY_KEYS in
# src/flight-logger.js.  Schema-v2 evidence must not reintroduce URL fields that
# the browser deliberately removes before exporting a flight log.
SAFE_RESOLVED_URL_ENUM_QUERY_VALUES = {
    "panoProfile": frozenset(("flight", "calibration")),
    "panoCaptureProfile": frozenset(("flight", "calibration")),
}
SAFE_RESOLVED_URL_NUMERIC_QUERY_KEYS = frozenset((
    "panoCaptureAnyway",
    "panoPreloadRequired",
    "panoWidth",
    "panoHeight",
    "panoFace",
    "panoVfov",
    "panoJpeg",
    "panoMs",
    "panoFaceFov",
    "panoTopPoleGuard",
    "panoBottomPoleGuard",
    "panoFrameDelayMs",
    "panoFaceTileTimeoutMs",
    "panoFaceTileQuietMs",
    "panoFacesPerSlice",
    "panoPreloadFrameDelayMs",
    "panoPreloadFaceTileTimeoutMs",
    "panoPreloadFaceTileQuietMs",
    "panoPreloadTimeoutMs",
    "da360TimeoutMs",
    "da360UploadScale",
    "da360UploadWidth",
    "da360UploadHeight",
    "depthMs",
    "yopoMaxFrameAgeMs",
    "panoramaTileSse",
    "flightTileSse",
    "placementTileSse",
    "resolutionScale",
    "placementResolutionScale",
    "tileCacheMb",
    "droneScale",
    "flightPreloadRadius",
    "flightPreloadMinCoverage",
    "flightPreloadViewTimeoutMs",
    "flightPreloadViewAttempts",
    "flightPreloadStrict",
))
SAFE_RESOLVED_URL_QUERY_KEYS = frozenset(SAFE_RESOLVED_URL_ENUM_QUERY_VALUES) \
    | SAFE_RESOLVED_URL_NUMERIC_QUERY_KEYS

_JAVASCRIPT_TRIM_CHARACTERS = (
    "\t\n\v\f\r \u00a0\u1680\u2000\u2001\u2002\u2003\u2004\u2005"
    "\u2006\u2007\u2008\u2009\u200a\u2028\u2029\u202f\u205f\u3000\ufeff"
)
_JAVASCRIPT_DECIMAL_NUMBER = re.compile(
    r"[+-]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[eE][+-]?[0-9]+)?\Z"
)
_JAVASCRIPT_RADIX_NUMBER = re.compile(
    r"(?:0[xX][0-9a-fA-F]+|0[bB][01]+|0[oO][0-7]+)\Z"
)
_INVALID_PERCENT_ESCAPE = re.compile(r"%(?![0-9a-fA-F]{2})")


def _finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _is_finite_javascript_number_string(value):
    """Mirror the logger's non-empty ``Number.isFinite(Number(value))`` gate."""
    if not isinstance(value, str):
        return False
    token = value.strip(_JAVASCRIPT_TRIM_CHARACTERS)
    if not token:
        return False
    try:
        if _JAVASCRIPT_RADIX_NUMBER.fullmatch(token):
            base = {"x": 16, "b": 2, "o": 8}[token[1].lower()]
            parsed = float(int(token[2:], base))
        elif _JAVASCRIPT_DECIMAL_NUMBER.fullmatch(token):
            parsed = float(token)
        else:
            return False
    except (OverflowError, ValueError):
        return False
    return math.isfinite(parsed)


def _validate_resolved_url(value):
    """Validate the sanitized browser URL without ever echoing secret values."""
    if not isinstance(value, str) or not value:
        return False, ["missing-or-not-string"]
    if value != value.strip() or any(character.isspace() for character in value):
        return False, ["invalid-url-syntax"]
    if "#" in value:
        return False, ["fragment-present"]
    if "\\" in value:
        return False, ["invalid-url-syntax"]

    try:
        parsed = urlsplit(value)
        # Accessing .port performs validation that urlsplit otherwise defers.
        parsed.port
    except (TypeError, UnicodeError, ValueError):
        return False, ["invalid-url-syntax"]
    if parsed.scheme.lower() not in {"http", "https"}:
        return False, ["invalid-url-scheme"]
    if not parsed.netloc or parsed.hostname is None:
        return False, ["missing-url-host"]
    if parsed.hostname.lower() not in {"127.0.0.1", "localhost"}:
        return False, ["non-local-url-host"]
    if parsed.username is not None or parsed.password is not None:
        return False, ["userinfo-present"]
    if parsed.path not in {"/", "/index.html"}:
        return False, ["invalid-entry-path"]
    if _INVALID_PERCENT_ESCAPE.search(parsed.path) \
            or _INVALID_PERCENT_ESCAPE.search(parsed.query):
        return False, ["invalid-query-encoding"]

    try:
        query = parse_qsl(
            parsed.query,
            keep_blank_values=True,
            strict_parsing=False,
            encoding="utf-8",
            errors="strict",
        )
    except (UnicodeDecodeError, ValueError):
        return False, ["invalid-query-encoding"]

    seen = set()
    for key, query_value in query:
        if key in seen:
            return False, ["duplicate-query-key"]
        seen.add(key)
        if key not in SAFE_RESOLVED_URL_QUERY_KEYS:
            return False, ["query-key-not-allowlisted"]
        enum_values = SAFE_RESOLVED_URL_ENUM_QUERY_VALUES.get(key)
        if enum_values is not None:
            if query_value not in enum_values:
                return False, ["invalid-enum-query-value"]
        elif not _is_finite_javascript_number_string(query_value):
            return False, ["invalid-numeric-query-value"]
    return True, []


def _identity_token(value):
    if value is None or isinstance(value, bool):
        return None
    token = str(value)
    if not token or token != token.strip() or len(token) > 128:
        return None
    if any(ord(character) < 32 for character in token):
        return None
    return token


def _generation_token(value):
    """Match the API contract: a non-negative JavaScript-safe integer token."""
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, int):
        token = str(value)
    elif isinstance(value, str):
        token = value
    else:
        return None
    if (
        not token
        or token != token.strip()
        or any(character not in "0123456789" for character in token)
    ):
        return None
    return token if int(token) <= MAX_SAFE_GENERATION else None


def _nonempty_fingerprint(value):
    return _identity_token(value)


def percentile(values, quantile):
    finite = []
    for value in values:
        parsed = _finite_number(value)
        if parsed is not None:
            finite.append(parsed)
    finite.sort()
    if not finite:
        return None
    index = max(0, min(len(finite) - 1, math.ceil(quantile * len(finite)) - 1))
    return finite[index]


def _candidate_planning_events(log):
    """Return events that claim an authorized, applied planning outcome."""
    return [
        item for item in log.get("perception", [])
        if isinstance(item, dict)
        and item.get("mode") == "planning"
        and item.get("outcome") == "applied"
        and item.get("planningAuthorized") is True
    ]


def _contradictory_planning_events(log):
    """Count planning records whose authorization/apply claims disagree."""
    violations = 0
    for item in log.get("perception", []):
        if not isinstance(item, dict) or item.get("mode") != "planning":
            continue
        outcome_applied = item.get("outcome") == "applied"
        trajectory_applied = item.get("trajectoryApplied") is True
        authorized = item.get("planningAuthorized") is True
        if outcome_applied and not (trajectory_applied and authorized):
            violations += 1
        elif trajectory_applied and not (outcome_applied and authorized):
            violations += 1
    return violations


def _validate_planning_event(item, allowed_depth_modes):
    identity = (
        _identity_token(item.get("goalId")),
        _generation_token(item.get("generation")),
        _identity_token(item.get("frameId")),
    )
    depth_mode = _identity_token(item.get("depthMode"))
    calibration_id = _nonempty_fingerprint(item.get("calibrationId"))
    service_fingerprint = _nonempty_fingerprint(item.get("serviceFingerprint"))
    apply_at = _finite_number(item.get("trajectoryAppliedAtMs"))
    capture_to_apply = _finite_number(item.get("captureToApplyMs"))
    server_ms = _finite_number(item.get("serverMs"))
    errors = []
    if item.get("trajectoryApplied") is not True:
        errors.append("trajectoryApplied")
    if any(token is None for token in identity):
        errors.append("identity")
    if depth_mode not in allowed_depth_modes:
        errors.append("depthMode")
    if calibration_id is None:
        errors.append("calibrationId")
    if service_fingerprint is None:
        errors.append("serviceFingerprint")
    if apply_at is None:
        errors.append("trajectoryAppliedAtMs")
    if capture_to_apply is None or capture_to_apply < 0:
        errors.append("captureToApplyMs")
    if server_ms is None or server_ms < 0:
        errors.append("serverMs")
    return {
        "item": item,
        "identity": identity,
        "depth_mode": depth_mode,
        "calibration_id": calibration_id,
        "service_fingerprint": service_fingerprint,
        "apply_at_ms": apply_at,
        "capture_to_apply_ms": capture_to_apply,
        "server_ms": server_ms,
        "errors": errors,
    }


def _physics_frame_times(log):
    """Return frame timestamps in their original log order."""
    start_ms = _finite_number(log.get("monotonicStartMs"))
    times = []
    invalid = 0
    for frame in log.get("frames", []):
        if not isinstance(frame, dict):
            invalid += 1
            continue
        recorded_at = _finite_number(frame.get("recordedAtMs"))
        if recorded_at is None and start_ms is not None:
            elapsed_s = _finite_number(frame.get("t"))
            if elapsed_s is not None:
                recorded_at = start_ms + elapsed_s * 1000.0
        if recorded_at is None:
            invalid += 1
        else:
            times.append(recorded_at)
    return times, invalid


def evaluate_log(
    log,
    warmup_samples=DEFAULT_WARMUP_FRAMES,
    *,
    min_duration_s=DEFAULT_MIN_DURATION_S,
    min_planning_frames=DEFAULT_MIN_PLANNING_FRAMES,
    min_physics_coverage=DEFAULT_MIN_PHYSICS_COVERAGE,
    planning_observation_hard_age_ms=DEFAULT_PLANNING_OBSERVATION_HARD_AGE_MS,
    required_schema_version=DEFAULT_REQUIRED_SCHEMA_VERSION,
    allowed_depth_modes=DEFAULT_ALLOWED_DEPTH_MODES,
    gates=None,
):
    effective_gates = dict(GATES if gates is None else gates)
    warmup_frames = max(0, int(warmup_samples))
    minimum_duration = _finite_number(min_duration_s)
    if minimum_duration is None or minimum_duration < 0:
        raise ValueError("min_duration_s must be a finite non-negative number")
    minimum_frames = max(1, int(min_planning_frames))
    minimum_physics_coverage = _finite_number(min_physics_coverage)
    if (
        minimum_physics_coverage is None
        or not 0 < minimum_physics_coverage <= 1
    ):
        raise ValueError("min_physics_coverage must be finite and in (0,1]")
    hard_observation_age_ms = _finite_number(planning_observation_hard_age_ms)
    if (
        hard_observation_age_ms is None
        or hard_observation_age_ms <= 0
        or hard_observation_age_ms > DEFAULT_PLANNING_OBSERVATION_HARD_AGE_MS
    ):
        raise ValueError(
            "planning_observation_hard_age_ms must be finite and in "
            f"(0,{DEFAULT_PLANNING_OBSERVATION_HARD_AGE_MS:g}]"
        )
    allowed_modes = tuple(dict.fromkeys(str(mode) for mode in allowed_depth_modes))

    candidates = _candidate_planning_events(log)
    contradictory_planning_events = _contradictory_planning_events(log)
    validated = [_validate_planning_event(item, set(allowed_modes)) for item in candidates]
    legal = [event for event in validated if not event["errors"]]
    legal_apply_times = [event["apply_at_ms"] for event in legal]
    legal_planning_intervals = [
        current - previous
        for previous, current in zip(legal_apply_times, legal_apply_times[1:])
    ]
    nonincreasing_planning_timestamps = sum(
        interval <= 0 for interval in legal_planning_intervals
    )

    unique = []
    seen = set()
    duplicate_count = 0
    for event in legal:
        if event["identity"] in seen:
            duplicate_count += 1
            continue
        seen.add(event["identity"])
        unique.append(event)

    measured = unique[warmup_frames:]
    apply_times = [event["apply_at_ms"] for event in measured]
    planning_intervals = [
        current - previous for previous, current in zip(apply_times, apply_times[1:])
    ]
    nonpositive_intervals = sum(interval <= 0 for interval in planning_intervals)
    measurement_duration_s = (
        (apply_times[-1] - apply_times[0]) / 1000.0 if len(apply_times) >= 2 else 0.0
    )
    mean_planning_hz = (
        (len(apply_times) - 1) / measurement_duration_s
        if len(apply_times) >= 2 and measurement_duration_s > 0 else 0.0
    )

    capture_to_apply = [event["capture_to_apply_ms"] for event in measured]
    server_values = [event["server_ms"] for event in measured]
    depth_modes = {event["depth_mode"] for event in unique}
    goal_ids = {event["identity"][0] for event in unique}
    generations = {event["identity"][1] for event in unique}
    calibration_ids = {event["calibration_id"] for event in unique}
    service_fingerprints = {event["service_fingerprint"] for event in unique}
    overage_planning_frames = sum(
        event["capture_to_apply_ms"] > hard_observation_age_ms for event in unique
    )

    physics_times, invalid_physics_timestamps = _physics_frame_times(log)
    all_physics_intervals = [
        current - previous
        for previous, current in zip(physics_times, physics_times[1:])
    ]
    nonincreasing_physics_timestamps = sum(
        interval <= 0 for interval in all_physics_intervals
    )
    if apply_times:
        physics_times = [
            timestamp for timestamp in physics_times
            if apply_times[0] <= timestamp <= apply_times[-1]
        ]
    else:
        physics_times = []
    physics_intervals = [
        current - previous for previous, current in zip(physics_times, physics_times[1:])
    ]
    physics_measurement_span_s = (
        (physics_times[-1] - physics_times[0]) / 1000.0
        if len(physics_times) >= 2 else 0.0
    )
    physics_span_coverage = (
        physics_measurement_span_s / measurement_duration_s
        if measurement_duration_s > 0 else 0.0
    )
    physics_gate_ms = effective_gates["physics_update_interval_p95_ms_max"]
    physics_measurement_coverage = (
        sum(min(max(interval, 0.0), physics_gate_ms) for interval in physics_intervals)
        / (measurement_duration_s * 1000.0)
        if measurement_duration_s > 0 and physics_gate_ms > 0 else 0.0
    )
    # A p95 computed from one or two strategically placed physics timestamps is
    # not meaningful evidence. Require enough intervals and cap each interval's
    # coverage contribution at the gate cadence, so a long unobserved gap cannot
    # be hidden behind a burst of dense timestamps plus one endpoint sample.
    required_physics_intervals = max(
        1,
        math.ceil(
            measurement_duration_s * 1000.0
            * minimum_physics_coverage / physics_gate_ms
        ),
    ) if physics_gate_ms > 0 else math.inf
    required_physics_frames = (
        required_physics_intervals + 1
        if math.isfinite(required_physics_intervals) else math.inf
    )

    invalid_field_counts = {}
    for event in validated:
        for field in event["errors"]:
            invalid_field_counts[field] = invalid_field_counts.get(field, 0) + 1

    schema_version = log.get("schemaVersion")
    resolved_url_safe, resolved_url_errors = _validate_resolved_url(
        log.get("resolvedUrl")
    )
    navigation_session = log.get("navigationSession")
    navigation_goal_id = _identity_token(
        navigation_session.get("goalId")
        if isinstance(navigation_session, dict) else None
    )
    navigation_generation = _generation_token(
        navigation_session.get("generation")
        if isinstance(navigation_session, dict) else None
    )
    navigation_session_valid = (
        navigation_goal_id is not None and navigation_generation is not None
    )
    all_planning_events = [
        item for item in log.get("perception", [])
        if isinstance(item, dict) and item.get("mode") == "planning"
    ]
    all_planning_identities = [
        (
            _identity_token(item.get("goalId")),
            _generation_token(item.get("generation")),
            _identity_token(item.get("frameId")),
        )
        for item in all_planning_events
    ]
    foreign_or_malformed_planning_events = sum(
        identity[0] != navigation_goal_id
        or identity[1] != navigation_generation
        or identity[2] is None
        for identity in all_planning_identities
    ) if navigation_session_valid else len(all_planning_events)
    valid_all_planning_identities = [
        identity for identity in all_planning_identities
        if all(token is not None for token in identity)
    ]
    duplicate_all_planning_identities = (
        len(valid_all_planning_identities)
        - len(set(valid_all_planning_identities))
    )
    all_planning_events_match_session = (
        bool(all_planning_events)
        and navigation_session_valid
        and foreign_or_malformed_planning_events == 0
    )
    planning_identity_matches_session = bool(unique) and navigation_session_valid
    if planning_identity_matches_session:
        planning_identity_matches_session = all(
            event["identity"][0] == navigation_goal_id
            and event["identity"][1] == navigation_generation
            for event in unique
        )
    session_start_ms = _finite_number(log.get("monotonicStartMs"))
    session_duration_s = _finite_number(log.get("duration_s"))
    session_end_ms = (
        session_start_ms + session_duration_s * 1000.0
        if session_start_ms is not None
        and session_duration_s is not None
        and session_duration_s > 0
        else None
    )
    planning_events_within_session = bool(legal_apply_times) and session_end_ms is not None
    if planning_events_within_session:
        planning_events_within_session = all(
            session_start_ms <= timestamp
            <= session_end_ms + SESSION_DURATION_ROUNDING_TOLERANCE_MS
            for timestamp in legal_apply_times
        )
    first_legal_planning_apply_ms = min(
        (event["apply_at_ms"] for event in unique), default=None
    )
    last_legal_planning_apply_ms = max(
        (event["apply_at_ms"] for event in unique), default=None
    )
    max_planning_session_boundary_gap_ms = (
        hard_observation_age_ms + SESSION_DURATION_ROUNDING_TOLERANCE_MS
    )
    planning_session_head_gap_ms = (
        max(0.0, first_legal_planning_apply_ms - session_start_ms)
        if session_start_ms is not None and first_legal_planning_apply_ms is not None
        else None
    )
    max_planning_session_tail_gap_ms = (
        max_planning_session_boundary_gap_ms
    )
    planning_session_tail_gap_ms = (
        max(0.0, session_end_ms - last_legal_planning_apply_ms)
        if session_end_ms is not None and last_legal_planning_apply_ms is not None
        else None
    )
    # Dense evidence in the middle must not certify a declared navigation
    # session that was stalled after the goal was set or before it ended. Reuse
    # the hard observation-age envelope at both boundaries, with only the
    # existing duration-rounding tolerance added.
    planning_starts_near_session_beginning = (
        planning_session_head_gap_ms is not None
        and planning_session_head_gap_ms <= max_planning_session_boundary_gap_ms
    )
    planning_reaches_session_end = (
        planning_session_tail_gap_ms is not None
        and planning_session_tail_gap_ms <= max_planning_session_tail_gap_ms
    )
    metrics = {
        "session_start_ms": session_start_ms,
        "session_duration_s": session_duration_s,
        "session_end_ms": session_end_ms,
        "first_legal_planning_apply_ms": first_legal_planning_apply_ms,
        "planning_session_head_gap_ms": planning_session_head_gap_ms,
        "last_legal_planning_apply_ms": last_legal_planning_apply_ms,
        "planning_session_tail_gap_ms": planning_session_tail_gap_ms,
        "measurement_duration_s": measurement_duration_s,
        "candidate_planning_events": len(candidates),
        "legal_unique_planning_frames_total": len(unique),
        "unique_planning_frames": len(measured),
        "duplicate_planning_frames": duplicate_count,
        "contradictory_planning_events": contradictory_planning_events,
        "mean_planning_hz": mean_planning_hz,
        "planning_interval_p95_ms": percentile(planning_intervals, 0.95),
        "capture_to_apply_p95_ms": percentile(capture_to_apply, 0.95),
        "physics_update_interval_p95_ms": percentile(physics_intervals, 0.95),
        "warm_server_p95_ms": percentile(server_values, 0.95),
        "warmup_frames_discarded": min(len(unique), warmup_frames),
        "invalid_planning_events": sum(bool(event["errors"]) for event in validated),
        "invalid_field_counts": invalid_field_counts,
        "invalid_physics_timestamps": invalid_physics_timestamps,
        "nonincreasing_planning_timestamps": nonincreasing_planning_timestamps,
        "nonincreasing_physics_timestamps": nonincreasing_physics_timestamps,
        "nonpositive_planning_intervals": nonpositive_intervals,
        "depth_modes": sorted(mode for mode in depth_modes if mode is not None),
        "calibration_ids": sorted(value for value in calibration_ids if value is not None),
        "service_fingerprints": sorted(
            value for value in service_fingerprints if value is not None
        ),
        "goal_ids": sorted(value for value in goal_ids if value is not None),
        "generations": sorted(value for value in generations if value is not None),
        "navigation_goal_id": navigation_goal_id,
        "navigation_generation": navigation_generation,
        "all_planning_events": len(all_planning_events),
        "foreign_or_malformed_planning_events": foreign_or_malformed_planning_events,
        "duplicate_all_planning_identities": duplicate_all_planning_identities,
        "resolved_url_errors": resolved_url_errors,
        "planning_observation_hard_age_ms": hard_observation_age_ms,
        "overage_planning_frames": overage_planning_frames,
        "maximum_capture_to_apply_ms": max(
            (event["capture_to_apply_ms"] for event in unique), default=None
        ),
        "physics_frames_in_measurement": len(physics_times),
        "physics_intervals_in_measurement": len(physics_intervals),
        "physics_measurement_span_s": physics_measurement_span_s,
        "physics_span_coverage": physics_span_coverage,
        "physics_measurement_coverage": physics_measurement_coverage,
    }
    checks = {
        "schema_version": isinstance(schema_version, int)
        and not isinstance(schema_version, bool)
        and schema_version >= required_schema_version,
        "resolved_url_safe": resolved_url_safe,
        "all_candidate_planning_events_valid": bool(candidates)
        and not any(event["errors"] for event in validated),
        "no_contradictory_planning_events": contradictory_planning_events == 0,
        "no_duplicate_planning_frames": duplicate_count == 0,
        "planning_timestamps_strictly_increasing": bool(legal)
        and nonincreasing_planning_timestamps == 0,
        "actual_trajectory_evidence": bool(unique)
        and all(event["item"].get("trajectoryApplied") is True for event in unique),
        "navigation_session_metadata": navigation_session_valid,
        "all_planning_events_match_navigation_session": all_planning_events_match_session,
        "all_planning_control_identities_unique": bool(all_planning_events)
        and foreign_or_malformed_planning_events == 0
        and duplicate_all_planning_identities == 0,
        "planning_identity_matches_navigation_session": planning_identity_matches_session,
        "goal_id_stable": len(goal_ids) == 1,
        "generation_stable": len(generations) == 1,
        "required_identity_fields": not any(
            "identity" in event["errors"] for event in validated
        ),
        "required_timing_fields": not any(
            set(event["errors"]) & {
                "trajectoryAppliedAtMs", "captureToApplyMs", "serverMs"
            }
            for event in validated
        ),
        "planning_observation_hard_age": bool(unique)
        and overage_planning_frames == 0,
        "depth_modes_allowed": bool(depth_modes)
        and depth_modes.issubset(set(allowed_modes))
        and not any("depthMode" in event["errors"] for event in validated),
        "depth_mode_stable": len(depth_modes) == 1,
        "calibration_id_present": bool(calibration_ids)
        and not any("calibrationId" in event["errors"] for event in validated),
        "calibration_id_stable": len(calibration_ids) == 1,
        "service_fingerprint_present": bool(service_fingerprints)
        and not any("serviceFingerprint" in event["errors"] for event in validated),
        "service_fingerprint_stable": len(service_fingerprints) == 1,
        "physics_timestamps_present": bool(physics_times)
        and invalid_physics_timestamps == 0,
        "physics_timestamps_strictly_increasing": bool(physics_times)
        and nonincreasing_physics_timestamps == 0,
        "physics_measurement_coverage": measurement_duration_s > 0
        and physics_measurement_coverage >= minimum_physics_coverage,
        "minimum_physics_frames": len(physics_times) >= required_physics_frames,
        "positive_planning_intervals": bool(planning_intervals)
        and nonpositive_intervals == 0,
        "session_metadata_valid": session_start_ms is not None
        and session_duration_s is not None
        and session_duration_s > 0,
        "planning_events_within_session": planning_events_within_session,
        "planning_starts_near_session_beginning": planning_starts_near_session_beginning,
        "planning_reaches_session_end": planning_reaches_session_end,
        "minimum_measurement_duration": measurement_duration_s >= minimum_duration,
        "minimum_planning_frames": len(measured) >= minimum_frames,
        "mean_planning_hz": mean_planning_hz
        >= effective_gates["mean_planning_hz_min"],
        "planning_interval_p95_ms": metrics["planning_interval_p95_ms"] is not None
        and metrics["planning_interval_p95_ms"]
        <= effective_gates["planning_interval_p95_ms_max"],
        "capture_to_apply_p95_ms": metrics["capture_to_apply_p95_ms"] is not None
        and metrics["capture_to_apply_p95_ms"]
        <= effective_gates["capture_to_apply_p95_ms_max"],
        "physics_update_interval_p95_ms": metrics["physics_update_interval_p95_ms"] is not None
        and metrics["physics_update_interval_p95_ms"]
        <= effective_gates["physics_update_interval_p95_ms_max"],
        "warm_server_p95_ms": metrics["warm_server_p95_ms"] is not None
        and metrics["warm_server_p95_ms"]
        <= effective_gates["warm_server_p95_ms_max"],
    }
    requirements = {
        "required_schema_version": required_schema_version,
        "warmup_frames": warmup_frames,
        "min_measurement_duration_s": minimum_duration,
        "min_planning_frames": minimum_frames,
        "min_physics_coverage": minimum_physics_coverage,
        "planning_observation_hard_age_ms": hard_observation_age_ms,
        "max_planning_session_head_gap_ms": max_planning_session_boundary_gap_ms,
        "max_planning_session_tail_gap_ms": max_planning_session_tail_gap_ms,
        "required_physics_frames": required_physics_frames,
        "allowed_depth_modes": list(allowed_modes),
    }
    return {
        "passed": all(checks.values()),
        "gates": effective_gates,
        "requirements": requirements,
        "checks": checks,
        "metrics": metrics,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="schema-v2 flight-log JSON")
    parser.add_argument(
        "--warmup-frames", "--warmup-samples", dest="warmup_frames",
        type=int, default=DEFAULT_WARMUP_FRAMES,
    )
    parser.add_argument("--min-duration-s", type=float, default=DEFAULT_MIN_DURATION_S)
    parser.add_argument(
        "--min-planning-frames", type=int, default=DEFAULT_MIN_PLANNING_FRAMES
    )
    parser.add_argument(
        "--min-physics-coverage", type=float,
        default=DEFAULT_MIN_PHYSICS_COVERAGE,
    )
    parser.add_argument(
        "--planning-observation-hard-age-ms", type=float,
        default=DEFAULT_PLANNING_OBSERVATION_HARD_AGE_MS,
    )
    parser.add_argument(
        "--required-schema-version", type=int,
        default=DEFAULT_REQUIRED_SCHEMA_VERSION,
    )
    parser.add_argument(
        "--allowed-depth-mode", action="append", dest="allowed_depth_modes",
        choices=DEFAULT_ALLOWED_DEPTH_MODES,
        help="repeat to replace the default allowed-mode set",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    if not math.isfinite(args.min_duration_s) or args.min_duration_s < 0:
        parser.error("--min-duration-s must be finite and non-negative")
    if not math.isfinite(args.min_physics_coverage) or not 0 < args.min_physics_coverage <= 1:
        parser.error("--min-physics-coverage must be finite and in (0,1]")
    if (
        not math.isfinite(args.planning_observation_hard_age_ms)
        or args.planning_observation_hard_age_ms <= 0
    ):
        parser.error("--planning-observation-hard-age-ms must be finite and positive")

    with args.log.open(encoding="utf-8") as stream:
        report = evaluate_log(
            json.load(stream),
            args.warmup_frames,
            min_duration_s=args.min_duration_s,
            min_planning_frames=args.min_planning_frames,
            min_physics_coverage=args.min_physics_coverage,
            planning_observation_hard_age_ms=args.planning_observation_hard_age_ms,
            required_schema_version=args.required_schema_version,
            allowed_depth_modes=args.allowed_depth_modes or DEFAULT_ALLOWED_DEPTH_MODES,
        )
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    sys.exit(main())
