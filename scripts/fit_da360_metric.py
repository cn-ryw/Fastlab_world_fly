#!/usr/bin/env python3
"""
Offline DA360 scale-only metric fitting: 1/z = a * pred_disp, b = 0.

Takes DA360 raw pred_disp (.npz from /depth/raw) and Cesium sparse anchors
(.json), deploys a scale-only model via robust regression, and outputs
metric-depth arrays together with a validation report.  The scale+shift fit is
retained only as a diagnostic: DA360 removes the additive disparity shift
internally, so an external non-zero b must never enter runtime calibration.

Usage:
    python scripts/fit_da360_metric.py \\
        --raw depth_raw.npz \\
        --anchors cesium_anchors.json \\
        --manifest capture_manifest.json \\
        --rgb capture_rgb.jpg \\
        --output experiment_data/metric_fit_sample/

Repeat all four artifact flags for multi-capture fitting. The fitter rejects
unpaired bundles, incomplete identity/projection contracts and duplicate
capture identities or poses before fitting. A runtime calibration is emitted
only after every held-out location and the pooled result pass all acceptance
gates.
"""
import argparse
import json
import math
import os
import sys
import warnings
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from PIL import Image, ImageOps

warnings.filterwarnings("ignore", category=RuntimeWarning)

EPS = np.float32(1e-6)
SEED = 42
HUBER_K = 1.345
MIN_HUBER_SCALE = 1e-6
METRIC_MIN_DEPTH_M = 0.5
METRIC_MAX_DEPTH_M = 20.0
NEAR_MAX_DEPTH_M = 10.0
MIN_LOCATIONS = 4
MIN_POSES_PER_LOCATION = 3
MIN_CAPTURES = MIN_LOCATIONS * MIN_POSES_PER_LOCATION
POSE_POSITION_EPS_M = 0.25
POSE_ANGLE_EPS_DEG = 5.0


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
    t = np.zeros(depth.shape, dtype=np.float32)
    t[valid] = 1.0 - (np.clip(depth[valid], near, far) - near) / max(far - near, 1e-6)
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
def _validate_fit_inputs(x, y, min_points):
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    y = np.asarray(y, dtype=np.float64).reshape(-1)
    if x.size != y.size:
        raise ValueError("x and y must have the same length")
    if x.size < min_points:
        raise ValueError(f"need at least {min_points} finite samples")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("fit samples must be finite")
    return x, y


def _linear_residuals(params, x, y):
    """Raw inverse-depth residuals; robustification is applied once by SciPy."""
    a, b = params
    return y - (a * x + b)


def _training_huber_scale(residuals):
    """Return a robust Huber threshold derived only from training residuals."""
    residuals = np.asarray(residuals, dtype=np.float64)
    residuals = residuals[np.isfinite(residuals)]
    if residuals.size == 0:
        raise ValueError("cannot derive Huber scale from empty residuals")
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    sigma = 1.4826 * mad
    return max(MIN_HUBER_SCALE, HUBER_K * sigma)


def fit_scale_shift(x, y, delta=None):
    """Huber-robust fit: y = a*x + b  with a > 0 constraint."""
    x, y = _validate_fit_inputs(x, y, min_points=2)
    if np.ptp(x) <= 1e-12:
        raise ValueError("scale+shift fit requires non-constant disparity samples")
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
    huber_scale = _training_huber_scale(y - (a0 * x + b0)) if delta is None else float(delta)
    if not np.isfinite(huber_scale) or huber_scale <= 0:
        raise ValueError("Huber scale must be positive and finite")

    result = least_squares(
        lambda p: _linear_residuals(p, x, y),
        [a0, b0],
        bounds=([1e-12, -np.inf], [np.inf, np.inf]),
        max_nfev=2000,
        loss='huber',
        f_scale=huber_scale,
    )
    a, b = float(result.x[0]), float(result.x[1])
    residuals = y - (a * x + b)
    return a, b, residuals


def fit_scale_only(x, y, delta=None):
    """Huber-robust fit: y = a*x  (no intercept)."""
    x, y = _validate_fit_inputs(x, y, min_points=1)
    nonzero = np.abs(x) > 1e-12
    if not np.any(nonzero):
        raise ValueError("scale-only fit requires non-zero disparity samples")
    a0 = max(1e-9, float(np.median(y[nonzero] / x[nonzero])))
    huber_scale = _training_huber_scale(y - a0 * x) if delta is None else float(delta)
    if not np.isfinite(huber_scale) or huber_scale <= 0:
        raise ValueError("Huber scale must be positive and finite")
    result = least_squares(
        lambda a: y - a[0] * x,
        [a0],
        bounds=([1e-12], [np.inf]),
        max_nfev=2000,
        loss='huber',
        f_scale=huber_scale,
    )
    a = float(result.x[0])
    residuals = y - a * x
    return a, residuals


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def compute_metrics(y_true, y_pred, valid=None):
    """Standard metrics in metres; non-positive/non-finite depths are invalid."""
    y_true = np.asarray(y_true, dtype=np.float64)
    y_pred = np.asarray(y_pred, dtype=np.float64)
    if y_true.shape != y_pred.shape:
        raise ValueError("metric arrays must have the same shape")
    if valid is None:
        valid = np.ones_like(y_true, dtype=bool)
    else:
        valid = np.asarray(valid, dtype=bool)
        if valid.shape != y_true.shape:
            raise ValueError("metric validity mask must match depth shape")
    valid = valid & np.isfinite(y_true) & np.isfinite(y_pred) & (y_true > 0) & (y_pred > 0)
    yt, yp = y_true[valid], y_pred[valid]
    if len(yt) < 2:
        return {
            "median_abs_rel": None,
            "p90_abs_rel": None,
            "rmse_m": None,
            "median_error_m": None,
            "p90_error_m": None,
            "n_valid": int(valid.sum()),
        }
    abs_rel = np.median(np.abs(yt - yp) / np.maximum(yt, EPS))
    p90_val = np.percentile(np.abs(yt - yp) / np.maximum(yt, EPS), 90)
    rmse = np.sqrt(np.mean((yt - yp) ** 2))
    abs_err = np.abs(yt - yp)
    return {
        "median_abs_rel": float(abs_rel),
        "p90_abs_rel": float(p90_val),
        "rmse_m": float(rmse),
        "median_error_m": float(np.median(abs_err)),
        "p90_error_m": float(np.percentile(abs_err, 90)),
        "n_valid": int(valid.sum()),
    }


def inverse_depth_to_metric_depth(inverse_depth, base_valid=None):
    """Convert positive inverse depth (1/m) to metres and return its validity."""
    inverse_depth = np.asarray(inverse_depth)
    valid = np.isfinite(inverse_depth) & (inverse_depth > 0)
    if base_valid is not None:
        base_valid = np.asarray(base_valid, dtype=bool)
        if base_valid.shape != inverse_depth.shape:
            raise ValueError("base validity mask must match inverse-depth shape")
        valid &= base_valid
    depth = np.full(inverse_depth.shape, np.nan, dtype=np.float32)
    depth[valid] = (1.0 / inverse_depth[valid]).astype(np.float32)
    return depth, valid


def map_pixel_center(coordinate, source_size, target_size):
    """Map an integer-centre pixel coordinate between equal-FOV resolutions."""
    if not all(np.isfinite(v) for v in (coordinate, source_size, target_size)):
        raise ValueError("pixel mapping values must be finite")
    if source_size <= 0 or target_size <= 0:
        raise ValueError("image dimensions must be positive")
    return (float(coordinate) + 0.5) * float(target_size) / float(source_size) - 0.5


def sample_wrapped_bilinear(array, valid_mask, u, v):
    """Bilinear ERP sample with horizontal seam wrap and vertical edge clamp."""
    array = np.asarray(array)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    if array.ndim != 2 or valid_mask.shape != array.shape:
        raise ValueError("bilinear sampler requires matching 2-D array and mask")
    if not np.isfinite(u) or not np.isfinite(v):
        return None
    height, width = array.shape
    if height == 0 or width == 0 or v < -0.5 or v > height - 0.5:
        return None

    wrapped_u = float(u) % width
    clamped_v = min(max(float(v), 0.0), height - 1.0)
    x0_raw = int(np.floor(wrapped_u))
    y0 = int(np.floor(clamped_v))
    x1_raw = x0_raw + 1
    y1 = min(y0 + 1, height - 1)
    fx = wrapped_u - x0_raw
    fy = clamped_v - y0
    x0, x1 = x0_raw % width, x1_raw % width

    weighted = (
        (x0, y0, (1.0 - fx) * (1.0 - fy)),
        (x1, y0, fx * (1.0 - fy)),
        (x0, y1, (1.0 - fx) * fy),
        (x1, y1, fx * fy),
    )
    value = 0.0
    for x, y, weight in weighted:
        if weight <= 1e-15:
            continue
        if not valid_mask[y, x] or not np.isfinite(array[y, x]):
            return None
        value += float(array[y, x]) * weight
    return value


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def _load_json_object(path, label):
    try:
        with open(path, encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid {label}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _require_mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def _require_string(mapping, key, label):
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label}.{key} must be a non-empty string")
    return value


def _require_number(mapping, key, label, *, positive=False):
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label}.{key} must be finite")
    value = float(value)
    if positive and value <= 0:
        raise ValueError(f"{label}.{key} must be positive")
    return value


def _require_int(mapping, key, label, *, positive=False):
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{label}.{key} must be an integer")
    value = int(value)
    if positive and value <= 0:
        raise ValueError(f"{label}.{key} must be positive")
    return value


def _validate_transform(value, label):
    transform = _require_mapping(value, label)
    position = _require_mapping(transform.get("position"), f"{label}.position")
    orientation = _require_mapping(transform.get("orientation"), f"{label}.orientation")
    position_vector = np.array([
        _require_number(position, axis, f"{label}.position") for axis in "xyz"
    ], dtype=np.float64)
    quaternion = np.array([
        _require_number(orientation, axis, f"{label}.orientation") for axis in "xyzw"
    ], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(f"{label}.orientation must be a unit quaternion")
    return position_vector, quaternion / norm


def _validate_state(value, label, require_acceleration=False):
    state = _require_mapping(value, label)
    for vector_name in ("position", "velocity"):
        vector = _require_mapping(state.get(vector_name), f"{label}.{vector_name}")
        for axis in "xyz":
            _require_number(vector, axis, f"{label}.{vector_name}")
    if require_acceleration:
        acceleration = _require_mapping(state.get("acceleration"), f"{label}.acceleration")
        for axis in "xyz":
            _require_number(acceleration, axis, f"{label}.acceleration")


def _artifact_from_manifest(manifest, key, actual_path):
    files = _require_mapping(manifest.get("files"), "manifest.files")
    entry = _require_mapping(files.get(key), f"manifest.files.{key}")
    name = _require_string(entry, "name", f"manifest.files.{key}")
    if Path(actual_path).name != name:
        raise ValueError(
            f"manifest {key} filename mismatch: {Path(actual_path).name!r} != {name!r}"
        )
    declared_bytes = _require_int(entry, "bytes", f"manifest.files.{key}", positive=True)
    if not Path(actual_path).is_file() or Path(actual_path).stat().st_size <= 0:
        raise ValueError(f"manifest {key} artifact is missing or empty")
    if Path(actual_path).stat().st_size != declared_bytes:
        raise ValueError(f"manifest {key} artifact size mismatch")


def _projection_contract(manifest, rgb_width, rgb_height):
    projection = _require_mapping(manifest.get("projectionConfig"), "manifest.projectionConfig")
    contract = {}
    for key in ("width", "height", "faceSize", "rgbWidth", "rgbHeight"):
        contract[key] = _require_int(projection, key, "manifest.projectionConfig", positive=True)
    for key in ("verticalFovDeg", "faceFovDeg", "topPoleGuardDeg", "bottomPoleGuardDeg"):
        contract[key] = _require_number(projection, key, "manifest.projectionConfig")
    contract["jpegQuality"] = _require_number(
        projection, "jpegQuality", "manifest.projectionConfig", positive=True
    )
    contract["uploadScale"] = _require_number(
        projection, "uploadScale", "manifest.projectionConfig", positive=True
    )
    if contract["rgbWidth"] != rgb_width or contract["rgbHeight"] != rgb_height:
        raise ValueError("manifest RGB dimensions do not match decoded RGB")
    expected_upload_scale = contract["rgbWidth"] / contract["width"]
    if not math.isclose(contract["uploadScale"], expected_upload_scale, rel_tol=0, abs_tol=1e-12):
        raise ValueError("manifest uploadScale does not match encoded/panorama width")
    if not 0 < contract["verticalFovDeg"] <= 180:
        raise ValueError("manifest projection verticalFovDeg must be in (0, 180]")
    return contract


def _raw_contract(metadata, height, width, rgb_width, rgb_height):
    contract = {
        "api_version": _require_int(metadata, "api_version", "raw.metadata", positive=True),
        "model": _require_string(metadata, "model", "raw.metadata"),
        "width": _require_int(metadata, "width", "raw.metadata", positive=True),
        "height": _require_int(metadata, "height", "raw.metadata", positive=True),
        "input_scale": _require_number(metadata, "input_scale", "raw.metadata", positive=True),
        "resample": _require_string(metadata, "resample", "raw.metadata"),
        "request_width": _require_int(metadata, "request_width", "raw.metadata", positive=True),
        "request_height": _require_int(metadata, "request_height", "raw.metadata", positive=True),
        "depth_mode": _require_string(metadata, "depth_mode", "raw.metadata"),
        "unit_pred_disp": _require_string(metadata, "unit_pred_disp", "raw.metadata"),
    }
    if contract["api_version"] != 2:
        raise ValueError("raw.metadata.api_version must be 2")
    if contract["width"] != width or contract["height"] != height:
        raise ValueError("raw output dimensions do not match metadata contract")
    if contract["request_width"] != rgb_width or contract["request_height"] != rgb_height:
        raise ValueError("raw request dimensions do not match decoded RGB")
    if contract["depth_mode"] != "da360-relative":
        raise ValueError("calibration input must use da360-relative raw disparity")
    expected_unit = "raw disparity (inverse depth), NOT per-frame normalized"
    if contract["unit_pred_disp"] != expected_unit:
        raise ValueError("raw disparity semantic contract is incompatible")
    if contract["resample"] not in {"bicubic", "bilinear"}:
        raise ValueError("raw.metadata.resample is unsupported")
    return contract


def _anchor_capture_provenance(anchor_metadata, projection_contract):
    """Verify that anchors came from the exact six-face RGB capture source."""
    label = "anchors.metadata"
    if anchor_metadata.get("raycastSource") != "panorama-capture-viewer":
        raise ValueError(
            "anchors.metadata.raycastSource must be panorama-capture-viewer"
        )
    if anchor_metadata.get("tilesetSharedWithRgb") is not True:
        raise ValueError("anchors.metadata.tilesetSharedWithRgb must be true")

    revision = _require_int(
        anchor_metadata, "panoramaCaptureRevision", label, positive=True
    )
    face_size = _require_int(
        anchor_metadata, "panoramaFaceSize", label, positive=True
    )
    if face_size != projection_contract["faceSize"]:
        raise ValueError(
            "anchors panoramaFaceSize does not match manifest projection faceSize"
        )

    source_image = _require_mapping(
        anchor_metadata.get("panoramaSourceImage"),
        "anchors.metadata.panoramaSourceImage",
    )
    source_width = _require_int(
        source_image, "width", "anchors.metadata.panoramaSourceImage", positive=True
    )
    source_height = _require_int(
        source_image, "height", "anchors.metadata.panoramaSourceImage", positive=True
    )
    source_vertical_fov = _require_number(
        source_image,
        "verticalFovDeg",
        "anchors.metadata.panoramaSourceImage",
        positive=True,
    )
    if source_width != projection_contract["width"] \
            or source_height != projection_contract["height"]:
        raise ValueError(
            "anchors panoramaSourceImage dimensions do not match manifest projection"
        )
    if not math.isclose(
        source_vertical_fov,
        projection_contract["verticalFovDeg"],
        rel_tol=0,
        abs_tol=1e-12,
    ):
        raise ValueError(
            "anchors panoramaSourceImage vertical FOV does not match manifest projection"
        )

    face_readiness = anchor_metadata.get("panoramaFaceTileReadiness")
    if not isinstance(face_readiness, list):
        raise ValueError(
            "anchors.metadata.panoramaFaceTileReadiness must be an array"
        )
    expected_faces = {"front", "right", "back", "left", "up", "down"}
    if len(face_readiness) != len(expected_faces):
        raise ValueError(
            "anchors panoramaFaceTileReadiness must contain exactly six faces"
        )
    seen_faces = set()
    for index, entry in enumerate(face_readiness):
        face_label = f"anchors.metadata.panoramaFaceTileReadiness[{index}]"
        face = _require_string(_require_mapping(entry, face_label), "face", face_label)
        if face in seen_faces:
            raise ValueError(f"duplicate panorama capture face: {face}")
        seen_faces.add(face)
        if entry.get("readyWhenCopied") is not True:
            raise ValueError(f"{face_label}.readyWhenCopied must be true")
    if seen_faces != expected_faces:
        missing = ", ".join(sorted(expected_faces - seen_faces))
        unexpected = ", ".join(sorted(seen_faces - expected_faces))
        raise ValueError(
            "panorama capture faces do not match the six-face projection "
            f"(missing={missing or '<none>'}; unexpected={unexpected or '<none>'})"
        )

    return {
        "raycastSource": "panorama-capture-viewer",
        "tilesetSharedWithRgb": True,
        "panoramaCaptureRevision": revision,
        "panoramaFaceSize": face_size,
        "panoramaSourceImage": {
            "width": source_width,
            "height": source_height,
            "verticalFovDeg": source_vertical_fov,
        },
        "panoramaFaces": sorted(seen_faces),
    }


def load_data(raw_path, anchors_path, manifest_path, rgb_path):
    """Load and verify one identity-bound RGB/raw/anchor/manifest bundle."""
    manifest = _load_json_object(manifest_path, "manifest")
    if manifest.get("schemaVersion") != 2:
        raise ValueError("manifest.schemaVersion must be 2")
    identity = {
        key: _require_string(manifest, key, "manifest")
        for key in ("sessionId", "captureId", "locationId", "frameId")
    }
    _require_number(manifest, "capturedAt", "manifest")
    _require_string(manifest, "exportedAt", "manifest")
    manifest_position, manifest_quaternion = _validate_transform(
        manifest.get("transform"), "manifest.transform"
    )
    _validate_state(manifest.get("actualState"), "manifest.actualState")
    _validate_state(manifest.get("referenceState"), "manifest.referenceState", True)
    _require_number(manifest, "yaw", "manifest")

    _artifact_from_manifest(manifest, "rgb", rgb_path)
    _artifact_from_manifest(manifest, "anchors", anchors_path)
    _artifact_from_manifest(manifest, "raw", raw_path)

    try:
        with Image.open(rgb_path) as encoded_rgb:
            rgb = ImageOps.exif_transpose(encoded_rgb).convert("RGB")
            rgb.load()
    except (OSError, ValueError) as error:
        raise ValueError(f"invalid RGB artifact: {error}") from error
    rgb_width, rgb_height = rgb.size
    if _require_int(manifest, "rgbWidth", "manifest", positive=True) != rgb_width \
            or _require_int(manifest, "rgbHeight", "manifest", positive=True) != rgb_height:
        raise ValueError("manifest top-level RGB dimensions do not match decoded RGB")
    projection_contract = _projection_contract(manifest, rgb_width, rgb_height)

    with np.load(raw_path, allow_pickle=False) as raw:
        required_arrays = {"pred_disp", "relative_depth", "valid_mask", "metadata_json"}
        missing = required_arrays - set(raw.files)
        if missing:
            raise ValueError(f"raw NPZ missing arrays: {', '.join(sorted(missing))}")
        pred_disp = np.asarray(raw["pred_disp"], dtype=np.float32).copy()
        relative_depth = np.asarray(raw["relative_depth"], dtype=np.float32).copy()
        valid_mask = np.asarray(raw["valid_mask"], dtype=bool).copy()
        try:
            metadata = json.loads(str(raw["metadata_json"]))
        except (TypeError, json.JSONDecodeError) as error:
            raise ValueError(f"raw metadata_json is invalid: {error}") from error
    if not isinstance(metadata, dict):
        raise ValueError("raw metadata_json must contain an object")
    if pred_disp.ndim != 2 or relative_depth.shape != pred_disp.shape \
            or valid_mask.shape != pred_disp.shape:
        raise ValueError("raw depth arrays must be matching 2-D arrays")
    height, width = pred_disp.shape
    epsilon = _require_number(metadata, "epsilon", "raw.metadata", positive=True)
    expected_valid = np.isfinite(pred_disp) & (pred_disp > epsilon)
    if not np.array_equal(valid_mask, expected_valid):
        raise ValueError("raw valid_mask is inconsistent with pred_disp and epsilon")
    if not np.allclose(
        relative_depth[expected_valid], 1.0 / pred_disp[expected_valid], rtol=1e-5, atol=1e-6
    ) or np.any(relative_depth[~expected_valid] != 0):
        raise ValueError("raw relative_depth is inconsistent with pred_disp")
    raw_contract = _raw_contract(
        metadata, height, width, rgb_width, rgb_height
    )
    raw_frame_id = _require_string(metadata, "frame_id", "raw.metadata")
    if raw_frame_id != identity["frameId"]:
        raise ValueError(f"anchor/raw frame mismatch: {identity['frameId']!r} != {raw_frame_id!r}")
    for raw_key, identity_key in (
        ("session_id", "sessionId"),
        ("capture_id", "captureId"),
        ("location_id", "locationId"),
    ):
        if _require_string(metadata, raw_key, "raw.metadata") != identity[identity_key]:
            raise ValueError(f"manifest/raw identity mismatch: {identity_key}")
    for manifest_key, raw_key in (("rawModel", "model"), ("rawWidth", "width"), ("rawHeight", "height")):
        manifest_value = manifest.get(manifest_key)
        if manifest_value is not None and manifest_value != metadata[raw_key]:
            raise ValueError(f"manifest {manifest_key} does not match raw metadata")

    anchor_data = _load_json_object(anchors_path, "anchors")
    anchors = anchor_data.get("anchors")
    failures = anchor_data.get("failures")
    if not isinstance(anchors, list) or not isinstance(failures, list):
        raise ValueError("anchors artifact requires anchors and failures arrays")
    anchor_metadata = _require_mapping(anchor_data.get("metadata"), "anchors.metadata")
    if anchor_metadata.get("schemaVersion") != 1:
        raise ValueError("anchors.metadata.schemaVersion must be 1")
    if anchor_metadata.get("tileState") != "ready":
        raise ValueError("anchors were captured before Cesium tiles reported ready")
    capture_provenance = _anchor_capture_provenance(
        anchor_metadata, projection_contract
    )
    anchor_identity = _require_mapping(anchor_metadata.get("identity"), "anchors.metadata.identity")
    for key, expected in identity.items():
        if _require_string(anchor_identity, key, "anchors.metadata.identity") != expected:
            raise ValueError(f"manifest/anchor identity mismatch: {key}")
    if anchor_metadata.get("transform") != manifest.get("transform"):
        raise ValueError("manifest/anchor transform mismatch")
    _validate_transform(anchor_metadata.get("transform"), "anchors.metadata.transform")

    image_metadata = _require_mapping(anchor_metadata.get("image"), "anchors.metadata.image")
    source_w = _require_int(image_metadata, "width", "anchors.metadata.image", positive=True)
    source_h = _require_int(image_metadata, "height", "anchors.metadata.image", positive=True)
    if source_w != rgb_width or source_h != rgb_height:
        raise ValueError("anchor image dimensions do not match decoded RGB")
    if image_metadata.get("pixelCoordinateConvention") != "integer-pixel-centres":
        raise ValueError("anchor pixel coordinate convention is incompatible")
    erp_metadata = _require_mapping(anchor_metadata.get("erp"), "anchors.metadata.erp")
    vertical_fov = _require_number(
        erp_metadata, "verticalFovDeg", "anchors.metadata.erp", positive=True
    )
    if vertical_fov != projection_contract["verticalFovDeg"]:
        raise ValueError("manifest/anchor ERP vertical FOV mismatch")
    if erp_metadata.get("sensorFrame") != "NWU(+x forward,+y left,+z up)" \
            or erp_metadata.get("componentFrame") != "(+x body-left,+y up,+z back)":
        raise ValueError("anchor ERP frame convention is incompatible")
    sampling_metadata = _require_mapping(
        anchor_metadata.get("sampling"), "anchors.metadata.sampling"
    )
    sampling_contract = {
        "gridCols": _require_int(sampling_metadata, "gridCols", "anchors.metadata.sampling", positive=True),
        "gridRows": _require_int(sampling_metadata, "gridRows", "anchors.metadata.sampling", positive=True),
        "maxRangeM": _require_number(sampling_metadata, "maxRangeM", "anchors.metadata.sampling", positive=True),
        "excludeTopDeg": _require_number(sampling_metadata, "excludeTopDeg", "anchors.metadata.sampling"),
        "excludeBottomDeg": _require_number(sampling_metadata, "excludeBottomDeg", "anchors.metadata.sampling"),
    }
    anchor_contract = {
        "image": dict(image_metadata),
        "erp": dict(erp_metadata),
        "sampling": sampling_contract,
        "projection": projection_contract,
        "capture_source": {
            key: value for key, value in capture_provenance.items()
            if key != "panoramaCaptureRevision"
        },
    }
    n_anchor_candidates = _require_int(anchor_metadata, "totalCells", "anchors.metadata", positive=True)
    expected_cells = sampling_contract["gridCols"] * sampling_contract["gridRows"]
    if n_anchor_candidates != expected_cells or len(anchors) + len(failures) != expected_cells:
        raise ValueError("anchor grid cell accounting is inconsistent")
    if _require_int(anchor_metadata, "validAnchors", "anchors.metadata") != len(anchors) \
            or _require_int(anchor_metadata, "failureCount", "anchors.metadata") != len(failures):
        raise ValueError("anchor success/failure counts are inconsistent")
    if _require_int(manifest, "validAnchors", "manifest") != len(anchors) \
            or _require_int(manifest, "failedAnchors", "manifest") != len(failures):
        raise ValueError("manifest/anchor success and failure counts disagree")

    x_vals, inv_depth_vals, location_ids, sampled_anchors = [], [], [], []
    seen_cells = set()
    for index, anchor in enumerate(anchors):
        if not isinstance(anchor, dict):
            raise ValueError(f"anchor {index} must be an object")
        col = _require_int(anchor, "col", f"anchor[{index}]")
        row = _require_int(anchor, "row", f"anchor[{index}]")
        cell = (col, row)
        if cell in seen_cells:
            raise ValueError(f"duplicate anchor grid cell: {cell}")
        seen_cells.add(cell)
        source_u = _require_number(anchor, "u", f"anchor[{index}]")
        source_v = _require_number(anchor, "v", f"anchor[{index}]")
        distance_m = _require_number(anchor, "distance", f"anchor[{index}]", positive=True)
        if distance_m > sampling_contract["maxRangeM"]:
            raise ValueError(f"anchor {index} exceeds the declared maximum range")
        if not (-0.5 <= source_u <= source_w - 0.5 and -0.5 <= source_v <= source_h - 0.5):
            raise ValueError(f"anchor {index} is outside the declared ERP image")
        raw_u = map_pixel_center(source_u, source_w, width)
        raw_v = map_pixel_center(source_v, source_h, height)
        disparity = sample_wrapped_bilinear(pred_disp, valid_mask, raw_u, raw_v)
        if disparity is None or not np.isfinite(disparity):
            continue
        x_vals.append(disparity)
        inv_depth_vals.append(1.0 / distance_m)
        location_ids.append(identity["locationId"])
        sampled_anchors.append({
            "u": source_u,
            "v": source_v,
            "raw_u": raw_u,
            "raw_v": raw_v,
            "distance_m": distance_m,
            "pred_disp": disparity,
        })

    x_array = np.asarray(x_vals, dtype=np.float64)
    inverse_depth_array = np.asarray(inv_depth_vals, dtype=np.float64)
    sampled_depth = np.full(inverse_depth_array.shape, np.nan, dtype=np.float64)
    positive_inverse = np.isfinite(inverse_depth_array) & (inverse_depth_array > 0)
    sampled_depth[positive_inverse] = 1.0 / inverse_depth_array[positive_inverse]
    n_range_anchors = int(np.count_nonzero(
        positive_inverse
        & (sampled_depth >= METRIC_MIN_DEPTH_M)
        & (sampled_depth <= METRIC_MAX_DEPTH_M)
    ))
    return {
        "pred_disp": pred_disp,
        "valid_mask": valid_mask,
        "H": height,
        "W": width,
        "x": x_array,
        "z": inverse_depth_array,
        "location_ids": np.asarray(location_ids, dtype=str),
        "sampled_anchors": sampled_anchors,
        "metadata": metadata,
        "anchor_metadata": anchor_metadata,
        "manifest": manifest,
        "raw_contract": raw_contract,
        "anchor_contract": anchor_contract,
        "n_anchors": n_anchor_candidates,
        "n_hit_anchors": len(anchors),
        "n_valid_anchors": len(x_array),
        "n_range_anchors": n_range_anchors,
        "valid_anchor_fraction": len(x_array) / n_anchor_candidates,
        "sample_id": identity["captureId"],
        "identity": identity,
        "pose_position": manifest_position,
        "pose_quaternion": manifest_quaternion,
    }


def _same_pose(first, second):
    position_distance = float(np.linalg.norm(first["pose_position"] - second["pose_position"]))
    quaternion_dot = float(abs(np.dot(first["pose_quaternion"], second["pose_quaternion"])))
    angle_deg = math.degrees(2.0 * math.acos(min(1.0, max(0.0, quaternion_dot))))
    return position_distance < POSE_POSITION_EPS_M and angle_deg < POSE_ANGLE_EPS_DEG


def combine_datasets(datasets):
    """Combine verified captures while rejecting duplicate identities or poses."""
    if not datasets:
        raise ValueError("at least one verified capture bundle is required")
    reference = datasets[0]
    seen_capture_ids = set()
    seen_frame_ids = set()
    location_captures = {}
    accepted = []
    for dataset in datasets:
        identity = dataset.get("identity")
        if not isinstance(identity, dict):
            raise ValueError("capture dataset is missing verified identity")
        capture_id = identity.get("captureId")
        frame_identity = (identity.get("sessionId"), identity.get("frameId"))
        if capture_id in seen_capture_ids:
            raise ValueError(f"duplicate captureId: {capture_id}")
        if frame_identity in seen_frame_ids:
            raise ValueError(f"duplicate session/frame identity: {frame_identity}")
        if dataset.get("raw_contract") != reference.get("raw_contract"):
            raise ValueError("capture raw inference contract mismatch")
        if dataset.get("anchor_contract") != reference.get("anchor_contract"):
            raise ValueError("capture ERP/projection/sampling contract mismatch")
        for prior in accepted:
            same_location = identity.get("locationId") == prior["identity"].get("locationId")
            if same_location and _same_pose(dataset, prior):
                raise ValueError(
                    f"duplicate capture pose: {capture_id} and {prior['identity']['captureId']}"
                )
        seen_capture_ids.add(capture_id)
        seen_frame_ids.add(frame_identity)
        accepted.append(dataset)
        location_captures.setdefault(identity.get("locationId"), []).append(capture_id)

    undersampled = {
        location: captures for location, captures in location_captures.items()
        if not location or len(captures) < MIN_POSES_PER_LOCATION
    }
    if undersampled:
        details = ", ".join(
            f"{location or '<missing>'}={len(captures)}" for location, captures in sorted(undersampled.items())
        )
        raise ValueError(
            f"each location requires >= {MIN_POSES_PER_LOCATION} distinct poses: {details}"
        )

    combined = dict(reference)
    combined["x"] = np.concatenate([dataset["x"] for dataset in datasets])
    combined["z"] = np.concatenate([dataset["z"] for dataset in datasets])
    combined["location_ids"] = np.concatenate([
        np.asarray(dataset["location_ids"], dtype=str) for dataset in datasets
    ])
    combined["n_anchors"] = sum(dataset["n_anchors"] for dataset in datasets)
    combined["n_hit_anchors"] = sum(dataset.get("n_hit_anchors", 0) for dataset in datasets)
    combined["n_valid_anchors"] = len(combined["x"])
    combined["valid_anchor_fraction"] = (
        combined["n_valid_anchors"] / combined["n_anchors"]
        if combined["n_anchors"] else 0.0
    )
    combined["capture_valid_anchor_fractions"] = {
        dataset["identity"]["captureId"]: float(dataset["valid_anchor_fraction"])
        for dataset in datasets
    }
    combined["capture_range_anchor_counts"] = {
        dataset["identity"]["captureId"]: int(dataset["n_range_anchors"])
        for dataset in datasets
    }
    combined["n_samples"] = len(datasets)
    combined["sample_ids"] = [dataset["identity"]["captureId"] for dataset in datasets]
    combined["capture_records"] = [
        {
            "sessionId": dataset["identity"]["sessionId"],
            "captureId": dataset["identity"]["captureId"],
            "locationId": dataset["identity"]["locationId"],
            "frameId": dataset["identity"]["frameId"],
        }
        for dataset in datasets
    ]
    combined["location_capture_counts"] = {
        location: len(captures) for location, captures in sorted(location_captures.items())
    }
    combined["preview_sample_id"] = reference["identity"]["captureId"]
    combined["bundle_validation_complete"] = True
    return combined


def _validation_folds(location_ids, n):
    """Build exactly one held-out fold for every collected location."""
    location_ids = np.asarray(location_ids, dtype=str).reshape(-1)
    if location_ids.size != n or np.any(location_ids == ""):
        raise ValueError("every in-range anchor requires a verified locationId")
    unique_locations = sorted(set(location_ids.tolist()))
    if len(unique_locations) < MIN_LOCATIONS:
        raise ValueError(f"need >= {MIN_LOCATIONS} locations for leave-one-location-out validation")
    folds = []
    for location in unique_locations:
        val_idx = np.flatnonzero(location_ids == location)
        train_idx = np.flatnonzero(location_ids != location)
        if val_idx.size < 2:
            raise ValueError(f"held-out location {location} has fewer than 2 in-range anchors")
        if train_idx.size < 4:
            raise ValueError(f"held-out location {location} leaves fewer than 4 training anchors")
        folds.append((train_idx, val_idx, location))
    return "leave-one-location-out", folds


def _fit_scale_shift_model(x, inverse_depth):
    """Fit the single authoritative Huber scale-and-shift model."""
    a, b, _ = fit_scale_shift(x, inverse_depth)
    return a, b


def _depth_metrics_from_inverse(
    true_inverse,
    predicted_inverse,
    min_true_depth_m=METRIC_MIN_DEPTH_M,
    max_true_depth_m=METRIC_MAX_DEPTH_M,
):
    true_depth, true_valid = inverse_depth_to_metric_depth(true_inverse)
    predicted_depth, predicted_valid = inverse_depth_to_metric_depth(predicted_inverse)
    valid = true_valid & predicted_valid
    if min_true_depth_m is not None:
        valid &= true_depth >= min_true_depth_m
    if max_true_depth_m is not None:
        valid &= true_depth <= max_true_depth_m
    return compute_metrics(true_depth, predicted_depth, valid)


def _cross_validate(x, inverse_depth, folds):
    scale_predictions = np.full(len(x), np.nan, dtype=np.float64)
    shift_predictions = np.full(len(x), np.nan, dtype=np.float64)
    fold_reports = []
    for train_idx, val_idx, label in folds:
        a_scale, _ = fit_scale_only(x[train_idx], inverse_depth[train_idx])
        a_shift, b_shift = _fit_scale_shift_model(x[train_idx], inverse_depth[train_idx])
        scale_predictions[val_idx] = a_scale * x[val_idx]
        shift_predictions[val_idx] = a_shift * x[val_idx] + b_shift
        fold_reports.append({
            "held_out": label,
            "n_train": int(len(train_idx)),
            "n_val": int(len(val_idx)),
            "scale_only": _depth_metrics_from_inverse(
                inverse_depth[val_idx], scale_predictions[val_idx]
            ),
            "scale_only_near_10m": _depth_metrics_from_inverse(
                inverse_depth[val_idx], scale_predictions[val_idx],
                max_true_depth_m=NEAR_MAX_DEPTH_M,
            ),
            "scale_shift": _depth_metrics_from_inverse(
                inverse_depth[val_idx], shift_predictions[val_idx]
            ),
            "scale_shift_near_10m": _depth_metrics_from_inverse(
                inverse_depth[val_idx], shift_predictions[val_idx],
                max_true_depth_m=NEAR_MAX_DEPTH_M,
            ),
        })

    scale_valid = np.isfinite(scale_predictions)
    shift_valid = np.isfinite(shift_predictions)
    return {
        "scale_only": _depth_metrics_from_inverse(
            inverse_depth[scale_valid], scale_predictions[scale_valid]
        ),
        "scale_shift": _depth_metrics_from_inverse(
            inverse_depth[shift_valid], shift_predictions[shift_valid]
        ),
        "scale_only_near_10m": _depth_metrics_from_inverse(
            inverse_depth[scale_valid], scale_predictions[scale_valid],
            max_true_depth_m=NEAR_MAX_DEPTH_M,
        ),
        "scale_shift_near_10m": _depth_metrics_from_inverse(
            inverse_depth[shift_valid], shift_predictions[shift_valid],
            max_true_depth_m=NEAR_MAX_DEPTH_M,
        ),
        "folds": fold_reports,
    }


def _metrics_pass_gate(metrics, near_metrics):
    return bool(
        metrics.get("median_abs_rel") is not None
        and metrics.get("p90_abs_rel") is not None
        and metrics["median_abs_rel"] <= 0.15
        and metrics["p90_abs_rel"] <= 0.30
        and near_metrics.get("p90_error_m") is not None
        and near_metrics["p90_error_m"] <= 1.0
    )


def _write_json_atomic(path, payload):
    path = Path(path)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    with open(temporary_path, "w", encoding="utf-8") as output:
        json.dump(payload, output, indent=2, allow_nan=False)
        output.write("\n")
    os.replace(temporary_path, path)


def _write_report(output_dir, report):
    _write_json_atomic(Path(output_dir) / "fit_report.json", report)


def _clear_calibration_outputs(output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale_name in ("depth_calibration.json", "depth_calibration_candidate.json"):
        (output_dir / stale_name).unlink(missing_ok=True)


def run_fitting(data, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    # Never leave a previously accepted calibration in a reused output
    # directory after a later run fails or is interrupted before validation.
    _clear_calibration_outputs(output_dir)

    x = np.asarray(data["x"], dtype=np.float64)
    inverse_depth = np.asarray(data["z"], dtype=np.float64)
    if x.shape != inverse_depth.shape or x.ndim != 1:
        report = {"success": False, "error": "x and z must be matching one-dimensional arrays"}
        _write_report(output_dir, report)
        return report
    source_n = len(x)
    location_ids = np.asarray(data.get("location_ids", []), dtype=str).reshape(-1)
    if location_ids.size != source_n:
        report = {
            "success": False,
            "error": "verified location IDs must match every calibration anchor",
            "n_total_anchors": int(data.get("n_anchors", source_n)),
            "n_valid_anchors": source_n,
        }
        _write_report(output_dir, report)
        return report

    finite = np.isfinite(x) & np.isfinite(inverse_depth) & (inverse_depth > 0)
    true_depth = np.full(inverse_depth.shape, np.nan, dtype=np.float64)
    true_depth[finite] = 1.0 / inverse_depth[finite]
    in_metric_range = finite & (true_depth >= METRIC_MIN_DEPTH_M) \
        & (true_depth <= METRIC_MAX_DEPTH_M)
    x = x[in_metric_range]
    inverse_depth = inverse_depth[in_metric_range]
    location_ids = location_ids[in_metric_range]
    n = len(x)
    if n < 6:
        report = {
            "success": False,
            "error": f"insufficient anchors in [{METRIC_MIN_DEPTH_M}, {METRIC_MAX_DEPTH_M}]m: {n} (need >=6)",
            "n_total_anchors": int(data.get("n_anchors", n)),
            "n_valid_anchors": source_n,
            "n_range_anchors": n,
        }
        _write_report(output_dir, report)
        return report

    try:
        validation_strategy, folds = _validation_folds(location_ids, n)
        validation = _cross_validate(x, inverse_depth, folds)
        final_a_scale, _ = fit_scale_only(x, inverse_depth)
        final_a_shift, final_b_shift = _fit_scale_shift_model(x, inverse_depth)
    except ValueError as error:
        report = {
            "success": False,
            "error": str(error),
            "n_total_anchors": int(data.get("n_anchors", n)),
            "n_valid_anchors": source_n,
            "n_range_anchors": n,
        }
        _write_report(output_dir, report)
        return report

    scale_metrics = validation["scale_only"]
    shift_metrics = validation["scale_shift"]

    # DA360's public prediction is scale-invariant disparity after the model's
    # learned internal shift correction.  The affine fit remains useful only
    # for diagnosing an input/data/model-contract mismatch; deployment must not
    # reintroduce a disparity shift.
    selected_model = "scale_only"
    selected_a = final_a_scale
    selected_b = 0.0

    report = {
        "success": True,
        "relation": "inverse_depth_1_per_m = a * pred_disp",
        "deployment_contract": {
            "model": "scale_only",
            "relation": "inverse_depth_1_per_m = a * pred_disp",
            "b": 0.0,
            "scale_shift_role": "diagnostic_only",
        },
        "output_units": "metres",
        "pixel_sampling": "integer-pixel-centres; resolution-scaled; horizontal-wrap bilinear",
        "robust_loss": "huber-once",
        "huber_scale_rule": (
            "1.345 * (1.4826 * training MAD) "
            "(minimum 1e-6 inverse metres)"
        ),
        "metric_gate_range_m": [METRIC_MIN_DEPTH_M, METRIC_MAX_DEPTH_M],
        "n_total_anchors": int(data.get("n_anchors", n)),
        "n_valid_anchors": source_n,
        "n_range_anchors": n,
        "valid_anchor_fraction": float(data.get(
            "valid_anchor_fraction", source_n / max(1, data.get("n_anchors", source_n))
        )),
        "n_samples": int(data.get("n_samples", 1)),
        "sample_ids": list(data.get("sample_ids", [data.get("sample_id", "unknown")])),
        "model": data.get("metadata", {}).get("model", "unknown"),
        "seed": SEED,
        "validation": {
            "strategy": validation_strategy,
            "folds": validation["folds"],
        },
        "scale_only": {
            "a": float(final_a_scale),
            "b": 0.0,
            **scale_metrics,
            "near_10m": validation["scale_only_near_10m"],
        },
        "scale_shift": {
            "diagnostic_only": True,
            "a": float(final_a_shift),
            "b": float(final_b_shift),
            **shift_metrics,
            "near_10m": validation["scale_shift_near_10m"],
        },
        "selected_model": selected_model,
        "calibration": {
            "a": float(selected_a),
            "b": float(selected_b),
            "relation": "inverse_depth_1_per_m = a * pred_disp",
        },
    }

    # Convert inverse-depth predictions to actual metric depths.  A non-positive
    # fitted inverse depth is invalid; clipping it would fabricate a far range.
    pred_disp = np.asarray(data["pred_disp"], dtype=np.float32)
    raw_valid = np.asarray(data["valid_mask"], dtype=bool)
    inverse_scale = final_a_scale * pred_disp
    inverse_shift = final_a_shift * pred_disp + final_b_shift
    depth_scale, valid_scale = inverse_depth_to_metric_depth(inverse_scale, raw_valid)
    depth_shift, valid_shift = inverse_depth_to_metric_depth(inverse_shift, raw_valid)
    selected_valid = valid_scale

    np.save(os.path.join(output_dir, "metric_depth_scale.npy"), depth_scale)
    np.save(os.path.join(output_dir, "metric_depth_shift.npy"), depth_shift)
    np.save(os.path.join(output_dir, "metric_valid_mask_scale.npy"), valid_scale.astype(np.uint8))
    np.save(os.path.join(output_dir, "metric_valid_mask_shift.npy"), valid_shift.astype(np.uint8))
    np.save(os.path.join(output_dir, "metric_valid_mask.npy"), selected_valid.astype(np.uint8))

    Image.fromarray(depth_to_color(depth_scale)).save(
        os.path.join(output_dir, "metric_preview_scale.png")
    )
    Image.fromarray(depth_to_color(depth_shift)).save(
        os.path.join(output_dir, "metric_preview_shift.png")
    )

    anchor_viz = np.zeros((data["H"], data["W"], 3), dtype=np.uint8)
    for anchor in data.get("sampled_anchors", []):
        u = int(round(anchor["raw_u"])) % data["W"]
        v = min(data["H"] - 1, max(0, int(round(anchor["raw_v"]))))
        anchor_viz[v, u] = [0, 255, 0]
    Image.fromarray(anchor_viz).save(os.path.join(output_dir, "anchor_overlay.png"))

    selected_metrics = report[selected_model]
    selected_near_metrics = selected_metrics["near_10m"]
    nonempty_locations = sorted(set(location_ids[location_ids != ""].tolist()))
    inference_metadata = data.get("metadata", {})
    capture_records = data.get("capture_records", [])
    capture_fractions = data.get("capture_valid_anchor_fractions", {})
    capture_range_counts = data.get("capture_range_anchor_counts", {})
    location_capture_counts = data.get("location_capture_counts", {})
    bundle_validation_complete = bool(data.get("bundle_validation_complete")) \
        and isinstance(capture_records, list) \
        and len(capture_records) == int(data.get("n_samples", -1))
    report["n_samples"] = len(capture_records) if bundle_validation_complete else 0
    report["sample_ids"] = [record["captureId"] for record in capture_records] \
        if bundle_validation_complete else []
    raw_contract = data.get("raw_contract", {})
    anchor_contract = data.get("anchor_contract", {})
    inference_contract_complete = bundle_validation_complete \
        and isinstance(raw_contract, dict) and isinstance(anchor_contract, dict) \
        and bool(raw_contract) and bool(anchor_contract)
    collected_locations = sorted(location_capture_counts) if isinstance(location_capture_counts, dict) else []
    held_out_locations = sorted(fold["held_out"] for fold in validation["folds"])
    fold_gate_results = {}
    for fold in validation["folds"]:
        fold_metrics = fold[selected_model]
        fold_near = fold[f"{selected_model}_near_10m"]
        fold_gate_results[fold["held_out"]] = {
            "passed": _metrics_pass_gate(fold_metrics, fold_near),
            "metrics": fold_metrics,
            "near_10m": fold_near,
        }
    all_capture_coverage_passed = bool(capture_fractions) and all(
        fraction >= 0.70 for fraction in capture_fractions.values()
    )
    all_captures_contribute_to_fit = bool(capture_range_counts) and all(
        count >= 2 for count in capture_range_counts.values()
    )
    all_locations_sampled = bool(location_capture_counts) and all(
        count >= MIN_POSES_PER_LOCATION for count in location_capture_counts.values()
    )
    collection_requirements = {
        "bundle_validation_complete": bundle_validation_complete,
        "locations": len(collected_locations),
        "locations_min": MIN_LOCATIONS,
        "captures": report["n_samples"],
        "captures_min": MIN_CAPTURES,
        "poses_per_location_min": MIN_POSES_PER_LOCATION,
        "location_capture_counts": location_capture_counts,
        "all_locations_sampled": all_locations_sampled,
        "held_out_locations": held_out_locations,
        "all_locations_held_out": held_out_locations == collected_locations == nonempty_locations,
        "leave_one_location_out": validation_strategy == "leave-one-location-out"
            and len(validation["folds"]) == len(collected_locations),
        "inference_contract_complete": bool(inference_contract_complete),
        "capture_valid_anchor_fractions": capture_fractions,
        "all_capture_coverage_passed": all_capture_coverage_passed,
        "capture_range_anchor_counts": capture_range_counts,
        "all_captures_contribute_to_fit": all_captures_contribute_to_fit,
    }
    passed_gate = (
        _metrics_pass_gate(selected_metrics, selected_near_metrics)
        and bool(fold_gate_results)
        and all(result["passed"] for result in fold_gate_results.values())
        and report["valid_anchor_fraction"] >= 0.70
        and collection_requirements["bundle_validation_complete"]
        and collection_requirements["locations"] >= collection_requirements["locations_min"]
        and collection_requirements["captures"] >= collection_requirements["captures_min"]
        and collection_requirements["all_locations_sampled"]
        and collection_requirements["all_locations_held_out"]
        and collection_requirements["leave_one_location_out"]
        and collection_requirements["inference_contract_complete"]
        and collection_requirements["all_capture_coverage_passed"]
        and collection_requirements["all_captures_contribute_to_fit"]
    )
    report["collection"] = collection_requirements
    report["acceptance"] = {
        "passed": bool(passed_gate),
        "range_m": [METRIC_MIN_DEPTH_M, METRIC_MAX_DEPTH_M],
        "median_abs_rel_max": 0.15,
        "p90_abs_rel_max": 0.30,
        "near_10m_p90_error_m_max": 1.0,
        "valid_anchor_fraction_min": 0.70,
        "per_location": fold_gate_results,
    }
    report["conclusion"] = (
        "metric calibration passed held-out gates"
        if passed_gate else "metric calibration did not pass held-out gates"
    )
    calibration_candidate = {
        "schema_version": 1,
        "accepted": bool(passed_gate),
        "a": float(selected_a),
        "b": float(selected_b),
        "depth_min_m": METRIC_MIN_DEPTH_M,
        "depth_max_m": METRIC_MAX_DEPTH_M,
        "model": inference_metadata.get("model"),
        "calibration_id": (
            f"da360-metric-v1-{inference_metadata.get('model')}-"
            f"{raw_contract.get('request_width')}x{raw_contract.get('request_height')}"
            if inference_contract_complete else None
        ),
        "width": inference_metadata.get("width"),
        "height": inference_metadata.get("height"),
        "input_scale": inference_metadata.get("input_scale"),
        "resample": inference_metadata.get("resample"),
        "requestWidth": raw_contract.get("request_width") if inference_contract_complete else None,
        "requestHeight": raw_contract.get("request_height") if inference_contract_complete else None,
        "input": raw_contract if inference_contract_complete else None,
        "projection": anchor_contract.get("projection") if inference_contract_complete else None,
        "selected_model": selected_model,
        "relation": "inverse_depth_1_per_m = a * pred_disp",
        "fit_report": "fit_report.json",
        "acceptance": report["acceptance"],
        "collection": report["collection"],
    }
    _write_json_atomic(Path(output_dir) / "depth_calibration_candidate.json", calibration_candidate)
    accepted_calibration = None
    if passed_gate:
        accepted_calibration = "depth_calibration.json"
        _write_json_atomic(Path(output_dir) / accepted_calibration, calibration_candidate)

    report["artifacts"] = {
        "scale_depth": "metric_depth_scale.npy",
        "scale_shift_depth": "metric_depth_shift.npy",
        "selected_valid_mask": "metric_valid_mask.npy",
        "anchor_overlay": "anchor_overlay.png",
        "preview_sample_id": data.get("preview_sample_id", data.get("sample_id", "unknown")),
        "calibration_candidate": "depth_calibration_candidate.json",
        "accepted_calibration": accepted_calibration,
    }

    # The report is written only after every conclusion and artifact field is
    # complete; atomic replace prevents consumers from reading partial JSON.
    _write_report(output_dir, report)
    print(json.dumps(report, indent=2, allow_nan=False))
    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Offline inverse-depth metric fitting")
    parser.add_argument(
        "--raw", required=True, action="append",
        help="DA360 raw .npz from /depth/raw; repeat with --anchors for multiple captures",
    )
    parser.add_argument(
        "--anchors", required=True, action="append",
        help="Cesium anchors .json; repeat in the same order as --raw",
    )
    parser.add_argument(
        "--manifest", required=True, action="append",
        help="Capture manifest .json; repeat in the same order as --raw",
    )
    parser.add_argument(
        "--rgb", required=True, action="append",
        help="Frozen capture RGB JPEG; repeat in the same order as --raw",
    )
    parser.add_argument("--output", default="experiment_data/metric_fit_sample", help="Output directory")
    args = parser.parse_args()

    counts = {len(args.raw), len(args.anchors), len(args.manifest), len(args.rgb)}
    if len(counts) != 1:
        parser.error("--raw, --anchors, --manifest and --rgb must have equal counts")
    _clear_calibration_outputs(args.output)
    try:
        captures = [
            load_data(raw, anchors, manifest, rgb)
            for raw, anchors, manifest, rgb in zip(
                args.raw, args.anchors, args.manifest, args.rgb
            )
        ]
        data = combine_datasets(captures)
    except ValueError as error:
        failure = {"success": False, "error": str(error), "stage": "bundle-validation"}
        _write_report(args.output, failure)
        print(f"Bundle validation failed: {error}", file=sys.stderr)
        sys.exit(1)
    report = run_fitting(data, args.output)

    if not report["success"]:
        print(f"Fitting failed: {report['error']}", file=sys.stderr)
        sys.exit(1)

    print(f"\nConclusion: {report['conclusion']}")


if __name__ == "__main__":
    main()
