"""Pure tests for fail-closed DA360 inverse-depth metric fitting."""

import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image, ImageOps

from scripts.fit_da360_metric import (
    _depth_metrics_from_inverse,
    combine_datasets,
    compute_metrics,
    inverse_depth_to_metric_depth,
    load_data,
    map_pixel_center,
    run_fitting,
    sample_wrapped_bilinear,
)


TRUE_A = 2.25
TRUE_B = 0.12
CHECKPOINT_SHA = "a" * 64
UNIT_PRED_DISP = "raw disparity (inverse depth), NOT per-frame normalized"
PANORAMA_FACES = ("front", "right", "back", "left", "up", "down")


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _rewrite_anchor_metadata(paths, mutate):
    """Mutate anchor metadata while preserving the bundle's SHA contract."""
    anchors_path = paths[1]
    manifest_path = paths[2]
    anchor_payload = json.loads(anchors_path.read_text(encoding="utf-8"))
    mutate(anchor_payload["metadata"])
    anchors_path.write_text(json.dumps(anchor_payload, indent=2), encoding="utf-8")
    anchors_sha = _sha256(anchors_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["anchorsSha256"] = anchors_sha
    manifest["files"]["anchors"]["sha256"] = anchors_sha
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def _state(position):
    return {
        "position": dict(zip("xyz", position)),
        "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def _write_bundle(
    root,
    *,
    capture_id="capture-001",
    session_id="session-001",
    location_id="A",
    frame_id="1",
    position=(0.0, 0.0, 0.0),
    color_seed=1,
    disparity_offset=0.0,
    distance_scale=1.0,
    source_size=(8, 4),
    raw_size=(4, 2),
    anchors_override=None,
):
    """Write one production-shaped immutable capture bundle and load paths."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    rgb_path = root / f"{capture_id}-rgb.jpg"
    raw_path = root / f"{capture_id}-raw.npz"
    anchors_path = root / f"{capture_id}-anchors.json"
    manifest_path = root / f"{capture_id}-manifest.json"

    source_width, source_height = source_size
    raw_width, raw_height = raw_size
    yy, xx = np.mgrid[:source_height, :source_width]
    rgb_array = np.stack([
        (xx * 19 + color_seed * 31) % 256,
        (yy * 47 + color_seed * 53) % 256,
        ((xx + yy) * 13 + color_seed * 71) % 256,
    ], axis=-1).astype(np.uint8)
    Image.fromarray(rgb_array).save(rgb_path, format="JPEG", quality=88)
    with Image.open(rgb_path) as encoded:
        decoded = ImageOps.exif_transpose(encoded).convert("RGB")
        decoded.load()
    decoded_sha = hashlib.sha256(np.asarray(decoded, dtype=np.uint8).tobytes()).hexdigest()

    base = np.linspace(0.08, 0.32, raw_width * raw_height, dtype=np.float32)
    pred_disp = base.reshape(raw_height, raw_width) + np.float32(disparity_offset)
    epsilon = 1e-6
    valid_mask = np.isfinite(pred_disp) & (pred_disp > epsilon)
    relative_depth = np.where(valid_mask, 1.0 / pred_disp, 0.0).astype(np.float32)
    raw_metadata = {
        "api_version": 2,
        "model": "synthetic-da360",
        "width": raw_width,
        "height": raw_height,
        "input_scale": 0.46,
        "resample": "bicubic",
        "checkpoint_sha256": CHECKPOINT_SHA,
        "request_width": source_width,
        "request_height": source_height,
        "decoded_rgb_sha256": decoded_sha,
        "frame_id": str(frame_id),
        "session_id": session_id,
        "capture_id": capture_id,
        "location_id": location_id,
        "depth_mode": "da360-relative",
        "epsilon": epsilon,
        "unit_pred_disp": UNIT_PRED_DISP,
    }
    np.savez(
        raw_path,
        pred_disp=pred_disp,
        relative_depth=relative_depth,
        valid_mask=valid_mask.astype(np.uint8),
        metadata_json=np.asarray(json.dumps(raw_metadata, sort_keys=True)),
    )

    transform = {
        "position": dict(zip("xyz", map(float, position))),
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    if anchors_override is None:
        anchors = []
        for row, source_v in enumerate((0.5, 2.5)):
            for col, source_u in enumerate((1.5, 5.5)):
                raw_u = map_pixel_center(source_u, source_width, raw_width)
                raw_v = map_pixel_center(source_v, source_height, raw_height)
                disparity = sample_wrapped_bilinear(pred_disp, valid_mask, raw_u, raw_v)
                inverse_depth = TRUE_A * disparity + TRUE_B
                anchors.append({
                    "col": col,
                    "row": row,
                    "u": source_u,
                    "v": source_v,
                    "distance": float(distance_scale / inverse_depth),
                })
    else:
        anchors = anchors_override
    anchor_metadata = {
        "schemaVersion": 1,
        "identity": {
            "sessionId": session_id,
            "captureId": capture_id,
            "locationId": location_id,
            "frameId": str(frame_id),
        },
        "image": {
            "width": source_width,
            "height": source_height,
            "pixelCoordinateConvention": "integer-pixel-centres",
        },
        "erp": {
            "verticalFovDeg": 180.0,
            "sensorFrame": "NWU(+x forward,+y left,+z up)",
            "componentFrame": "(+x body-left,+y up,+z back)",
        },
        "sampling": {
            "gridCols": 2,
            "gridRows": 2,
            "maxRangeM": 100.0,
            "excludeTopDeg": 0.0,
            "excludeBottomDeg": 0.0,
        },
        "transform": transform,
        "totalCells": 4,
        "validAnchors": len(anchors),
        "failureCount": 4 - len(anchors),
        "raycastSource": "panorama-capture-viewer",
        "tilesetSharedWithRgb": True,
        "panoramaFaceSize": 144,
        "panoramaCaptureRevision": 1,
        "panoramaSourceImage": {
            "width": 384,
            "height": 192,
            "verticalFovDeg": 180.0,
        },
        "panoramaFaceTileReadiness": [
            {"face": face, "readyWhenCopied": True}
            for face in PANORAMA_FACES
        ],
        "tileState": "ready",
        "timestamp": 1,
    }
    anchor_payload = {
        "anchors": anchors,
        "failures": [
            {"col": index, "row": 99, "reason": "no_hit"}
            for index in range(4 - len(anchors))
        ],
        "metadata": anchor_metadata,
    }
    anchors_path.write_text(json.dumps(anchor_payload, indent=2), encoding="utf-8")

    actual_state = _state(position)
    reference_state = {**_state(position), "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0}}
    projection = {
        "width": 384,
        "height": 192,
        "faceSize": 144,
        "verticalFovDeg": 180.0,
        "faceFovDeg": 130.0,
        "topPoleGuardDeg": 0.0,
        "bottomPoleGuardDeg": 0.0,
        "jpegQuality": 0.74,
        "uploadScale": source_width / 384,
        "rgbWidth": source_width,
        "rgbHeight": source_height,
    }
    rgb_sha, raw_sha, anchors_sha = map(_sha256, (rgb_path, raw_path, anchors_path))
    manifest = {
        "schemaVersion": 2,
        "sessionId": session_id,
        "captureId": capture_id,
        "locationId": location_id,
        "frameId": str(frame_id),
        "capturedAt": 100.0 + color_seed,
        "exportedAt": "2026-08-09T12:00:00.000Z",
        "rgbWidth": source_width,
        "rgbHeight": source_height,
        "rgbSha256": rgb_sha,
        "rawSha256": raw_sha,
        "anchorsSha256": anchors_sha,
        "files": {
            "rgb": {"name": rgb_path.name, "sha256": rgb_sha},
            "raw": {"name": raw_path.name, "sha256": raw_sha},
            "anchors": {"name": anchors_path.name, "sha256": anchors_sha},
        },
        "rawModel": "synthetic-da360",
        "rawWidth": raw_width,
        "rawHeight": raw_height,
        "transform": transform,
        "actualState": actual_state,
        "referenceState": reference_state,
        "yaw": 0.0,
        "projectionConfig": projection,
        "validAnchors": len(anchors),
        "failedAnchors": 4 - len(anchors),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return raw_path, anchors_path, manifest_path, rgb_path


def _load_bundle(*paths):
    return load_data(*paths)


def _four_location_dataset(tmp_path, *, bad_location=None):
    datasets = []
    capture_index = 0
    for location_index, location in enumerate("ABCD"):
        for pose_index in range(3):
            capture_index += 1
            capture_id = f"capture-{capture_index:03d}"
            paths = _write_bundle(
                tmp_path / capture_id,
                capture_id=capture_id,
                session_id="session-fit",
                location_id=location,
                frame_id=str(capture_index),
                position=(location_index * 20.0 + pose_index, 0.0, 0.0),
                color_seed=capture_index,
                disparity_offset=capture_index * 0.002,
                distance_scale=1.8 if location == bad_location else 1.0,
            )
            datasets.append(_load_bundle(*paths))
    return combine_datasets(datasets)


def test_pixel_center_mapping_and_horizontal_wrap(tmp_path):
    pred_disp = np.array([[0, 10, 20, 30], [0, 10, 20, 30]], dtype=np.float32)
    anchors = [
        {"col": 0, "row": 0, "u": 1.5, "v": 0.5, "distance": 2.0},
        {"col": 1, "row": 0, "u": 7.5, "v": 0.5, "distance": 4.0},
    ]
    paths = _write_bundle(
        tmp_path / "seam",
        capture_id="seam",
        location_id="seam-location",
        source_size=(8, 4),
        raw_size=(4, 2),
        anchors_override=anchors,
    )
    # Replace the raw values, then repair all bundle hashes and decoded semantic
    # fields through a fresh fixture is unnecessarily opaque; the pure sampler
    # assertions below lock the same mapping directly.
    assert map_pixel_center(1.5, 8, 4) == 0.5
    assert map_pixel_center(7.5, 8, 4) == 3.5
    assert sample_wrapped_bilinear(pred_disp, np.ones_like(pred_disp, bool), 0.5, 0) == 5.0
    assert sample_wrapped_bilinear(pred_disp, np.ones_like(pred_disp, bool), 3.5, 0) == 15.0
    data = _load_bundle(*paths)
    np.testing.assert_allclose(data["z"], [0.5, 0.25])


def test_bilinear_sample_rejects_invalid_contributor():
    values = np.array([[1.0, 3.0]], dtype=np.float32)
    valid = np.array([[True, False]])
    assert sample_wrapped_bilinear(values, valid, 0.5, 0) is None
    assert sample_wrapped_bilinear(values, valid, 0.0, 0) == 1.0


@pytest.mark.parametrize("missing_field", ["sessionId", "captureId", "locationId", "frameId"])
def test_load_rejects_missing_manifest_identity(tmp_path, missing_field):
    paths = _write_bundle(tmp_path / missing_field)
    manifest_path = paths[2]
    manifest = json.loads(manifest_path.read_text())
    del manifest[missing_field]
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match=missing_field):
        _load_bundle(*paths)


def test_load_rejects_manifest_hash_tamper(tmp_path):
    paths = _write_bundle(tmp_path / "tamper")
    manifest_path = paths[2]
    manifest = json.loads(manifest_path.read_text())
    manifest["rgbSha256"] = "b" * 64
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="rgbSha256"):
        _load_bundle(*paths)


def test_load_rejects_artifact_tamper_and_frame_mismatch(tmp_path):
    paths = _write_bundle(tmp_path / "bytes")
    paths[3].write_bytes(paths[3].read_bytes() + b"tampered")
    with pytest.raises(ValueError, match="rgb SHA-256 mismatch"):
        _load_bundle(*paths)

    paths = _write_bundle(tmp_path / "frame", frame_id="raw-frame")
    raw_path, anchors_path, manifest_path, rgb_path = paths
    manifest = json.loads(manifest_path.read_text())
    manifest["frameId"] = "other-frame"
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="identity mismatch|frame mismatch"):
        load_data(raw_path, anchors_path, manifest_path, rgb_path)


@pytest.mark.parametrize(
    "field",
    [
        "raycastSource",
        "tilesetSharedWithRgb",
        "panoramaFaceSize",
        "panoramaCaptureRevision",
        "panoramaSourceImage",
        "panoramaFaceTileReadiness",
    ],
)
def test_load_rejects_missing_panorama_provenance(tmp_path, field):
    paths = _write_bundle(tmp_path / f"missing-{field}")
    _rewrite_anchor_metadata(paths, lambda metadata: metadata.pop(field))
    with pytest.raises(ValueError, match=field):
        _load_bundle(*paths)


def test_load_rejects_duplicate_or_incomplete_panorama_faces(tmp_path):
    paths = _write_bundle(tmp_path / "duplicate-face")

    def duplicate_face(metadata):
        metadata["panoramaFaceTileReadiness"][1]["face"] = "front"

    _rewrite_anchor_metadata(paths, duplicate_face)
    with pytest.raises(ValueError, match="duplicate panorama capture face"):
        _load_bundle(*paths)

    paths = _write_bundle(tmp_path / "incomplete-faces")
    _rewrite_anchor_metadata(
        paths, lambda metadata: metadata["panoramaFaceTileReadiness"].pop()
    )
    with pytest.raises(ValueError, match="exactly six faces"):
        _load_bundle(*paths)


@pytest.mark.parametrize("ready_value", [False, 1, "true", None])
def test_load_rejects_face_not_explicitly_ready(tmp_path, ready_value):
    paths = _write_bundle(tmp_path / f"not-ready-{ready_value!s}")
    _rewrite_anchor_metadata(
        paths,
        lambda metadata: metadata["panoramaFaceTileReadiness"][0].update(
            readyWhenCopied=ready_value
        ),
    )
    with pytest.raises(ValueError, match="readyWhenCopied must be true"):
        _load_bundle(*paths)


@pytest.mark.parametrize(
    ("case", "mutate", "message"),
    [
        (
            "ray-source",
            lambda metadata: metadata.update(raycastSource="main-viewer"),
            "raycastSource",
        ),
        (
            "shared-tileset",
            lambda metadata: metadata.update(tilesetSharedWithRgb=False),
            "tilesetSharedWithRgb",
        ),
        (
            "capture-revision",
            lambda metadata: metadata.update(panoramaCaptureRevision=0),
            "panoramaCaptureRevision",
        ),
        (
            "face-size",
            lambda metadata: metadata.update(panoramaFaceSize=96),
            "panoramaFaceSize",
        ),
        (
            "source-width",
            lambda metadata: metadata["panoramaSourceImage"].update(width=192),
            "dimensions",
        ),
        (
            "source-height",
            lambda metadata: metadata["panoramaSourceImage"].update(height=96),
            "dimensions",
        ),
        (
            "source-vfov",
            lambda metadata: metadata["panoramaSourceImage"].update(verticalFovDeg=120),
            "vertical FOV",
        ),
        (
            "unknown-face",
            lambda metadata: metadata["panoramaFaceTileReadiness"][0].update(face="px"),
            "six-face projection",
        ),
        (
            "tile-state",
            lambda metadata: metadata.update(tileState="loading"),
            "tiles reported ready",
        ),
    ],
)
def test_load_rejects_panorama_provenance_mismatch(tmp_path, case, mutate, message):
    paths = _write_bundle(tmp_path / case)
    _rewrite_anchor_metadata(paths, mutate)
    with pytest.raises(ValueError, match=message):
        _load_bundle(*paths)


def test_inverse_depth_conversion_and_metric_range():
    depth, valid = inverse_depth_to_metric_depth(np.array([0.5, 0.0, -1.0, np.nan]))
    np.testing.assert_allclose(depth[valid], [2.0])
    assert valid.tolist() == [True, False, False, False]

    true_depth = np.array([0.5, 10.0, 20.0, 0.499, 20.001])
    predicted_depth = np.array([0.5, 10.0, 20.0, 100.0, 0.1])
    metrics = _depth_metrics_from_inverse(1 / true_depth, 1 / predicted_depth)
    near = _depth_metrics_from_inverse(1 / true_depth, 1 / predicted_depth, max_true_depth_m=10.0)
    assert metrics["n_valid"] == 3
    assert near["n_valid"] == 2
    assert metrics["p90_error_m"] == 0.0


def test_metrics_are_computed_in_metres():
    metrics = compute_metrics(np.array([1.0, 10.0]), np.array([2.0, 11.0]))
    assert np.isclose(metrics["median_abs_rel"], 0.55)
    assert np.isclose(metrics["median_error_m"], 1.0)
    assert np.isclose(metrics["rmse_m"], 1.0)


def test_duplicate_capture_frame_rgb_and_pose_are_rejected(tmp_path):
    first = _load_bundle(*_write_bundle(
        tmp_path / "first", capture_id="one", frame_id="1", position=(0, 0, 0), color_seed=1
    ))
    second = _load_bundle(*_write_bundle(
        tmp_path / "second", capture_id="two", frame_id="2", position=(1, 0, 0), color_seed=2
    ))

    duplicate = copy.deepcopy(second)
    duplicate["identity"]["captureId"] = "one"
    with pytest.raises(ValueError, match="duplicate captureId"):
        combine_datasets([first, duplicate])

    duplicate = copy.deepcopy(second)
    duplicate["identity"]["frameId"] = "1"
    with pytest.raises(ValueError, match="duplicate session/frame"):
        combine_datasets([first, duplicate])

    duplicate = copy.deepcopy(second)
    duplicate["rgb_sha256"] = first["rgb_sha256"]
    with pytest.raises(ValueError, match="duplicate RGB"):
        combine_datasets([first, duplicate])

    duplicate = copy.deepcopy(second)
    duplicate["pose_position"] = first["pose_position"].copy()
    duplicate["pose_quaternion"] = -first["pose_quaternion"].copy()
    with pytest.raises(ValueError, match="duplicate capture pose"):
        combine_datasets([first, duplicate])


def test_every_location_requires_three_distinct_poses(tmp_path):
    datasets = []
    for index in range(2):
        datasets.append(_load_bundle(*_write_bundle(
            tmp_path / f"short-{index}",
            capture_id=f"short-{index}",
            frame_id=str(index),
            position=(index, 0, 0),
            color_seed=index + 10,
            disparity_offset=index * 0.01,
        )))
    with pytest.raises(ValueError, match="requires >= 3 distinct poses"):
        combine_datasets(datasets)


def test_local_pose_deduplication_is_scoped_to_location(tmp_path):
    datasets = []
    capture_index = 0
    for location in ("site-A", "site-B"):
        for pose_index in range(3):
            capture_index += 1
            datasets.append(_load_bundle(*_write_bundle(
                tmp_path / f"capture-{capture_index}",
                capture_id=f"capture-{capture_index}",
                location_id=location,
                frame_id=str(capture_index),
                position=(pose_index, 0, 0),
                color_seed=capture_index + 30,
                disparity_offset=capture_index * 0.003,
            )))

    combined = combine_datasets(datasets)
    assert combined["location_capture_counts"] == {"site-A": 3, "site-B": 3}


def test_known_scale_shift_all_locations_pass_and_candidate_is_bound(tmp_path):
    data = _four_location_dataset(tmp_path / "captures")
    output_dir = tmp_path / "fit"
    report = run_fitting(data, output_dir)

    assert report["success"]
    assert report["acceptance"]["passed"]
    assert report["validation"]["strategy"] == "leave-one-location-out"
    assert len(report["validation"]["folds"]) == 4
    assert all(item["passed"] for item in report["acceptance"]["per_location"].values())
    assert np.isclose(report["scale_shift"]["a"], TRUE_A, rtol=1e-5)
    assert np.isclose(report["scale_shift"]["b"], TRUE_B, rtol=1e-5)
    assert report["selected_model"] == "scale_shift"

    calibration = json.loads((output_dir / "depth_calibration.json").read_text())
    assert calibration["accepted"] is True
    assert calibration["checkpoint_sha256"] == CHECKPOINT_SHA
    assert calibration["requestWidth"] == 8
    assert calibration["requestHeight"] == 4
    assert calibration["input"]["request_width"] == 8
    assert calibration["projection"]["jpegQuality"] == 0.74
    assert calibration["projection"]["rgbWidth"] == 8
    assert len(calibration["dataset_fingerprint_sha256"]) == 64


def test_one_bad_held_out_location_rejects_aggregate_candidate(tmp_path):
    data = _four_location_dataset(tmp_path / "captures", bad_location="D")
    output_dir = tmp_path / "fit"
    report = run_fitting(data, output_dir)
    assert report["success"]
    assert report["acceptance"]["per_location"]["D"]["passed"] is False
    assert report["acceptance"]["passed"] is False
    assert not (output_dir / "depth_calibration.json").exists()
    assert json.loads((output_dir / "depth_calibration_candidate.json").read_text())["accepted"] is False


def test_unverified_synthetic_data_never_emits_accepted_calibration(tmp_path):
    x = np.linspace(0.08, 0.32, 24)
    data = {
        "x": x,
        "z": TRUE_A * x + TRUE_B,
        "location_ids": np.repeat(list("ABCD"), 6),
        "pred_disp": x.reshape(4, 6).astype(np.float32),
        "valid_mask": np.ones((4, 6), dtype=bool),
        "H": 4,
        "W": 6,
        "metadata": {},
        "n_anchors": len(x),
    }
    report = run_fitting(data, tmp_path / "unverified")
    assert report["success"]
    assert report["acceptance"]["passed"] is False
    assert report["collection"]["bundle_validation_complete"] is False
    assert not (tmp_path / "unverified" / "depth_calibration.json").exists()


def test_small_dataset_fails_cleanly_and_removes_stale_calibration(tmp_path):
    output_dir = tmp_path / "small"
    output_dir.mkdir()
    (output_dir / "depth_calibration.json").write_text('{"accepted": true}')
    data = {
        "x": np.arange(5, dtype=np.float64),
        "z": np.ones(5, dtype=np.float64),
        "location_ids": np.repeat("A", 5),
        "n_anchors": 5,
    }
    report = run_fitting(data, output_dir)
    assert not report["success"]
    assert "need >=6" in report["error"]
    assert not (output_dir / "depth_calibration.json").exists()
    assert json.loads((output_dir / "fit_report.json").read_text()) == report
