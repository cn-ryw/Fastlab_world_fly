"""Pure tests for offline DA360 inverse-depth metric fitting."""

import json

import numpy as np

from scripts.fit_da360_metric import (
    combine_datasets,
    compute_metrics,
    inverse_depth_to_metric_depth,
    load_data,
    map_pixel_center,
    run_fitting,
    sample_wrapped_bilinear,
)


def _write_raw(path, pred_disp, valid_mask=None, metadata_extra=None):
    pred_disp = np.asarray(pred_disp, dtype=np.float32)
    if valid_mask is None:
        valid_mask = np.ones(pred_disp.shape, dtype=np.uint8)
    metadata = {"model": "synthetic-da360", "width": pred_disp.shape[1], "height": pred_disp.shape[0]}
    metadata.update(metadata_extra or {})
    np.savez(
        path,
        pred_disp=pred_disp,
        valid_mask=np.asarray(valid_mask, dtype=np.uint8),
        metadata_json=np.array(json.dumps(metadata)),
    )


def test_pixel_center_mapping_and_horizontal_wrap(tmp_path):
    raw_path = tmp_path / "raw.npz"
    anchor_path = tmp_path / "anchors.json"
    pred_disp = np.array([[0, 10, 20, 30], [0, 10, 20, 30]], dtype=np.float32)
    _write_raw(raw_path, pred_disp)
    anchor_path.write_text(json.dumps({
        "metadata": {"imageWidth": 8, "imageHeight": 4, "locationId": "seam"},
        "anchors": [
            # Maps to raw (0.5, 0), halfway between columns 0 and 1.
            {"u": 1.5, "v": 0.5, "distance": 2.0},
            # Maps to raw (3.5, 0), halfway across the ERP 3->0 seam.
            {"u": 7.5, "v": 0.5, "distance": 4.0},
        ],
    }))

    data = load_data(raw_path, anchor_path)
    np.testing.assert_allclose(data["x"], [5.0, 15.0])
    np.testing.assert_allclose(data["z"], [0.5, 0.25])
    assert map_pixel_center(1.5, 8, 4) == 0.5
    assert map_pixel_center(7.5, 8, 4) == 3.5
    assert sample_wrapped_bilinear(pred_disp, np.ones_like(pred_disp, bool), 3.5, 0) == 15.0


def test_bilinear_sample_rejects_invalid_contributor():
    values = np.array([[1.0, 3.0]], dtype=np.float32)
    valid = np.array([[True, False]])
    assert sample_wrapped_bilinear(values, valid, 0.5, 0) is None
    assert sample_wrapped_bilinear(values, valid, 0.0, 0) == 1.0


def test_load_rejects_anchor_raw_frame_mismatch(tmp_path):
    raw_path = tmp_path / "raw.npz"
    anchor_path = tmp_path / "anchors.json"
    _write_raw(raw_path, [[1.0]], metadata_extra={"frame_id": "raw-frame"})
    anchor_path.write_text(json.dumps({
        "metadata": {"imageWidth": 1, "imageHeight": 1, "frameId": "anchor-frame"},
        "anchors": [{"u": 0, "v": 0, "distance": 1}],
    }))
    try:
        load_data(raw_path, anchor_path)
    except ValueError as error:
        assert "frame mismatch" in str(error)
    else:
        raise AssertionError("mismatched frame IDs must be rejected")


def test_inverse_depth_conversion_marks_nonpositive_invalid():
    depth, valid = inverse_depth_to_metric_depth(np.array([0.5, 0.0, -1.0, np.nan]))
    np.testing.assert_allclose(depth[valid], [2.0])
    assert valid.tolist() == [True, False, False, False]
    assert np.isnan(depth[~valid]).all()


def test_metrics_are_computed_in_metres():
    metrics = compute_metrics(np.array([1.0, 10.0]), np.array([2.0, 11.0]))
    assert np.isclose(metrics["median_abs_rel"], 0.55)
    assert np.isclose(metrics["median_error_m"], 1.0)
    assert np.isclose(metrics["rmse_m"], 1.0)


def test_known_scale_shift_and_location_holdout(tmp_path):
    x = np.linspace(0.05, 0.65, 24, dtype=np.float64)
    true_a, true_b = 2.25, 0.12
    inverse_depth = true_a * x + true_b
    pred_disp = x.reshape(4, 6).astype(np.float32)
    output_dir = tmp_path / "fit"
    data = {
        "x": x,
        "z": inverse_depth,
        "location_ids": np.repeat(["A", "B", "C", "D"], 6),
        "pred_disp": pred_disp,
        "valid_mask": np.ones(pred_disp.shape, dtype=bool),
        "H": 4,
        "W": 6,
        "metadata": {
            "model": "synthetic-da360",
            "width": 6,
            "height": 4,
            "resample": "bicubic",
            "checkpoint_sha256": "a" * 64,
        },
        "n_anchors": len(x),
        "valid_anchor_fraction": 1.0,
        "sampled_anchors": [],
        "n_samples": 12,
        "sample_ids": [f"capture-{index}" for index in range(12)],
    }

    report = run_fitting(data, output_dir)

    assert report["success"]
    assert report["validation"]["strategy"] == "leave-one-location-out"
    assert len(report["validation"]["folds"]) == 4
    assert np.isclose(report["scale_shift"]["a"], true_a, rtol=1e-6)
    assert np.isclose(report["scale_shift"]["b"], true_b, rtol=1e-6)
    assert report["selected_model"] == "scale_shift"
    expected_depth = 1.0 / (true_a * pred_disp + true_b)
    np.testing.assert_allclose(
        np.load(output_dir / "metric_depth_shift.npy"), expected_depth, rtol=1e-6
    )

    on_disk = json.loads((output_dir / "fit_report.json").read_text())
    assert on_disk["conclusion"] == report["conclusion"]
    assert on_disk["relation"] == "inverse_depth_1_per_m = a * pred_disp + b"
    assert on_disk["output_units"] == "metres"
    calibration = json.loads((output_dir / "depth_calibration.json").read_text())
    assert calibration["accepted"] is True
    assert calibration["checkpoint_sha256"] == "a" * 64


def test_multiple_capture_combination_preserves_location_groups():
    def capture(sample_id, location, offset):
        pred = np.array([[0.1 + offset, 0.2 + offset]], dtype=np.float32)
        return {
            "x": pred.reshape(-1).astype(np.float64),
            "z": (2.0 * pred.reshape(-1) + 0.1).astype(np.float64),
            "location_ids": np.repeat(location, 2),
            "pred_disp": pred,
            "valid_mask": np.ones_like(pred, dtype=bool),
            "H": 1,
            "W": 2,
            "metadata": {"model": "same", "width": 2, "height": 1},
            "anchor_metadata": {},
            "n_anchors": 2,
            "n_valid_anchors": 2,
            "valid_anchor_fraction": 1.0,
            "sampled_anchors": [],
            "sample_id": sample_id,
        }

    combined = combine_datasets([
        capture("capture-a", "A", 0.0),
        capture("capture-b", "B", 0.2),
    ])
    assert combined["n_samples"] == 2
    assert combined["sample_ids"] == ["capture-a", "capture-b"]
    assert combined["location_ids"].tolist() == ["A", "A", "B", "B"]
    assert combined["preview_sample_id"] == "capture-a"


def test_small_dataset_fails_cleanly_and_writes_complete_report(tmp_path):
    output_dir = tmp_path / "small"
    data = {
        "x": np.arange(5, dtype=np.float64),
        "z": np.ones(5, dtype=np.float64),
        "n_anchors": 5,
    }
    report = run_fitting(data, output_dir)
    assert not report["success"]
    assert "need >=6" in report["error"]
    assert json.loads((output_dir / "fit_report.json").read_text()) == report
