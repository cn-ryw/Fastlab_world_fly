"""GPU-free contract tests for standalone and combined DA360 Flask apps."""

import io
import hashlib
import json
import os
import sys
import tempfile
import threading
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image

try:
    import flask  # noqa: F401
except ModuleNotFoundError:
    import pytest

    pytest.skip(
        "backend contract tests run in the project image when Flask is unavailable",
        allow_module_level=True,
    )

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import combined_server  # noqa: E402
from da360_server import DA360Runner, create_app, depth_to_polar_scan  # noqa: E402


class FakeDepthRunner:
    model_name = "DA360_fake"
    device = "cpu"
    width = 8
    height = 4
    checkpoint_width = 16
    checkpoint_height = 8
    input_scale = 0.5
    resample_name = "bicubic"
    use_amp = False
    channels_last = False
    depth_mode = "da360-relative"
    calibration = None
    checkpoint_sha256 = "ab" * 32
    checkpoint_coverage = 1.0
    checkpoint_missing_keys = 0
    checkpoint_unexpected_keys = 0

    def __init__(self):
        self.infer_calls = 0

    def infer(self, _image):
        self.infer_calls += 1
        return np.linspace(1.0, 8.0, self.width * self.height, dtype=np.float32).reshape(
            self.height, self.width
        )

    def infer_depth(self, image, _projection_config=None):
        return self.infer(image)

    def infer_raw(self, _image):
        pred_disp = np.linspace(
            2.0, 10.0, self.width * self.height, dtype=np.float32
        ).reshape(self.height, self.width)
        return {
            "pred_disp": pred_disp,
            "relative_depth": 1.0 / pred_disp,
            "valid_mask": np.ones_like(pred_disp, dtype=np.uint8),
            "metadata": {
                "model": self.model_name,
                "device": self.device,
                "width": self.width,
                "height": self.height,
                "resample": self.resample_name,
                "unit_pred_disp": "raw disparity (inverse depth), NOT per-frame normalized",
                "unit_relative_depth": "1/pred_disp (not divided by frame min)",
            },
        }


class FakeYopoRunner:
    device = "cpu"
    model_path = "/models/fake-yopo.pth"
    checkpoint_sha256 = "cd" * 32
    checkpoint_coverage = 1.0
    checkpoint_missing_keys = 0
    checkpoint_unexpected_keys = 0
    config_name = "fake.yaml"
    config_sha256 = "ef" * 32
    base_config_name = "traj_opt.yaml"
    base_config_sha256 = "12" * 32
    effective_config_sha256 = "34" * 32

    def __init__(self):
        self.lock = threading.Lock()
        self.last_call = None
        self.infer_calls = 0

    def infer(self, **kwargs):
        self.infer_calls += 1
        self.last_call = kwargs
        return np.arange(9, dtype=np.float32), 0.25, 1.125


def jpeg_bytes(width=16, height=8):
    output = io.BytesIO()
    Image.new("RGB", (width, height), (80, 120, 160)).save(output, "JPEG")
    return output.getvalue()


class BackendContractTests(unittest.TestCase):
    @staticmethod
    def expected_service_fingerprint():
        identity = {
            "api_version": 2,
            "da360_checkpoint_sha256": "ab" * 32,
            "yopo_checkpoint_sha256": "cd" * 32,
            "yopo_effective_config_sha256": "34" * 32,
        }
        canonical = json.dumps(
            identity, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ).encode("ascii")
        return hashlib.sha256(canonical).hexdigest()

    def test_yopo_overlay_is_selected_before_global_config_import(self):
        source = (SCRIPTS_DIR / "yopo_bridge.py").read_text(encoding="utf-8")
        self.assertLess(
            source.index('os.environ.setdefault("YOPO_CONFIG", DEFAULT_CONFIG_NAME)'),
            source.index("from config.config import cfg"),
        )
        self.assertIn("loaded_config_path != config_path.resolve()", source)

    def test_yopo_height_override_is_segment_bounded(self):
        source = (SCRIPTS_DIR / "yopo_bridge.py").read_text(encoding="utf-8")
        self.assertIn("YOPO_MAX_VERTICAL_ENDPOINT_STEP_M = 4.0", source)
        self.assertIn("endstate_w[:, 2, 0] = np.clip(", source)

    @contextmanager
    def clients(self):
        standalone = create_app(FakeDepthRunner())
        standalone.config.update(TESTING=True)
        old_depth = combined_server.da360_runner
        old_yopo = combined_server.yopo_runner
        with combined_server._depth_cache_lock:
            old_cache = dict(combined_server._depth_cache)
            combined_server._depth_cache.update({
                "data": None,
                "ts": 0,
                "frame_id": None,
                "depth_mode": None,
                "calibration_id": None,
            })
        combined_server.da360_runner = FakeDepthRunner()
        combined_server.yopo_runner = FakeYopoRunner()
        combined_server.app.config.update(TESTING=True)
        try:
            yield {
                "standalone": standalone.test_client(),
                "combined": combined_server.app.test_client(),
            }
        finally:
            combined_server.da360_runner = old_depth
            combined_server.yopo_runner = old_yopo
            with combined_server._depth_cache_lock:
                combined_server._depth_cache.clear()
                combined_server._depth_cache.update(old_cache)

    def test_health_contract_is_shared(self):
        with self.clients() as clients:
            for name, client in clients.items():
                with self.subTest(app=name):
                    response = client.get("/health")
                    self.assertEqual(response.status_code, 200)
                    payload = response.get_json()
                    self.assertIs(payload["ok"], True)
                    self.assertEqual(payload["api_version"], 2)
                    self.assertEqual(payload["depth_mode"], "da360-relative")
                    self.assertIs(payload["calibration"]["loaded"], False)
                    self.assertIsNone(payload["calibration"]["id"])
                    self.assertIsNone(payload["calibration"]["request_width"])
                    self.assertIsNone(payload["calibration"]["request_height"])
                    self.assertEqual(payload["resample"], "bicubic")
                    self.assertIs(payload["channels_last"], False)

    def test_depth_contract_is_shared(self):
        with self.clients() as clients:
            for name, client in clients.items():
                with self.subTest(app=name):
                    response = client.post(
                        "/depth?frame_id=frame-7&goal_id=goal-2&generation=3",
                        data=jpeg_bytes(),
                        content_type="image/jpeg",
                    )
                    self.assertEqual(response.status_code, 200)
                    payload = response.get_json()
                    required = {
                        "api_version", "depth_image", "depth_scale", "latency_ms", "timings_ms",
                        "model", "device", "width", "height", "request_width", "request_height",
                        "depth_mode", "calibration_id", "frame_id", "goal_id", "generation",
                        "input_sha256", "polar_scan",
                    }
                    self.assertLessEqual(required, payload.keys())
                    self.assertTrue(payload["depth_image"].startswith("data:image/jpeg;base64,"))
                    self.assertEqual(payload["frame_id"], "frame-7")
                    self.assertEqual(payload["goal_id"], "goal-2")
                    self.assertEqual(payload["generation"], "3")
                    scan = payload["polar_scan"]
                    self.assertEqual(scan["depth_mode"], "da360-relative")
                    self.assertEqual(scan["unit"], "x-near-reference")
                    self.assertEqual(scan["radius"], 20.0)
                    self.assertEqual(len(scan["values"]), FakeDepthRunner.width)

    def test_polar_scan_relative_is_explicitly_non_metric(self):
        depth = np.full((12, 96), 4.0, dtype=np.float32)
        depth[:, 48:52] = 2.0
        scan = depth_to_polar_scan(depth, "da360-relative")
        self.assertEqual(scan["unit"], "x-near-reference")
        self.assertEqual(scan["normalization"], "per-frame-depth-p02")
        self.assertAlmostEqual(min(value for value in scan["values"] if value), 1.0)
        self.assertEqual(scan["angle_positive"], "body-left")

    def test_polar_scan_reorders_native_erp_columns_to_increasing_body_left_angle(self):
        # Native YOPO ERP columns are +pi -> -pi, while the public polar scan
        # enumerates -pi -> +pi with a positive body-left angle step.  Unique
        # values make a left/right regression directly observable.
        depth = np.array([[10.0, 20.0, 30.0, 40.0]], dtype=np.float32)
        scan = depth_to_polar_scan(
            depth,
            "da360-metric",
            angular_bins=4,
        )
        self.assertEqual(scan["angle_start_deg"], -135.0)
        self.assertEqual(scan["angle_step_deg"], 90.0)
        self.assertEqual(scan["angle_positive"], "body-left")
        self.assertEqual(scan["values"], [40.0, 30.0, 20.0, 10.0])

    def test_polar_scan_metric_preserves_metres_and_invalid_bins(self):
        depth = np.full((20, 96), 8.0, dtype=np.float32)
        depth[:, 9:12] = np.nan
        scan = depth_to_polar_scan(depth, "da360-metric")
        self.assertEqual(scan["unit"], "metres")
        self.assertIsNone(scan["normalization"])
        self.assertAlmostEqual(scan["values"][75], 8.0, delta=0.05)
        self.assertIsNone(scan["values"][85])
        self.assertLess(scan["valid_fraction"], 1.0)

    def test_polar_scan_uses_runtime_vertical_fov(self):
        depth = np.full((12, 96), 8.0, dtype=np.float32)
        scan = depth_to_polar_scan(
            depth,
            "da360-metric",
            vertical_fov_deg=120.0,
        )
        self.assertEqual(scan["vertical_fov_deg"], 120.0)
        self.assertAlmostEqual(scan["values"][0], 8.0 * np.cos(np.deg2rad(5.0)), places=3)
        with self.assertRaisesRegex(ValueError, "vertical FOV"):
            depth_to_polar_scan(depth, "da360-metric", vertical_fov_deg=200.0)

    def test_raw_contract_is_shared(self):
        with self.clients() as clients:
            for name, client in clients.items():
                with self.subTest(app=name):
                    response = client.post(
                        "/depth/raw?frame_id=frame-1&session_id=session-1&"
                        "capture_id=capture-1&location_id=site-1",
                        data=jpeg_bytes(),
                        content_type="image/jpeg",
                        headers={"Origin": "http://127.0.0.1:8080"},
                    )
                    self.assertEqual(response.status_code, 200)
                    self.assertEqual(response.content_type, "application/x-npz")
                    archive = np.load(io.BytesIO(response.data), allow_pickle=False)
                    self.assertLessEqual(
                        {"pred_disp", "relative_depth", "valid_mask", "metadata_json"},
                        set(archive.files),
                    )
                    metadata = json.loads(str(archive["metadata_json"]))
                    self.assertEqual(metadata["frame_id"], "frame-1")
                    self.assertEqual(metadata["session_id"], "session-1")
                    self.assertEqual(metadata["capture_id"], "capture-1")
                    self.assertEqual(metadata["location_id"], "site-1")
                    self.assertEqual(metadata["api_version"], 2)
                    self.assertEqual(metadata["resample"], "bicubic")
                    self.assertEqual(len(metadata["decoded_rgb_sha256"]), 64)
                    self.assertEqual(response.headers["X-Frame-ID"], "frame-1")
                    self.assertEqual(response.headers["X-Session-ID"], "session-1")
                    self.assertEqual(response.headers["X-Capture-ID"], "capture-1")
                    self.assertEqual(response.headers["X-Location-ID"], "site-1")
                    self.assertEqual(
                        response.headers["Access-Control-Allow-Origin"],
                        "http://127.0.0.1:8080",
                    )
                    exposed = {
                        item.strip().lower()
                        for item in response.headers[
                            "Access-Control-Expose-Headers"
                        ].split(",")
                    }
                    self.assertLessEqual(
                        {
                            "x-frame-id",
                            "x-session-id",
                            "x-capture-id",
                            "x-location-id",
                            "x-da360-model",
                            "x-da360-width",
                            "x-da360-height",
                            "x-da360-latency-ms",
                        },
                        exposed,
                    )

    def test_api_cors_is_allowlisted(self):
        with self.clients() as clients:
            for name, client in clients.items():
                with self.subTest(app=name):
                    allowed = client.options(
                        "/depth", headers={"Origin": "http://127.0.0.1:8080"}
                    )
                    self.assertEqual(allowed.status_code, 204)
                    self.assertEqual(
                        allowed.headers["Access-Control-Allow-Origin"],
                        "http://127.0.0.1:8080",
                    )
                    allowed_headers = {
                        item.strip().lower()
                        for item in allowed.headers[
                            "Access-Control-Allow-Headers"
                        ].split(",")
                    }
                    self.assertLessEqual(
                        {
                            "x-frame-id",
                            "x-session-id",
                            "x-capture-id",
                            "x-location-id",
                            "x-goal-id",
                            "x-generation",
                            "x-projection-config",
                        },
                        allowed_headers,
                    )
                    denied = client.options(
                        "/depth", headers={"Origin": "https://attacker.invalid"}
                    )
                    self.assertEqual(denied.status_code, 403)
                    self.assertNotIn("Access-Control-Allow-Origin", denied.headers)

    def test_request_size_limit_is_enforced(self):
        app = create_app(FakeDepthRunner())
        app.config.update(TESTING=True, MAX_CONTENT_LENGTH=64)
        response = app.test_client().post(
            "/depth", data=b"x" * 65, content_type="image/jpeg"
        )
        self.assertEqual(response.status_code, 413)

    def test_invalid_and_oversized_decoded_images_are_rejected(self):
        app = create_app(FakeDepthRunner())
        app.config.update(TESTING=True)
        client = app.test_client()
        invalid = client.post("/depth", data=b"not-a-jpeg", content_type="image/jpeg")
        self.assertEqual(invalid.status_code, 400)
        with patch.dict(os.environ, {"DA360_MAX_IMAGE_PIXELS": "100"}):
            oversized = client.post(
                "/depth", data=jpeg_bytes(16, 8), content_type="image/jpeg"
            )
        self.assertEqual(oversized.status_code, 400)
        self.assertIn("decoded image is too large", oversized.get_json()["error"])

    def test_relative_plan_full_returns_preview_but_no_applicable_trajectory(self):
        with self.clients() as clients:
            query = (
                "px=1&py=2&pz=3&rpx=4&rpy=5&rpz=6&"
                "gx=10&gy=2&gz=4&vx=0&vy=0&vz=0&ax=0&ay=0&az=0&yaw=0"
                "&frame_id=f9&goal_id=g4&generation=5"
            )
            response = clients["combined"].post(
                f"/yopo/plan_full?{query}", data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertEqual(payload["frame_id"], "f9")
            self.assertEqual(payload["goal_id"], "g4")
            self.assertEqual(payload["generation"], "5")
            self.assertEqual(payload["depth_mode"], "da360-relative")
            self.assertTrue(payload["depth_image"].startswith("data:image/jpeg;base64,"))
            self.assertEqual(payload["polar_scan"]["unit"], "x-near-reference")
            self.assertIs(payload["planning_authorized"], False)
            self.assertEqual(
                payload["service_fingerprint"], self.expected_service_fingerprint()
            )
            self.assertEqual(
                payload["planning_reason"], "da360-relative-is-preview-only"
            )
            self.assertNotIn("endstate", payload)
            self.assertNotIn("traj_time", payload)
            self.assertEqual(payload["planning_origin"]["actual"], [1.0, 2.0, 3.0])
            self.assertEqual(payload["planning_origin"]["reference"], [4.0, 5.0, 6.0])
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)
            self.assertIsNone(combined_server.yopo_runner.last_call)

    def test_metric_plan_full_authorizes_trajectory_and_preserves_origins(self):
        with self.clients() as clients:
            combined_server.da360_runner.depth_mode = "da360-metric"
            combined_server.da360_runner.calibration = {
                "id": "calib-1",
                "accuracy_accepted": True,
                "automatic_accuracy_gate_passed": False,
                "acceptance_method": "manual-user",
                "acceptance_scope": "sim-to-sim",
            }
            query = (
                "px=1&py=2&pz=3&rpx=4&rpy=5&rpz=6&"
                "gx=10&gy=2&gz=4&vx=0&vy=0&vz=0&ax=0&ay=0&az=0&yaw=0"
                "&frame_id=f9&goal_id=g4&generation=5"
            )
            response = clients["combined"].post(
                f"/yopo/plan_full?{query}", data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertIs(payload["planning_authorized"], True)
            self.assertEqual(payload["planning_reason"], "validated-da360-metric")
            self.assertIs(payload["calibration_accuracy_accepted"], True)
            self.assertIs(payload["calibration_automatic_gate_passed"], False)
            self.assertEqual(payload["calibration_acceptance_method"], "manual-user")
            self.assertEqual(payload["calibration_acceptance_scope"], "sim-to-sim")
            self.assertEqual(payload["calibration_id"], "calib-1")
            self.assertEqual(
                payload["service_fingerprint"], self.expected_service_fingerprint()
            )
            self.assertEqual(len(payload["endstate"]), 9)
            self.assertEqual(payload["traj_time"], 1.125)
            call = combined_server.yopo_runner.last_call
            np.testing.assert_array_equal(call["pos"], [1.0, 2.0, 3.0])
            np.testing.assert_array_equal(call["reference_pos"], [4.0, 5.0, 6.0])

    def test_unaccepted_metric_candidate_is_explicitly_experimental_but_runs_yopo(self):
        with self.clients() as clients:
            combined_server.da360_runner.depth_mode = "da360-metric"
            combined_server.da360_runner.calibration = {
                "id": "calib-experimental",
                "accuracy_accepted": False,
            }
            query = (
                "px=1&py=2&pz=3&rpx=4&rpy=5&rpz=6&"
                "gx=10&gy=2&gz=4&vx=0&vy=0&vz=0&ax=0&ay=0&az=0&yaw=0"
                "&frame_id=f9&goal_id=g4&generation=5"
            )
            response = clients["combined"].post(
                f"/yopo/plan_full?{query}", data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertIs(payload["planning_authorized"], True)
            self.assertEqual(
                payload["planning_reason"],
                "experimental-unaccepted-da360-metric",
            )
            self.assertEqual(payload["calibration_id"], "calib-experimental")
            self.assertIs(payload["calibration_accuracy_accepted"], False)
            self.assertEqual(len(payload["endstate"]), 9)
            self.assertEqual(combined_server.yopo_runner.infer_calls, 1)
            health = clients["combined"].get("/yopo/health").get_json()
            self.assertIs(health["calibration_accuracy_accepted"], False)
            self.assertEqual(
                health["planning_reason"],
                "experimental-unaccepted-da360-metric",
            )

    def test_metric_planning_is_blocked_without_complete_service_fingerprint(self):
        with self.clients() as clients:
            combined_server.da360_runner.depth_mode = "da360-metric"
            combined_server.da360_runner.calibration = {"id": "calib-1"}
            combined_server.yopo_runner.effective_config_sha256 = None
            query = (
                "px=1&py=2&pz=3&rpx=4&rpy=5&rpz=6&"
                "gx=10&gy=2&gz=4&vx=0&vy=0&vz=0&ax=0&ay=0&az=0&yaw=0"
                "&frame_id=f9&goal_id=g4&generation=5"
            )
            response = clients["combined"].post(
                f"/yopo/plan_full?{query}", data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertIs(payload["planning_authorized"], False)
            self.assertEqual(
                payload["planning_reason"], "service-fingerprint-unavailable"
            )
            self.assertIsNone(payload["service_fingerprint"])
            self.assertNotIn("endstate", payload)
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)

    def test_combined_plan_full_rejects_incomplete_state(self):
        with self.clients() as clients:
            response = clients["combined"].post(
                "/yopo/plan_full?frame_id=f&goal_id=g&generation=1&px=0",
                data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(response.status_code, 400)
            self.assertIn("missing planning field", response.get_json()["error"])
            self.assertEqual(combined_server.da360_runner.infer_calls, 0)

    def test_combined_plan_full_requires_identity_and_acceleration(self):
        with self.clients() as clients:
            state = (
                "px=1&py=2&pz=3&rpx=4&rpy=5&rpz=6&"
                "gx=10&gy=2&gz=4&vx=0&vy=0&vz=0&ax=0&ay=0&az=0&yaw=0"
            )
            missing_identity = clients["combined"].post(
                f"/yopo/plan_full?{state}", data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(missing_identity.status_code, 400)
            self.assertIn("frame_id", missing_identity.get_json()["error"])

            missing_acceleration = clients["combined"].post(
                "/yopo/plan_full?px=1&py=2&pz=3&rpx=4&rpy=5&rpz=6&"
                "gx=10&gy=2&gz=4&vx=0&vy=0&vz=0&yaw=0&"
                "frame_id=f9&goal_id=g4&generation=5",
                data=jpeg_bytes(), content_type="image/jpeg",
            )
            self.assertEqual(missing_acceleration.status_code, 400)
            self.assertIn("missing planning field: ax", missing_acceleration.get_json()["error"])
            self.assertEqual(combined_server.da360_runner.infer_calls, 0)

    def test_combined_yopo_health_exposes_fingerprints(self):
        with self.clients() as clients:
            response = clients["combined"].get("/yopo/health")
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertEqual(payload["checkpoint_sha256"], "cd" * 32)
            self.assertEqual(payload["checkpoint_coverage"], 1.0)
            self.assertEqual(payload["config"], "fake.yaml")
            self.assertEqual(payload["config_sha256"], "ef" * 32)
            self.assertEqual(payload["base_config"], "traj_opt.yaml")
            self.assertEqual(payload["base_config_sha256"], "12" * 32)
            self.assertEqual(payload["effective_config_sha256"], "34" * 32)
            self.assertIs(payload["planning_authorized"], False)

    def test_relative_two_stage_plan_is_blocked_without_running_yopo(self):
        with self.clients() as clients:
            body = {
                "depth": np.ones((4, 8), dtype=np.float32).tolist(),
                "pose": {"x": 1, "y": 2, "z": 3},
                "goal": {"x": 10, "y": 2, "z": 4},
                "frame_id": "legacy-frame",
                "goal_id": "legacy-goal",
                "generation": "7",
            }
            response = clients["combined"].post("/yopo/plan", json=body)
            self.assertEqual(response.status_code, 409)
            payload = response.get_json()
            self.assertIs(payload["planning_authorized"], False)
            self.assertEqual(
                payload["planning_reason"], "da360-relative-is-preview-only"
            )
            self.assertEqual(payload["frame_id"], "legacy-frame")
            self.assertNotIn("endstate", payload)
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)

    def test_metric_two_stage_plan_uses_matching_server_cache_and_reference(self):
        with self.clients() as clients:
            combined_server.da360_runner.depth_mode = "da360-metric"
            combined_server.da360_runner.calibration = {"id": "calib-legacy"}
            combined_server._cache_depth(
                np.ones((4, 8), dtype=np.float32),
                {"frame_id": "legacy-frame"},
            )
            body = {
                "pose": {"x": 1, "y": 2, "z": 3},
                "reference_pose": {"x": 4, "y": 5, "z": 6},
                "goal": {"x": 10, "y": 2, "z": 4},
                "vel": {"vx": 0, "vy": 0, "vz": 0},
                "acc": {"ax": 0, "ay": 0, "az": 0},
                "yaw": 0,
                "frame_id": "legacy-frame",
                "goal_id": "legacy-goal",
                "generation": "7",
            }
            response = clients["combined"].post("/yopo/plan", json=body)
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertIs(payload["planning_authorized"], True)
            self.assertEqual(payload["planning_reason"], "validated-da360-metric")
            self.assertEqual(payload["frame_id"], "legacy-frame")
            self.assertEqual(payload["goal_id"], "legacy-goal")
            self.assertEqual(payload["generation"], "7")
            self.assertEqual(payload["planning_origin"]["actual"], [1.0, 2.0, 3.0])
            self.assertEqual(payload["planning_origin"]["reference"], [4.0, 5.0, 6.0])
            call = combined_server.yopo_runner.last_call
            np.testing.assert_array_equal(
                call["depth_arr"], np.ones((4, 8), dtype=np.float32)
            )
            np.testing.assert_array_equal(call["pos"], [1.0, 2.0, 3.0])
            np.testing.assert_array_equal(call["reference_pos"], [4.0, 5.0, 6.0])

    def test_metric_two_stage_plan_rejects_request_body_depth(self):
        with self.clients() as clients:
            combined_server.da360_runner.depth_mode = "da360-metric"
            combined_server.da360_runner.calibration = {"id": "calib-legacy"}
            body = {
                "depth": np.ones((4, 8), dtype=np.float32).tolist(),
                "pose": {"x": 1, "y": 2, "z": 3},
                "reference_pose": {"x": 4, "y": 5, "z": 6},
                "goal": {"x": 10, "y": 2, "z": 4},
                "vel": {"vx": 0, "vy": 0, "vz": 0},
                "acc": {"ax": 0, "ay": 0, "az": 0},
                "yaw": 0,
                "frame_id": "legacy-frame",
                "goal_id": "legacy-goal",
                "generation": "7",
            }
            response = clients["combined"].post("/yopo/plan", json=body)
            self.assertEqual(response.status_code, 400)
            self.assertIn("server-cached depth", response.get_json()["error"])
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)

    def test_metric_two_stage_plan_rejects_missing_reference(self):
        with self.clients() as clients:
            combined_server.da360_runner.depth_mode = "da360-metric"
            combined_server.da360_runner.calibration = {"id": "calib-legacy"}
            combined_server._cache_depth(
                np.ones((4, 8), dtype=np.float32),
                {"frame_id": "legacy-frame"},
            )
            body = {
                "pose": {"x": 1, "y": 2, "z": 3},
                "goal": {"x": 10, "y": 2, "z": 4},
                "vel": {"vx": 0, "vy": 0, "vz": 0},
                "acc": {"ax": 0, "ay": 0, "az": 0},
                "yaw": 0,
                "frame_id": "legacy-frame",
                "goal_id": "legacy-goal",
                "generation": "7",
            }
            response = clients["combined"].post("/yopo/plan", json=body)
            self.assertEqual(response.status_code, 400)
            self.assertIn("reference_pose or reference_pos", response.get_json()["error"])
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)

    def test_metric_two_stage_plan_rejects_cache_identity_mismatch(self):
        with self.clients() as clients:
            combined_server.da360_runner.depth_mode = "da360-metric"
            combined_server.da360_runner.calibration = {"id": "calib-current"}
            combined_server._cache_depth(
                np.ones((4, 8), dtype=np.float32),
                {"frame_id": "cached-frame"},
            )
            base_body = {
                "pose": {"x": 1, "y": 2, "z": 3},
                "reference_pos": {"x": 4, "y": 5, "z": 6},
                "goal": {"x": 10, "y": 2, "z": 4},
                "vel": {"vx": 0, "vy": 0, "vz": 0},
                "acc": {"ax": 0, "ay": 0, "az": 0},
                "yaw": 0,
                "frame_id": "requested-frame",
                "goal_id": "legacy-goal",
                "generation": "7",
            }
            response = clients["combined"].post("/yopo/plan", json=base_body)
            self.assertEqual(response.status_code, 409)
            self.assertIn("frame_id mismatch", response.get_json()["error"])
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)

            base_body["frame_id"] = "cached-frame"
            with combined_server._depth_cache_lock:
                combined_server._depth_cache["calibration_id"] = "calib-stale"
            response = clients["combined"].post("/yopo/plan", json=base_body)
            self.assertEqual(response.status_code, 409)
            self.assertIn("calibration mismatch", response.get_json()["error"])
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)

            with combined_server._depth_cache_lock:
                combined_server._depth_cache["calibration_id"] = "calib-current"
                combined_server._depth_cache["depth_mode"] = "da360-relative"
            response = clients["combined"].post("/yopo/plan", json=base_body)
            self.assertEqual(response.status_code, 409)
            self.assertIn("depth mode mismatch", response.get_json()["error"])
            self.assertEqual(combined_server.yopo_runner.infer_calls, 0)

    def test_calibration_is_fail_closed_and_fingerprint_bound(self):
        runner = object.__new__(DA360Runner)
        runner.model_name = "DA360_fake"
        runner.width = 8
        runner.height = 4
        runner.resample_name = "bicubic"
        runner.checkpoint_sha256 = "ab" * 32
        calibration = {
            "schema_version": 1,
            "accepted": True,
            "a": 1.25,
            "b": 0.05,
            "depth_min_m": 0.5,
            "depth_max_m": 20.0,
            "model": runner.model_name,
            "width": runner.width,
            "height": runner.height,
            "resample": runner.resample_name,
            "checkpoint_sha256": runner.checkpoint_sha256,
            "requestWidth": 16,
            "requestHeight": 8,
            "input": {
                "model": runner.model_name,
                "width": runner.width,
                "height": runner.height,
                "resample": runner.resample_name,
                "checkpoint_sha256": runner.checkpoint_sha256,
                "request_width": 16,
                "request_height": 8,
            },
            "projection": {
                "width": 32,
                "height": 16,
                "faceSize": 16,
                "rgbWidth": 16,
                "rgbHeight": 8,
                "verticalFovDeg": 180,
                "faceFovDeg": 130,
                "topPoleGuardDeg": 0,
                "bottomPoleGuardDeg": 0,
                "jpegQuality": 0.74,
                "uploadScale": 0.5,
            },
            "dataset_fingerprint_sha256": "de" * 32,
            "selected_model": "scale_shift",
            "relation": "inverse_depth_1_per_m = a * pred_disp + b",
            "acceptance": {"passed": True},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "depth_calibration.json"

            path.write_text(json.dumps(calibration), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                loaded = runner._load_depth_calibration()
            self.assertEqual(loaded["a"], 1.25)
            self.assertEqual(len(loaded["id"]), 16)
            self.assertIs(loaded["accuracy_accepted"], True)
            self.assertIs(loaded["automatic_accuracy_gate_passed"], True)
            self.assertEqual(loaded["acceptance_method"], "automatic")
            self.assertEqual(loaded["acceptance_scope"], "accuracy-gates")
            self.assertEqual((loaded["request_width"], loaded["request_height"]), (16, 8))

            runner.calibration = loaded
            infer_calls = []
            runner.infer_raw = lambda image: infer_calls.append(image.size) or {
                "pred_disp": np.full((runner.height, runner.width), 0.5, dtype=np.float32)
            }
            metric = runner.infer_metric(
                Image.new("RGB", (16, 8)), calibration["projection"]
            )
            self.assertTrue(np.all(np.isfinite(metric)))
            self.assertEqual(infer_calls, [(16, 8)])
            with self.assertRaisesRegex(ValueError, "input size mismatch"):
                runner.infer_metric(
                    Image.new("RGB", (8, 4)), calibration["projection"]
                )
            with self.assertRaisesRegex(ValueError, "runtime projection config must be an object"):
                runner.infer_metric(Image.new("RGB", (16, 8)))
            changed_projection = dict(calibration["projection"], verticalFovDeg=160)
            with self.assertRaisesRegex(ValueError, "projection mismatch for verticalFovDeg"):
                runner.infer_metric(Image.new("RGB", (16, 8)), changed_projection)
            self.assertEqual(infer_calls, [(16, 8)])

            rejected = dict(calibration, accepted=False)
            rejected["acceptance"] = {"passed": False}
            path.write_text(json.dumps(rejected), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                experimental = runner._load_depth_calibration()
            self.assertIs(experimental["accuracy_accepted"], False)
            self.assertIs(experimental["automatic_accuracy_gate_passed"], False)
            self.assertIsNone(experimental["acceptance_method"])

            manually_accepted = dict(
                rejected,
                accepted=True,
                manual_acceptance={
                    "accepted": True,
                    "accepted_by": "project-owner",
                    "accepted_at": "2026-08-11",
                    "scope": "sim-to-sim",
                    "basis": "user-reviewed live depth",
                },
            )
            path.write_text(json.dumps(manually_accepted), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                manually_loaded = runner._load_depth_calibration()
            self.assertIs(manually_loaded["accuracy_accepted"], True)
            self.assertIs(manually_loaded["automatic_accuracy_gate_passed"], False)
            self.assertEqual(manually_loaded["acceptance_method"], "manual-user")
            self.assertEqual(manually_loaded["acceptance_scope"], "sim-to-sim")

            wrong_scope = dict(manually_accepted)
            wrong_scope["manual_acceptance"] = dict(
                manually_accepted["manual_acceptance"], scope="real-world"
            )
            path.write_text(json.dumps(wrong_scope), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                with self.assertRaisesRegex(RuntimeError, "scope must be sim-to-sim"):
                    runner._load_depth_calibration()

            inconsistent = dict(rejected, accepted=True)
            path.write_text(json.dumps(inconsistent), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                with self.assertRaisesRegex(RuntimeError, "acceptance statuses disagree"):
                    runner._load_depth_calibration()

            mismatched = dict(calibration, checkpoint_sha256="cd" * 32)
            path.write_text(json.dumps(mismatched), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                with self.assertRaisesRegex(RuntimeError, "checkpoint_sha256 mismatch"):
                    runner._load_depth_calibration()


if __name__ == "__main__":
    unittest.main()
