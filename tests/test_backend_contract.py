"""GPU-free contract tests for standalone and combined DA360 Flask apps."""

import io
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

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import combined_server  # noqa: E402
from da360_server import DA360Runner, create_app  # noqa: E402


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

    def infer_depth(self, image):
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

    def __init__(self):
        self.lock = threading.Lock()
        self.last_call = None

    def infer(self, **kwargs):
        self.last_call = kwargs
        return np.arange(9, dtype=np.float32), 0.25, 1.125


def jpeg_bytes(width=16, height=8):
    output = io.BytesIO()
    Image.new("RGB", (width, height), (80, 120, 160)).save(output, "JPEG")
    return output.getvalue()


class BackendContractTests(unittest.TestCase):
    @contextmanager
    def clients(self):
        standalone = create_app(FakeDepthRunner())
        standalone.config.update(TESTING=True)
        old_depth = combined_server.da360_runner
        old_yopo = combined_server.yopo_runner
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
                    self.assertEqual(payload["calibration"], {"id": None, "loaded": False})
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
                        "input_sha256",
                    }
                    self.assertLessEqual(required, payload.keys())
                    self.assertTrue(payload["depth_image"].startswith("data:image/jpeg;base64,"))
                    self.assertEqual(payload["frame_id"], "frame-7")
                    self.assertEqual(payload["goal_id"], "goal-2")
                    self.assertEqual(payload["generation"], "3")

    def test_raw_contract_is_shared(self):
        with self.clients() as clients:
            for name, client in clients.items():
                with self.subTest(app=name):
                    response = client.post(
                        "/depth/raw?frame_id=capture-1",
                        data=jpeg_bytes(),
                        content_type="image/jpeg",
                    )
                    self.assertEqual(response.status_code, 200)
                    self.assertEqual(response.content_type, "application/x-npz")
                    archive = np.load(io.BytesIO(response.data), allow_pickle=False)
                    self.assertLessEqual(
                        {"pred_disp", "relative_depth", "valid_mask", "metadata_json"},
                        set(archive.files),
                    )
                    metadata = json.loads(str(archive["metadata_json"]))
                    self.assertEqual(metadata["frame_id"], "capture-1")
                    self.assertEqual(metadata["api_version"], 2)
                    self.assertEqual(metadata["resample"], "bicubic")
                    self.assertEqual(len(metadata["decoded_rgb_sha256"]), 64)
                    self.assertEqual(response.headers["X-Frame-ID"], "capture-1")

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

    def test_combined_plan_full_echoes_identity_and_depth_mode(self):
        with self.clients() as clients:
            query = (
                "px=1&py=2&pz=3&rpx=4&rpy=5&rpz=6&"
                "gx=10&gy=2&gz=4&vx=0&vy=0&vz=0&yaw=0"
                "&frame_id=f9&goal_id=g4&generation=5"
            )
            response = clients["combined"].post(
                f"/yopo/plan_full?{query}", data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertEqual(len(payload["endstate"]), 9)
            self.assertEqual(payload["frame_id"], "f9")
            self.assertEqual(payload["goal_id"], "g4")
            self.assertEqual(payload["generation"], "5")
            self.assertEqual(payload["depth_mode"], "da360-relative")
            self.assertEqual(payload["planning_origin"]["actual"], [1.0, 2.0, 3.0])
            self.assertEqual(payload["planning_origin"]["reference"], [4.0, 5.0, 6.0])
            call = combined_server.yopo_runner.last_call
            np.testing.assert_array_equal(call["pos"], [1.0, 2.0, 3.0])
            np.testing.assert_array_equal(call["reference_pos"], [4.0, 5.0, 6.0])

    def test_combined_plan_full_rejects_incomplete_state(self):
        with self.clients() as clients:
            response = clients["combined"].post(
                "/yopo/plan_full?px=0", data=jpeg_bytes(), content_type="image/jpeg"
            )
            self.assertEqual(response.status_code, 400)
            self.assertIn("missing planning field", response.get_json()["error"])
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

    def test_two_stage_plan_reference_origin_defaults_to_actual(self):
        with self.clients() as clients:
            body = {
                "depth": np.ones((4, 8), dtype=np.float32).tolist(),
                "pose": {"x": 1, "y": 2, "z": 3},
                "goal": {"x": 10, "y": 2, "z": 4},
                "vel": {"vx": 0, "vy": 0, "vz": 0},
                "acc": {"ax": 0, "ay": 0, "az": 0},
                "yaw": 0,
            }
            response = clients["combined"].post("/yopo/plan", json=body)
            self.assertEqual(response.status_code, 200)
            payload = response.get_json()
            self.assertEqual(payload["planning_origin"]["actual"], [1.0, 2.0, 3.0])
            self.assertEqual(payload["planning_origin"]["reference"], [1.0, 2.0, 3.0])
            call = combined_server.yopo_runner.last_call
            np.testing.assert_array_equal(call["pos"], call["reference_pos"])

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

            rejected = dict(calibration, accepted=False)
            path.write_text(json.dumps(rejected), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                with self.assertRaisesRegex(RuntimeError, "acceptance gates"):
                    runner._load_depth_calibration()

            mismatched = dict(calibration, checkpoint_sha256="cd" * 32)
            path.write_text(json.dumps(mismatched), encoding="utf-8")
            with patch.dict(os.environ, {"DA360_DEPTH_CALIB_PATH": str(path)}):
                with self.assertRaisesRegex(RuntimeError, "checkpoint_sha256 mismatch"):
                    runner._load_depth_calibration()


if __name__ == "__main__":
    unittest.main()
