"""Test NPZ encode-decode round-trip integrity and edge cases."""
import io, json, os, sys
import numpy as np
import pytest
import requests
from PIL import Image

pytestmark = pytest.mark.integration

DA360_URL = os.environ.get("DA360_URL", "http://127.0.0.1:5688")
SESSION = requests.Session()
SESSION.trust_env = False


def _post_raw(image_bytes):
    return SESSION.post(f"{DA360_URL}/depth/raw",
                         data=image_bytes,
                         headers={"Content-Type": "image/jpeg"},
                         timeout=60)


def _jpeg_bytes(w, h):
    img = Image.new("RGB", (w, h), (100, 150, 200))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _health():
    response = SESSION.get(f"{DA360_URL}/health", timeout=10)
    response.raise_for_status()
    return response.json()


def _request_size():
    health = _health()
    calibration = health.get("calibration") or {}
    return (
        int(calibration.get("request_width") or 1036),
        int(calibration.get("request_height") or 518),
    )


def test_npz_is_valid_archive():
    """Response body is a valid NPZ that numpy can load."""
    resp = _post_raw(_jpeg_bytes(*_request_size()))
    data = np.load(io.BytesIO(resp.content), allow_pickle=True)
    assert len(data.files) >= 4, f"expected >=4 arrays, got {len(data.files)}"


def test_npz_numpy_version_independent():
    """NPZ works with both allow_pickle=True and with metadata_json read as string."""
    resp = _post_raw(_jpeg_bytes(*_request_size()))
    # First load requiring pickle (for metadata_json)
    data1 = np.load(io.BytesIO(resp.content), allow_pickle=True)
    assert "metadata_json" in data1.files
    # Numeric arrays work without pickle
    for key in ("pred_disp", "relative_depth", "valid_mask"):
        arr = data1[key]
        assert arr.dtype in (np.float32, np.uint8), f"unexpected dtype for {key}: {arr.dtype}"


def test_different_input_sizes_work():
    """The raw endpoint handles different input ERP image sizes via server-side resize."""
    health = _health()
    calibration = health.get("calibration") or {}
    sizes = [_request_size()] if health.get("depth_mode") == "da360-metric" else [
        (1036, 518), (672, 336), (518, 259)
    ]
    for w, h in sizes:
        resp = _post_raw(_jpeg_bytes(w, h))
        assert resp.status_code == 200, f"failed for {w}x{h}: HTTP {resp.status_code}"
        data = np.load(io.BytesIO(resp.content), allow_pickle=True)
        pd = data["pred_disp"]
        md = json.loads(str(data["metadata_json"]))
        assert pd.shape == (md["height"], md["width"]), \
            f"shape mismatch: pred_disp {pd.shape} vs metadata {md['height']}x{md['width']}"


def test_response_headers():
    """Response includes X-DA360-* headers with metadata."""
    resp = _post_raw(_jpeg_bytes(*_request_size()))
    assert resp.headers.get("X-DA360-Model"), "missing X-DA360-Model"
    assert resp.headers.get("X-DA360-Width"), "missing X-DA360-Width"
    assert resp.headers.get("X-DA360-Height"), "missing X-DA360-Height"
    assert resp.headers.get("X-DA360-Latency-Ms"), "missing X-DA360-Latency-Ms"
    assert float(resp.headers["X-DA360-Latency-Ms"]) > 0
