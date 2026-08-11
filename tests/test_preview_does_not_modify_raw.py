"""Verify that calling /depth (preview) does NOT mutate the raw output of /depth/raw."""
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


def _post_depth(image_bytes):
    health = SESSION.get(f"{DA360_URL}/health", timeout=10).json()
    projection = (health.get("calibration") or {}).get("projection")
    headers = {"Content-Type": "image/jpeg"}
    if projection:
        headers["X-Projection-Config"] = json.dumps(projection, separators=(",", ":"))
    return SESSION.post(f"{DA360_URL}/depth",
                         data=image_bytes,
                         headers=headers,
                         timeout=60)


def _request_size():
    health = SESSION.get(f"{DA360_URL}/health", timeout=10).json()
    calibration = health.get("calibration") or {}
    return (
        int(calibration.get("request_width") or 1036),
        int(calibration.get("request_height") or 518),
    )


def _jpeg_bytes():
    img = Image.new("RGB", _request_size(), (50, 180, 80))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def test_preview_does_not_modify_raw():
    """Calling /depth between two /depth/raw calls produces identical raw output."""
    jpg = _jpeg_bytes()

    # First raw call
    r1 = _post_raw(jpg)
    assert r1.status_code == 200
    data1 = np.load(io.BytesIO(r1.content), allow_pickle=True)
    disp1 = data1["pred_disp"].copy()

    # Interleaved preview call
    r_preview = _post_depth(jpg)
    assert r_preview.status_code == 200, f"/depth failed: {r_preview.text}"

    # Second raw call — must be identical
    r2 = _post_raw(jpg)
    assert r2.status_code == 200
    data2 = np.load(io.BytesIO(r2.content), allow_pickle=True)
    disp2 = data2["pred_disp"]

    assert np.allclose(disp1, disp2, rtol=1e-6, atol=1e-8), \
        "pred_disp changed after /depth call — preview side-effects raw output!"


def test_multiple_preview_calls_stable():
    """Multiple interleaved /depth calls do not accumulate side effects."""
    jpg = _jpeg_bytes()

    # Initial raw
    r1 = _post_raw(jpg)
    disp_ref = np.load(io.BytesIO(r1.content), allow_pickle=True)["pred_disp"].copy()

    for _ in range(3):
        _post_depth(jpg)  # interleaved preview

    # Final raw
    r2 = _post_raw(jpg)
    disp_final = np.load(io.BytesIO(r2.content), allow_pickle=True)["pred_disp"]

    assert np.allclose(disp_ref, disp_final, rtol=1e-6, atol=1e-8), \
        "pred_disp changed after multiple /depth calls"
