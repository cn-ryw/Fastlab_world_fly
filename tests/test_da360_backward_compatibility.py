"""Verify the existing /depth endpoint is unchanged after adding /depth/raw."""
import io, os, sys
import requests
from PIL import Image

DA360_URL = os.environ.get("DA360_URL", "http://127.0.0.1:5688")


def _post_depth(image_bytes):
    return requests.post(f"{DA360_URL}/depth",
                         data=image_bytes,
                         headers={"Content-Type": "image/jpeg"},
                         timeout=60)


def _jpeg_bytes():
    img = Image.new("RGB", (1036, 518), (100, 150, 200))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def test_depth_returns_200():
    """Original /depth endpoint still works."""
    resp = _post_depth(_jpeg_bytes())
    assert resp.status_code == 200, f"HTTP {resp.status_code}: {resp.text}"


def test_depth_has_required_fields():
    """/depth response includes depth_image, depth_scale, latency_ms, model, etc."""
    resp = _post_depth(_jpeg_bytes())
    data = resp.json()
    for field in ("depth_image", "depth_scale", "latency_ms", "model", "device",
                  "width", "height", "timings_ms"):
        assert field in data, f"missing field: {field}"


def test_depth_image_is_base64_data_url():
    """depth_image field is a valid base64 data URL."""
    resp = _post_depth(_jpeg_bytes())
    img = resp.json()["depth_image"]
    assert img.startswith("data:image/"), f"not a data URL: {img[:50]}"
    assert "base64" in img


def test_depth_scale_has_metadata():
    """depth_scale includes valid flag, unit, min, max."""
    resp = _post_depth(_jpeg_bytes())
    ds = resp.json()["depth_scale"]
    for field in ("valid", "unit", "min", "max", "near", "far"):
        assert field in ds, f"missing depth_scale.{field}"


def test_depth_accepts_multiple_content_types():
    """/depth accepts image/jpeg, multipart upload, and JSON-wrapped base64."""
    jpg = _jpeg_bytes()
    # image/jpeg
    r1 = _post_depth(jpg)
    assert r1.status_code == 200
    # multipart
    r2 = requests.post(f"{DA360_URL}/depth",
                       files={"image": ("test.jpg", jpg, "image/jpeg")}, timeout=60)
    assert r2.status_code == 200
    # JSON-wrapped base64
    import base64
    b64 = base64.b64encode(jpg).decode()
    r3 = requests.post(f"{DA360_URL}/depth",
                       json={"image": f"data:image/jpeg;base64,{b64}"}, timeout=60)
    assert r3.status_code == 200


def test_health_endpoint():
    """/health returns ok=True with model and device info."""
    resp = requests.get(f"{DA360_URL}/health", timeout=10)
    assert resp.status_code == 200
    data = resp.json()
    assert data["ok"] is True
    assert "model" in data
    assert "device" in data
