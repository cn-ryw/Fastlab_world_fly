"""Test /depth/raw endpoint: output format, dtypes, shapes, and invariants."""
import io, json, os, sys
import numpy as np
import requests
from PIL import Image

DA360_URL = os.environ.get("DA360_URL", "http://127.0.0.1:5688")


def _post_raw(image_bytes):
    return requests.post(f"{DA360_URL}/depth/raw",
                         data=image_bytes,
                         headers={"Content-Type": "image/jpeg"},
                         timeout=60)


def _jpeg_bytes(width=1036, height=518):
    img = Image.new("RGB", (width, height), (100, 150, 200))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def test_raw_returns_200():
    """Raw endpoint returns HTTP 200 for a valid JPEG."""
    resp = _post_raw(_jpeg_bytes())
    assert resp.status_code == 200, resp.text


def test_raw_content_type_npz():
    """Response Content-Type is application/x-npz."""
    resp = _post_raw(_jpeg_bytes())
    assert resp.headers.get("Content-Type") == "application/x-npz"


def test_raw_contains_required_keys():
    """NPZ must contain pred_disp, relative_depth, valid_mask, metadata_json."""
    resp = _post_raw(_jpeg_bytes())
    data = np.load(io.BytesIO(resp.content), allow_pickle=True)
    for key in ("pred_disp", "relative_depth", "valid_mask", "metadata_json"):
        assert key in data.files, f"missing key: {key}"


def test_pred_disp_dtype_shape():
    """pred_disp is float32, 2D, and NOT in [0,1]."""
    resp = _post_raw(_jpeg_bytes())
    data = np.load(io.BytesIO(resp.content), allow_pickle=True)
    pd = data["pred_disp"]
    assert pd.dtype == np.float32, f"dtype: {pd.dtype}"
    assert len(pd.shape) == 2, f"shape: {pd.shape}"
    # Not per-frame normalized: range should be far from [0,1]
    assert pd.max() > 1.0, f"pred_disp max={pd.max()} — looks per-frame normalized"
    assert pd.min() > 0.0, "pred_disp min <= 0"


def test_relative_depth_formula():
    """relative_depth = 1 / max(pred_disp, epsilon) for valid pixels."""
    resp = _post_raw(_jpeg_bytes())
    data = np.load(io.BytesIO(resp.content), allow_pickle=True)
    pd = data["pred_disp"]
    rd = data["relative_depth"]
    vm = data["valid_mask"].astype(bool)
    md = json.loads(str(data["metadata_json"]))
    eps = md["epsilon"]
    expected = np.where(vm, 1.0 / np.maximum(pd, np.float32(eps)), 0.0)
    assert np.allclose(rd, expected, rtol=1e-4), "relative_depth != 1/max(pred_disp, eps)"


def test_valid_mask_binary():
    """valid_mask is uint8 and only 0 or 1."""
    resp = _post_raw(_jpeg_bytes())
    data = np.load(io.BytesIO(resp.content), allow_pickle=True)
    vm = data["valid_mask"]
    assert vm.dtype == np.uint8, f"valid_mask dtype: {vm.dtype}"
    assert set(np.unique(vm)).issubset({0, 1}), f"non-binary values: {np.unique(vm)}"


def test_metadata_fields():
    """metadata_json contains model, device, width, height, unit descriptions."""
    resp = _post_raw(_jpeg_bytes())
    data = np.load(io.BytesIO(resp.content), allow_pickle=True)
    md = json.loads(str(data["metadata_json"]))
    for field in ("model", "device", "width", "height", "unit_pred_disp", "unit_relative_depth"):
        assert field in md, f"missing metadata field: {field}"
    assert "not per-frame normalized" in md["unit_pred_disp"].lower()
    assert "not divided by frame min" in md["unit_relative_depth"].lower()
