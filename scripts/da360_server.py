#!/usr/bin/env python3
"""Flask API for DA360 panoramic depth inference."""

import argparse
import base64
import io
import json
import math
import os
import sys
import threading
import time
from contextlib import nullcontext
from pathlib import Path

try:
    import numpy as np
    from PIL import Image, ImageOps
except ImportError as exc:
    raise SystemExit(
        "Missing DA360 API dependencies. Install at least: "
        "pip install numpy pillow flask torch torchvision opencv-python timm"
    ) from exc

try:
    from flask import Flask, jsonify, request
except ImportError as exc:
    raise SystemExit("Missing Flask. Install with: pip install flask") from exc
from werkzeug.exceptions import HTTPException

try:
    import torch
except ImportError:
    # Flask contract and security tests use fake runners and intentionally do
    # not install the multi-gigabyte CUDA stack.  Real runner construction
    # below still fails fast with an actionable error.
    torch = None


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DA360_ROOT = Path(os.environ.get("DA360_ROOT", PROJECT_ROOT / "third_party" / "DA360")).resolve()
DEFAULT_MODEL_NAME = os.environ.get("DA360_MODEL", "large")
DEFAULT_MODEL = Path(os.environ.get(
    "DA360_MODEL_PATH",
    DA360_ROOT / "checkpoints" / f"DA360_{DEFAULT_MODEL_NAME}.pth",
))
PATCH_SIZE = 14
DEFAULT_INPUT_SCALE = 0.46
API_VERSION = 2
DEFAULT_MAX_CONTENT_LENGTH = 8 * 1024 * 1024
DEFAULT_ALLOWED_ORIGINS = (
    "http://127.0.0.1:8080",
    "http://localhost:8080",
)
PROJECTION_CONFIG_HEADER = "X-Projection-Config"
PROJECTION_INTEGER_FIELDS = (
    "width", "height", "faceSize", "rgbWidth", "rgbHeight",
)
PROJECTION_FLOAT_FIELDS = (
    "verticalFovDeg", "faceFovDeg", "topPoleGuardDeg",
    "bottomPoleGuardDeg", "jpegQuality", "uploadScale",
)


def env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", ""}


def env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def env_int(name, default):
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _normalize_projection_config(value, label="projection config"):
    """Validate and canonicalize the RGB/ERP settings bound to calibration."""
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    required = set(PROJECTION_INTEGER_FIELDS + PROJECTION_FLOAT_FIELDS)
    missing = sorted(required - set(value))
    if missing:
        raise ValueError(f"{label} is incomplete: " + ", ".join(missing))

    normalized = {}
    for field in PROJECTION_INTEGER_FIELDS:
        raw = value[field]
        if isinstance(raw, bool):
            raise ValueError(f"{label}.{field} must be a positive integer")
        try:
            parsed = int(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label}.{field} must be a positive integer") from exc
        if parsed <= 0 or float(raw) != parsed:
            raise ValueError(f"{label}.{field} must be a positive integer")
        normalized[field] = parsed

    for field in PROJECTION_FLOAT_FIELDS:
        try:
            parsed = float(value[field])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label}.{field} must be finite") from exc
        if not math.isfinite(parsed):
            raise ValueError(f"{label}.{field} must be finite")
        normalized[field] = parsed

    for field in ("verticalFovDeg", "faceFovDeg"):
        if not 0 < normalized[field] <= 180:
            raise ValueError(f"{label}.{field} must be in (0, 180]")
    for field in ("topPoleGuardDeg", "bottomPoleGuardDeg"):
        if not 0 <= normalized[field] < 90:
            raise ValueError(f"{label}.{field} must be in [0, 90)")
    if not 0 < normalized["jpegQuality"] <= 1:
        raise ValueError(f"{label}.jpegQuality must be in (0, 1]")
    if normalized["uploadScale"] <= 0:
        raise ValueError(f"{label}.uploadScale must be positive")
    return normalized


def _allowed_origins():
    configured = os.environ.get("DA360_ALLOWED_ORIGINS", "")
    if configured.strip():
        return {item.strip().rstrip("/") for item in configured.split(",") if item.strip()}
    return set(DEFAULT_ALLOWED_ORIGINS)


def configure_api_security(app):
    """Apply the same request-size and local-origin policy to every API app."""
    app.config["MAX_CONTENT_LENGTH"] = env_int(
        "DA360_MAX_CONTENT_LENGTH", DEFAULT_MAX_CONTENT_LENGTH
    )
    allowed = _allowed_origins()
    identity_headers = (
        "X-Frame-ID",
        "X-Session-ID",
        "X-Capture-ID",
        "X-Location-ID",
        "X-Goal-ID",
        "X-Generation",
        PROJECTION_CONFIG_HEADER,
    )
    exposed_headers = identity_headers + (
        "X-DA360-Model",
        "X-DA360-Width",
        "X-DA360-Height",
        "X-DA360-Latency-Ms",
    )

    @app.before_request
    def reject_untrusted_origin():
        origin = request.headers.get("Origin", "").rstrip("/")
        if origin and origin not in allowed:
            return jsonify({"error": "origin not allowed"}), 403
        return None

    @app.after_request
    def add_api_headers(response):
        origin = request.headers.get("Origin", "").rstrip("/")
        if origin and origin in allowed:
            response.headers["Access-Control-Allow-Origin"] = origin
            response.headers.add("Vary", "Origin")
            response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = (
                "Content-Type, " + ", ".join(identity_headers)
            )
            response.headers["Access-Control-Expose-Headers"] = ", ".join(
                exposed_headers
            )
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Cache-Control"] = "no-store"
        return response

    return app


def load_torch_checkpoint(path, device):
    """Load tensor-only checkpoints unless unsafe pickle is explicitly enabled."""
    try:
        return torch.load(path, map_location=device, weights_only=True)
    except TypeError as exc:
        if not env_bool("DA360_ALLOW_UNSAFE_CHECKPOINT", False):
            raise RuntimeError(
                "this PyTorch version does not support safe weights_only loading; "
                "upgrade PyTorch or explicitly set DA360_ALLOW_UNSAFE_CHECKPOINT=1 "
                "for a verified local checkpoint"
            ) from exc
        print("[DA360] WARNING: unsafe checkpoint pickle loading explicitly enabled", file=sys.stderr)
        return torch.load(path, map_location=device, weights_only=False)
    except Exception as exc:
        if not env_bool("DA360_ALLOW_UNSAFE_CHECKPOINT", False):
            raise RuntimeError(
                "DA360 checkpoint was rejected by the safe tensor-only loader; "
                "set DA360_ALLOW_UNSAFE_CHECKPOINT=1 only for a verified local file"
            ) from exc
        print("[DA360] WARNING: unsafe checkpoint pickle loading explicitly enabled", file=sys.stderr)
        return torch.load(path, map_location=device, weights_only=False)


def decode_data_url(data_url):
    if not data_url:
        raise ValueError("empty image")
    if "," in data_url:
        data_url = data_url.split(",", 1)[1]
    try:
        raw = base64.b64decode(data_url, validate=True)
        image = Image.open(io.BytesIO(raw))
        return _validate_input_image(image)
    except ValueError:
        raise
    except Exception as exc:
        raise ValueError(f"invalid image: {exc}") from exc


def _validate_input_image(image):
    width, height = image.size
    maximum_pixels = env_int("DA360_MAX_IMAGE_PIXELS", 16 * 1024 * 1024)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if width * height > maximum_pixels:
        raise ValueError(
            f"decoded image is too large: {width}x{height} > {maximum_pixels} pixels"
        )
    try:
        return ImageOps.exif_transpose(image).convert("RGB")
    except Exception as exc:
        raise ValueError(f"invalid image: {exc}") from exc


def decode_request_image(req):
    if req.files:
        first_file = next(iter(req.files.values()))
        try:
            image = Image.open(first_file.stream)
            return _validate_input_image(image)
        except ValueError:
            raise
        except Exception as exc:
            raise ValueError(f"invalid image: {exc}") from exc

    content_type = (req.content_type or "").split(";", 1)[0].strip().lower()
    if content_type.startswith("image/") or content_type == "application/octet-stream":
        try:
            image = Image.open(io.BytesIO(req.get_data()))
            return _validate_input_image(image)
        except ValueError:
            raise
        except Exception as exc:
            raise ValueError(f"invalid image: {exc}") from exc

    is_json = content_type == "application/json" or content_type.endswith("+json")
    if not is_json:
        raise ValueError("No image data received")
    try:
        data = req.get_json(silent=False)
    except Exception as exc:  # pylint: disable=broad-except
        raise ValueError(f"Invalid JSON body: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("JSON body must be an object")
    if "image" not in data:
        raise ValueError("No image data received")
    return decode_data_url(data["image"])


def image_to_tensor(image, width, height, device, mean, std, resample, channels_last=False):
    if image.size != (width, height):
        image = image.resize((width, height), resample)
    arr = np.asarray(image, dtype=np.float32) / 255.0
    tensor = torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0)
    tensor = tensor.to(device, non_blocking=True)
    tensor = (tensor - mean) / std
    if channels_last:
        tensor = tensor.contiguous(memory_format=torch.channels_last)
    return tensor


def depth_to_color(depth, sample_limit=65536):
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        return np.zeros((*depth.shape, 3), dtype=np.uint8), {
            "valid": False,
            "unit": "relative_to_nearest",
        }

    valid_values = depth[valid]
    if valid_values.size > sample_limit:
        valid_values = valid_values[::max(1, int(np.ceil(valid_values.size / sample_limit)))]
    near = np.percentile(valid_values, 2.0)
    far = np.percentile(valid_values, 98.0)
    if not np.isfinite(near) or not np.isfinite(far) or far <= near:
        near = float(depth[valid].min())
        far = float(depth[valid].max() + 1e-6)

    t = 1.0 - (np.clip(depth, near, far) - near) / max(far - near, 1e-6)
    t = np.clip(t, 0.0, 1.0)
    stops = np.array([
        [4, 3, 30],
        [20, 25, 210],
        [0, 210, 255],
        [92, 255, 120],
        [255, 238, 67],
        [255, 64, 43],
        [210, 38, 255],
    ], dtype=np.float32)
    scaled = t * (len(stops) - 1)
    lo = np.floor(scaled).astype(np.int32)
    hi = np.clip(lo + 1, 0, len(stops) - 1)
    frac = (scaled - lo)[..., None]
    color = stops[lo] * (1.0 - frac) + stops[hi] * frac
    color[~valid] = 0
    full_valid_values = depth[valid]
    scale = {
        "valid": True,
        "unit": "relative_to_nearest",
        "nearest": 1.0,
        "near": float(near),
        "far": float(far),
        "min": float(full_valid_values.min()),
        "max": float(full_valid_values.max()),
        "near_percentile": 2.0,
        "far_percentile": 98.0,
    }
    return color.astype(np.uint8), scale


def depth_to_polar_scan(
    depth,
    depth_mode="da360-relative",
    angular_bins=96,
    pitch_band_deg=12.0,
    distance_percentile=20.0,
    vertical_fov_deg=180.0,
):
    """Reduce one ERP depth frame to a compact horizontal clearance scan.

    The scan is derived from the numerical DA360 output, never from the
    coloured preview JPEG.  Each angular bin reports a robust near surface in
    a narrow band around the horizon.  Slant ranges are projected onto the
    horizontal plane before aggregation.

    Relative DA360 output is normalized by this frame's second-percentile
    depth and is therefore explicitly labelled ``x-near-reference``.  Only
    validated metric/truth modes are labelled in metres.
    """
    values = np.asarray(depth, dtype=np.float32)
    if values.ndim != 2 or values.shape[0] <= 0 or values.shape[1] <= 0:
        raise ValueError("polar scan requires a non-empty 2-D depth array")
    if not isinstance(angular_bins, int) or angular_bins <= 0:
        raise ValueError("polar scan angular_bins must be a positive integer")
    if not np.isfinite(pitch_band_deg) or not 0 < pitch_band_deg < 90:
        raise ValueError("polar scan pitch band must be between 0 and 90 degrees")
    if not np.isfinite(distance_percentile) or not 0 <= distance_percentile <= 100:
        raise ValueError("polar scan percentile must be in [0, 100]")
    if not np.isfinite(vertical_fov_deg) or not 0 < vertical_fov_deg <= 180:
        raise ValueError("polar scan vertical FOV must be in (0, 180]")

    height, width = values.shape
    bin_count = min(angular_bins, width)
    row_pitch_deg = (
        float(vertical_fov_deg) / 2.0
        - (np.arange(height, dtype=np.float32) + 0.5)
        * (float(vertical_fov_deg) / height)
    )
    row_indices = np.flatnonzero(np.abs(row_pitch_deg) <= pitch_band_deg)
    if row_indices.size == 0:
        # Very small contract-test tensors may have no centre inside the
        # configured band.  Select the closest row(s), symmetrically on even H.
        nearest_count = min(height, 2 if height % 2 == 0 else 1)
        row_indices = np.argsort(np.abs(row_pitch_deg))[:nearest_count]

    horizontal_factor = np.cos(np.deg2rad(row_pitch_deg[row_indices])).reshape(-1, 1)
    horizontal_depth = values[row_indices, :] * horizontal_factor
    horizontal_valid = np.isfinite(horizontal_depth) & (horizontal_depth > 0)

    metric = depth_mode in {"da360-metric", "cesium-truth"}
    reference = 1.0
    if not metric:
        # Normalize the same horizontal quantity that the scan publishes, so
        # ``1x`` remains a truthful nearest-reference distance.
        if np.any(horizontal_valid):
            reference = float(np.percentile(horizontal_depth[horizontal_valid], 2.0))
        if not np.isfinite(reference) or reference <= 0:
            reference = 1.0

    # Vectorized sampling keeps the visualization off the 15 Hz critical path.
    # At production resolution, three adjacent ERP columns contribute to each
    # 3.75-degree bin; tiny test tensors use one column and avoid overlap.
    centre_columns = np.floor(
        (np.arange(bin_count, dtype=np.float32) + 0.5) * width / bin_count
    ).astype(np.int64)
    half_width = max(0, int(np.floor((width / bin_count) * 0.25)))
    column_offsets = np.arange(-half_width, half_width + 1, dtype=np.int64)
    sample_columns = (centre_columns[:, None] + column_offsets[None, :]) % width
    samples = horizontal_depth[:, sample_columns].transpose(0, 2, 1).reshape(-1, bin_count)
    sample_valid = np.isfinite(samples) & (samples > 0)
    valid_counts = sample_valid.sum(axis=0)
    # Ignore invalid values without np.nanpercentile's per-column Python loop.
    # Linear rank interpolation matches NumPy's default percentile semantics.
    ordered = np.sort(np.where(sample_valid, samples, np.inf), axis=0)
    ranks = (np.maximum(valid_counts, 1) - 1) * (distance_percentile / 100.0)
    lower = np.floor(ranks).astype(np.int64)
    upper = np.ceil(ranks).astype(np.int64)
    fraction = ranks - lower
    bin_indices = np.arange(bin_count)
    distances = np.full(bin_count, np.nan, dtype=np.float64)
    valid_bin_indices = bin_indices[valid_counts > 0]
    distances[valid_bin_indices] = (
        ordered[lower[valid_bin_indices], valid_bin_indices]
        * (1.0 - fraction[valid_bin_indices])
        + ordered[upper[valid_bin_indices], valid_bin_indices]
        * fraction[valid_bin_indices]
    )
    if not metric:
        distances = distances / reference
    # YOPO ERP columns run from yaw +pi toward -pi as u increases:
    #   u=0 -> back/+pi, W/4 -> body-left/+pi/2,
    #   W/2 -> front/0, 3W/4 -> body-right/-pi/2.
    # The compact polar schema deliberately uses increasing body-left angles
    # (-pi -> +pi), so reverse only this visualization array.  The full depth
    # tensor passed to YOPO remains in its native training layout.
    scan_values = [
        round(float(distance), 3)
        if valid_counts[index] > 0 and np.isfinite(distance) and distance > 0
        else None
        for index, distance in reversed(list(enumerate(distances)))
    ]
    valid_bins = sum(item is not None for item in scan_values)

    angle_step_deg = 360.0 / bin_count
    return {
        "schema_version": 1,
        "depth_mode": depth_mode,
        "unit": "metres" if metric else "x-near-reference",
        "radius": 20.0,
        "angle_start_deg": -180.0 + angle_step_deg * 0.5,
        "angle_step_deg": angle_step_deg,
        "angle_positive": "body-left",
        "pitch_band_deg": [-float(pitch_band_deg), float(pitch_band_deg)],
        "vertical_fov_deg": float(vertical_fov_deg),
        "distance_percentile": float(distance_percentile),
        "normalization": None if metric else "per-frame-depth-p02",
        "valid_fraction": round(valid_bins / bin_count, 4),
        "values": scan_values,
    }


def encode_image(image, output_format="jpeg", jpeg_quality=72):
    out = io.BytesIO()
    fmt = (output_format or "jpeg").lower()
    if fmt in {"jpg", "jpeg"}:
        Image.fromarray(image).save(out, format="JPEG", quality=int(jpeg_quality), optimize=False)
        mime = "image/jpeg"
    else:
        Image.fromarray(image).save(out, format="PNG")
        mime = "image/png"
    return f"data:{mime};base64," + base64.b64encode(out.getvalue()).decode("ascii")


def resolve_input_size(base_width, base_height, input_scale=None, input_width=None, input_height=None):
    base_w_patches = max(1, int(round(base_width / PATCH_SIZE)))
    base_h_patches = max(1, int(round(base_height / PATCH_SIZE)))

    if input_width and input_height:
        width_patches = max(1, int(round(input_width / PATCH_SIZE)))
        height_patches = max(1, int(round(input_height / PATCH_SIZE)))
    elif input_width:
        width_patches = max(1, int(round(input_width / PATCH_SIZE)))
        height_patches = max(1, int(round(width_patches * base_h_patches / base_w_patches)))
    elif input_height:
        height_patches = max(1, int(round(input_height / PATCH_SIZE)))
        width_patches = max(1, int(round(height_patches * base_w_patches / base_h_patches)))
    else:
        scale = max(0.2, min(1.0, float(input_scale if input_scale is not None else 1.0)))
        height_patches = max(1, int(round(base_h_patches * scale)))
        width_patches = max(1, int(round(height_patches * base_w_patches / base_h_patches)))

    width_patches = min(base_w_patches, max(16, width_patches))
    height_patches = min(base_h_patches, max(8, height_patches))
    return width_patches * PATCH_SIZE, height_patches * PATCH_SIZE


class DA360Runner:
    def __init__(self, model_path, input_scale=DEFAULT_INPUT_SCALE, input_width=None, input_height=None):
        if torch is None:
            raise RuntimeError("Missing PyTorch. Install DA360 dependencies before starting inference.")
        if not DA360_ROOT.is_dir():
            raise FileNotFoundError(f"DA360 repo is missing: {DA360_ROOT}")
        if not Path(model_path).is_file():
            raise FileNotFoundError(f"DA360 checkpoint is missing: {model_path}")

        sys.path.insert(0, str(DA360_ROOT))
        import networks  # pylint: disable=import-error,import-outside-toplevel

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
            try:
                torch.set_float32_matmul_precision("high")
            except Exception as exc:  # pylint: disable=broad-except
                print(f"[DA360] set_float32_matmul_precision failed: {exc}", file=sys.stderr)

        checkpoint = load_torch_checkpoint(model_path, self.device)
        checkpoint.setdefault("net", "DA360")
        checkpoint.setdefault("dinov2_encoder", "vits")
        checkpoint.setdefault("height", 518)
        checkpoint.setdefault("width", 1036)
        self.checkpoint_height = int(checkpoint["height"])
        self.checkpoint_width = int(checkpoint["width"])
        self.width, self.height = resolve_input_size(
            self.checkpoint_width,
            self.checkpoint_height,
            input_scale=input_scale,
            input_width=input_width,
            input_height=input_height,
        )
        self.input_scale = self.width / max(1, self.checkpoint_width)

        net_cls = getattr(networks, checkpoint["net"])
        self.model = net_cls(
            self.height,
            self.width,
            dinov2_encoder=checkpoint["dinov2_encoder"],
        ).to(self.device)
        model_state = self.model.state_dict()
        compatible_state = {}
        for key, value in checkpoint.items():
            if key not in model_state or not hasattr(value, "shape"):
                continue
            if tuple(value.shape) == tuple(model_state[key].shape):
                compatible_state[key] = value
        total_numel = sum(value.numel() for value in model_state.values())
        loaded_numel = sum(model_state[key].numel() for key in compatible_state)
        self.checkpoint_coverage = loaded_numel / max(1, total_numel)
        minimum_coverage = env_float("DA360_MIN_CHECKPOINT_COVERAGE", 0.95)
        if self.checkpoint_coverage < minimum_coverage:
            raise RuntimeError(
                "DA360 checkpoint coverage is too low: "
                f"{self.checkpoint_coverage:.2%} < {minimum_coverage:.2%} "
                f"({len(compatible_state)}/{len(model_state)} state entries matched)"
            )
        incompatible = self.model.load_state_dict(compatible_state, strict=False)
        self.checkpoint_missing_keys = len(incompatible.missing_keys)
        self.checkpoint_unexpected_keys = len(incompatible.unexpected_keys)
        self.model.eval()
        self.model_name = Path(model_path).stem
        self.use_amp = self.device.type == "cuda" and env_bool("DA360_AMP", True)
        # channels_last 在 RTX 5070 Ti + PyTorch 2.8 + Flask 主进程中会导致
        # model forward 从 ~25ms 退化到 ~5.5s（200x 减速）。docker exec 子进程不受影响。
        # 根因是 PyTorch CUDA kernel 调度与 Flask 线程模型的交互问题，非代码逻辑错误。
        # 默认禁用；若换 GPU/PyTorch 版本可重新启用。
        self.channels_last = self.device.type == "cuda" and env_bool("DA360_CHANNELS_LAST", False)
        if self.channels_last:
            self.model = self.model.to(memory_format=torch.channels_last)
        if env_bool("DA360_TORCH_COMPILE", False) and hasattr(torch, "compile"):
            self.model = torch.compile(self.model)
        # Background async compilation: if requested, start torch.compile in a
        # daemon thread so it does not block the /health check. The compiled
        # model swaps in when ready; inference falls back to eager until then.
        # Skip if sync compile was already applied (avoid double-compile).
        elif env_bool("DA360_TORCH_COMPILE_ASYNC", False) and hasattr(torch, "compile"):
            self._compiled_model = None
            self._compile_error = None
            def _compile_async():
                try:
                    compiled = torch.compile(self.model)
                    # one warmup forward in the SAME autocast context as real inference
                    warm = torch.randn(1, 3, self.height, self.width, device=self.device)
                    if self.channels_last:
                        warm = warm.contiguous(memory_format=torch.channels_last)
                    with torch.inference_mode():
                        amp_ctx = torch.cuda.amp.autocast() if self.use_amp else nullcontext()
                        with amp_ctx:
                            compiled(warm)
                    self._compiled_model = compiled
                except Exception as exc:
                    self._compile_error = exc
                    print(f"[DA360] async compile failed: {exc}", file=sys.stderr)
            t = threading.Thread(target=_compile_async, daemon=True, name="da360-compile")
            t.start()
            # Check before each inference whether the compiled model is ready
            _orig_infer = self.infer
            _orig_infer_raw = self.infer_raw
            def _infer_with_compile_swap(image):
                if self._compiled_model is not None and self.model is not self._compiled_model:
                    self.model = self._compiled_model
                return _orig_infer(image)
            def _infer_raw_with_compile_swap(image):
                if self._compiled_model is not None and self.model is not self._compiled_model:
                    self.model = self._compiled_model
                return _orig_infer_raw(image)
            self.infer = _infer_with_compile_swap
            self.infer_raw = _infer_raw_with_compile_swap
        self.mean = torch.tensor([0.485, 0.456, 0.406], device=self.device).view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225], device=self.device).view(1, 3, 1, 1)
        resample_name = os.environ.get("DA360_RESAMPLE", "bicubic").strip().lower()
        self.resample_name = "bicubic" if resample_name == "bicubic" else "bilinear"
        self.resample = Image.Resampling.BICUBIC if resample_name == "bicubic" else Image.Resampling.BILINEAR
        mode = os.environ.get("DA360_DEPTH_MODE", "da360-relative").strip().lower()
        mode_aliases = {
            "relative": "da360-relative",
            "da360-relative": "da360-relative",
            "metric": "da360-metric",
            "da360-metric": "da360-metric",
        }
        if mode not in mode_aliases:
            raise ValueError(
                "DA360_DEPTH_MODE must be da360-relative or da360-metric, "
                f"got {mode!r}"
            )
        self.depth_mode = mode_aliases[mode]
        self.calibration = self._load_depth_calibration()
        if self.depth_mode == "da360-metric" and self.calibration is None:
            raise RuntimeError(
                "DA360 metric mode requires a valid DA360_DEPTH_CALIB_PATH; "
                "refusing to silently use relative depth"
            )
        self.lock = threading.Lock()

        if os.environ.get("DA360_NO_WARMUP") != "1":
            warmup = Image.new("RGB", (self.width, self.height), (0, 0, 0))
            self.infer(warmup)

    def infer(self, image):
        import time as _time
        _t = [_time.time()]
        def _lap(label):
            _t.append(_time.time())
            if _t[-1] - _t[-2] > 0.1:  # 只打印 >100ms 的步骤
                print(f"  [da360 infer] {label}: {1000*(_t[-1]-_t[-2]):.0f}ms", flush=True)
        # CPU work (resize, H2D, normalize) outside the GPU lock so the next
        # frame's decode/resize overlaps the current frame's GPU forward.
        tensor = image_to_tensor(
            image, self.width, self.height, self.device,
            self.mean, self.std, self.resample, channels_last=self.channels_last,
        )
        _lap("image_to_tensor")
        with self.lock:
            _lap("lock-acquired")
            with torch.inference_mode():
                amp_context = torch.cuda.amp.autocast() if self.use_amp else nullcontext()
                with amp_context:
                    outputs = self.model(tensor)
            _lap("model-forward")
        disp = outputs["pred_disp"].detach().float().cpu().numpy()[0, 0]
        _lap("cpu-copy")
        depth = 1.0 / np.maximum(disp, 1e-6)
        valid = np.isfinite(depth) & (depth > 0)
        if np.any(valid):
            depth = depth / max(float(depth[valid].min()), 1e-6)
        scale = env_float("DA360_DEPTH_SCALE", 1.0)
        if abs(scale - 1.0) > 1e-6:
            depth = depth * scale
        _lap("postprocess")
        if _t[-1] - _t[0] > 0.5:
            print(f"  [da360 infer] TOTAL: {1000*(_t[-1]-_t[0]):.0f}ms  ({self.width}x{self.height})", flush=True)
        return depth

    def infer_metric(self, image, projection_config=None):
        """Run DA360 inference and convert raw pred_disp to metric depth.

        使用启动时验证并冻结的线性标定参数 1/z = a·pred_disp + b，
        跳过 per-frame min-归一化步骤。没有有效标定时直接失败，避免
        把相对深度静默冒充为米制深度。

        Returns
        -------
        np.ndarray  float32[H,W]  metric depth (metres), clamped to [min_dis, max_dis]
        """
        calib = self.calibration
        if calib is None:
            raise RuntimeError("metric inference requested without a validated calibration")
        request_size = (int(image.width), int(image.height))
        calibrated_size = (calib["request_width"], calib["request_height"])
        if request_size != calibrated_size:
            raise ValueError(
                "metric calibration input size mismatch: "
                f"{request_size[0]}x{request_size[1]} != "
                f"{calibrated_size[0]}x{calibrated_size[1]}"
            )
        runtime_projection = _normalize_projection_config(
            projection_config, "runtime projection config"
        )
        if (runtime_projection["rgbWidth"], runtime_projection["rgbHeight"]) != request_size:
            raise ValueError("runtime projection RGB dimensions do not match decoded image")
        for field, expected in calib["projection"].items():
            actual = runtime_projection[field]
            matches = actual == expected if field in PROJECTION_INTEGER_FIELDS else math.isclose(
                actual, expected, rel_tol=0, abs_tol=1e-9
            )
            if not matches:
                raise ValueError(
                    f"metric calibration projection mismatch for {field}: "
                    f"{actual!r} != {expected!r}"
                )
        raw = self.infer_raw(image)
        pred_disp = raw["pred_disp"]
        a, b = calib["a"], calib["b"]
        inverse_depth = a * pred_disp + b
        valid = np.isfinite(inverse_depth) & (inverse_depth > 1e-6)
        metric = np.full(pred_disp.shape, np.nan, dtype=np.float32)
        metric[valid] = np.clip(
            1.0 / inverse_depth[valid],
            calib["depth_min_m"],
            calib["depth_max_m"],
        )
        return metric.astype(np.float32)

    def infer_depth(self, image, projection_config=None):
        """Run the explicitly configured relative or metric depth path."""
        if self.depth_mode == "da360-metric":
            return self.infer_metric(image, projection_config)
        return self.infer(image)

    def _load_depth_calibration(self):
        """Load and validate calibration exactly once during runner startup."""
        path = os.environ.get("DA360_DEPTH_CALIB_PATH", "")
        if not path:
            return None
        try:
            calibration_path = Path(path).resolve(strict=True)
            with calibration_path.open(encoding="utf-8") as f:
                calib = json.load(f)
            if calib.get("schema_version") != 1:
                raise ValueError("unsupported or missing calibration schema_version")
            accuracy_accepted = calib.get("accepted")
            acceptance_report = calib.get("acceptance")
            if not isinstance(accuracy_accepted, bool):
                raise ValueError("calibration accepted status must be boolean")
            if not isinstance(acceptance_report, dict) or not isinstance(
                    acceptance_report.get("passed"), bool):
                raise ValueError("calibration acceptance report status must be boolean")
            automatic_gate_passed = acceptance_report["passed"]
            manual_acceptance = calib.get("manual_acceptance")
            manual_accepted = False
            acceptance_method = "automatic" if automatic_gate_passed else None
            acceptance_scope = "accuracy-gates" if automatic_gate_passed else None
            if manual_acceptance is not None:
                if not isinstance(manual_acceptance, dict):
                    raise ValueError("calibration manual_acceptance must be an object")
                manual_accepted = manual_acceptance.get("accepted") is True
                if manual_acceptance.get("accepted") is not True:
                    raise ValueError("calibration manual acceptance must be explicitly true")
                required_manual_fields = {
                    "accepted_by": manual_acceptance.get("accepted_by"),
                    "accepted_at": manual_acceptance.get("accepted_at"),
                    "scope": manual_acceptance.get("scope"),
                    "basis": manual_acceptance.get("basis"),
                }
                invalid_manual_fields = [
                    field for field, value in required_manual_fields.items()
                    if not isinstance(value, str) or not value.strip()
                ]
                if invalid_manual_fields:
                    raise ValueError(
                        "calibration manual acceptance is missing non-empty fields: "
                        + ", ".join(invalid_manual_fields)
                    )
                if manual_acceptance["scope"] != "sim-to-sim":
                    raise ValueError(
                        "calibration manual acceptance scope must be sim-to-sim"
                    )
                acceptance_method = "manual-user"
                acceptance_scope = manual_acceptance["scope"]
            provenance_accepted = automatic_gate_passed or manual_accepted
            if accuracy_accepted is not provenance_accepted:
                raise ValueError("calibration acceptance statuses disagree")
            expected_relation = "inverse_depth_1_per_m = a * pred_disp + b"
            if calib.get("relation") != expected_relation:
                raise ValueError("calibration inverse-depth relation is missing or incompatible")
            a = float(calib["a"])
            b = float(calib["b"])
            min_depth = float(calib.get("depth_min_m", 0.04))
            max_depth = float(calib.get("depth_max_m", 20.0))
            if not all(math.isfinite(value) for value in (a, b, min_depth, max_depth)):
                raise ValueError("a, b and depth limits must be finite")
            if a <= 0:
                raise ValueError("calibration slope a must be positive")
            if min_depth <= 0 or max_depth <= min_depth:
                raise ValueError("calibration depth range must satisfy 0 < min < max")

            context = calib.get("input", {})
            if not isinstance(context, dict):
                raise ValueError("calibration input contract must be an object")
            expected = {
                "model": self.model_name,
                "width": self.width,
                "height": self.height,
                "resample": self.resample_name,
            }
            observed = {
                key: calib.get(key, context.get(key)) for key in expected
            }
            missing = [key for key, value in observed.items() if value is None]
            if missing:
                raise ValueError(
                    "calibration is missing inference contract fields: "
                    + ", ".join(missing)
                )
            for key, expected_value in expected.items():
                actual = observed[key]
                if key in {"width", "height"}:
                    actual = int(actual)
                else:
                    actual = str(actual)
                if actual != expected_value:
                    raise ValueError(
                        f"calibration {key} mismatch: {actual!r} != {expected_value!r}"
                    )

            request_width = int(calib.get("requestWidth", context.get("request_width", 0)))
            request_height = int(calib.get("requestHeight", context.get("request_height", 0)))
            if request_width <= 0 or request_height <= 0:
                raise ValueError("calibration requestWidth/requestHeight must be positive")
            if int(context.get("request_width", 0)) != request_width \
                    or int(context.get("request_height", 0)) != request_height:
                raise ValueError("calibration request dimensions disagree with input contract")

            projection = _normalize_projection_config(
                calib.get("projection"), "calibration projection contract"
            )
            if int(projection["rgbWidth"]) != request_width \
                    or int(projection["rgbHeight"]) != request_height:
                raise ValueError("calibration projection RGB dimensions mismatch")
            expected_upload_scale = request_width / int(projection["width"])
            if not math.isclose(
                    projection["uploadScale"], expected_upload_scale,
                    rel_tol=0, abs_tol=1e-12):
                raise ValueError("calibration uploadScale does not match request/panorama width")

            declared_id = str(calib.get("calibration_id", "")).strip()
            if declared_id and not all(
                    character.isalnum() or character in {"-", "_", "."}
                    for character in declared_id):
                raise ValueError("calibration_id contains unsupported characters")
            calibration_id = declared_id or (
                f"{calibration_path.stem}-v{calib['schema_version']}-"
                f"{self.model_name}-{request_width}x{request_height}"
            )
            return {
                "a": a,
                "b": b,
                "depth_min_m": min_depth,
                "depth_max_m": max_depth,
                "id": calibration_id,
                "version": calib["schema_version"],
                "request_width": request_width,
                "request_height": request_height,
                "projection": projection,
                # Accuracy acceptance is provenance, not a runtime gate.  This
                # allows an explicitly selected metric candidate to be tested
                # without ever treating raw relative depth as metric input.
                "accuracy_accepted": accuracy_accepted,
                "automatic_accuracy_gate_passed": automatic_gate_passed,
                "acceptance_method": acceptance_method,
                "acceptance_scope": acceptance_scope,
            }
        except Exception as exc:
            raise RuntimeError(f"invalid DA360 calibration {path}: {exc}") from exc

    def infer_raw(self, image):
        """Return raw pred_disp without per-frame min-normalization.

        Returns
        -------
        dict with keys:
            pred_disp       — float32[H,W] raw disparity from DA360 (no per-frame rescale)
            relative_depth  — float32[H,W] = 1 / max(pred_disp, epsilon); NOT divided by min
            valid_mask      — bool[H,W]   finite(pred_disp) & pred_disp > epsilon
            metadata         — dict with model info and inference context
        """
        # CPU work outside GPU lock (same as infer())
        tensor = image_to_tensor(
            image,
            self.width,
            self.height,
            self.device,
            self.mean,
            self.std,
            self.resample,
            channels_last=self.channels_last,
        )
        with self.lock:
            with torch.inference_mode():
                amp_context = torch.cuda.amp.autocast() if self.use_amp else nullcontext()
                with amp_context:
                    outputs = self.model(tensor)
        disp = outputs["pred_disp"].detach().float().cpu().numpy()[0, 0]

        eps = np.float32(1e-6)
        valid = np.isfinite(disp) & (disp > eps)
        rel_depth = np.where(valid, np.float32(1.0) / np.maximum(disp, eps), np.float32(0.0))

        return {
            "pred_disp": disp,
            "relative_depth": rel_depth,
            "valid_mask": valid.astype(np.uint8),
            "metadata": {
                "model": self.model_name,
                "device": str(self.device),
                "width": self.width,
                "height": self.height,
                "checkpoint_width": self.checkpoint_width,
                "checkpoint_height": self.checkpoint_height,
                "input_scale": self.input_scale,
                "resample": self.resample_name,
                "amp": self.use_amp,
                "depth_mode": self.depth_mode,
                "calibration_id": self.calibration["id"] if self.calibration else None,
                "epsilon": float(eps),
                "unit_pred_disp": "raw disparity (inverse depth), NOT per-frame normalized",
                "unit_relative_depth": "1/pred_disp (not divided by frame min)",
            },
        }


def _runner_calibration_id(runner):
    calibration = getattr(runner, "calibration", None)
    return calibration.get("id") if calibration else None


def runner_health_payload(runner):
    """Build the stable health contract shared by standalone and combined."""
    calibration = getattr(runner, "calibration", None)
    return {
        "ok": True,
        "api_version": API_VERSION,
        "model": runner.model_name,
        "device": str(runner.device),
        "width": runner.width,
        "height": runner.height,
        "checkpoint_width": getattr(runner, "checkpoint_width", runner.width),
        "checkpoint_height": getattr(runner, "checkpoint_height", runner.height),
        "input_scale": runner.input_scale,
        "resample": runner.resample_name,
        "amp": runner.use_amp,
        "channels_last": runner.channels_last,
        "depth_scale": env_float("DA360_DEPTH_SCALE", 1.0),
        "depth_mode": getattr(runner, "depth_mode", "da360-relative"),
        "calibration": {
            "loaded": calibration is not None,
            "id": _runner_calibration_id(runner),
            "accuracy_accepted": calibration.get("accuracy_accepted")
                if calibration else None,
            "automatic_accuracy_gate_passed": calibration.get(
                "automatic_accuracy_gate_passed"
            ) if calibration else None,
            "acceptance_method": calibration.get("acceptance_method")
                if calibration else None,
            "acceptance_scope": calibration.get("acceptance_scope")
                if calibration else None,
            "request_width": calibration.get("request_width") if calibration else None,
            "request_height": calibration.get("request_height") if calibration else None,
            "version": calibration.get("version") if calibration else None,
        },
        "checkpoint": {
            "coverage": getattr(runner, "checkpoint_coverage", None),
            "missing_keys": getattr(runner, "checkpoint_missing_keys", None),
            "unexpected_keys": getattr(runner, "checkpoint_unexpected_keys", None),
        },
    }


def _request_image_metadata(image):
    projection_header = request.headers.get(PROJECTION_CONFIG_HEADER)
    projection_config = None
    if projection_header is not None:
        if len(projection_header) > 4096:
            raise ValueError("runtime projection config header is too large")
        try:
            projection_payload = json.loads(projection_header)
        except json.JSONDecodeError as exc:
            raise ValueError("runtime projection config must be valid JSON") from exc
        projection_config = _normalize_projection_config(
            projection_payload, "runtime projection config"
        )
    metadata = {
        "frame_id": request.args.get("frame_id") or request.headers.get("X-Frame-ID"),
        "session_id": request.args.get("session_id") or request.headers.get("X-Session-ID"),
        "capture_id": request.args.get("capture_id") or request.headers.get("X-Capture-ID"),
        "location_id": request.args.get("location_id") or request.headers.get("X-Location-ID"),
        "goal_id": request.args.get("goal_id") or request.headers.get("X-Goal-ID"),
        "generation": request.args.get("generation") or request.headers.get("X-Generation"),
        "request_width": image.width,
        "request_height": image.height,
        "projection_config": projection_config,
    }
    return metadata


def _infer_configured_depth(runner, image, request_metadata=None):
    infer_depth = getattr(runner, "infer_depth", None)
    if infer_depth is None:
        return runner.infer(image)
    projection_config = (request_metadata or {}).get("projection_config")
    return infer_depth(image, projection_config)


def register_depth_routes(app, runner_provider, on_depth=None, endpoint_prefix="da360"):
    """Register the DA360 contract on a Flask app.

    ``runner_provider`` is a callable so the combined service can register its
    routes before the two GPU runners finish loading. ``on_depth`` receives the
    predicted array and immutable request metadata and is used only by the
    legacy cached ``/yopo/plan`` path.
    """
    def current_runner():
        runner = runner_provider()
        if runner is None:
            return None
        return runner

    def health():
        runner = current_runner()
        if runner is None:
            return jsonify({"ok": False, "error": "DA360 not initialized"}), 503
        return jsonify(runner_health_payload(runner))

    def depth_raw():
        if request.method == "OPTIONS":
            return ("", 204)
        runner = current_runner()
        if runner is None:
            return jsonify({"error": "DA360 not initialized"}), 503
        started = time.perf_counter()
        try:
            image = decode_request_image(request)
            request_metadata = _request_image_metadata(image)
            raw = runner.infer_raw(image)
            metadata = dict(raw["metadata"])
            metadata.update(request_metadata)
            metadata.update({
                "api_version": API_VERSION,
                "depth_mode": getattr(runner, "depth_mode", "da360-relative"),
                "calibration_id": _runner_calibration_id(runner),
            })
            buf = io.BytesIO()
            np.savez_compressed(
                buf,
                pred_disp=np.asarray(raw["pred_disp"], dtype=np.float32),
                relative_depth=np.asarray(raw["relative_depth"], dtype=np.float32),
                valid_mask=np.asarray(raw["valid_mask"], dtype=np.uint8),
                metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
            )
            latency_ms = (time.perf_counter() - started) * 1000.0
            frame_token = str(request_metadata["frame_id"] or "unassigned")
            frame_token = "".join(
                character if character.isalnum() or character in {"-", "_"} else "_"
                for character in frame_token[:80]
            )
            def response_token(field):
                value = str(request_metadata[field] or "")[:128]
                return "".join(
                    character if character.isalnum() or character in {"-", "_", "."} else "_"
                    for character in value
                )
            filename = f"depth_raw_{frame_token}.npz"
            return app.response_class(
                buf.getvalue(),
                status=200,
                mimetype="application/x-npz",
                headers={
                    "Content-Disposition": f"attachment; filename={filename}",
                    "X-DA360-Model": str(metadata["model"]),
                    "X-DA360-Width": str(metadata["width"]),
                    "X-DA360-Height": str(metadata["height"]),
                    "X-DA360-Latency-Ms": str(latency_ms),
                    "X-Frame-ID": frame_token if request_metadata["frame_id"] else "",
                    "X-Session-ID": response_token("session_id"),
                    "X-Capture-ID": response_token("capture_id"),
                    "X-Location-ID": response_token("location_id"),
                },
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except HTTPException:
            raise
        except Exception as exc:  # pylint: disable=broad-except
            print(f"[DA360] raw inference failed: {exc}", file=sys.stderr)
            return jsonify({"error": "raw depth inference failed"}), 500

    def depth():
        if request.method == "OPTIONS":
            return ("", 204)
        runner = current_runner()
        if runner is None:
            return jsonify({"error": "DA360 not initialized"}), 503
        started = time.perf_counter()
        try:
            timings = {}
            mark = time.perf_counter()
            image = decode_request_image(request)
            request_metadata = _request_image_metadata(image)
            timings["decode_ms"] = (time.perf_counter() - mark) * 1000.0
            mark = time.perf_counter()
            pred_depth = _infer_configured_depth(runner, image, request_metadata)
            timings["infer_ms"] = (time.perf_counter() - mark) * 1000.0
            if on_depth is not None:
                on_depth(pred_depth, request_metadata)
            mark = time.perf_counter()
            polar_scan = depth_to_polar_scan(
                pred_depth,
                getattr(runner, "depth_mode", "da360-relative"),
                vertical_fov_deg=(
                    request_metadata.get("projection_config") or {}
                ).get("verticalFovDeg", 180.0),
            )
            timings["polar_ms"] = (time.perf_counter() - mark) * 1000.0
            mark = time.perf_counter()
            colored, depth_scale = depth_to_color(pred_depth)
            if getattr(runner, "depth_mode", "da360-relative") == "da360-metric":
                depth_scale["unit"] = "metres"
            timings["color_ms"] = (time.perf_counter() - mark) * 1000.0
            mark = time.perf_counter()
            depth_image = encode_image(
                colored,
                os.environ.get("DA360_OUTPUT_FORMAT", "jpeg"),
                env_int("DA360_JPEG_QUALITY", 72),
            )
            timings["encode_ms"] = (time.perf_counter() - mark) * 1000.0
            return jsonify({
                "api_version": API_VERSION,
                "depth_image": depth_image,
                "depth_scale": depth_scale,
                "polar_scan": polar_scan,
                "depth_mode": getattr(runner, "depth_mode", "da360-relative"),
                "calibration_id": _runner_calibration_id(runner),
                "latency_ms": (time.perf_counter() - started) * 1000.0,
                "timings_ms": timings,
                "model": runner.model_name,
                "device": str(runner.device),
                "width": runner.width,
                "height": runner.height,
                "request_width": request_metadata["request_width"],
                "request_height": request_metadata["request_height"],
                "input_pixels": runner.width * runner.height,
                "request_pixels": request_metadata["request_width"] * request_metadata["request_height"],
                "frame_id": request_metadata["frame_id"],
                "goal_id": request_metadata["goal_id"],
                "generation": request_metadata["generation"],
            })
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except HTTPException:
            raise
        except Exception as exc:  # pylint: disable=broad-except
            print(f"[DA360] inference failed: {exc}", file=sys.stderr)
            return jsonify({"error": "depth inference failed"}), 500

    app.add_url_rule("/health", f"{endpoint_prefix}_health", health, methods=["GET"])
    app.add_url_rule(
        "/depth/raw", f"{endpoint_prefix}_depth_raw", depth_raw, methods=["POST", "OPTIONS"]
    )
    app.add_url_rule("/depth", f"{endpoint_prefix}_depth", depth, methods=["POST", "OPTIONS"])
    return app


def create_app(runner):
    app = Flask(__name__)
    configure_api_security(app)
    register_depth_routes(app, lambda: runner)
    return app


def parse_args():
    parser = argparse.ArgumentParser(description="Start the DA360 panoramic depth API.")
    parser.add_argument("--model-path", default=str(DEFAULT_MODEL), help="Path to DA360 .pth checkpoint.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=5688, type=int)
    parser.add_argument("--input-scale", default=env_float("DA360_INPUT_SCALE", DEFAULT_INPUT_SCALE), type=float)
    parser.add_argument("--input-width", default=env_int("DA360_INPUT_WIDTH", 0), type=int)
    parser.add_argument("--input-height", default=env_int("DA360_INPUT_HEIGHT", 0), type=int)
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    runner = DA360Runner(
        args.model_path,
        input_scale=args.input_scale,
        input_width=args.input_width or None,
        input_height=args.input_height or None,
    )
    app = create_app(runner)
    print(f"DA360 API running at http://127.0.0.1:{args.port}")
    print(f"Model: {args.model_path}")
    print(f"Device: {runner.device}")
    print(f"Input: {runner.width}x{runner.height} (checkpoint {runner.checkpoint_width}x{runner.checkpoint_height})")
    print(f"Depth mode: {runner.depth_mode}; calibration={_runner_calibration_id(runner) or 'none'}")
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)


if __name__ == "__main__":
    main()
