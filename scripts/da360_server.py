#!/usr/bin/env python3
"""Flask API for DA360 panoramic depth inference."""

import argparse
import base64
import io
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
        "pip install numpy pillow flask flask-cors torch torchvision opencv-python timm"
    ) from exc

try:
    from flask import Flask, jsonify, request
except ImportError as exc:
    raise SystemExit("Missing Flask. Install with: pip install flask flask-cors") from exc

try:
    from flask_cors import CORS
except ImportError:
    CORS = None

try:
    import torch
except ImportError as exc:
    raise SystemExit("Missing PyTorch. Install DA360 dependencies before starting this server.") from exc


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DA360_ROOT = Path(os.environ.get("DA360_ROOT", PROJECT_ROOT / "third_party" / "DA360")).resolve()
DEFAULT_MODEL_NAME = os.environ.get("DA360_MODEL", "large")
DEFAULT_MODEL = Path(os.environ.get(
    "DA360_MODEL_PATH",
    DA360_ROOT / "checkpoints" / f"DA360_{DEFAULT_MODEL_NAME}.pth",
))
PATCH_SIZE = 14
DEFAULT_INPUT_SCALE = 1.0


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


def load_torch_checkpoint(path, device):
    try:
        return torch.load(path, map_location=device, weights_only=False)
    except TypeError:
        return torch.load(path, map_location=device)


def decode_data_url(data_url):
    if not data_url:
        raise ValueError("empty image")
    if "," in data_url:
        data_url = data_url.split(",", 1)[1]
    raw = base64.b64decode(data_url)
    image = Image.open(io.BytesIO(raw))
    image = ImageOps.exif_transpose(image).convert("RGB")
    return image


def decode_request_image(req):
    if req.files:
        first_file = next(iter(req.files.values()))
        image = Image.open(first_file.stream)
        return ImageOps.exif_transpose(image).convert("RGB")

    content_type = (req.content_type or "").split(";", 1)[0].strip().lower()
    if content_type.startswith("image/") or content_type == "application/octet-stream":
        image = Image.open(io.BytesIO(req.get_data()))
        return ImageOps.exif_transpose(image).convert("RGB")

    data = req.get_json(silent=True) or {}
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
            except Exception:
                pass

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
        self.model.load_state_dict(compatible_state, strict=False)
        self.model.eval()
        self.model_name = Path(model_path).stem
        self.use_amp = self.device.type == "cuda" and env_bool("DA360_AMP", True)
        self.channels_last = self.device.type == "cuda" and env_bool("DA360_CHANNELS_LAST", True)
        if self.channels_last:
            self.model = self.model.to(memory_format=torch.channels_last)
        if env_bool("DA360_TORCH_COMPILE", False) and hasattr(torch, "compile"):
            self.model = torch.compile(self.model)
        self.mean = torch.tensor([0.485, 0.456, 0.406], device=self.device).view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225], device=self.device).view(1, 3, 1, 1)
        resample_name = os.environ.get("DA360_RESAMPLE", "bilinear").strip().lower()
        self.resample = Image.Resampling.BICUBIC if resample_name == "bicubic" else Image.Resampling.BILINEAR
        self.lock = threading.Lock()

        if os.environ.get("DA360_NO_WARMUP") != "1":
            warmup = Image.new("RGB", (self.width, self.height), (0, 0, 0))
            self.infer(warmup)

    def infer(self, image):
        with self.lock:
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
            with torch.inference_mode():
                amp_context = torch.cuda.amp.autocast() if self.use_amp else nullcontext()
                with amp_context:
                    outputs = self.model(tensor)
            disp = outputs["pred_disp"].detach().float().cpu().numpy()[0, 0]
        depth = 1.0 / np.maximum(disp, 1e-6)
        valid = np.isfinite(depth) & (depth > 0)
        if np.any(valid):
            depth = depth / max(float(depth[valid].min()), 1e-6)
        return depth


def create_app(runner):
    app = Flask(__name__)
    if CORS is not None:
        CORS(app, resources={r"/*": {"origins": "*"}})

    @app.after_request
    def add_cors_headers(response):
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return response

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify({
            "ok": True,
            "model": runner.model_name,
            "device": str(runner.device),
            "width": runner.width,
            "height": runner.height,
            "checkpoint_width": runner.checkpoint_width,
            "checkpoint_height": runner.checkpoint_height,
            "input_scale": runner.input_scale,
            "amp": runner.use_amp,
            "channels_last": runner.channels_last,
        })

    @app.route("/depth", methods=["POST", "OPTIONS"])
    def depth():
        if request.method == "OPTIONS":
            return ("", 204)
        started = time.time()

        try:
            image = decode_request_image(request)
            request_width, request_height = image.size
            pred_depth = runner.infer(image)
            colored, depth_scale = depth_to_color(pred_depth)
            return jsonify({
                "depth_image": encode_image(
                    colored,
                    os.environ.get("DA360_OUTPUT_FORMAT", "jpeg"),
                    env_int("DA360_JPEG_QUALITY", 72),
                ),
                "depth_scale": depth_scale,
                "latency_ms": (time.time() - started) * 1000.0,
                "model": runner.model_name,
                "device": str(runner.device),
                "width": runner.width,
                "height": runner.height,
                "request_width": request_width,
                "request_height": request_height,
                "input_pixels": runner.width * runner.height,
                "request_pixels": request_width * request_height,
            })
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:  # pylint: disable=broad-except
            print(f"[DA360] inference failed: {exc}", file=sys.stderr)
            return jsonify({"error": str(exc)}), 500

    return app


def parse_args():
    parser = argparse.ArgumentParser(description="Start the DA360 panoramic depth API.")
    parser.add_argument("--model-path", default=str(DEFAULT_MODEL), help="Path to DA360 .pth checkpoint.")
    parser.add_argument("--host", default="0.0.0.0")
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
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)


if __name__ == "__main__":
    main()
