#!/usr/bin/env python3
"""Flask API for DA360 panoramic depth inference."""

import argparse
import base64
import io
import json
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
DEFAULT_INPUT_SCALE = 0.65


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
        self.model.load_state_dict(compatible_state, strict=False)
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
        resample_name = os.environ.get("DA360_RESAMPLE", "bilinear").strip().lower()
        self.resample_name = "bicubic" if resample_name == "bicubic" else "bilinear"
        self.resample = Image.Resampling.BICUBIC if resample_name == "bicubic" else Image.Resampling.BILINEAR
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

    def infer_metric(self, image):
        """Run DA360 inference and convert raw pred_disp to metric depth.

        使用离线拟合的线性标定参数 1/z = a·pred_disp + b，
        跳过 per-frame min-归一化步骤。标定参数文件路径由环境变量
        DA360_DEPTH_CALIB_PATH 指定；文件不存在时回退到 env_float 的
        DA360_DEPTH_SCALE 粗调（当前默认行为）。

        Returns
        -------
        np.ndarray  float32[H,W]  metric depth (metres), clamped to [min_dis, max_dis]
        """
        raw = self.infer_raw(image)
        pred_disp = raw["pred_disp"]
        calib = self._load_depth_calibration()
        if calib is not None:
            a, b = calib["a"], calib["b"]
            min_d = calib.get("depth_min_m", self.min_dis)
            max_d = calib.get("depth_max_m", self.max_dis)
            metric = 1.0 / np.maximum(a * pred_disp + b, 1e-6)
            metric = np.clip(metric, min_d, max_d)
        else:
            # 无标定文件时退回旧行为：per-frame min-归一化 + 常数缩放
            depth = 1.0 / np.maximum(pred_disp, 1e-6)
            valid = np.isfinite(depth) & (depth > 0)
            if np.any(valid):
                depth = depth / max(float(depth[valid].min()), 1e-6)
            scale = env_float("DA360_DEPTH_SCALE", 1.0)
            metric = depth * scale
        return metric.astype(np.float32)

    @staticmethod
    def _load_depth_calibration():
        """加载标定参数文件。文件不存在或格式无效时返回 None。"""
        path = os.environ.get("DA360_DEPTH_CALIB_PATH", "")
        if not path:
            return None
        try:
            with open(path) as f:
                calib = json.load(f)
            a = float(calib.get("a", 0))
            b = float(calib.get("b", 0))
            if abs(a) < 1e-12:
                return None
            return {"a": a, "b": b,
                    "depth_min_m": float(calib.get("depth_min_m", 0.04)),
                    "depth_max_m": float(calib.get("depth_max_m", 20.0))}
        except Exception:
            return None

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
                "epsilon": float(eps),
                "unit_pred_disp": "raw disparity (inverse depth), NOT per-frame normalized",
                "unit_relative_depth": "1/pred_disp (not divided by frame min)",
            },
        }


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
            "resample": runner.resample_name,
            "amp": runner.use_amp,
            "channels_last": runner.channels_last,
            "depth_scale": env_float("DA360_DEPTH_SCALE", 1.0),
        })

    @app.route("/depth/raw", methods=["POST", "OPTIONS"])
    def depth_raw():
        """Return raw pred_disp, relative_depth, and valid_mask in .npz format.

        Input:  Content-Type: image/jpeg  (ERP RGB JPEG)
        Output: Content-Type: application/x-npz
                Contains: pred_disp (float32), relative_depth (float32),
                          valid_mask (uint8), metadata_json (str)
        """
        if request.method == "OPTIONS":
            return ("", 204)
        started = time.time()

        try:
            image = decode_request_image(request)
            request_width, request_height = image.size
            raw = runner.infer_raw(image)

            buf = io.BytesIO()
            np.savez_compressed(
                buf,
                pred_disp=raw["pred_disp"],
                relative_depth=raw["relative_depth"],
                valid_mask=raw["valid_mask"],
                metadata_json=json.dumps(raw["metadata"]),
            )
            raw_bytes = buf.getvalue()

            response = app.response_class(
                raw_bytes,
                status=200,
                mimetype="application/x-npz",
                headers={
                    "Content-Disposition": "attachment; filename=depth_raw.npz",
                    "X-DA360-Model": raw["metadata"]["model"],
                    "X-DA360-Width": str(raw["metadata"]["width"]),
                    "X-DA360-Height": str(raw["metadata"]["height"]),
                    "X-DA360-Latency-Ms": str((time.time() - started) * 1000.0),
                },
            )
            return response
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:
            print(f"[DA360] raw inference failed: {exc}", file=sys.stderr)
            return jsonify({"error": str(exc)}), 500
    @app.route("/depth", methods=["POST", "OPTIONS"])
    def depth():
        if request.method == "OPTIONS":
            return ("", 204)
        started = time.time()

        try:
            timings = {}
            mark = time.time()
            image = decode_request_image(request)
            timings["decode_ms"] = (time.time() - mark) * 1000.0
            request_width, request_height = image.size
            mark = time.time()
            pred_depth = runner.infer(image)
            timings["infer_ms"] = (time.time() - mark) * 1000.0
            mark = time.time()
            colored, depth_scale = depth_to_color(pred_depth)
            timings["color_ms"] = (time.time() - mark) * 1000.0
            mark = time.time()
            depth_image = encode_image(
                colored,
                os.environ.get("DA360_OUTPUT_FORMAT", "jpeg"),
                env_int("DA360_JPEG_QUALITY", 72),
            )
            timings["encode_ms"] = (time.time() - mark) * 1000.0
            return jsonify({
                "depth_image": depth_image,
                "depth_scale": depth_scale,
                "latency_ms": (time.time() - started) * 1000.0,
                "timings_ms": timings,
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
