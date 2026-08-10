#!/usr/bin/env python3
"""Combined DA360 + YOPO server — one process and one HTTP port.

DA360:  (/health, /depth, /depth/raw)
YOPO:   (/yopo/health, /yopo/plan, /yopo/plan_full)

/yopo/plan_full accepts a JPEG image (same as DA360 /depth) plus pose/goal.
It always returns DA360 depth; YOPO runs only when the selected depth mode is
explicitly authorized for planning (currently an accepted metric calibration).
"""

import argparse
import math
import os
import sys
import threading
import time
from pathlib import Path

# ── Path setup ──────────────────────────────────────────────────────────
DA360_ROOT = Path(os.environ.get("DA360_ROOT", "/opt/DA360")).resolve()
YOPO_ROOT = Path(os.environ.get("YOPO_ROOT", "/opt/YOPO_360")).resolve()
SERVER_DIR = Path(__file__).resolve().parent

sys.path.insert(0, str(DA360_ROOT))
sys.path.insert(0, str(YOPO_ROOT / "YOPO"))
sys.path.insert(0, str(SERVER_DIR))

# ── Imports ─────────────────────────────────────────────────────────────
import numpy as np
from flask import Flask, jsonify, request
from werkzeug.exceptions import HTTPException

# DA360 imports
from da360_server import (
    API_VERSION,
    DA360Runner,
    DEFAULT_INPUT_SCALE,
    _infer_configured_depth,
    _request_image_metadata,
    configure_api_security,
    decode_request_image,
    depth_to_color,
    encode_image,
    env_float,
    env_int,
    register_depth_routes,
)

app = Flask(__name__)
configure_api_security(app)
da360_runner = None
yopo_runner = None
# 服务端缓存最近一次 DA360 推理结果（由 /depth 写入，/yopo/plan 读取，避免前端回传 1.4MB）
_depth_cache = {
    "data": None,
    "ts": 0,
    "frame_id": None,
    "depth_mode": None,
    "calibration_id": None,
}
_depth_cache_lock = threading.Lock()


def _cache_depth(pred_depth, request_metadata):
    """Cache the latest preview for the legacy two-stage planning endpoint."""
    calibration = getattr(da360_runner, "calibration", None)
    with _depth_cache_lock:
        _depth_cache["data"] = np.asarray(pred_depth, dtype=np.float32).copy()
        _depth_cache["ts"] = time.monotonic()
        _depth_cache["frame_id"] = request_metadata.get("frame_id")
        _depth_cache["depth_mode"] = getattr(
            da360_runner, "depth_mode", "da360-relative"
        )
        _depth_cache["calibration_id"] = (
            calibration.get("id") if calibration else None
        )


register_depth_routes(
    app,
    lambda: da360_runner,
    on_depth=_cache_depth,
    endpoint_prefix="combined_da360",
)


def _planning_authorization():
    """Separate API availability from permission to apply a YOPO trajectory."""
    if da360_runner is None:
        return False, "da360-not-initialized"
    depth_mode = getattr(da360_runner, "depth_mode", "da360-relative")
    if depth_mode != "da360-metric":
        return False, "da360-relative-is-preview-only"
    calibration = getattr(da360_runner, "calibration", None)
    if not calibration or not calibration.get("id"):
        return False, "metric-calibration-not-loaded"
    if yopo_runner is None:
        return False, "yopo-not-initialized"
    return True, "validated-da360-metric"


# ── YOPO endpoints ───────────────────────────────────────────────────────
@app.route("/yopo/health", methods=["GET"])
def yopo_health():
    if yopo_runner is None:
        return jsonify({"ok": False, "error": "YOPO not initialized"}), 503
    planning_authorized, planning_reason = _planning_authorization()
    return jsonify({
        "ok": True,
        "api_version": API_VERSION,
        "device": yopo_runner.device,
        "model": Path(str(getattr(yopo_runner, "model_path", "unknown"))).name,
        "checkpoint_sha256": getattr(yopo_runner, "checkpoint_sha256", None),
        "checkpoint_coverage": getattr(yopo_runner, "checkpoint_coverage", None),
        "checkpoint_missing_keys": getattr(yopo_runner, "checkpoint_missing_keys", None),
        "checkpoint_unexpected_keys": getattr(yopo_runner, "checkpoint_unexpected_keys", None),
        "config": getattr(yopo_runner, "config_name", None)
            or os.environ.get("YOPO_CONFIG", "x5_cruise15_18m_a12_mask_wc3.yaml"),
        "config_sha256": getattr(yopo_runner, "config_sha256", None),
        "base_config": getattr(yopo_runner, "base_config_name", None) or "traj_opt.yaml",
        "base_config_sha256": getattr(yopo_runner, "base_config_sha256", None),
        "effective_config_sha256": getattr(
            yopo_runner, "effective_config_sha256", None
        ),
        "planning_authorized": planning_authorized,
        "planning_reason": planning_reason,
    })


def _finite_float(value, field):
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a number") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field} must be finite")
    return parsed


def _validate_yopo_result(endstate, score, traj_time, actual_position=None):
    endstate = np.asarray(endstate, dtype=np.float32).reshape(-1)
    if endstate.size != 9 or not np.all(np.isfinite(endstate)):
        raise RuntimeError("YOPO returned an invalid 9-element axis-major endstate")
    try:
        score = _finite_float(score, "score")
        traj_time = _finite_float(traj_time, "traj_time")
    except ValueError as exc:
        raise RuntimeError(f"YOPO returned an invalid numeric result: {exc}") from exc
    if traj_time <= 0 or traj_time > env_float("YOPO_MAX_TRAJ_TIME", 10.0):
        raise RuntimeError(f"YOPO returned invalid traj_time={traj_time}")
    endpoint_position = endstate[[0, 3, 6]]
    endpoint_velocity = endstate[[1, 4, 7]]
    endpoint_acceleration = endstate[[2, 5, 8]]
    if np.linalg.norm(endpoint_velocity) > env_float("YOPO_MAX_ENDPOINT_SPEED", 40.0):
        raise RuntimeError("YOPO endpoint speed exceeds the safety envelope")
    if np.linalg.norm(endpoint_acceleration) > env_float("YOPO_MAX_ENDPOINT_ACCEL", 40.0):
        raise RuntimeError("YOPO endpoint acceleration exceeds the safety envelope")
    if actual_position is not None:
        actual_position = np.asarray(actual_position, dtype=np.float32).reshape(3)
        if np.linalg.norm(endpoint_position - actual_position) > env_float(
                "YOPO_MAX_ENDPOINT_DISPLACEMENT", 100.0):
            raise RuntimeError("YOPO endpoint displacement exceeds the safety envelope")
    return endstate, score, traj_time


def _response_identity(mapping=None, *, required=False):
    mapping = mapping if isinstance(mapping, dict) else {}
    identity = {
        "frame_id": request.args.get("frame_id")
            or request.headers.get("X-Frame-ID") or mapping.get("frame_id"),
        "goal_id": request.args.get("goal_id")
            or request.headers.get("X-Goal-ID") or mapping.get("goal_id"),
        "generation": request.args.get("generation")
            or request.headers.get("X-Generation") or mapping.get("generation"),
    }
    identity = {
        key: None if value is None else str(value)
        for key, value in identity.items()
    }
    if required:
        for key in ("frame_id", "goal_id"):
            token = identity[key]
            if not token or token != token.strip() or len(token) > 128 \
                    or any(ord(character) < 32 for character in token):
                raise ValueError(f"{key} must be a non-empty identity token")
        generation = identity["generation"]
        if not generation or any(character not in "0123456789" for character in generation) \
                or int(generation) > 2**53 - 1:
            raise ValueError("generation must be a non-negative safe integer")
    return identity


def _json_vector(mapping, keys, field):
    if not isinstance(mapping, dict):
        raise ValueError(f"{field} must be an object")
    return np.array(
        [_finite_float(mapping.get(key), f"{field}.{key}") for key in keys],
        dtype=np.float32,
    )


@app.route("/yopo/plan", methods=["POST", "OPTIONS"])
def yopo_plan():
    """Accept depth array + pose + goal → return trajectory endpoint."""
    if request.method == "OPTIONS":
        return ("", 204)
    try:
        data = request.get_json(force=True)
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        planning_authorized, planning_reason = _planning_authorization()
        calibration = getattr(da360_runner, "calibration", None)
        planning_metadata = {
            "api_version": API_VERSION,
            "depth_mode": getattr(da360_runner, "depth_mode", "da360-relative"),
            "calibration_id": calibration.get("id") if calibration else None,
            "planning_authorized": planning_authorized,
            "planning_reason": planning_reason,
        }
        identity = _response_identity(data, required=planning_authorized)
        planning_metadata.update(identity)
        if not planning_authorized:
            return jsonify(planning_metadata), 409
        if "depth" in data:
            raise ValueError(
                "authorized two-stage planning only accepts server-cached depth"
            )
        started = time.perf_counter()
        pose = data["pose"]
        if "reference_pose" in data:
            reference_pose = data["reference_pose"]
        elif "reference_pos" in data:
            reference_pose = data["reference_pos"]
        else:
            raise ValueError("reference_pose or reference_pos is required")
        goal = data["goal"]
        vel = data["vel"]
        yaw = _finite_float(data["yaw"], "yaw")
        acc = data["acc"]
        actual_position = _json_vector(pose, ("x", "y", "z"), "pose")
        reference_position = _json_vector(
            reference_pose, ("x", "y", "z"), "reference_pose"
        )
        goal_position = _json_vector(goal, ("x", "y", "z"), "goal")
        velocity = _json_vector(vel, ("vx", "vy", "vz"), "vel")
        acceleration = _json_vector(acc, ("ax", "ay", "az"), "acc")

        current_depth_mode = getattr(
            da360_runner, "depth_mode", "da360-relative"
        )
        current_calibration = getattr(da360_runner, "calibration", None)
        current_calibration_id = (
            current_calibration.get("id") if current_calibration else None
        )
        with _depth_cache_lock:
            if _depth_cache["data"] is None:
                return jsonify({"error": "server depth cache is empty"}), 400
            cache_age = time.monotonic() - _depth_cache["ts"]
            if cache_age > env_float("DA360_DEPTH_CACHE_MAX_AGE", 2.0):
                return jsonify({"error": "cached depth is stale"}), 409
            cached_frame_id = _depth_cache["frame_id"]
            if cached_frame_id is not None:
                cached_frame_id = str(cached_frame_id)
            if cached_frame_id != identity["frame_id"]:
                return jsonify({"error": "cached depth frame_id mismatch"}), 409
            if _depth_cache["depth_mode"] != current_depth_mode:
                return jsonify({"error": "cached depth mode mismatch"}), 409
            if _depth_cache["calibration_id"] != current_calibration_id:
                return jsonify({"error": "cached depth calibration mismatch"}), 409
            depth_arr = _depth_cache["data"].copy()

        with yopo_runner.lock:
            endstate, score, traj_time = yopo_runner.infer(
                depth_arr=depth_arr,
                pos=actual_position,
                reference_pos=reference_position,
                vel=velocity,
                acc=acceleration,
                goal=goal_position,
                yaw=yaw,
            )
        endstate, score, traj_time = _validate_yopo_result(
            endstate, score, traj_time, actual_position
        )
        response_payload = {
            "endstate": endstate.tolist(),
            "score": float(score),
            "traj_time": traj_time,
            "latency_ms": (time.perf_counter() - started) * 1000.0,
            "planning_origin": {
                "actual": actual_position.tolist(),
                "reference": reference_position.tolist(),
            },
        }
        response_payload.update(planning_metadata)
        return jsonify(response_payload)
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"error": str(exc)}), 400
    except HTTPException:
        raise
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[YOPO] planning failed: {exc}", file=sys.stderr)
        return jsonify({"error": "YOPO planning failed"}), 500


@app.route("/yopo/plan_full", methods=["POST", "OPTIONS"])
def yopo_plan_full():
    """Return depth and, only when authorized, a one-shot YOPO trajectory."""
    if request.method == "OPTIONS":
        return ("", 204)
    if da360_runner is None:
        return jsonify({"error": "DA360 not initialized"}), 503
    try:
        started = time.perf_counter()
        identity = _response_identity(required=True)

        # Validate all state before spending GPU time on the image.
        def query_number(name, header):
            value = request.args.get(name, request.headers.get(header))
            if value is None:
                raise ValueError(f"missing planning field: {name}")
            return _finite_float(value, name)

        pose_x = query_number("px", "X-Pose-X")
        pose_y = query_number("py", "X-Pose-Y")
        pose_z = query_number("pz", "X-Pose-Z")
        reference_x = query_number("rpx", "X-Reference-X")
        reference_y = query_number("rpy", "X-Reference-Y")
        reference_z = query_number("rpz", "X-Reference-Z")
        goal_x = query_number("gx", "X-Goal-X")
        goal_y = query_number("gy", "X-Goal-Y")
        goal_z = query_number("gz", "X-Goal-Z")
        vel_vx = query_number("vx", "X-Vel-X")
        vel_vy = query_number("vy", "X-Vel-Y")
        vel_vz = query_number("vz", "X-Vel-Z")
        drone_yaw = query_number("yaw", "X-Yaw")
        acc_x = query_number("ax", "X-Acc-X")
        acc_y = query_number("ay", "X-Acc-Y")
        acc_z = query_number("az", "X-Acc-Z")

        # 1. Decode JPEG → DA360 depth
        t0 = time.perf_counter()
        image = decode_request_image(request)
        request_metadata = _request_image_metadata(image)
        t_decode = time.perf_counter()
        pred_depth = _infer_configured_depth(da360_runner, image, request_metadata)
        t1 = time.perf_counter()

        # 生成小 JPEG 深度图供前端显示（原项目做法，~6KB，不塞 1.3MB depth_array）
        colored, depth_scale = depth_to_color(pred_depth)
        if getattr(da360_runner, "depth_mode", "da360-relative") == "da360-metric":
            depth_scale["unit"] = "metres"
        depth_jpeg = encode_image(colored, "jpeg", env_int("DA360_JPEG_QUALITY", 72))
        t_color = time.perf_counter()

        # 2. Only validated metric depth may authorize an applicable trajectory.
        # Relative DA360 remains useful as a live preview, but scale-normalized
        # depth must never be silently fed to a metric-trained YOPO policy.
        planning_authorized, planning_reason = _planning_authorization()
        endstate = score = traj_time = None
        if planning_authorized:
            with yopo_runner.lock:
                endstate, score, traj_time = yopo_runner.infer(
                    depth_arr=pred_depth,
                    pos=np.array([pose_x, pose_y, pose_z], dtype=np.float32),
                    reference_pos=np.array(
                        [reference_x, reference_y, reference_z], dtype=np.float32
                    ),
                    vel=np.array([vel_vx, vel_vy, vel_vz], dtype=np.float32),
                    acc=np.array([acc_x, acc_y, acc_z], dtype=np.float32),
                    goal=np.array([goal_x, goal_y, goal_z], dtype=np.float32),
                    yaw=drone_yaw,
                )
            endstate, score, traj_time = _validate_yopo_result(
                endstate, score, traj_time, [pose_x, pose_y, pose_z]
            )
        t2 = time.perf_counter()
        calibration = getattr(da360_runner, "calibration", None)
        resp_payload = {
            "api_version": API_VERSION,
            "depth_image": depth_jpeg,
            "depth_scale": depth_scale,
            "depth_mode": getattr(da360_runner, "depth_mode", "da360-relative"),
            "calibration_id": calibration.get("id") if calibration else None,
            "planning_authorized": planning_authorized,
            "planning_reason": planning_reason,
            "latency_ms": (time.perf_counter() - started) * 1000.0,
            "timings_ms": {
                "decode_ms": (t_decode - t0) * 1000.0,
                "da360_ms": (t1 - t_decode) * 1000.0,
                "color_encode_ms": (t_color - t1) * 1000.0,
                "yopo_ms": (t2 - t_color) * 1000.0,
            },
            "planning_origin": {
                "actual": [pose_x, pose_y, pose_z],
                "reference": [reference_x, reference_y, reference_z],
            },
        }
        if planning_authorized:
            resp_payload.update({
                "endstate": endstate.tolist(),
                "score": float(score),
                "traj_time": traj_time,
            })
        resp_payload.update(identity)
        resp = jsonify(resp_payload)
        t3 = time.perf_counter()
        print(
            f"[plan_full] decode={1000*(t_decode-t0):.0f}ms "
            f"da360={1000*(t1-t_decode):.0f}ms "
            f"color+jpeg={1000*(t_color-t1):.0f}ms "
            f"yopo={1000*(t2-t_color):.0f}ms "
            f"authorized={planning_authorized} reason={planning_reason} "
            f"json={1000*(t3-t2):.0f}ms total={1000*(t3-started):.0f}ms",
            flush=True,
        )
        return resp
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"error": str(exc)}), 400
    except HTTPException:
        raise
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[plan_full] failed: {exc}", file=sys.stderr)
        return jsonify({"error": "full planning failed"}), 500


# ── Main ──────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Combined DA360 + YOPO server")
    p.add_argument("--da360-model", default=os.environ.get("DA360_MODEL_PATH",
        str(Path("/models/DA360_large.pth"))))
    p.add_argument("--yopo-model", default=os.environ.get("YOPO_MODEL_PATH",
        str(Path("/models/epoch10.pth"))))
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", default=5688, type=int)
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


def main():
    global da360_runner, yopo_runner

    try:
        import torch  # pylint: disable=import-outside-toplevel
    except ImportError as exc:
        raise RuntimeError("combined DA360+YOPO service requires PyTorch") from exc

    args = parse_args()
    input_scale = env_float("DA360_INPUT_SCALE", DEFAULT_INPUT_SCALE)

    # 顺序加载：并行线程加载两个 CUDA 模型会导致 DA360 warmup 期间
    # CUDA context 竞争，模型前向传递从 ~25ms 退化到 ~5.5s。
    # 原项目 da360_server.py 只跑单一模型，没有这个问题。
    if not torch.cuda.is_available():
        raise RuntimeError("combined DA360+YOPO service requires a CUDA GPU")
    torch.zeros(1, device="cuda")
    print("[combined] Loading DA360 model...")
    da360_runner = DA360Runner(args.da360_model, input_scale=input_scale)
    print(f"[combined] DA360 ready: {da360_runner.model_name} on {da360_runner.device}")

    print("[combined] Loading YOPO model...")
    # Deferred so the Flask app and fake-runner contract tests do not require
    # the external YOPO checkout at import time.
    from yopo_bridge import YopoRunner  # pylint: disable=import-outside-toplevel
    yopo_runner = YopoRunner(args.yopo_model)
    yopo_runner.lock = threading.Lock()
    print(f"[combined] YOPO ready on {yopo_runner.device}")

    print(f"[combined] Starting on {args.host}:{args.port} (single-threaded)")
    # threaded=False：跨线程 CUDA 首次推理慢 8 倍（~25ms→~220ms）。
    # 前端 gate 已保证请求串行，多线程无性能收益。
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=False)


if __name__ == "__main__":
    main()
