#!/usr/bin/env python3
"""Combined DA360 + YOPO server — one process and one HTTP port.

DA360:  (/health, /depth, /depth/raw)
YOPO:   (/yopo/health, /yopo/plan, /yopo/plan_full)

/yopo/plan_full accepts a JPEG image (same as DA360 /depth) plus pose/goal,
internally runs DA360 inference → depth array → YOPO inference → trajectory.
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
_depth_cache = {"data": None, "ts": 0}
_depth_cache_lock = threading.Lock()


def _cache_depth(pred_depth, request_metadata):
    """Cache the latest preview for the legacy two-stage planning endpoint."""
    with _depth_cache_lock:
        _depth_cache["data"] = np.asarray(pred_depth, dtype=np.float32).copy()
        _depth_cache["ts"] = time.monotonic()
        _depth_cache["frame_id"] = request_metadata.get("frame_id")


register_depth_routes(
    app,
    lambda: da360_runner,
    on_depth=_cache_depth,
    endpoint_prefix="combined_da360",
)


# ── YOPO endpoints ───────────────────────────────────────────────────────
@app.route("/yopo/health", methods=["GET"])
def yopo_health():
    if yopo_runner is None:
        return jsonify({"ok": False, "error": "YOPO not initialized"}), 503
    return jsonify({
        "ok": True,
        "api_version": API_VERSION,
        "device": yopo_runner.device,
        "model": Path(str(getattr(yopo_runner, "model_path", "unknown"))).name,
        "checkpoint_sha256": getattr(yopo_runner, "checkpoint_sha256", None)
            or os.environ.get("YOPO_MODEL_SHA256") or None,
        "checkpoint_coverage": getattr(yopo_runner, "checkpoint_coverage", None),
        "checkpoint_missing_keys": getattr(yopo_runner, "checkpoint_missing_keys", None),
        "checkpoint_unexpected_keys": getattr(yopo_runner, "checkpoint_unexpected_keys", None),
        "config": getattr(yopo_runner, "config_name", None)
            or os.environ.get("YOPO_CONFIG", "x5_cruise15_18m_a12_mask_wc3.yaml"),
        "config_sha256": getattr(yopo_runner, "config_sha256", None)
            or os.environ.get("YOPO_CONFIG_SHA256") or None,
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


def _response_identity():
    return {
        "frame_id": request.args.get("frame_id") or request.headers.get("X-Frame-ID"),
        "goal_id": request.args.get("goal_id") or request.headers.get("X-Goal-ID"),
        "generation": request.args.get("generation") or request.headers.get("X-Generation"),
    }


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
    if yopo_runner is None:
        return jsonify({"error": "YOPO not initialized"}), 503
    try:
        data = request.get_json(force=True)
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        started = time.perf_counter()
        # 优先用请求中的 depth 数组；未提供时从服务端缓存取（省去前端 1.4MB JSON 回传）
        if "depth" in data:
            depth_arr = np.array(data["depth"], dtype=np.float32)
        elif _depth_cache["data"] is not None:
            with _depth_cache_lock:
                cache_age = time.monotonic() - _depth_cache["ts"]
                if cache_age > env_float("DA360_DEPTH_CACHE_MAX_AGE", 2.0):
                    return jsonify({"error": "cached depth is stale"}), 409
                depth_arr = _depth_cache["data"].copy()
        else:
            return jsonify({"error": "no depth provided and cache empty"}), 400
        pose = data["pose"]
        reference_pose = data.get("reference_pose", data.get("reference_pos", pose))
        goal = data["goal"]
        vel = data.get("vel", {"vx": 0, "vy": 0, "vz": 0})
        yaw = _finite_float(data.get("yaw", 0.0), "yaw")
        acc = data.get("acc", {"ax": 0, "ay": 0, "az": 0})
        actual_position = _json_vector(pose, ("x", "y", "z"), "pose")
        reference_position = _json_vector(
            reference_pose, ("x", "y", "z"), "reference_pose"
        )
        goal_position = _json_vector(goal, ("x", "y", "z"), "goal")
        velocity = _json_vector(vel, ("vx", "vy", "vz"), "vel")
        acceleration = _json_vector(acc, ("ax", "ay", "az"), "acc")

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
        return jsonify({
            "api_version": API_VERSION,
            "endstate": endstate.tolist(),
            "score": float(score),
            "traj_time": traj_time,
            "latency_ms": (time.perf_counter() - started) * 1000.0,
            "planning_origin": {
                "actual": actual_position.tolist(),
                "reference": reference_position.tolist(),
            },
        })
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"error": str(exc)}), 400
    except HTTPException:
        raise
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[YOPO] planning failed: {exc}", file=sys.stderr)
        return jsonify({"error": "YOPO planning failed"}), 500


@app.route("/yopo/plan_full", methods=["POST", "OPTIONS"])
def yopo_plan_full():
    """JPEG → DA360 → depth → YOPO → trajectory. One-shot planning call."""
    if request.method == "OPTIONS":
        return ("", 204)
    if da360_runner is None or yopo_runner is None:
        return jsonify({"error": "servers not initialized"}), 503
    try:
        started = time.perf_counter()

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
        acc_x = _finite_float(request.args.get("ax", 0), "ax")
        acc_y = _finite_float(request.args.get("ay", 0), "ay")
        acc_z = _finite_float(request.args.get("az", 0), "az")

        # 1. Decode JPEG → DA360 depth
        t0 = time.perf_counter()
        image = decode_request_image(request)
        t_decode = time.perf_counter()
        pred_depth = _infer_configured_depth(da360_runner, image)
        t1 = time.perf_counter()

        # 生成小 JPEG 深度图供前端显示（原项目做法，~6KB，不塞 1.3MB depth_array）
        colored, depth_scale = depth_to_color(pred_depth)
        if getattr(da360_runner, "depth_mode", "da360-relative") == "da360-metric":
            depth_scale["unit"] = "metres"
        depth_jpeg = encode_image(colored, "jpeg", env_int("DA360_JPEG_QUALITY", 72))
        t_color = time.perf_counter()

        # 2. YOPO inference
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
            "endstate": endstate.tolist(),
            "score": float(score),
            "traj_time": traj_time,
            "depth_image": depth_jpeg,
            "depth_scale": depth_scale,
            "depth_mode": getattr(da360_runner, "depth_mode", "da360-relative"),
            "calibration_id": calibration.get("id") if calibration else None,
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
        resp_payload.update(_response_identity())
        resp = jsonify(resp_payload)
        t3 = time.perf_counter()
        print(
            f"[plan_full] decode={1000*(t_decode-t0):.0f}ms "
            f"da360={1000*(t1-t_decode):.0f}ms "
            f"color+jpeg={1000*(t_color-t1):.0f}ms "
            f"yopo={1000*(t2-t_color):.0f}ms "
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
