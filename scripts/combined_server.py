#!/usr/bin/env python3
"""Combined DA360 + YOPO server — single process, two ports.

DA360:  port 5688  (/health, /depth, /depth/raw)
YOPO:   port 5699  (/yopo/health, /yopo/plan, /yopo/plan_full)

/yopo/plan_full accepts a JPEG image (same as DA360 /depth) plus pose/goal,
internally runs DA360 inference → depth array → YOPO inference → trajectory.
"""

import argparse
import os
import sys
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
import torch
from flask import Flask, jsonify, request

# DA360 imports
from da360_server import DA360Runner, env_float
from da360_server import DEFAULT_INPUT_SCALE

# YOPO imports
from yopo_bridge import YopoRunner

app = Flask(__name__)
try:
    from flask_cors import CORS
    CORS(app)
except ImportError:
    pass
da360_runner = None
yopo_runner = None


# ── DA360 endpoints ──────────────────────────────────────────────────────
@app.route("/health", methods=["GET"])
def da360_health():
    if da360_runner is None:
        return jsonify({"ok": False, "error": "DA360 not initialized"}), 503
    return jsonify({
        "ok": True,
        "model": da360_runner.model_name,
        "device": str(da360_runner.device),
        "width": da360_runner.width,
        "height": da360_runner.height,
        "input_scale": da360_runner.input_scale,
        "resample": da360_runner.resample_name,
        "amp": da360_runner.use_amp,
        "channels_last": da360_runner.channels_last,
        "depth_scale": env_float("DA360_DEPTH_SCALE", 1.0),
    })


@app.route("/depth", methods=["POST", "OPTIONS"])
def da360_depth():
    if request.method == "OPTIONS":
        return ("", 204)
    if da360_runner is None:
        return jsonify({"error": "DA360 not initialized"}), 503
    try:
        from da360_server import (
            decode_request_image, depth_to_color, encode_image, env_int,
        )
        import time
        started = time.time()
        image = decode_request_image(request)
        pred_depth = da360_runner.infer(image)
        colored, depth_scale = depth_to_color(pred_depth)
        depth_image = encode_image(colored, "jpeg", env_int("DA360_JPEG_QUALITY", 72))
        return jsonify({
            "depth_image": depth_image,
            "depth_scale": depth_scale,
            "latency_ms": (time.time() - started) * 1000.0,
            "model": da360_runner.model_name,
            "device": str(da360_runner.device),
            "width": da360_runner.width,
            "height": da360_runner.height,
        })
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ── YOPO endpoints ───────────────────────────────────────────────────────
@app.route("/yopo/health", methods=["GET"])
def yopo_health():
    if yopo_runner is None:
        return jsonify({"ok": False, "error": "YOPO not initialized"}), 503
    return jsonify({"ok": True, "device": yopo_runner.device})


@app.route("/yopo/plan", methods=["POST", "OPTIONS"])
def yopo_plan():
    """Accept depth array + pose + goal → return trajectory endpoint."""
    if request.method == "OPTIONS":
        return ("", 204)
    if yopo_runner is None:
        return jsonify({"error": "YOPO not initialized"}), 503
    try:
        import time
        data = request.get_json(force=True)
        started = time.time()
        depth_arr = np.array(data["depth"], dtype=np.float32)
        pose = data["pose"]
        goal = data["goal"]
        vel = data.get("vel", {"vx": 0, "vy": 0, "vz": 0})
        yaw = float(data.get("yaw", 0.0))
        acc = data.get("acc", {"ax": 0, "ay": 0, "az": 0})

        endstate, score, traj_time = yopo_runner.infer(
            depth_arr=depth_arr,
            pos=np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float32),
            vel=np.array([vel["vx"], vel["vy"], vel["vz"]], dtype=np.float32),
            acc=np.array([acc["ax"], acc["ay"], acc["az"]], dtype=np.float32),
            goal=np.array([goal["x"], goal["y"], goal["z"]], dtype=np.float32),
            yaw=yaw,
        )
        return jsonify({
            "endstate": endstate.tolist(),
            "score": float(score),
            "traj_time": traj_time,
            "latency_ms": (time.time() - started) * 1000.0,
        })
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@app.route("/yopo/plan_full", methods=["POST", "OPTIONS"])
def yopo_plan_full():
    """JPEG → DA360 → depth → YOPO → trajectory. One-shot planning call."""
    if request.method == "OPTIONS":
        return ("", 204)
    if da360_runner is None or yopo_runner is None:
        return jsonify({"error": "servers not initialized"}), 503
    try:
        import io, time
        from PIL import Image
        from da360_server import image_to_tensor, decode_request_image

        started = time.time()

        # 1. Decode JPEG → DA360 depth
        if request.content_type and "image/jpeg" in request.content_type:
            image = Image.open(io.BytesIO(request.get_data())).convert("RGB")
        else:
            image = decode_request_image(request)

        pred_depth = da360_runner.infer(image)

        # 2. Extract pose + goal from headers/query params
        pose_x = float(request.args.get("px", request.headers.get("X-Pose-X", 0)))
        pose_y = float(request.args.get("py", request.headers.get("X-Pose-Y", 2)))
        pose_z = float(request.args.get("pz", request.headers.get("X-Pose-Z", 0)))
        goal_x = float(request.args.get("gx", request.headers.get("X-Goal-X", 10)))
        goal_y = float(request.args.get("gy", request.headers.get("X-Goal-Y", 2)))
        goal_z = float(request.args.get("gz", request.headers.get("X-Goal-Z", 10)))
        vel_vx = float(request.args.get("vx", request.headers.get("X-Vel-X", 0)))
        vel_vy = float(request.args.get("vy", request.headers.get("X-Vel-Y", 0)))
        vel_vz = float(request.args.get("vz", request.headers.get("X-Vel-Z", 0)))
        drone_yaw = float(request.args.get("yaw", request.headers.get("X-Yaw", 0)))

        # 3. YOPO inference
        endstate, score, traj_time = yopo_runner.infer(
            depth_arr=pred_depth,
            pos=np.array([pose_x, pose_y, pose_z], dtype=np.float32),
            vel=np.array([vel_vx, vel_vy, vel_vz], dtype=np.float32),
            acc=np.zeros(3, dtype=np.float32),
            goal=np.array([goal_x, goal_y, goal_z], dtype=np.float32),
            yaw=drone_yaw,
        )
        total_ms = (time.time() - started) * 1000.0
        return jsonify({
            "endstate": endstate.tolist(),
            "score": float(score),
            "traj_time": traj_time,
            "latency_ms": total_ms,
        })
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ── Main ──────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Combined DA360 + YOPO server")
    p.add_argument("--da360-model", default=os.environ.get("DA360_MODEL_PATH",
        str(Path("/models/DA360_large.pth"))))
    p.add_argument("--yopo-model", default=os.environ.get("YOPO_MODEL_PATH",
        str(Path("/models/epoch10.pth"))))
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", default=5688, type=int)
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


def main():
    global da360_runner, yopo_runner

    args = parse_args()
    input_scale = env_float("DA360_INPUT_SCALE", DEFAULT_INPUT_SCALE)

    # Load DA360
    print("[combined] Loading DA360 model...")
    da360_runner = DA360Runner(args.da360_model, input_scale=input_scale)
    print(f"[combined] DA360 ready: {da360_runner.model_name} on {da360_runner.device}")

    # Load YOPO
    print("[combined] Loading YOPO model...")
    yopo_runner = YopoRunner(args.yopo_model)
    print(f"[combined] YOPO ready on {yopo_runner.device}")

    print(f"[combined] Starting on 0.0.0.0:{args.port}")
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=False)


if __name__ == "__main__":
    main()
