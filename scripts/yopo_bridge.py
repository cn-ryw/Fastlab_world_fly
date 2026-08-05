#!/usr/bin/env python3
"""YOPO planning bridge — Flask API that wraps YOPO inference for the web simulator.

POST /yopo/plan
  Input JSON: { depth: float32[192][384], pose: {x,y,z,qx,qy,qz,qw}, goal: {x,y,z},
                vel: {vx,vy,vz}, yaw: float }
  Output JSON: { endstate: [px,py,pz,vx,vy,vz,ax,ay,az], traj_time: 1.125,
                 score: float, fixed_height: 0.8 }
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
from flask import Flask, jsonify, request
from scipy.spatial.transform import Rotation as R

# ── path setup ──────────────────────────────────────────────────────────
YOPO_ROOT = Path(os.environ.get("YOPO_ROOT", "/opt/YOPO_360")).resolve()
sys.path.insert(0, str(YOPO_ROOT / "YOPO"))

from config.config import cfg  # noqa: E402
from policy.yopo_network import YopoNetwork  # noqa: E402
from policy.depth_mask import fill_invalid_depth, valid_depth_mask  # noqa: E402
from policy.state_transform import StateTransform  # noqa: E402
from policy.poly_solver import Poly5Solver  # noqa: E402

DEFAULT_MODEL = os.environ.get("YOPO_MODEL_PATH", str(YOPO_ROOT / "YOPO/saved/YOPO_55/epoch10.pth"))
DEFAULT_CONFIG = os.environ.get("YOPO_CONFIG", "x5_cruise15_18m_a12_mask_wc3.yaml")
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def _load_model(model_path, device):
    """Load YOPO model weights."""
    if not Path(model_path).is_file():
        raise FileNotFoundError(f"YOPO checkpoint missing: {model_path}")
    payload = torch.load(model_path, map_location=device, weights_only=False)
    state_dict = payload.get("model_state_dict", payload)
    net = YopoNetwork(
        observation_dim=9,
        output_dim=10,
        hidden_state=64,
    )
    net.load_state_dict(state_dict, strict=False)
    net.to(device)
    net.eval()
    return net


# ── Flask app ────────────────────────────────────────────────────────────
app = Flask(__name__)
runner = None  # set by main()


@app.route("/yopo/health", methods=["GET"])
def health():
    if runner is None:
        return jsonify({"ok": False, "error": "not initialized"}), 503
    return jsonify({"ok": True, "model": str(runner.model_path), "device": runner.device})


@app.route("/yopo/plan", methods=["POST", "OPTIONS"])
def plan():
    if request.method == "OPTIONS":
        return ("", 204)

    if runner is None:
        return jsonify({"error": "YOPO bridge not initialized"}), 503

    try:
        data = request.get_json(force=True)
        started = time.time()

        # --- parse inputs ---
        depth_arr = np.array(data["depth"], dtype=np.float32)  # [192, 384]
        pose = data["pose"]    # {x,y,z,qx,qy,qz,qw}
        goal = data["goal"]    # {x,y,z}
        vel = data.get("vel", {"vx": 0, "vy": 0, "vz": 0})
        yaw = float(data.get("yaw", 0.0))
        acc = data.get("acc", {"ax": 0, "ay": 0, "az": 0})

        endstate, score, traj_time = runner.infer(
            depth_arr=depth_arr,
            pos=np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float32),
            vel=np.array([vel["vx"], vel["vy"], vel["vz"]], dtype=np.float32),
            acc=np.array([acc["ax"], acc["ay"], acc["az"]], dtype=np.float32),
            goal=np.array([goal["x"], goal["y"], goal["z"]], dtype=np.float32),
            yaw=yaw,
        )

        return jsonify({
            "endstate": endstate.tolist(),      # [px,py,pz,vx,vy,vz,ax,ay,az]
            "score": float(score),
            "traj_time": traj_time,
            "fixed_height": runner.fixed_height,
            "latency_ms": (time.time() - started) * 1000.0,
        })

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ── Runner ────────────────────────────────────────────────────────────────
class YopoRunner:
    def __init__(self, model_path, device=DEVICE):
        self.model_path = model_path
        self.device = device

        self.net = _load_model(model_path, device)
        self.width = cfg["image_width"]
        self.height = cfg["image_height"]
        self.in_channels = cfg["image_channels"]
        self.max_dis = cfg["depth_max_m"]
        self.min_dis = cfg["depth_min_m"]
        self.mask_valid_threshold = cfg["depth_mask_threshold"]
        self.fixed_height = cfg["fixed_height"]
        self.traj_time = 2 * cfg["radio_range"] / cfg["vel_max_train"]
        self.Rotation_bc = R.from_euler("z", 0, degrees=True).as_matrix()  # identity

        self.state_transform = StateTransform()
        self._warmup()

    def _warmup(self):
        dummy_depth = torch.zeros(1, self.in_channels, self.height, self.width, device=self.device)
        dummy_obs = self.state_transform.prepare_input(
            torch.zeros(1, 9, device=self.device)
        )
        with torch.inference_mode():
            self.net(dummy_depth, dummy_obs)

    @torch.inference_mode()
    def infer(self, depth_arr, pos, vel, acc, goal, yaw):
        """Run YOPO inference and return the decoded best endstate.

        Returns
        -------
        endstate : np.ndarray  shape [9]  (px,py,pz,vx,vy,vz,ax,ay,az) world-frame
        score    : float
        traj_time : float
        """
        # 1. preprocess depth
        if depth_arr.shape[0] != self.height or depth_arr.shape[1] != self.width:
            import cv2
            depth_arr = cv2.resize(depth_arr, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

        raw_finite = np.isfinite(depth_arr)
        depth_norm = np.minimum(
            np.where(raw_finite, depth_arr, np.nan), self.max_dis
        ) / self.max_dis
        valid = valid_depth_mask(depth_norm, None, threshold=self.mask_valid_threshold,
                                 minimum_depth=self.min_dis / self.max_dis)
        depth_norm = fill_invalid_depth(depth_norm, valid, mode="valid_mean")
        stacked = np.stack([depth_norm, valid.astype(np.float32)], axis=0)
        depth_input = torch.from_numpy(stacked.reshape(1, self.in_channels, self.height, self.width)).to(self.device)

        # 2. observation
        Rotation_wb = R.from_euler("Z", yaw, degrees=False).as_matrix()
        Rotation_wc = np.dot(Rotation_wb, self.Rotation_bc)
        Rotation_cw = Rotation_wc.T

        vel_c = np.dot(Rotation_cw, vel)
        acc_c = np.dot(Rotation_cw, acc)
        goal_w = goal - pos
        goal_c = np.dot(Rotation_cw, goal_w)

        obs = np.concatenate([vel_c, acc_c, goal_c], axis=0).astype(np.float32)
        obs_norm = self.state_transform.normalize_obs(torch.from_numpy(obs[None, :]))
        obs_input = self.state_transform.prepare_input(obs_norm.to(self.device))

        # 3. forward
        endstate_pred, score_pred = self.net(depth_input, obs_input)
        endstate_pred, score_pred = endstate_pred.cpu().numpy(), score_pred.cpu().numpy()

        # 4. decode best candidate
        traj_num = cfg["horizon_num"] * cfg["vertical_num"] * cfg["radio_num"]  # 72
        endstate_flat = endstate_pred.reshape(9, traj_num).T  # [72, 9]
        score_flat = score_pred.reshape(traj_num)
        action_id = int(np.argmin(score_flat))
        lattice_id = traj_num - 1 - action_id
        endstate = self.state_transform.pred_to_endstate_cpu(
            endstate_flat[action_id, :][np.newaxis, :],
            torch.tensor([lattice_id], dtype=torch.long)
        )
        score = float(score_flat[action_id])

        endstate_c = endstate.reshape(-1, 3, 3).transpose(0, 2, 1)
        endstate_w = np.matmul(Rotation_wc, endstate_c)

        # z is forced to fixed_height
        endstate_w[:, 2, 0] = self.fixed_height - pos[2]

        best = endstate_w[0]
        result = np.array([
            best[0, 0] + pos[0], best[0, 1], best[0, 2],  # px, vx, ax
            best[1, 0] + pos[1], best[1, 1], best[1, 2],  # py, vy, ay
            best[2, 0] + pos[2], best[2, 1], best[2, 2],  # pz, vz, az
        ], dtype=np.float32)

        return result, score, self.traj_time


# ── main ──────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="YOPO planning bridge API")
    p.add_argument("--model-path", default=DEFAULT_MODEL)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", default=5699, type=int)
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


def main():
    global runner
    args = parse_args()
    os.environ.setdefault("YOPO_CONFIG", DEFAULT_CONFIG)
    runner = YopoRunner(args.model_path)
    print(f"[yopo-bridge] model loaded: {runner.model_path}, device={runner.device}")
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=False)


if __name__ == "__main__":
    main()
