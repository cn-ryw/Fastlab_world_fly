#!/usr/bin/env python3
"""YOPO planning bridge — Flask API that wraps YOPO inference for the web simulator.

POST /yopo/plan
  Input JSON: { depth: float32[192][384], pose: {x,y,z,qx,qy,qz,qw}, goal: {x,y,z},
                reference_pose: {x,y,z}, vel: {vx,vy,vz}, yaw: float }
  Output JSON: { endstate: [px,vx,ax, py,vy,ay, pz,vz,az], traj_time: 1.125,
                 score: float, fixed_height: float }
  注：响应里的 fixed_height 是**训练配置值**（0.8m），仅供参考。实际规划所用的
  高度平面取自请求中的 goal.y（sim 为 Y-Up），不再是这个常量。

endstate 采用**轴主序**：每个轴连续排布 位置/速度/加速度，坐标为 sim 世界系
(x=east, y=up, z=north)。这与参考实现 test_yopo_ros.py 的
endstate_w[id, axis, order] 布局一致，消费方是 drone.js:setYopoTrajectory。
历史教训：本 docstring 曾错写成量主序 [px,py,pz,vx,vy,vz,ax,ay,az]，
前端照着它实现，导致参考轨迹发散、无人机掉高。改动此顺序必须同步更新
drone.js 与 tests/test_yopo_endstate_layout.js。
"""

import argparse
import json
import os
import sys
import time
from contextlib import nullcontext
from pathlib import Path

import numpy as np
import torch
from flask import Flask, jsonify, request
from scipy.spatial.transform import Rotation as R

# ── path setup ──────────────────────────────────────────────────────────
YOPO_ROOT = Path(os.environ.get("YOPO_ROOT", "/opt/YOPO_360")).resolve()
sys.path.insert(0, str(YOPO_ROOT / "YOPO"))

# config.config materializes its global ``cfg`` at import time. Establish the
# overlay first so the standalone bridge cannot name one YAML while actually
# running with the base-only configuration.
DEFAULT_CONFIG_NAME = "x5_cruise15_18m_a12_mask_wc3.yaml"
os.environ.setdefault("YOPO_CONFIG", DEFAULT_CONFIG_NAME)

from config.config import cfg  # noqa: E402
from policy.yopo_network import YopoNetwork  # noqa: E402
from policy.depth_mask import fill_invalid_depth, valid_depth_mask  # noqa: E402
from policy.state_transform import StateTransform  # noqa: E402
from policy.poly_solver import Poly5Solver  # noqa: E402

DEFAULT_MODEL = os.environ.get("YOPO_MODEL_PATH", str(YOPO_ROOT / "YOPO/saved/YOPO_55/epoch10.pth"))
DEFAULT_CONFIG = os.environ["YOPO_CONFIG"]
BASE_CONFIG = "traj_opt.yaml"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
YOPO_MAX_VERTICAL_ENDPOINT_STEP_M = 4.0


def _load_model(model_path, device):
    """Load YOPO weights with safe deserialization and coverage checks."""
    if not Path(model_path).is_file():
        raise FileNotFoundError(f"YOPO checkpoint missing: {model_path}")
    try:
        payload = torch.load(model_path, map_location=device, weights_only=True)
    except TypeError as exc:
        if os.environ.get("YOPO_ALLOW_UNSAFE_CHECKPOINT", "0").strip().lower() \
                not in {"1", "true", "yes", "on"}:
            raise RuntimeError(
                "this PyTorch version does not support safe weights_only loading; "
                "upgrade PyTorch or explicitly set YOPO_ALLOW_UNSAFE_CHECKPOINT=1 "
                "for a verified local checkpoint"
            ) from exc
        print("[YOPO] WARNING: unsafe checkpoint pickle loading explicitly enabled", file=sys.stderr)
        payload = torch.load(model_path, map_location=device, weights_only=False)
    except Exception as exc:
        if os.environ.get("YOPO_ALLOW_UNSAFE_CHECKPOINT", "0").strip().lower() \
                not in {"1", "true", "yes", "on"}:
            raise RuntimeError(
                "YOPO checkpoint was rejected by the safe tensor-only loader; "
                "set YOPO_ALLOW_UNSAFE_CHECKPOINT=1 only for a verified local file"
            ) from exc
        print("[YOPO] WARNING: unsafe checkpoint pickle loading explicitly enabled", file=sys.stderr)
        payload = torch.load(model_path, map_location=device, weights_only=False)
    state_dict = payload.get("model_state_dict", payload)
    net = YopoNetwork(
        observation_dim=9,
        output_dim=10,
        hidden_state=64,
    )
    model_state = net.state_dict()
    compatible = {
        key: value for key, value in state_dict.items()
        if key in model_state
        and hasattr(value, "shape")
        and tuple(value.shape) == tuple(model_state[key].shape)
    }
    total_numel = sum(value.numel() for value in model_state.values())
    loaded_numel = sum(model_state[key].numel() for key in compatible)
    coverage = loaded_numel / max(1, total_numel)
    try:
        minimum_coverage = float(os.environ.get("YOPO_MIN_CHECKPOINT_COVERAGE", "0.99"))
    except ValueError:
        minimum_coverage = 0.99
    if coverage < minimum_coverage:
        raise RuntimeError(
            f"YOPO checkpoint coverage is too low: {coverage:.2%} < {minimum_coverage:.2%}"
        )
    incompatible = net.load_state_dict(compatible, strict=False)
    net.checkpoint_coverage = coverage
    net.checkpoint_missing_keys = len(incompatible.missing_keys)
    net.checkpoint_unexpected_keys = len(incompatible.unexpected_keys)
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
    return jsonify({
        "ok": True,
        "model": Path(str(runner.model_path)).name,
        "strategy": os.environ.get("MINDCLOUD_YOPO_STRATEGY", "baseline"),
        "device": runner.device,
        "checkpoint_coverage": runner.checkpoint_coverage,
        "checkpoint_missing_keys": runner.checkpoint_missing_keys,
        "checkpoint_unexpected_keys": runner.checkpoint_unexpected_keys,
        "config": runner.config_name,
        "base_config": runner.base_config_name,
    })


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
        reference_pose = data.get("reference_pose", data.get("reference_pos", pose))
        goal = data["goal"]    # {x,y,z}
        vel = data.get("vel", {"vx": 0, "vy": 0, "vz": 0})
        yaw = float(data.get("yaw", 0.0))
        acc = data.get("acc", {"ax": 0, "ay": 0, "az": 0})

        endstate, score, traj_time = runner.infer(
            depth_arr=depth_arr,
            pos=np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float32),
            reference_pos=np.array([
                reference_pose["x"], reference_pose["y"], reference_pose["z"]
            ], dtype=np.float32),
            vel=np.array([vel["vx"], vel["vy"], vel["vz"]], dtype=np.float32),
            acc=np.array([acc["ax"], acc["ay"], acc["az"]], dtype=np.float32),
            goal=np.array([goal["x"], goal["y"], goal["z"]], dtype=np.float32),
            yaw=yaw,
        )

        return jsonify({
            "endstate": endstate.tolist(),      # [px,vx,ax, py,vy,ay, pz,vz,az]（轴主序）
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

        # Match the authoritative ROS inference contract before YopoNetwork
        # constructs StateTransform/LatticePrimitive.  In training mode the
        # primitive silently ignores cfg["velocity"] and always uses the
        # training scales, which is incorrect for runtime profiles.
        cfg["train"] = False
        self.net = _load_model(model_path, device)
        self.checkpoint_coverage = self.net.checkpoint_coverage
        self.checkpoint_missing_keys = self.net.checkpoint_missing_keys
        self.checkpoint_unexpected_keys = self.net.checkpoint_unexpected_keys
        self.config_name = os.environ.get("YOPO_CONFIG", DEFAULT_CONFIG)
        config_path = YOPO_ROOT / "YOPO" / "config" / self.config_name
        if not config_path.is_file():
            raise FileNotFoundError(f"YOPO config missing: {config_path}")
        loaded_config_path = Path(cfg.config_path).resolve()
        if loaded_config_path != config_path.resolve():
            raise RuntimeError(
                "YOPO runtime config mismatch: imported "
                f"{loaded_config_path} but expected {config_path.resolve()}"
            )
        self.base_config_name = BASE_CONFIG
        base_config_path = YOPO_ROOT / "YOPO" / "config" / self.base_config_name
        if not base_config_path.is_file():
            raise FileNotFoundError(f"YOPO base config missing: {base_config_path}")
        # channels_last 在 RTX 5070 Ti + PyTorch 2.8 + Flask 主进程中有 200x 减速 bug，
        # 症状同 DA360_CHANNELS_LAST。默认禁用，换 GPU/PyTorch 版本可重新启用。
        self.use_amp = device == "cuda" and os.environ.get("YOPO_AMP", "1") != "0"
        self.use_channels_last = device == "cuda" and os.environ.get("YOPO_CHANNELS_LAST", "0") != "0"
        if self.use_channels_last:
            self.net = self.net.to(memory_format=torch.channels_last)
        self.width = cfg["image_width"]
        self.height = cfg["image_height"]
        self.in_channels = cfg["image_channels"]
        self.max_dis = cfg["depth_max_m"]
        self.min_dis = cfg["depth_min_m"]
        self.mask_valid_threshold = cfg["depth_mask_threshold"]
        self.fixed_height = cfg["fixed_height"]
        self.Rotation_bc = R.from_euler("z", 0, degrees=True).as_matrix()  # identity

        self.state_transform = StateTransform()
        self.traj_time = float(
            self.state_transform.lattice_primitive.segment_time
        )
        self.last_plan_diagnostics = None
        self._warmup()

    def _warmup(self):
        dummy_depth = torch.zeros(1, self.in_channels, self.height, self.width, device=self.device)
        dummy_obs = self.state_transform.prepare_input(
            torch.zeros(1, 9, device=self.device)
        )
        with torch.inference_mode():
            self.net(dummy_depth, dummy_obs)

    @torch.inference_mode()
    def infer(self, depth_arr, pos, vel, acc, goal, yaw, reference_pos=None):
        """Run YOPO inference and return the decoded best endstate.

        ``pos`` is the actual vehicle origin used to translate the decoded
        world endpoint. ``reference_pos`` is the active polynomial reference
        used only for the network goal observation (goal-reference).  Keeping
        these origins separate matches the authoritative ROS implementation.

        Returns
        -------
        endstate : np.ndarray shape [9], axis-major
                   (px,vx,ax, py,vy,ay, pz,vz,az) world-frame
        score    : float
        traj_time : float
        """
        # Never let a failed inference expose diagnostics from the previous
        # request to a caller that snapshots this field under the runner lock.
        self.last_plan_diagnostics = None

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
        depth_tensor = torch.from_numpy(stacked.reshape(1, self.in_channels, self.height, self.width))
        if self.use_channels_last:
            depth_tensor = depth_tensor.contiguous(memory_format=torch.channels_last)
        depth_input = depth_tensor.to(self.device)

        # 2. coordinate mapping: sim (east, up, north) → YOPO world (x=east, y=north, z=up)
        # Sim: x=east, y=up, z=north
        # YOPO world: x=east, y=north, z=up
        # -> YOPO x = sim_x, YOPO y = sim_z, YOPO z = sim_y
        yopo_pos = np.array([pos[0], pos[2], pos[1]], dtype=np.float32)
        if reference_pos is None:
            reference_pos = pos
        yopo_reference_pos = np.array(
            [reference_pos[0], reference_pos[2], reference_pos[1]], dtype=np.float32
        )
        yopo_vel = np.array([vel[0], vel[2], vel[1]], dtype=np.float32) if vel is not None else np.zeros(3, dtype=np.float32)
        yopo_acc = np.array([acc[0], acc[2], acc[1]], dtype=np.float32) if acc is not None else np.zeros(3, dtype=np.float32)
        yopo_goal = np.array([goal[0], goal[2], goal[1]], dtype=np.float32)

        # 高度平面跟随目标高度，对齐参考实现 test_yopo_ros.py:callback_set_goal_3d
        # (`self.fixed_height = data.pose.position.z`)。
        #
        # 这里刻意不做任何高度平移：网络输入 obs = [vel_c, acc_c, goal_c] 中
        # goal_c = R_cw·(goal − pos) 只含**相对**位移，绝对高度根本不进网络。
        # 此前的 altitude_shift 把 pos.z 和 goal.z 同减一个量，对 obs 是恒等变换
        # （数值验证误差 < 3e-15），既没有让模型"看到训练分布"，还额外引入了
        # 一次需要在输出端撤销的偏移。已删除。
        height_plane = float(yopo_goal[2])
        if not np.isfinite(height_plane):
            height_plane = float(yopo_pos[2])   # 目标高度无效时退化为保持当前高度

        # Sim yaw in degrees (0=south, +clockwise) → YOPO yaw in radians (0=east)
        # Sim yaw=0(south) → YOPO yaw=-π/2; yaw=90(west) → YOPO yaw=π
        # Mapping: yopo = -(sim_yaw + 90°) → deg2rad(-yaw - 90)
        yopo_yaw_rad = np.deg2rad(-yaw - 90.0)
        Rotation_wb = R.from_euler("Z", yopo_yaw_rad, degrees=False).as_matrix()
        Rotation_wc = np.dot(Rotation_wb, self.Rotation_bc)
        Rotation_cw = Rotation_wc.T

        vel_c = np.dot(Rotation_cw, yopo_vel)
        acc_c = np.dot(Rotation_cw, yopo_acc)
        goal_w = yopo_goal - yopo_reference_pos
        goal_c = np.dot(Rotation_cw, goal_w)

        obs = np.concatenate([vel_c, acc_c, goal_c], axis=0).astype(np.float32)
        obs_norm = self.state_transform.normalize_obs(torch.from_numpy(obs[None, :]))
        obs_input = self.state_transform.prepare_input(obs_norm.to(self.device))

        # 3. forward (AMP autocast for fp16 matmul on CUDA)
        amp_ctx = torch.cuda.amp.autocast() if self.use_amp else nullcontext()
        with amp_ctx:
            endstate_pred, score_pred = self.net(depth_input, obs_input)
        endstate_pred, score_pred = endstate_pred.cpu().numpy(), score_pred.cpu().numpy()

        # 4. decode best candidate
        traj_num = cfg["horizon_num"] * cfg["vertical_num"] * cfg["radio_num"]
        endstate_flat = endstate_pred.reshape(9, traj_num).T
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

        # 参考实现把末端直接拉到高度平面；浏览器现在会检查整段 quintic
        # 的 50m/s / 80m/s² 极值。若用户一次改变几十米高度，硬拉到目标
        # 会让每一条 1.125s 轨迹都被安全门禁拒绝，规划链永久停在 hold。
        # 因此只对这个已有的高度覆盖做单段可达限幅，连续重规划逐段逼近；
        # 网络生成的水平动作、速度和加速度保持原样，最终仍由消费端复核。
        height_error = height_plane - yopo_pos[2]
        endstate_w[:, 2, 0] = np.clip(
            height_error,
            -YOPO_MAX_VERTICAL_ENDPOINT_STEP_M,
            YOPO_MAX_VERTICAL_ENDPOINT_STEP_M,
        )

        best = endstate_w[0]
        yopo_result = np.array([
            best[0, 0] + yopo_pos[0], best[0, 1], best[0, 2],
            best[1, 0] + yopo_pos[1], best[1, 1], best[1, 2],
            best[2, 0] + yopo_pos[2], best[2, 1], best[2, 2],
        ], dtype=np.float32)

        # 5. 换回 sim 坐标系 (x=east, y=up, z=north)；保持**轴主序**
        #    [px,vx,ax, py,vy,ay, pz,vz,az]，消费方 drone.js:setYopoTrajectory
        result = np.array([
            yopo_result[0], yopo_result[1], yopo_result[2],  # px, vx, ax = YOPO x
            yopo_result[6], yopo_result[7], yopo_result[8],  # py, vy, ay = YOPO z → sim y
            yopo_result[3], yopo_result[4], yopo_result[5],  # pz, vz, az = YOPO y → sim z
        ], dtype=np.float32)

        # Compact, selected-candidate-only metadata. ``selected_endstate_raw``
        # is the nine-value normalized network output before lattice decoding:
        # [yaw_offset, pitch_offset, radial, vpx, vpy, vpz, apx, apy, apz].
        # NumPy scalars/arrays are converted here so the HTTP layer never has
        # to serialize framework-specific values or all candidate tensors.
        selected_endstate_raw = np.asarray(
            endstate_flat[action_id], dtype=np.float64
        ).reshape(9)
        terminal_velocity = result[[1, 4, 7]].astype(np.float64)
        terminal_acceleration = result[[2, 5, 8]].astype(np.float64)
        endpoint_displacement = (
            result[[0, 3, 6]].astype(np.float64)
            - np.asarray(pos, dtype=np.float64).reshape(3)
        )
        diagnostic_values = np.concatenate([
            selected_endstate_raw,
            terminal_velocity,
            terminal_acceleration,
            endpoint_displacement,
            np.array([score, self.traj_time], dtype=np.float64),
        ])
        if not np.all(np.isfinite(diagnostic_values)):
            raise RuntimeError("YOPO selected candidate diagnostics are non-finite")
        primitive = self.state_transform.lattice_primitive
        self.last_plan_diagnostics = {
            "selected_endstate_raw": selected_endstate_raw.tolist(),
            "selected_candidate_id": int(action_id),
            "selected_action_id": int(action_id),
            "selected_lattice_id": int(lattice_id),
            "selected_score": float(score),
            "terminal_speed_mps": float(np.linalg.norm(terminal_velocity)),
            "terminal_acceleration_mps2": float(
                np.linalg.norm(terminal_acceleration)
            ),
            "endpoint_displacement_m": float(
                np.linalg.norm(endpoint_displacement)
            ),
            "trajectory_time_s": float(self.traj_time),
            "candidate_count": int(traj_num),
            "velocity_scale_mps": float(primitive.vel_max),
            "acceleration_scale_mps2": float(primitive.acc_max),
        }

        return result, score, self.traj_time


# ── main ──────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="YOPO planning bridge API")
    p.add_argument("--model-path", default=DEFAULT_MODEL)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", default=5699, type=int)
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


def main():
    global runner
    args = parse_args()
    runner = YopoRunner(args.model_path)
    print(f"[yopo-bridge] model loaded: {runner.model_path}, device={runner.device}")
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=False)


if __name__ == "__main__":
    main()
