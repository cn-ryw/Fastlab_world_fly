#!/usr/bin/env python3
"""Create public, privacy-safe figures and an aggregate summary from flight logs."""

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

CJK_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(CJK_FONT_PATH)
CJK_FONT_NAME = font_manager.FontProperties(fname=CJK_FONT_PATH).get_name()

plt.rcParams.update({
    "font.family": CJK_FONT_NAME,
    "font.sans-serif": [CJK_FONT_NAME, "DejaVu Sans", "sans-serif"],
    "font.size": 7.0,
    "axes.titlesize": 8.0,
    "axes.labelsize": 7.0,
    "xtick.labelsize": 6.5,
    "ytick.labelsize": 6.5,
    "legend.fontsize": 6.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "svg.fonttype": "none",
    "pdf.fonttype": 42,
})

BLUE = "#1769aa"
TEAL = "#07847e"
ORANGE = "#e87500"
GRAY = "#667085"
RED = "#c93c37"


def finite(value, default=0.0):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def percentile(values, q):
    clean = [finite(v, float("nan")) for v in values]
    clean = [v for v in clean if math.isfinite(v)]
    return float(np.percentile(clean, q)) if clean else 0.0


def load_run(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("schemaVersion") != 2:
        raise ValueError(f"{path}: expected schemaVersion=2")
    frames = payload.get("frames") or []
    goal = payload.get("goal") or {}
    if not frames or not all(k in goal for k in ("x", "y", "z")):
        raise ValueError(f"{path}: missing frames or goal")

    t = np.asarray([finite(f.get("t")) for f in frames])
    x = np.asarray([finite(f.get("x")) for f in frames])
    y = np.asarray([finite(f.get("y")) for f in frames])
    z = np.asarray([finite(f.get("z")) for f in frames])
    vx = np.asarray([finite(f.get("vx")) for f in frames])
    vy = np.asarray([finite(f.get("vy")) for f in frames])
    vz = np.asarray([finite(f.get("vz")) for f in frames])
    speed = np.sqrt(vx * vx + vy * vy + vz * vz)
    gx, gy, gz = (finite(goal[k]) for k in ("x", "y", "z"))
    distance = np.sqrt((x - gx) ** 2 + (y - gy) ** 2 + (z - gz) ** 2)

    perception = payload.get("perception") or []
    applied = [event for event in perception if event.get("trajectoryApplied") is True or event.get("outcome") == "applied"]
    perf = payload.get("perf") or {}
    duration = finite(payload.get("duration_s"), t[-1] if len(t) else 0.0)
    computed_hz = len({event.get("frameId") for event in applied if event.get("frameId") is not None}) / duration if duration > 0 else 0.0
    planning_hz = finite(perf.get("trajectoryInstallHz"), finite(perf.get("uniquePlanningHz"), computed_hz))
    if planning_hz <= 0:
        planning_hz = computed_hz

    cta = [event.get("captureToApplyMs") for event in applied if event.get("captureToApplyMs") is not None]
    da360 = [event.get("da360Ms") for event in perception if event.get("da360Ms") is not None]
    yopo = [event.get("yopoMs") for event in perception if event.get("yopoMs") is not None]
    capture_to_apply_p95 = finite(perf.get("captureToApplyP95Ms"), percentile(cta, 95))
    da360_p95 = finite(perf.get("da360P95Ms"), percentile(da360, 95))
    yopo_p95 = finite(perf.get("yopoP95Ms"), percentile(yopo, 95))

    stamp = str(payload.get("startTime", "unknown")).replace("-", "").replace(":", "")
    log_id = stamp[:8] + "-" + stamp[9:15] if len(stamp) >= 15 else Path(path).stem.replace("flight-log-", "").replace("T", "-").replace("-", "")
    if "13:02" in str(payload.get("startTime")):
        log_id = "20260814-130247"
    elif "13:03" in str(payload.get("startTime")):
        log_id = "20260814-130423"

    return {
        "payload": payload,
        "log_id": log_id,
        "t": t,
        "x": x,
        "y": y,
        "z": z,
        "speed": speed,
        "distance": distance,
        "goal": (gx, gy, gz),
        "applied": applied,
        "duration_s": duration,
        "planning_hz": planning_hz,
        "capture_to_apply_p95_ms": capture_to_apply_p95,
        "da360_p95_ms": da360_p95,
        "yopo_p95_ms": yopo_p95,
    }


def save_figure(fig, stem):
    fig.savefig(stem.with_suffix(".png"), dpi=300, bbox_inches="tight", facecolor="white")
    svg_path = stem.with_suffix(".svg")
    fig.savefig(svg_path, bbox_inches="tight", facecolor="white")
    svg_text = svg_path.read_text(encoding="utf-8")
    svg_path.write_text(
        "\n".join(line.rstrip() for line in svg_text.splitlines()) + "\n",
        encoding="utf-8",
    )
    fig.savefig(stem.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_run(run, output_dir):
    fig = plt.figure(figsize=(7.2, 4.8), constrained_layout=True)
    grid = fig.add_gridspec(2, 2)
    ax_path = fig.add_subplot(grid[:, 0])
    ax_dist = fig.add_subplot(grid[0, 1])
    ax_speed = fig.add_subplot(grid[1, 1])

    scatter = ax_path.scatter(run["x"], run["z"], c=run["t"], s=5, cmap="viridis", linewidths=0, rasterized=True)
    ax_path.plot(run["x"], run["z"], color=BLUE, linewidth=0.7, alpha=0.5)
    ax_path.scatter(run["x"][0], run["z"][0], marker="o", s=28, color=TEAL, label="起点", zorder=4)
    gx, gy, gz = run["goal"]
    ax_path.scatter(gx, gz, marker="*", s=70, color=ORANGE, edgecolor="white", linewidth=0.6, label="目标", zorder=5)
    ax_path.add_patch(plt.Circle((gx, gz), 4.0, fill=False, color=ORANGE, linestyle="--", linewidth=1.0, label="4 m 到达半径"))
    ax_path.set_title("a  水平飞行轨迹（全部物理帧）", loc="left", fontweight="bold")
    ax_path.set_xlabel("世界 X / m")
    ax_path.set_ylabel("世界 Z / m")
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.grid(alpha=0.18, linewidth=0.5)
    ax_path.legend(loc="best")
    cbar = fig.colorbar(scatter, ax=ax_path, fraction=0.046, pad=0.03)
    cbar.set_label("时间 / s")

    ax_dist.plot(run["t"], run["distance"], color=TEAL, linewidth=1.2)
    ax_dist.axhline(4.0, color=ORANGE, linestyle="--", linewidth=1.0, label="4 m 到达半径")
    ax_dist.scatter(run["t"][np.argmin(run["distance"])], np.min(run["distance"]), color=ORANGE, s=20, zorder=3)
    ax_dist.set_title("b  目标距离", loc="left", fontweight="bold")
    ax_dist.set_xlabel("时间 / s")
    ax_dist.set_ylabel("三维距离 / m")
    ax_dist.grid(alpha=0.18, linewidth=0.5)
    ax_dist.legend(loc="best")

    ax_speed.plot(run["t"], run["speed"], color=BLUE, linewidth=1.1)
    ax_speed.fill_between(run["t"], 0, run["speed"], color=BLUE, alpha=0.12)
    ax_speed.set_title("c  三维速度", loc="left", fontweight="bold")
    ax_speed.set_xlabel("时间 / s")
    ax_speed.set_ylabel("速度 / m/s")
    ax_speed.grid(alpha=0.18, linewidth=0.5)

    fig.suptitle(
        f"零碰撞成功飞行 {run['log_id']}  |  规划 {run['planning_hz']:.1f} Hz  |  Capture-to-Apply p95 {run['capture_to_apply_p95_ms']:.0f} ms",
        fontsize=9.0,
        fontweight="bold",
    )
    save_figure(fig, output_dir / f"demo-flight-{run['log_id']}")


def plot_comparison(runs, output_dir):
    labels = [run["log_id"][-6:] for run in runs]
    metrics = [
        ("轨迹安装频率", "Hz", [run["planning_hz"] for run in runs], 15.0),
        ("Capture-to-Apply p95", "ms", [run["capture_to_apply_p95_ms"] for run in runs], 150.0),
        ("DA360 p95", "ms", [run["da360_p95_ms"] for run in runs], 50.0),
        ("YOPO p95", "ms", [run["yopo_p95_ms"] for run in runs], 10.0),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(7.2, 2.45), constrained_layout=True)
    colors = [TEAL, BLUE]
    for index, (ax, (title, unit, values, target)) in enumerate(zip(axes, metrics)):
        bars = ax.bar(labels, values, color=colors[:len(values)], width=0.58)
        ax.axhline(target, color=ORANGE, linestyle="--", linewidth=1.0, label=f"参考 {target:g} {unit}")
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.1f}", ha="center", va="bottom", fontsize=6.5)
        ax.set_title(chr(ord("a") + index) + "  " + title, loc="left", fontweight="bold")
        ax.set_ylabel(unit)
        ax.grid(axis="y", alpha=0.18, linewidth=0.5)
        ax.legend(loc="best")
    fig.suptitle("两次零碰撞成功到达：规划与推理时延比较", fontsize=9.0, fontweight="bold")
    fig.text(0.5, -0.01, "橙色虚线为目标或门禁参考；两次短时演示均未通过持续 15 Hz 正式验收。", ha="center", fontsize=6.5, color=GRAY)
    save_figure(fig, output_dir / "demo-flight-comparison")


def public_summary(runs):
    result = []
    for run in runs:
        payload = run["payload"]
        reasons = []
        if run["duration_s"] < 60.0:
            reasons.append("duration_below_60s")
        if run["planning_hz"] < 15.0:
            reasons.append("planning_rate_below_15hz")
        result.append({
            "log_id": run["log_id"],
            "navigation_kind": payload.get("navigationKind"),
            "arrived": bool(payload.get("arrived")),
            "collision_count": len(payload.get("collisions") or []),
            "duration_s": round(run["duration_s"], 3),
            "planning_rate_hz": round(run["planning_hz"], 3),
            "capture_to_apply_p95_ms": round(run["capture_to_apply_p95_ms"], 3),
            "da360_p95_ms": round(run["da360_p95_ms"], 3),
            "yopo_p95_ms": round(run["yopo_p95_ms"], 3),
            "max_speed_mps": round(float(np.max(run["speed"])), 3),
            "min_goal_distance_m": round(float(np.min(run["distance"])), 3),
            "formal_gate_passed": False,
            "formal_gate_failure_reasons": reasons,
        })
    return {
        "schema_version": 1,
        "claim": "Two collision-free simulator runs reached the 4 m goal region.",
        "formal_acceptance_note": "Demo evidence only; not a sustained 15 Hz or real-flight acceptance result.",
        "runs": result,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", help="schema-v2 flight logs")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    runs = [load_run(path) for path in args.logs]
    for run in runs:
        plot_run(run, args.output_dir)
    plot_comparison(runs, args.output_dir)
    summary = public_summary(runs)
    args.summary.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
