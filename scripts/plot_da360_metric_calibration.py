#!/usr/bin/env python3
"""从公开脱敏摘要生成 DA360 米制标定中文数据图。"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch


SITE_COLORS = {
    "A": "#3775BA",
    "B": "#E58B2A",
    "C": "#8B6BB8",
    "D": "#42949E",
}
BLUE = "#3775BA"
RED = "#C44E52"
GREEN = "#2A8C62"
INK = "#263238"
MUTED = "#66737F"
GRID = "#DCE3E8"


def register_cjk_font():
    candidates = (
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf"),
        Path("/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"),
    )
    for path in candidates:
        if path.exists():
            font_manager.fontManager.addfont(path)
            return font_manager.FontProperties(fname=path).get_name()
    raise RuntimeError("未找到可用于简体中文绘图的 Noto Sans CJK 字体")


def configure_style():
    cjk_family = register_cjk_font()
    mpl.rcParams.update({
        "font.family": cjk_family,
        "font.sans-serif": [
            cjk_family,
            "Noto Sans CJK SC",
            "Source Han Sans SC",
            "Microsoft YaHei",
            "SimHei",
            "DejaVu Sans",
        ],
        "font.size": 9,
        "axes.titlesize": 12,
        "axes.labelsize": 10,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.9,
        "svg.fonttype": "none",
        "svg.hashsalt": "mindcloud-da360-metric-calibration",
        "pdf.fonttype": 42,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
    })


def panel_label(ax, label):
    ax.text(
        -0.10,
        1.04,
        label,
        transform=ax.transAxes,
        fontsize=13,
        fontweight="bold",
        color=INK,
        ha="left",
        va="bottom",
    )


def metric_card(ax, x, y, width, height, title, value, gate, passed):
    color = GREEN if passed else RED
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.014,rounding_size=0.018",
        transform=ax.transAxes,
        linewidth=1.1,
        edgecolor=color,
        facecolor="#F4FAF7" if passed else "#FFF6F5",
    )
    ax.add_patch(patch)
    ax.text(x + 0.04, y + height * 0.70, title, transform=ax.transAxes,
            fontsize=8.5, color=MUTED, va="center")
    ax.text(x + 0.04, y + height * 0.39, value, transform=ax.transAxes,
            fontsize=13, fontweight="bold", color=color, va="center")
    ax.text(x + 0.04, y + height * 0.14, gate, transform=ax.transAxes,
            fontsize=7.5, color=MUTED, va="center")


def make_figure(data):
    configure_style()
    fig = plt.figure(figsize=(14.2, 7.7))
    grid = fig.add_gridspec(
        2,
        2,
        left=0.065,
        right=0.975,
        bottom=0.10,
        top=0.82,
        width_ratios=[1.35, 1.0],
        height_ratios=[1.0, 0.92],
        wspace=0.25,
        hspace=0.40,
    )
    ax_scale = fig.add_subplot(grid[:, 0])
    ax_lolo = fig.add_subplot(grid[0, 1])
    ax_summary = fig.add_subplot(grid[1, 1])

    fig.suptitle(
        "DA360 米制尺度标定：固定全局尺度未通过跨地点验证",
        x=0.065,
        y=0.955,
        ha="left",
        fontsize=20,
        fontweight="bold",
        color=INK,
    )
    collection = data["collection"]
    fig.text(
        0.065,
        0.895,
        (
            f"{collection['sites']} 个地点 · {collection['captures']} 次采集 · "
            f"{collection['anchorsInRange']} 个 0.5–20 m 锚点 · "
            "部署公式 1/D = a·p（b = 0）"
        ),
        ha="left",
        fontsize=10.5,
        color=MUTED,
    )

    captures = data["captures"]
    x = np.arange(len(captures))
    scales = np.array([item["scaleA"] for item in captures]) * 1e3
    labels = [item["id"] for item in captures]
    for site in SITE_COLORS:
        indices = [i for i, item in enumerate(captures) if item["site"] == site]
        values = scales[indices]
        ax_scale.plot(indices, values, color=SITE_COLORS[site], lw=2.0, alpha=0.72)
        ax_scale.scatter(indices, values, s=78, color=SITE_COLORS[site],
                         edgecolor="white", linewidth=1.0, zorder=3,
                         label=f"地点 {site}")
    global_scale = data["deployment"]["a"] * 1e3
    ax_scale.axhline(global_scale, color=INK, ls="--", lw=1.5,
                     label=f"全局尺度 a = {global_scale:.3f}×10^-3")
    ratio = float(scales.max() / scales.min())
    max_index = int(np.argmax(scales))
    ax_scale.annotate(
        f"最大/最小 = {ratio:.2f}×\n{labels[max_index]} 为主要异常点",
        xy=(max_index, scales[max_index]),
        xytext=(max_index + 1.0, scales[max_index] - 0.12),
        arrowprops={"arrowstyle": "->", "color": RED, "lw": 1.2},
        color=RED,
        fontsize=9,
        ha="left",
        va="top",
    )
    ax_scale.set_title("单帧尺度随采集场景变化", loc="left", fontweight="bold", pad=12)
    ax_scale.set_ylabel("单帧拟合尺度 a（×10^-3）")
    ax_scale.set_xlabel("采集编号（地点 + 次序）")
    ax_scale.set_xticks(x)
    ax_scale.set_xticklabels(labels, rotation=45, ha="right", rotation_mode="anchor")
    ax_scale.set_ylim(0.5, 3.45)
    ax_scale.grid(axis="y", color=GRID, lw=0.8)
    ax_scale.legend(loc="upper left", ncol=2, fontsize=8, frameon=False)
    panel_label(ax_scale, "a")

    folds = data["leaveOneLocationOut"]
    sites = [item["site"] for item in folds]
    n_values = [item["n"] for item in folds]
    median = np.array([item["medianAbsRel"] for item in folds]) * 100
    p90 = np.array([item["p90AbsRel"] for item in folds]) * 100
    fx = np.arange(len(sites))
    for xi, low, high in zip(fx, median, p90):
        ax_lolo.plot([xi, xi], [low, high], color="#B6C0C8", lw=1.5, zorder=1)
    ax_lolo.scatter(fx - 0.07, median, s=62, color=BLUE, marker="o",
                    edgecolor="white", linewidth=0.8, zorder=3, label="中位 AbsRel")
    ax_lolo.scatter(fx + 0.07, p90, s=62, color=RED, marker="s",
                    edgecolor="white", linewidth=0.8, zorder=3, label="P90 AbsRel")
    gates = data["gates"]
    ax_lolo.axhline(gates["medianAbsRelMax"] * 100, color=BLUE, ls="--", lw=1.2,
                    label="中位门槛 15%")
    ax_lolo.axhline(gates["p90AbsRelMax"] * 100, color=RED, ls=":", lw=1.5,
                    label="P90 门槛 30%")
    for xi, med, tail in zip(fx, median, p90):
        ax_lolo.text(xi - 0.08, med + 6, f"{med:.1f}%", ha="right", va="bottom",
                     fontsize=7.5, color=BLUE)
        tail_y = tail - 9 if tail > 195 else tail + 6
        tail_va = "top" if tail > 195 else "bottom"
        ax_lolo.text(xi + 0.08, tail_y, f"{tail:.1f}%", ha="left", va=tail_va,
                     fontsize=7.5, color=RED)
    ax_lolo.set_title("四地点留一验证均未通过误差门槛", loc="left",
                      fontweight="bold", pad=10)
    ax_lolo.set_ylabel("绝对相对误差")
    ax_lolo.set_xticks(fx)
    ax_lolo.set_xticklabels([f"地点 {s}\nn={n}" for s, n in zip(sites, n_values)])
    ax_lolo.set_ylim(0, 235)
    ax_lolo.set_yticks([0, 50, 100, 150, 200])
    ax_lolo.set_yticklabels(["0%", "50%", "100%", "150%", "200%"])
    ax_lolo.grid(axis="y", color=GRID, lw=0.8)
    ax_lolo.legend(loc="upper left", ncol=2, fontsize=7.5, frameon=False)
    panel_label(ax_lolo, "b")

    ax_summary.set_axis_off()
    panel_label(ax_summary, "c")
    ax_summary.text(0.0, 1.02, "自动精度门禁结果", transform=ax_summary.transAxes,
                    fontsize=12, fontweight="bold", color=INK, va="bottom")
    global_metrics = data["globalScaleOnly"]
    coverage = collection["validAnchorFraction"]
    metric_card(ax_summary, 0.00, 0.56, 0.47, 0.34,
                "有效锚点覆盖率", f"{coverage * 100:.1f}%",
                "门槛：≥ 70%（通过）", coverage >= gates["validAnchorFractionMin"])
    metric_card(ax_summary, 0.51, 0.56, 0.47, 0.34,
                "全局中位 AbsRel", f"{global_metrics['medianAbsRel'] * 100:.1f}%",
                "门槛：≤ 15%（未通过）", False)
    metric_card(ax_summary, 0.00, 0.16, 0.47, 0.34,
                "全局 P90 AbsRel", f"{global_metrics['p90AbsRel'] * 100:.1f}%",
                "门槛：≤ 30%（未通过）", False)
    metric_card(ax_summary, 0.51, 0.16, 0.47, 0.34,
                "10 m 内 P90 误差", f"{global_metrics['near10mP90ErrorM']:.2f} m",
                "门槛：≤ 1 m（未通过）", False)
    ax_summary.text(
        0.0,
        0.02,
        "结论：当前固定尺度仅作为人工检查后的 sim-to-sim 基线，\n"
        "不代表自动精度门禁、跨场景尺度或真实传感器标定已经通过。",
        transform=ax_summary.transAxes,
        fontsize=8.5,
        color=INK,
        va="bottom",
        linespacing=1.35,
    )
    return fig


def main():
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data",
        type=Path,
        default=repo_root / "docs/data/da360-metric-calibration-summary.json",
    )
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=repo_root / "docs/assets/figures/da360-metric-calibration.zh-CN",
    )
    args = parser.parse_args()
    data = json.loads(args.data.read_text(encoding="utf-8"))
    if data["deployment"]["b"] != 0.0:
        raise ValueError("公开 DA360 标定图只接受 b=0 的 scale-only 数据")
    fig = make_figure(data)
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    output = lambda extension: args.output_prefix.parent / (
        args.output_prefix.name + extension
    )
    fig.savefig(output(".png"), dpi=360, bbox_inches="tight")
    svg_path = output(".svg")
    fig.savefig(svg_path, bbox_inches="tight")
    svg_text = svg_path.read_text(encoding="utf-8")
    svg_path.write_text(
        "\n".join(line.rstrip() for line in svg_text.splitlines()) + "\n",
        encoding="utf-8",
    )
    fig.savefig(output(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
