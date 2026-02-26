#!/usr/bin/env python3
"""
Generate an MP4 video with:
1) XY trajectory playback
2) Z-axis variation over time

Input CSV must contain columns:
  t_s, gt_x_m, gt_y_m, gt_z_m
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
from pathlib import Path
from typing import List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter


def load_series(csv_path: Path) -> Tuple[List[float], List[float], List[float], List[float]]:
    t: List[float] = []
    x: List[float] = []
    y: List[float] = []
    z: List[float] = []

    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t.append(float(row["t_s"]))
                x.append(float(row["gt_x_m"]))
                y.append(float(row["gt_y_m"]))
                z.append(float(row["gt_z_m"]))
            except (KeyError, ValueError):
                continue

    if len(t) < 2:
        raise RuntimeError(f"not enough valid rows in {csv_path}")
    return t, x, y, z


def _expand_range(vmin: float, vmax: float, ratio: float = 0.08) -> Tuple[float, float]:
    if not math.isfinite(vmin) or not math.isfinite(vmax):
        return -1.0, 1.0
    span = max(vmax - vmin, 1e-6)
    pad = span * ratio
    return vmin + (-pad), vmax + pad


def render_video(
    t: List[float],
    x: List[float],
    y: List[float],
    z: List[float],
    output: Path,
    fps: int,
    stride: int,
    title: str,
) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg not found in PATH")

    indices = list(range(0, len(t), max(1, stride)))
    if indices[-1] != len(t) - 1:
        indices.append(len(t) - 1)

    x_min, x_max = _expand_range(min(x), max(x), 0.08)
    y_min, y_max = _expand_range(min(y), max(y), 0.08)
    z_min, z_max = _expand_range(min(z), max(z), 0.12)

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0], wspace=0.22)
    ax_xy = fig.add_subplot(gs[0, 0])
    ax_z = fig.add_subplot(gs[0, 1])

    # static style
    ax_xy.set_title("Trajectory (map XY)")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.set_xlim(x_min, x_max)
    ax_xy.set_ylim(y_min, y_max)
    ax_xy.grid(alpha=0.25)
    ax_xy.set_aspect("equal", adjustable="box")

    ax_z.set_title("Z variation vs time")
    ax_z.set_xlabel("time (s)")
    ax_z.set_ylabel("z (m)")
    ax_z.set_xlim(t[0], t[-1])
    ax_z.set_ylim(z_min, z_max)
    ax_z.grid(alpha=0.25)

    # static background lines
    ax_xy.plot(x, y, color="#cbd5e1", linewidth=1.4, label="full path")
    z_baseline = sum(z) / len(z)
    ax_z.axhline(z_baseline, color="#94a3b8", linewidth=1.2, linestyle="--", label=f"mean z={z_baseline:.3f}m")

    # dynamic artists
    (traj_line,) = ax_xy.plot([], [], color="#2563eb", linewidth=2.4, label="traversed")
    (traj_head,) = ax_xy.plot([], [], marker="o", markersize=7, color="#dc2626", linestyle="None", label="robot")
    (z_line,) = ax_z.plot([], [], color="#0f766e", linewidth=2.2, label="z")
    (z_head,) = ax_z.plot([], [], marker="o", markersize=7, color="#dc2626", linestyle="None")

    txt = fig.text(0.03, 0.95, "", fontsize=12, family="monospace")
    fig.suptitle(title, fontsize=18, y=0.995)
    ax_xy.legend(loc="best", fontsize=9)
    ax_z.legend(loc="best", fontsize=9)

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=fps, codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])

    with writer.saving(fig, output.as_posix(), dpi=120):
        for k, idx in enumerate(indices, start=1):
            traj_line.set_data(x[: idx + 1], y[: idx + 1])
            traj_head.set_data([x[idx]], [y[idx]])
            z_line.set_data(t[: idx + 1], z[: idx + 1])
            z_head.set_data([t[idx]], [z[idx]])

            txt.set_text(
                f"sample={idx + 1}/{len(t)}  frame={k}/{len(indices)}  "
                f"t={t[idx]:.2f}s  x={x[idx]:.3f}m  y={y[idx]:.3f}m  z={z[idx]:.3f}m"
            )
            writer.grab_frame()

    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create trajectory + z variation mp4 from loc_vs_ground_truth CSV.")
    parser.add_argument("--csv", type=Path, required=True, help="CSV path, e.g. analysis_drift/loc_vs_ground_truth.csv")
    parser.add_argument("--output", type=Path, required=True, help="output MP4 path")
    parser.add_argument("--fps", type=int, default=25, help="video fps")
    parser.add_argument("--stride", type=int, default=1, help="sample stride to speed up rendering")
    parser.add_argument("--title", type=str, default="Wave-Road Trajectory and Z Variation")
    args = parser.parse_args()

    t, x, y, z = load_series(args.csv.resolve())
    render_video(t, x, y, z, args.output.resolve(), args.fps, args.stride, args.title)
    print(f"[ok] video: {args.output.resolve()}")


if __name__ == "__main__":
    main()
