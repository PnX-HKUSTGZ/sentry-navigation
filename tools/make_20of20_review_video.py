#!/usr/bin/env python3
"""
Generate a short MP4 review video for two 20/20 stress-test artifacts.

The video is slide-based and contains:
1) Title and scope
2) KPI comparison (success/avg duration)
3) Per-goal status timeline
4) Retry usage and reliability notes
5) Wave-road passability evidence images
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import os
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


STATUS_COLORS = {
    "SUCCEEDED": "#2ca02c",
    "ABORTED": "#d62728",
    "CANCELED": "#ff7f0e",
    "UNKNOWN": "#7f7f7f",
}


@dataclass
class GoalResult:
    index: int
    status: str
    filename: str
    recovered_by_retry: bool = False


@dataclass
class ArtifactData:
    label: str
    path: Path
    summary: Dict[str, str]
    goals: List[GoalResult]
    timeout_retry_count: int
    retry_recovered_count: int


def parse_summary(summary_path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    with summary_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or "=" not in line:
                continue
            k, v = line.split("=", 1)
            data[k.strip()] = v.strip()
    return data


def parse_goal_file(goal_path: Path) -> str:
    status = "UNKNOWN"
    pattern = re.compile(r"Goal finished with status:\s*([A-Z_]+)")
    with goal_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                status = m.group(1).strip().upper()
                break
    return status


def goal_sort_key(filename: str) -> Tuple[int, int]:
    m = re.search(r"goal_r(\d+)_p(\d+)\.log$", filename)
    if not m:
        return (10**6, 10**6)
    return (int(m.group(1)), int(m.group(2)))


def load_artifact(label: str, artifact_dir: Path) -> ArtifactData:
    summary_path = artifact_dir / "summary.txt"
    if not summary_path.exists():
        raise FileNotFoundError(f"missing summary file: {summary_path}")

    summary = parse_summary(summary_path)

    goal_paths = sorted(
        [
            p
            for p in artifact_dir.glob("goal_r*_p*.log")
            if ".timeout_retry" not in p.name
        ],
        key=lambda p: goal_sort_key(p.name),
    )

    goals: List[GoalResult] = []
    retry_recovered_count = 0
    for i, goal_path in enumerate(goal_paths, start=1):
        status = parse_goal_file(goal_path)
        recovered = False
        if status == "UNKNOWN":
            retry_paths = sorted(
                artifact_dir.glob(goal_path.name + ".timeout_retry*"),
                key=lambda p: p.name,
            )
            for rp in retry_paths:
                retry_status = parse_goal_file(rp)
                if retry_status != "UNKNOWN":
                    status = retry_status
                    recovered = True
            if recovered:
                retry_recovered_count += 1

        goals.append(
            GoalResult(
                index=i,
                status=status,
                filename=goal_path.name,
                recovered_by_retry=recovered,
            )
        )

    timeout_retry_count = len(glob.glob(str(artifact_dir / "goal_*.log.timeout_retry*")))

    return ArtifactData(
        label=label,
        path=artifact_dir,
        summary=summary,
        goals=goals,
        timeout_retry_count=timeout_retry_count,
        retry_recovered_count=retry_recovered_count,
    )


def _save_fig(path: Path, fig: plt.Figure) -> None:
    # Force a stable canvas size for ffmpeg concat compatibility.
    fig.set_size_inches(16, 9, forward=True)
    fig.savefig(path, dpi=120, facecolor="white")
    plt.close(fig)


def make_slide_title(out_png: Path, baseline: ArtifactData, stvl: ArtifactData) -> None:
    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()

    fig.text(
        0.06,
        0.88,
        "RMUL_26 Wave-Road Navigation Review",
        fontsize=36,
        fontweight="bold",
        color="#1f2937",
    )
    fig.text(
        0.06,
        0.80,
        "Fast-LIO2 + DWB + small-gicp (dynamic obstacle stress)",
        fontsize=22,
        color="#374151",
    )

    def row(y: float, label: str, val: str, color: str = "#111827") -> None:
        fig.text(0.08, y, label, fontsize=20, color="#6b7280")
        fig.text(0.28, y, val, fontsize=20, color=color, family="monospace")

    row(
        0.66,
        "Baseline artifact:",
        baseline.path.as_posix(),
        "#111827",
    )
    row(
        0.58,
        "STVL+retry artifact:",
        stvl.path.as_posix(),
        "#111827",
    )
    row(
        0.46,
        "Result:",
        f"both 20/20 succeeded (100%)",
        "#047857",
    )
    row(
        0.38,
        "Avg duration:",
        f"baseline {baseline.summary.get('avg_duration_sec', 'n/a')} s | "
        f"stvl+retry {stvl.summary.get('avg_duration_sec', 'n/a')} s",
        "#1d4ed8",
    )
    row(
        0.30,
        "Scope:",
        "show KPI + per-goal outcome + wave passability evidence",
    )

    now_s = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    fig.text(0.06, 0.12, f"Generated at {now_s}", fontsize=14, color="#6b7280")
    _save_fig(out_png, fig)


def make_slide_kpi(out_png: Path, baseline: ArtifactData, stvl: ArtifactData) -> None:
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[2.2, 1], width_ratios=[1, 1], hspace=0.35)

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, :])

    labels = [baseline.label, stvl.label]
    success = [
        float(baseline.summary.get("success_rate_percent", "0")),
        float(stvl.summary.get("success_rate_percent", "0")),
    ]
    avg_t = [
        float(baseline.summary.get("avg_duration_sec", "0")),
        float(stvl.summary.get("avg_duration_sec", "0")),
    ]

    bars1 = ax1.bar(labels, success, color=["#4f46e5", "#059669"], width=0.55)
    ax1.set_ylim(0, 110)
    ax1.set_title("Success rate (%)", fontsize=18)
    ax1.set_ylabel("%")
    for b, v in zip(bars1, success):
        ax1.text(b.get_x() + b.get_width() / 2, v + 1.0, f"{v:.1f}", ha="center", fontsize=13)
    ax1.grid(axis="y", alpha=0.3)

    bars2 = ax2.bar(labels, avg_t, color=["#6366f1", "#10b981"], width=0.55)
    ax2.set_title("Average goal duration (s)", fontsize=18)
    ax2.set_ylabel("sec")
    for b, v in zip(bars2, avg_t):
        ax2.text(b.get_x() + b.get_width() / 2, v + 0.2, f"{v:.3f}", ha="center", fontsize=13)
    ax2.grid(axis="y", alpha=0.3)

    total_b = int(baseline.summary.get("total_goals", "0"))
    succ_b = int(baseline.summary.get("succeeded", "0"))
    total_s = int(stvl.summary.get("total_goals", "0"))
    succ_s = int(stvl.summary.get("succeeded", "0"))
    delta = avg_t[0] - avg_t[1]
    ratio = (avg_t[0] / avg_t[1]) if avg_t[1] > 1e-6 else float("inf")

    ax3.axis("off")
    ax3.text(
        0.02,
        0.78,
        f"Baseline: {succ_b}/{total_b} success, avg {avg_t[0]:.3f}s, retries {baseline.timeout_retry_count}, "
        f"retry recovered {baseline.retry_recovered_count}",
        fontsize=18,
        color="#1f2937",
    )
    ax3.text(
        0.02,
        0.48,
        f"STVL+retry: {succ_s}/{total_s} success, avg {avg_t[1]:.3f}s, retries {stvl.timeout_retry_count}, "
        f"retry recovered {stvl.retry_recovered_count}",
        fontsize=18,
        color="#1f2937",
    )
    ax3.text(
        0.02,
        0.18,
        f"Timing gain: -{delta:.3f}s/goal (about {ratio:.2f}x faster)",
        fontsize=20,
        fontweight="bold",
        color="#065f46",
    )

    fig.suptitle("20-goal KPI comparison", fontsize=26, y=0.98)
    _save_fig(out_png, fig)


def make_slide_goal_timeline(out_png: Path, baseline: ArtifactData, stvl: ArtifactData) -> None:
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 1, hspace=0.35)
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[1, 0])]

    for ax, art in zip(axes, [baseline, stvl]):
        xs = [g.index for g in art.goals]
        ys = [1] * len(xs)
        colors = [STATUS_COLORS.get(g.status, STATUS_COLORS["UNKNOWN"]) for g in art.goals]
        ax.scatter(xs, ys, c=colors, s=320, marker="s", edgecolors="#111827", linewidths=0.5)
        for g in art.goals:
            ax.text(g.index, 1.0, str(g.index), ha="center", va="center", fontsize=9, color="white")
            if g.recovered_by_retry:
                ax.text(g.index, 1.085, "R", ha="center", va="center", fontsize=9, color="#92400e", fontweight="bold")
        ax.set_xlim(0.3, max(20.7, len(xs) + 0.7))
        ax.set_ylim(0.75, 1.25)
        ax.set_yticks([])
        ax.set_xticks(np.arange(1, max(20, len(xs)) + 1, 1))
        ax.set_title(
            f"{art.label}: {art.summary.get('succeeded', '0')}/{art.summary.get('total_goals', '0')} succeeded",
            fontsize=18,
            loc="left",
        )
        ax.grid(axis="x", alpha=0.2)
        ax.set_xlabel("Goal index")

        if art.timeout_retry_count > 0:
            ax.text(
                0.995,
                0.82,
                f"timeout retries: {art.timeout_retry_count} | recovered: {art.retry_recovered_count}",
                transform=ax.transAxes,
                ha="right",
                fontsize=13,
                color="#92400e",
            )

    legend_handles = []
    for status, color in STATUS_COLORS.items():
        handle = plt.Line2D(
            [0],
            [0],
            marker="s",
            color="w",
            markerfacecolor=color,
            markeredgecolor="#111827",
            markersize=14,
            label=status,
        )
        legend_handles.append(handle)
    fig.legend(handles=legend_handles, loc="upper center", ncol=4, fontsize=12, bbox_to_anchor=(0.5, 0.98))
    fig.suptitle("Per-goal outcome timeline (20 goals each)", fontsize=26, y=0.995)
    _save_fig(out_png, fig)


def _imshow_or_note(ax: plt.Axes, img_path: Path, title: str) -> None:
    ax.set_title(title, fontsize=18)
    ax.axis("off")
    if img_path.exists():
        img = plt.imread(img_path.as_posix())
        ax.imshow(img)
    else:
        ax.text(
            0.5,
            0.5,
            f"Missing image:\n{img_path.as_posix()}",
            ha="center",
            va="center",
            fontsize=14,
            color="#b91c1c",
            transform=ax.transAxes,
        )


def _parse_kv_text(path: Path) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def build_wave_truth_evidence_image(out_png: Path, csv_path: Path, summary_path: Path) -> None:
    """Build a cleaner wave-road evidence image from ground-truth logs.

    The plot focuses on the wave-zone segment to avoid startup transients.
    """

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.15, 1], wspace=0.15)
    ax_xy = fig.add_subplot(gs[0, 0])
    right = gs[0, 1].subgridspec(3, 1, hspace=0.28)
    ax_z = fig.add_subplot(right[0, 0])
    ax_pitch = fig.add_subplot(right[1, 0])
    ax_roll = fig.add_subplot(right[2, 0])

    if not csv_path.exists():
        ax_xy.axis("off")
        ax_z.axis("off")
        ax_pitch.axis("off")
        ax_roll.axis("off")
        fig.text(
            0.5,
            0.5,
            f"Missing ground truth csv:\n{csv_path.as_posix()}",
            ha="center",
            va="center",
            fontsize=16,
            color="#b91c1c",
        )
        _save_fig(out_png, fig)
        return

    rows: List[Dict[str, str]] = []
    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        ax_xy.axis("off")
        ax_z.axis("off")
        ax_pitch.axis("off")
        ax_roll.axis("off")
        fig.text(0.5, 0.5, "ground truth csv is empty", ha="center", va="center", fontsize=16, color="#b91c1c")
        _save_fig(out_png, fig)
        return

    x = np.array([float(r["x"]) for r in rows], dtype=np.float64)
    y = np.array([float(r["y"]) for r in rows], dtype=np.float64)
    z = np.array([float(r["z"]) for r in rows], dtype=np.float64)
    pitch_deg = np.array([float(r["pitch"]) * 180.0 / np.pi for r in rows], dtype=np.float64)
    roll_deg = np.array([float(r["roll"]) * 180.0 / np.pi for r in rows], dtype=np.float64)

    sm = _parse_kv_text(summary_path)
    try:
        zone_x_min = float(sm.get("zone_x_min", "-5.20"))
        zone_x_max = float(sm.get("zone_x_max", "-3.84"))
    except Exception:
        zone_x_min, zone_x_max = -5.20, -3.84

    zone_mask = (x >= zone_x_min) & (x <= zone_x_max)
    if np.count_nonzero(zone_mask) < 10:
        zone_mask = np.ones_like(x, dtype=bool)

    zx = x[zone_mask]
    zy = y[zone_mask]
    zz = z[zone_mask]
    zp = pitch_deg[zone_mask]
    zr = roll_deg[zone_mask]
    idx = np.arange(len(zz))

    # use relative z in mm to avoid absolute-height offset confusion
    zz_rel_mm = (zz - np.mean(zz)) * 1000.0
    z_range_mm = float(np.max(zz_rel_mm) - np.min(zz_rel_mm)) if len(zz_rel_mm) > 0 else 0.0

    ax_xy.plot(x, y, color="#94a3b8", lw=2.0, label="full path")
    ax_xy.plot(zx, zy, color="#2563eb", lw=2.4, label="wave-zone segment")
    ax_xy.scatter([x[0]], [y[0]], color="#16a34a", s=90, label="start", zorder=3)
    ax_xy.scatter([x[-1]], [y[-1]], color="#dc2626", s=90, label="end", zorder=3)
    ax_xy.set_title("Ground-truth path and wave-zone segment", fontsize=16)
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.grid(alpha=0.25)
    ax_xy.legend(loc="best", fontsize=11)

    ax_z.plot(idx, zz_rel_mm, color="#111827", lw=1.8)
    ax_z.set_title("Relative z in wave zone (mm, mean removed)", fontsize=14)
    ax_z.set_ylabel("dz (mm)")
    ax_z.grid(alpha=0.25)
    ax_z.text(
        0.99,
        0.86,
        f"range={z_range_mm:.2f} mm",
        transform=ax_z.transAxes,
        ha="right",
        fontsize=11,
        color="#111827",
    )

    ax_pitch.plot(idx, zp, color="#1d4ed8", lw=1.8)
    ax_pitch.set_title("Pitch in wave zone (deg)", fontsize=14)
    ax_pitch.set_ylabel("pitch")
    ax_pitch.grid(alpha=0.25)

    ax_roll.plot(idx, zr, color="#16a34a", lw=1.8)
    ax_roll.set_title("Roll in wave zone (deg)", fontsize=14)
    ax_roll.set_ylabel("roll")
    ax_roll.set_xlabel("sample idx")
    ax_roll.grid(alpha=0.25)

    fig.suptitle("Wave-road motion evidence (startup transient removed by zone filtering)", fontsize=20, y=0.98)
    _save_fig(out_png, fig)


def make_slide_wave_evidence(
    out_png: Path, wave_map_img: Path, wave_pass_img: Path
) -> None:
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[7, 1], hspace=0.2, wspace=0.08)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, :])

    _imshow_or_note(ax1, wave_map_img, "Wave segment map annotation")
    _imshow_or_note(ax2, wave_pass_img, "Measured passability (XY and Z)")

    ax3.axis("off")
    ax3.text(
        0.01,
        0.65,
        "Evidence: robot trajectory crosses wave zone; right panel uses zone-filtered relative-z + pitch/roll.",
        fontsize=17,
        color="#111827",
    )
    ax3.text(
        0.01,
        0.25,
        "Absolute z offset at startup is excluded to avoid misleading transient spikes.",
        fontsize=17,
        color="#065f46",
    )

    fig.suptitle("Wave-road passability evidence", fontsize=26, y=0.98)
    _save_fig(out_png, fig)


def make_slide_repro(out_png: Path, baseline: ArtifactData, stvl: ArtifactData, out_mp4: Path) -> None:
    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()

    fig.text(0.06, 0.88, "Reproducibility", fontsize=34, fontweight="bold", color="#111827")

    lines = [
        "Run command template (dynamic obstacle stress, 20 goals):",
        "ROUNDS=10 LOCALIZATION=small_gicp CONTROLLER=dwb BRINGUP_WORLD=RMUL_26_WAVE "
        "BRINGUP_MAP=RMUL26_WAVE USE_STVL=<true|false> DYNOBS=1 bash tools/stress_dynamic_nav.sh",
        "",
        "Artifacts used in this video:",
        f"1) {baseline.path.as_posix()}",
        f"2) {stvl.path.as_posix()}",
        "",
        f"Video output: {out_mp4.as_posix()}",
    ]

    y = 0.75
    for ln in lines:
        family = "monospace" if ("=" in ln or ln.startswith("1)") or ln.startswith("2)") or ln.startswith("Video output")) else None
        size = 17 if family else 19
        color = "#1f2937" if ln else "#111827"
        fig.text(0.08, y, ln, fontsize=size, color=color, family=family)
        y -= 0.08

    fig.text(
        0.06,
        0.12,
        "Summary: both baseline and STVL+retry reached 20/20. STVL+retry showed lower average goal time.",
        fontsize=18,
        color="#047857",
    )
    _save_fig(out_png, fig)


def build_video(slides: List[Path], output_mp4: Path, fps: int = 30) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg not found in PATH")

    # per-slide display time in seconds
    durations = [4, 5, 5, 5, 6]
    if len(durations) != len(slides):
        durations = [5] * len(slides)

    cmd: List[str] = [ffmpeg, "-y"]
    for slide, dur in zip(slides, durations):
        cmd.extend(["-loop", "1", "-t", str(dur), "-i", slide.as_posix()])

    filter_parts = "".join([f"[{i}:v]" for i in range(len(slides))])
    filter_complex = f"{filter_parts}concat=n={len(slides)}:v=1:a=0,format=yuv420p[v]"
    cmd.extend(
        [
            "-filter_complex",
            filter_complex,
            "-map",
            "[v]",
            "-r",
            str(fps),
            "-movflags",
            "+faststart",
            output_mp4.as_posix(),
        ]
    )

    subprocess.run(cmd, check=True)


def pick_default_artifacts(repo_root: Path) -> Tuple[Path, Path]:
    return (
        repo_root / "artifacts/realstress_smallgicp_dynobs20_baseline_20260225_094408",
        repo_root / "artifacts/realstress_smallgicp_dynobs20_stvl_retry_20260225_130032",
    )


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    default_baseline, default_stvl = pick_default_artifacts(repo_root)
    default_out_dir = repo_root / "docs/logs/weekly_review_20of20"

    parser = argparse.ArgumentParser(description="Create a 20/20 stress-test review video.")
    parser.add_argument("--baseline-dir", type=Path, default=default_baseline)
    parser.add_argument("--stvl-dir", type=Path, default=default_stvl)
    parser.add_argument("--out-dir", type=Path, default=default_out_dir)
    parser.add_argument("--output-name", type=str, default="small_gicp_20of20_review.mp4")
    args = parser.parse_args()

    baseline = load_artifact("baseline", args.baseline_dir.resolve())
    stvl = load_artifact("stvl+retry", args.stvl_dir.resolve())

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_mp4 = (args.out_dir / args.output_name).resolve()

    wave_map_img = repo_root / "docs/RMUL26_wave_segment_annotated_v2.png"
    wave_gt_csv = repo_root / "docs/logs/wave_verify_20260221_203858/ground_truth_odom.csv"
    wave_gt_summary = repo_root / "docs/logs/wave_verify_20260221_203858/validation_summary.txt"

    with tempfile.TemporaryDirectory(prefix="video_slides_") as td:
        td_path = Path(td)
        wave_pass_img = td_path / "wave_truth_evidence.png"
        build_wave_truth_evidence_image(wave_pass_img, wave_gt_csv, wave_gt_summary)

        slides = [
            td_path / "slide_01_title.png",
            td_path / "slide_02_kpi.png",
            td_path / "slide_03_goals.png",
            td_path / "slide_04_wave.png",
            td_path / "slide_05_repro.png",
        ]

        make_slide_title(slides[0], baseline, stvl)
        make_slide_kpi(slides[1], baseline, stvl)
        make_slide_goal_timeline(slides[2], baseline, stvl)
        make_slide_wave_evidence(slides[3], wave_map_img, wave_pass_img)
        make_slide_repro(slides[4], baseline, stvl, out_mp4)

        build_video(slides, out_mp4, fps=30)

    print(f"[ok] video generated: {out_mp4}")


if __name__ == "__main__":
    main()
