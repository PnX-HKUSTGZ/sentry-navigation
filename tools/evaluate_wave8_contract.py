#!/usr/bin/env python3
"""Evaluate wave-road trajectory contract from localization-vs-ground-truth CSV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--csv", required=True, help="Path to analysis_drift/loc_vs_ground_truth.csv")
    p.add_argument(
        "--summary",
        default="",
        help="Optional stress summary.txt path (for action success gate)",
    )
    p.add_argument("--x-col", default="gt_x_m", help="Ground-truth x column")
    p.add_argument("--y-col", default="gt_y_m", help="Ground-truth y column")
    p.add_argument("--loc-x-col", default="loc_x_m", help="Localization x column")
    p.add_argument("--loc-y-col", default="loc_y_m", help="Localization y column")
    p.add_argument("--t-col", default="t_s", help="Time column in seconds")
    p.add_argument("--wave-x-min", type=float, default=-1.40)
    p.add_argument("--wave-x-max", type=float, default=0.52)
    p.add_argument("--wave-y-min", type=float, default=2.20)
    p.add_argument("--wave-y-max", type=float, default=3.20)
    p.add_argument("--wavelength", type=float, default=0.24)
    p.add_argument(
        "--fit-window-sec",
        type=float,
        default=2.0,
        help="Use early samples in [0, fit_window_sec] to fit rigid alignment",
    )
    p.add_argument("--min-waves", type=float, default=7.5)
    p.add_argument("--min-inside-time", type=float, default=2.0)
    p.add_argument("--max-wave-aligned-p95", type=float, default=0.35)
    p.add_argument("--max-wave-aligned-max", type=float, default=0.55)
    p.add_argument(
        "--require-action-success",
        type=int,
        choices=[0, 1],
        default=1,
        help="If 1, require summary.txt succeeded>=1",
    )
    return p.parse_args()


def parse_float(row: Dict[str, str], key: str) -> Optional[float]:
    raw = row.get(key, "")
    if raw == "":
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def read_csv_points(
    csv_path: Path,
    x_col: str,
    y_col: str,
    loc_x_col: str,
    loc_y_col: str,
    t_col: str,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    gt_pts: List[Tuple[float, float]] = []
    loc_pts: List[Tuple[float, float]] = []
    ts: List[float] = []

    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            gt_x = parse_float(row, x_col)
            gt_y = parse_float(row, y_col)
            loc_x = parse_float(row, loc_x_col)
            loc_y = parse_float(row, loc_y_col)
            t = parse_float(row, t_col)
            if (
                gt_x is None
                or gt_y is None
                or loc_x is None
                or loc_y is None
                or t is None
            ):
                continue
            gt_pts.append((gt_x, gt_y))
            loc_pts.append((loc_x, loc_y))
            ts.append(t)

    if not gt_pts:
        raise RuntimeError("no valid samples parsed from csv")

    return (
        np.asarray(gt_pts, dtype=float),
        np.asarray(loc_pts, dtype=float),
        np.asarray(ts, dtype=float),
    )


def rigid_align_2d(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Solve dst ~= R*src + t via SVD."""
    if src.shape[0] < 2 or dst.shape[0] < 2:
        return np.eye(2), np.zeros(2)
    src_centroid = np.mean(src, axis=0)
    dst_centroid = np.mean(dst, axis=0)
    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid
    h = src_centered.T @ dst_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = dst_centroid - r @ src_centroid
    return r, t


def safe_stat(arr: np.ndarray, fn) -> float:
    if arr.size == 0:
        return float("nan")
    return float(fn(arr))


def parse_summary(path: Path) -> Dict[str, float]:
    result: Dict[str, float] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        try:
            result[key] = float(value)
        except ValueError:
            continue
    return result


def print_metric(key: str, value) -> None:
    if isinstance(value, float):
        if math.isnan(value):
            print(f"{key}=nan")
        else:
            print(f"{key}={value:.6f}")
        return
    print(f"{key}={value}")


def main() -> int:
    args = parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    gt, loc, ts = read_csv_points(
        csv_path,
        args.x_col,
        args.y_col,
        args.loc_x_col,
        args.loc_y_col,
        args.t_col,
    )
    t0 = float(ts[0])
    ts_rel = ts - t0

    if args.wave_x_max <= args.wave_x_min:
        raise ValueError("wave-x-max must be greater than wave-x-min")
    if args.wave_y_max <= args.wave_y_min:
        raise ValueError("wave-y-max must be greater than wave-y-min")
    if args.wavelength <= 0.0:
        raise ValueError("wavelength must be > 0")

    wave_inside = (
        (gt[:, 0] >= args.wave_x_min)
        & (gt[:, 0] <= args.wave_x_max)
        & (gt[:, 1] >= args.wave_y_min)
        & (gt[:, 1] <= args.wave_y_max)
    )
    inside_idx = np.where(wave_inside)[0]

    wave_length = args.wave_x_max - args.wave_x_min
    if inside_idx.size > 0:
        inside_x = gt[inside_idx, 0]
        inside_t = ts_rel[inside_idx]
        inside_x_min = float(np.min(inside_x))
        inside_x_max = float(np.max(inside_x))
        inside_x_span = inside_x_max - inside_x_min
        coverage_ratio = inside_x_span / wave_length
        waves_crossed = inside_x_span / args.wavelength
        inside_time_span = float(np.max(inside_t) - np.min(inside_t))
    else:
        inside_x_min = float("nan")
        inside_x_max = float("nan")
        inside_x_span = 0.0
        coverage_ratio = 0.0
        waves_crossed = 0.0
        inside_time_span = 0.0

    fit_mask = ts_rel <= args.fit_window_sec
    if int(np.count_nonzero(fit_mask)) < 4:
        fit_mask = np.arange(len(ts_rel)) < min(20, len(ts_rel))
    r, trans = rigid_align_2d(loc[fit_mask], gt[fit_mask])
    loc_aligned = (r @ loc.T).T + trans
    aligned_err = np.linalg.norm(loc_aligned - gt, axis=1)
    aligned_err_wave = aligned_err[wave_inside]

    slope_m_per_min = float("nan")
    if aligned_err.size >= 2:
        coef = np.polyfit(ts_rel, aligned_err, 1)
        slope_m_per_min = float(coef[0] * 60.0)

    summary_vals = parse_summary(Path(args.summary)) if args.summary else {}
    succeeded = int(summary_vals.get("succeeded", -1))
    total_goals = int(summary_vals.get("total_goals", -1))

    coverage_pass = waves_crossed >= args.min_waves and inside_time_span >= args.min_inside_time
    drift_wave_p95 = safe_stat(aligned_err_wave, lambda a: np.percentile(a, 95))
    drift_wave_max = safe_stat(aligned_err_wave, np.max)
    drift_pass = (
        aligned_err_wave.size > 0
        and drift_wave_p95 <= args.max_wave_aligned_p95
        and drift_wave_max <= args.max_wave_aligned_max
    )
    nav_pass = True
    if args.require_action_success == 1:
        nav_pass = succeeded >= 1

    contract_pass = coverage_pass and drift_pass and nav_pass

    print_metric("csv", csv_path)
    print_metric("samples_total", int(gt.shape[0]))
    print_metric("wave_inside_count", int(inside_idx.size))
    print_metric("wave_x_min", args.wave_x_min)
    print_metric("wave_x_max", args.wave_x_max)
    print_metric("wave_y_min", args.wave_y_min)
    print_metric("wave_y_max", args.wave_y_max)
    print_metric("wave_length_m", wave_length)
    print_metric("inside_x_min", inside_x_min)
    print_metric("inside_x_max", inside_x_max)
    print_metric("inside_x_span_m", inside_x_span)
    print_metric("inside_time_span_s", inside_time_span)
    print_metric("coverage_ratio", coverage_ratio)
    print_metric("estimated_waves_crossed", waves_crossed)
    print_metric("fit_window_sec", args.fit_window_sec)
    print_metric("aligned_err_all_mean_m", safe_stat(aligned_err, np.mean))
    print_metric("aligned_err_all_p95_m", safe_stat(aligned_err, lambda a: np.percentile(a, 95)))
    print_metric("aligned_err_all_max_m", safe_stat(aligned_err, np.max))
    print_metric("aligned_err_wave_mean_m", safe_stat(aligned_err_wave, np.mean))
    print_metric("aligned_err_wave_p95_m", drift_wave_p95)
    print_metric("aligned_err_wave_max_m", drift_wave_max)
    print_metric("aligned_err_slope_m_per_min", slope_m_per_min)
    print_metric("summary_total_goals", total_goals)
    print_metric("summary_succeeded", succeeded)
    print_metric("gate_min_waves", args.min_waves)
    print_metric("gate_min_inside_time_s", args.min_inside_time)
    print_metric("gate_max_wave_aligned_p95_m", args.max_wave_aligned_p95)
    print_metric("gate_max_wave_aligned_max_m", args.max_wave_aligned_max)
    print_metric("gate_require_action_success", args.require_action_success)
    print_metric("coverage_pass", int(coverage_pass))
    print_metric("drift_pass", int(drift_pass))
    print_metric("nav_pass", int(nav_pass))
    print_metric("contract_pass", int(contract_pass))

    return 0 if contract_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
