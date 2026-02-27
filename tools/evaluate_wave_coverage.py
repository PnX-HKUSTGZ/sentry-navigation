#!/usr/bin/env python3
"""Quantify how much of a wave strip was truly traversed from trajectory CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List, Optional, Tuple


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--csv", required=True, help="Path to loc_vs_ground_truth.csv")
    p.add_argument("--x-col", default="gt_x_m", help="Column name for x in meters")
    p.add_argument("--y-col", default="gt_y_m", help="Column name for y in meters")
    p.add_argument("--t-col", default="t_s", help="Optional timestamp column")
    p.add_argument("--wave-x-min", type=float, default=-5.44)
    p.add_argument("--wave-x-max", type=float, default=-3.52)
    p.add_argument("--wave-y-min", type=float, default=1.85)
    p.add_argument("--wave-y-max", type=float, default=2.55)
    p.add_argument("--wavelength", type=float, default=0.24, help="Wave length in meters")
    return p.parse_args()


def read_rows(path: Path) -> List[dict]:
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def maybe_float(row: dict, key: str) -> Optional[float]:
    val = row.get(key)
    if val is None or val == "":
        return None
    try:
        return float(val)
    except ValueError:
        return None


def main() -> int:
    args = parse_args()
    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    rows = read_rows(csv_path)
    if not rows:
        print("status=EMPTY")
        return 1

    pts: List[Tuple[float, float, Optional[float]]] = []
    for r in rows:
        x = maybe_float(r, args.x_col)
        y = maybe_float(r, args.y_col)
        t = maybe_float(r, args.t_col)
        if x is None or y is None:
            continue
        pts.append((x, y, t))

    if not pts:
        print("status=NO_VALID_POINTS")
        return 1

    wx0, wx1 = args.wave_x_min, args.wave_x_max
    wy0, wy1 = args.wave_y_min, args.wave_y_max
    wspan = wx1 - wx0
    if wspan <= 0:
        raise ValueError("wave_x_max must be greater than wave_x_min")
    if args.wavelength <= 0:
        raise ValueError("wavelength must be positive")

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    inside_idx = [
        i
        for i, (x, y, _) in enumerate(pts)
        if wx0 <= x <= wx1 and wy0 <= y <= wy1
    ]

    print(f"csv={csv_path}")
    print(f"overall_x_min={min(xs):.6f}")
    print(f"overall_x_max={max(xs):.6f}")
    print(f"overall_y_min={min(ys):.6f}")
    print(f"overall_y_max={max(ys):.6f}")
    print(f"wave_x_min={wx0:.6f}")
    print(f"wave_x_max={wx1:.6f}")
    print(f"wave_y_min={wy0:.6f}")
    print(f"wave_y_max={wy1:.6f}")
    print(f"wave_length_m={wspan:.6f}")
    print(f"wavelength_m={args.wavelength:.6f}")
    print(f"inside_count={len(inside_idx)}")

    if not inside_idx:
        print("status=NO_WAVE_CROSSING")
        return 0

    inside_x = [xs[i] for i in inside_idx]
    inside_y = [ys[i] for i in inside_idx]
    xmin, xmax = min(inside_x), max(inside_x)
    xspan = xmax - xmin
    cov = xspan / wspan
    est_waves = xspan / args.wavelength

    print(f"inside_x_min={xmin:.6f}")
    print(f"inside_x_max={xmax:.6f}")
    print(f"inside_y_min={min(inside_y):.6f}")
    print(f"inside_y_max={max(inside_y):.6f}")
    print(f"inside_x_span_m={xspan:.6f}")
    print(f"coverage_ratio={cov:.6f}")
    print(f"estimated_waves_crossed={est_waves:.3f}")

    t0 = pts[inside_idx[0]][2]
    t1 = pts[inside_idx[-1]][2]
    if t0 is not None and t1 is not None:
        print(f"inside_time_start_s={t0:.6f}")
        print(f"inside_time_end_s={t1:.6f}")
        print(f"inside_time_span_s={t1 - t0:.6f}")

    print("status=OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
