#!/usr/bin/env python3
"""Analyze localization drift against /ground_truth/odom from rosbag2 sqlite.

Primary source is /amcl_pose. If /amcl_pose is sparse (common QoS mismatch in bagging),
the script falls back to /tf composition:
  map->odom + odom->base_link( or odom->base_link_fake ) -> map->base
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


@dataclass
class WaveBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    top_mean: float

    def contains(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return (x >= self.x_min) & (x <= self.x_max) & (y >= self.y_min) & (y <= self.y_max)


@dataclass
class Pose2D:
    t: float
    x: float
    y: float
    yaw: float


def normalize_angle_rad(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def compose_pose2d(a: Pose2D, b: Pose2D) -> Pose2D:
    # T_world_c = T_world_a o T_a_c
    ca = math.cos(a.yaw)
    sa = math.sin(a.yaw)
    x = a.x + ca * b.x - sa * b.y
    y = a.y + sa * b.x + ca * b.y
    yaw = math.atan2(math.sin(a.yaw + b.yaw), math.cos(a.yaw + b.yaw))
    t = max(a.t, b.t)
    return Pose2D(t=t, x=x, y=y, yaw=yaw)


def detect_settle_time_sec(t: np.ndarray, z: np.ndarray) -> float:
    if len(t) < 20:
        return float(t[0]) if len(t) else 0.0
    dt = np.diff(t)
    dz = np.diff(z)
    v = np.divide(dz, dt, out=np.zeros_like(dz), where=dt > 1e-6)
    win = 16
    if len(v) < win:
        return float(t[0])
    rms = np.array([np.sqrt(np.mean(v[i : i + win] ** 2)) for i in range(len(v) - win + 1)])
    idx = np.where(rms < 0.005)[0]
    return float(t[idx[0]]) if len(idx) else float(t[0])


def load_wave_bounds(world_path: Path) -> WaveBounds:
    text = world_path.read_text(encoding="utf-8", errors="ignore")
    pattern = re.compile(
        r"<collision name='c_(\d+)'>\s*<pose>"
        r"([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+[-0-9.eE+]+\s+[-0-9.eE+]+\s+[-0-9.eE+]+"
        r"</pose>.*?<size>([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)</size>",
        re.S,
    )
    rows = []
    for m in pattern.finditer(text):
        rows.append(
            (
                int(m.group(1)),
                float(m.group(2)),
                float(m.group(3)),
                float(m.group(4)),
                float(m.group(5)),
                float(m.group(6)),
                float(m.group(7)),
            )
        )
    if not rows:
        raise RuntimeError(f"no wave strip collisions found in world: {world_path}")
    arr = np.array(sorted(rows), dtype=float)
    xc = arr[:, 1]
    yc = arr[:, 2]
    zc = arr[:, 3]
    dx = arr[:, 4]
    wy = arr[:, 5]
    hz = arr[:, 6]
    return WaveBounds(
        x_min=float((xc - dx / 2.0).min()),
        x_max=float((xc + dx / 2.0).max()),
        y_min=float((yc - wy / 2.0).min()),
        y_max=float((yc + wy / 2.0).max()),
        top_mean=float(np.mean(zc + hz / 2.0)),
    )


def stats_line(prefix: str, values: np.ndarray) -> list[str]:
    if len(values) == 0:
        return [
            f"{prefix}_count=0",
            f"{prefix}_mean=nan",
            f"{prefix}_median=nan",
            f"{prefix}_p95=nan",
            f"{prefix}_max=nan",
            f"{prefix}_std=nan",
        ]
    return [
        f"{prefix}_count={len(values)}",
        f"{prefix}_mean={np.mean(values):.6f}",
        f"{prefix}_median={np.median(values):.6f}",
        f"{prefix}_p95={np.percentile(values, 95):.6f}",
        f"{prefix}_max={np.max(values):.6f}",
        f"{prefix}_std={np.std(values):.6f}",
    ]


def drift_slope_per_minute(t: np.ndarray, err: np.ndarray) -> float:
    if len(t) < 2:
        return float("nan")
    coeff = np.polyfit(t, err, 1)
    return float(coeff[0] * 60.0)


def build_tf_localization_pose(cur: sqlite3.Cursor, tf_topic_id: int) -> list[tuple[float, float, float, float]]:
    tf_type = get_message("tf2_msgs/msg/TFMessage")
    map_odom: list[Pose2D] = []
    odom_base: list[Pose2D] = []

    for ts_ns, data in cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tf_topic_id,)):
        msg = deserialize_message(data, tf_type)
        t_msg = ts_ns * 1e-9
        for tr in msg.transforms:
            parent = tr.header.frame_id
            child = tr.child_frame_id
            x = tr.transform.translation.x
            y = tr.transform.translation.y
            q = tr.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            if parent == "map" and child == "odom":
                map_odom.append(Pose2D(t=t_msg, x=x, y=y, yaw=yaw))
            elif parent == "odom" and (child == "base_link" or child == "base_link_fake"):
                odom_base.append(Pose2D(t=t_msg, x=x, y=y, yaw=yaw))

    if not map_odom or not odom_base:
        return []

    map_odom = sorted(map_odom, key=lambda p: p.t)
    odom_base = sorted(odom_base, key=lambda p: p.t)
    mo_t = np.array([p.t for p in map_odom], dtype=float)
    ob_t = np.array([p.t for p in odom_base], dtype=float)

    rows = []
    for ob in odom_base:
        idx = int(np.searchsorted(mo_t, ob.t, side="left"))
        if idx >= len(map_odom):
            idx = len(map_odom) - 1
        cand = [idx]
        if idx > 0:
            cand.append(idx - 1)
        best = min(cand, key=lambda i: abs(mo_t[i] - ob.t))
        mo = map_odom[best]
        # avoid composing stale map->odom with far-away timestamp
        if abs(mo.t - ob.t) > 0.40:
            continue
        mb = compose_pose2d(mo, ob)
        rows.append((mb.t, mb.x, mb.y, mb.yaw))

    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact", required=True, help="artifact dir containing rosbag_dataset")
    parser.add_argument("--world", default="", help="optional .world path for wave-zone subset stats")
    parser.add_argument(
        "--source",
        choices=["auto", "amcl", "tf"],
        default="auto",
        help="localization pose source: amcl topic or tf-composed map->base",
    )
    parser.add_argument(
        "--min-amcl-samples",
        type=int,
        default=20,
        help="minimum /amcl_pose samples to trust amcl source in auto mode",
    )
    parser.add_argument(
        "--min-amcl-rate-hz",
        type=float,
        default=3.0,
        help="minimum /amcl_pose rate to trust amcl source in auto mode",
    )
    args = parser.parse_args()

    artifact = Path(args.artifact)
    db = artifact / "rosbag_dataset" / "rosbag_dataset_0.db3"
    if not db.exists():
        raise FileNotFoundError(f"bag not found: {db}")

    out_dir = artifact / "analysis_drift"
    out_dir.mkdir(parents=True, exist_ok=True)

    wave_bounds: Optional[WaveBounds] = None
    if args.world:
        world = Path(args.world)
        if world.exists():
            wave_bounds = load_wave_bounds(world)
        else:
            raise FileNotFoundError(f"world not found: {world}")

    conn = sqlite3.connect(str(db))
    cur = conn.cursor()
    cur.execute("select id, name, type from topics")
    topics = {name: (tid, typ) for tid, name, typ in cur.fetchall()}

    if "/ground_truth/odom" not in topics:
        raise RuntimeError("missing /ground_truth/odom in bag")

    odom_id, odom_type = topics["/ground_truth/odom"]
    Odom = get_message(odom_type)
    odom_rows = []
    for ts_ns, data in cur.execute(
        "select timestamp, data from messages where topic_id=? order by timestamp", (odom_id,)
    ):
        msg = deserialize_message(data, Odom)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        odom_rows.append((ts_ns * 1e-9, p.x, p.y, p.z, yaw))
    if not odom_rows:
        raise RuntimeError("insufficient /ground_truth/odom data")

    amcl_rows = []
    if "/amcl_pose" in topics:
        amcl_id, amcl_type = topics["/amcl_pose"]
        Amcl = get_message(amcl_type)
        for ts_ns, data in cur.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp", (amcl_id,)
        ):
            msg = deserialize_message(data, Amcl)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            amcl_rows.append((ts_ns * 1e-9, p.x, p.y, yaw))

    tf_rows = []
    if "/tf" in topics:
        tf_id, _ = topics["/tf"]
        tf_rows = build_tf_localization_pose(cur, tf_id)

    conn.close()

    amcl_rate_hz = 0.0
    if len(amcl_rows) >= 2:
        amcl_dt = amcl_rows[-1][0] - amcl_rows[0][0]
        if amcl_dt > 1e-6:
            amcl_rate_hz = (len(amcl_rows) - 1) / amcl_dt

    source = args.source
    if source == "auto":
        if len(amcl_rows) >= args.min_amcl_samples and amcl_rate_hz >= args.min_amcl_rate_hz:
            source = "amcl"
        elif len(tf_rows) >= 20:
            source = "tf"
        elif len(amcl_rows) > 0:
            source = "amcl"
        else:
            raise RuntimeError("no usable localization source: amcl and tf both insufficient")
    elif source == "amcl" and len(amcl_rows) == 0:
        raise RuntimeError("source=amcl requested but /amcl_pose missing")
    elif source == "tf" and len(tf_rows) == 0:
        raise RuntimeError("source=tf requested but /tf map->odom/odom->base chain unavailable")

    loc_rows = amcl_rows if source == "amcl" else tf_rows

    odom = np.array(odom_rows, dtype=float)
    loc = np.array(loc_rows, dtype=float)
    t0 = odom[0, 0]
    odom[:, 0] -= t0
    loc[:, 0] -= t0

    loc_t = loc[:, 0]
    idx = np.searchsorted(loc_t, odom[:, 0], side="left")
    idx = np.clip(idx, 0, len(loc) - 1)
    loc_sync = loc[idx]

    dx = loc_sync[:, 1] - odom[:, 1]
    dy = loc_sync[:, 2] - odom[:, 2]
    dist = np.sqrt(dx * dx + dy * dy)
    yaw_err = np.degrees(normalize_angle_rad(loc_sync[:, 3] - odom[:, 4]))
    dt_sync = np.abs(loc_sync[:, 0] - odom[:, 0])

    settle_t = detect_settle_time_sec(odom[:, 0], odom[:, 3])
    post_settle = odom[:, 0] >= settle_t
    wave_mask = wave_bounds.contains(odom[:, 1], odom[:, 2]) if wave_bounds else np.zeros(len(odom), dtype=bool)

    csv_path = out_dir / "loc_vs_ground_truth.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_s",
                "gt_x_m",
                "gt_y_m",
                "gt_z_m",
                "loc_x_m",
                "loc_y_m",
                "dx_m",
                "dy_m",
                "xy_err_m",
                "yaw_err_deg",
                "loc_sync_dt_s",
                "inside_wave",
                "post_settle",
            ]
        )
        for i in range(len(odom)):
            writer.writerow(
                [
                    odom[i, 0],
                    odom[i, 1],
                    odom[i, 2],
                    odom[i, 3],
                    loc_sync[i, 1],
                    loc_sync[i, 2],
                    dx[i],
                    dy[i],
                    dist[i],
                    yaw_err[i],
                    dt_sync[i],
                    int(wave_mask[i]),
                    int(post_settle[i]),
                ]
            )

    summary = []
    summary.append(f"artifact={artifact}")
    summary.append(f"bag={db}")
    summary.append(f"pose_source={source}")
    summary.append(f"odom_samples={len(odom)}")
    summary.append(f"loc_samples={len(loc)}")
    summary.append(f"amcl_samples_raw={len(amcl_rows)}")
    summary.append(f"amcl_rate_hz_raw={amcl_rate_hz:.6f}")
    summary.append(f"tf_pose_samples_raw={len(tf_rows)}")
    summary.append(f"sync_max_dt_s={np.max(dt_sync):.6f}")
    summary.append(f"sync_p95_dt_s={np.percentile(dt_sync,95):.6f}")
    summary.append(f"settle_time_s={settle_t:.6f}")
    summary.append(f"final_xy_err_m={dist[-1]:.6f}")
    summary.append(f"final_yaw_err_deg={yaw_err[-1]:.6f}")
    summary.append(f"xy_err_slope_m_per_min={drift_slope_per_minute(odom[:, 0], dist):.6f}")
    summary.append(f"yaw_err_slope_deg_per_min={drift_slope_per_minute(odom[:, 0], yaw_err):.6f}")
    summary.extend(stats_line("xy_err_all_m", dist))
    summary.extend(stats_line("yaw_err_all_deg", np.abs(yaw_err)))
    summary.extend(stats_line("xy_err_post_settle_m", dist[post_settle]))
    summary.extend(stats_line("yaw_err_post_settle_deg", np.abs(yaw_err[post_settle])))

    if wave_bounds:
        summary.append(f"wave_x_min={wave_bounds.x_min:.6f}")
        summary.append(f"wave_x_max={wave_bounds.x_max:.6f}")
        summary.append(f"wave_y_min={wave_bounds.y_min:.6f}")
        summary.append(f"wave_y_max={wave_bounds.y_max:.6f}")
        summary.append(f"wave_top_mean={wave_bounds.top_mean:.6f}")
        summary.extend(stats_line("xy_err_wave_m", dist[wave_mask]))
        summary.extend(stats_line("yaw_err_wave_deg", np.abs(yaw_err[wave_mask])))
        summary.extend(stats_line("xy_err_wave_post_settle_m", dist[wave_mask & post_settle]))
        summary.extend(stats_line("yaw_err_wave_post_settle_deg", np.abs(yaw_err[wave_mask & post_settle])))

    (out_dir / "drift_summary.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")

    fig, ax = plt.subplots(1, 2, figsize=(12, 5), dpi=150)
    loc_label = "amcl" if source == "amcl" else "tf(map->base)"
    ax[0].plot(odom[:, 1], odom[:, 2], color="#111827", lw=1.6, label="ground_truth")
    ax[0].plot(loc_sync[:, 1], loc_sync[:, 2], color="#2563eb", lw=1.2, alpha=0.85, label=f"{loc_label} (synced)")
    if wave_bounds:
        rx = [
            wave_bounds.x_min,
            wave_bounds.x_max,
            wave_bounds.x_max,
            wave_bounds.x_min,
            wave_bounds.x_min,
        ]
        ry = [
            wave_bounds.y_min,
            wave_bounds.y_min,
            wave_bounds.y_max,
            wave_bounds.y_max,
            wave_bounds.y_min,
        ]
        ax[0].plot(rx, ry, "r--", lw=1.2, label="wave zone")
    ax[0].scatter([odom[0, 1]], [odom[0, 2]], color="green", s=42, label="start")
    ax[0].scatter([odom[-1, 1]], [odom[-1, 2]], color="red", s=42, label="end")
    ax[0].set_title("Trajectory: localization vs ground truth")
    ax[0].set_xlabel("x (m)")
    ax[0].set_ylabel("y (m)")
    ax[0].grid(alpha=0.25)
    ax[0].legend(fontsize=8)

    ax[1].plot(odom[:, 0], dist, color="#1d4ed8", lw=1.3, label="xy error (m)")
    ax[1].plot(odom[:, 0], np.abs(yaw_err) / 45.0, color="#059669", lw=1.1, label="|yaw err| / 45")
    ax[1].axvline(settle_t, color="gray", ls="--", lw=1.0, label="settle")
    if wave_bounds and np.any(wave_mask):
        t0w = odom[np.where(wave_mask)[0][0], 0]
        t1w = odom[np.where(wave_mask)[0][-1], 0]
        ax[1].axvspan(t0w, t1w, color="red", alpha=0.08, label="wave crossing")
    ax[1].set_title(f"Localization error timeline ({source})")
    ax[1].set_xlabel("t (s)")
    ax[1].grid(alpha=0.25)
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / "localization_drift_plot.png", bbox_inches="tight")
    plt.close(fig)

    print(f"[ok] {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
