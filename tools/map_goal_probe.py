#!/usr/bin/env python3
"""
Check whether a map-frame goal is free/occupied/unknown on a Nav2 map yaml+pgm.

Output is line-based key=value pairs for easy shell parsing.
"""

from __future__ import annotations

import argparse
import ast
import math
import pathlib
import sys
from typing import Dict, List, Optional, Tuple


def parse_map_yaml(path: pathlib.Path) -> Dict[str, object]:
    cfg: Dict[str, object] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if key == "image":
            cfg["image"] = value.strip("'\"")
        elif key == "resolution":
            cfg["resolution"] = float(value)
        elif key == "origin":
            cfg["origin"] = ast.literal_eval(value)
        elif key == "negate":
            cfg["negate"] = int(float(value))
        elif key == "occupied_thresh":
            cfg["occupied_thresh"] = float(value)
        elif key == "free_thresh":
            cfg["free_thresh"] = float(value)
        elif key == "mode":
            cfg["mode"] = value.strip("'\"")

    required = {"image", "resolution", "origin", "occupied_thresh", "free_thresh", "negate"}
    missing = required - set(cfg.keys())
    if missing:
        raise ValueError(f"map yaml missing fields: {sorted(missing)}")
    return cfg


def _read_token(buf: bytes, idx: int) -> Tuple[bytes, int]:
    n = len(buf)
    while idx < n and chr(buf[idx]).isspace():
        idx += 1
    while idx < n and buf[idx] == ord("#"):
        while idx < n and buf[idx] not in (ord("\n"), ord("\r")):
            idx += 1
        while idx < n and chr(buf[idx]).isspace():
            idx += 1
    start = idx
    while idx < n and not chr(buf[idx]).isspace():
        idx += 1
    return buf[start:idx], idx


def read_pgm(path: pathlib.Path) -> Tuple[int, int, int, List[int]]:
    data = path.read_bytes()
    tok, i = _read_token(data, 0)
    magic = tok.decode("ascii")
    if magic not in {"P2", "P5"}:
        raise ValueError(f"unsupported pgm magic: {magic}")
    tok, i = _read_token(data, i)
    width = int(tok.decode("ascii"))
    tok, i = _read_token(data, i)
    height = int(tok.decode("ascii"))
    tok, i = _read_token(data, i)
    maxval = int(tok.decode("ascii"))
    if maxval <= 0 or maxval > 255:
        raise ValueError(f"unsupported maxval: {maxval}")

    while i < len(data) and chr(data[i]).isspace():
        i += 1

    if magic == "P5":
        count = width * height
        pixels = list(data[i : i + count])
        if len(pixels) != count:
            raise ValueError("truncated pgm binary data")
        return width, height, maxval, pixels

    payload = data[i:].decode("ascii", errors="ignore")
    vals: List[int] = []
    for raw in payload.split():
        if raw.startswith("#"):
            continue
        vals.append(int(raw))
    if len(vals) < width * height:
        raise ValueError("truncated pgm ascii data")
    return width, height, maxval, vals[: width * height]


def occ_prob(px: int, negate: int, maxval: int) -> float:
    if negate:
        return float(px) / float(maxval)
    return float(maxval - px) / float(maxval)


def world_to_cell(x: float, y: float, origin: List[float], resolution: float, height: int) -> Tuple[int, int]:
    mx = int((x - origin[0]) / resolution)
    my = int((y - origin[1]) / resolution)
    py = height - 1 - my
    return mx, py


def cell_to_world(mx: int, py: int, origin: List[float], resolution: float, height: int) -> Tuple[float, float]:
    my = height - 1 - py
    x = origin[0] + (mx + 0.5) * resolution
    y = origin[1] + (my + 0.5) * resolution
    return x, y


def classify(
    x: float,
    y: float,
    cfg: Dict[str, object],
    width: int,
    height: int,
    maxval: int,
    pixels: List[int],
) -> Dict[str, object]:
    mx, py = world_to_cell(x, y, cfg["origin"], float(cfg["resolution"]), height)  # type: ignore[arg-type]
    out: Dict[str, object] = {"mx": mx, "py": py}
    if mx < 0 or mx >= width or py < 0 or py >= height:
        out["status"] = "OUT"
        return out
    idx = py * width + mx
    px = pixels[idx]
    p = occ_prob(px, int(cfg["negate"]), maxval)
    occ_th = float(cfg["occupied_thresh"])
    free_th = float(cfg["free_thresh"])
    if p >= occ_th:
        status = "OCC"
    elif p <= free_th:
        status = "FREE"
    else:
        status = "UNK"
    out.update({"status": status, "pixel": px, "occ_prob": p})
    return out


def nearest_free(
    x: float,
    y: float,
    cfg: Dict[str, object],
    width: int,
    height: int,
    maxval: int,
    pixels: List[int],
    max_radius_m: float,
) -> Optional[Tuple[float, float, float]]:
    res = float(cfg["resolution"])
    origin = cfg["origin"]  # type: ignore[assignment]
    free_th = float(cfg["free_thresh"])
    mx0, py0 = world_to_cell(x, y, origin, res, height)
    max_cells = int(max_radius_m / res)
    best: Optional[Tuple[float, float, float]] = None

    def try_cell(mx: int, py: int) -> None:
        nonlocal best
        if mx < 0 or mx >= width or py < 0 or py >= height:
            return
        idx = py * width + mx
        p = occ_prob(pixels[idx], int(cfg["negate"]), maxval)
        if p > free_th:
            return
        wx, wy = cell_to_world(mx, py, origin, res, height)
        d = math.hypot(wx - x, wy - y)
        if best is None or d < best[0]:
            best = (d, wx, wy)

    for c in range(max_cells + 1):
        if c == 0:
            try_cell(mx0, py0)
        else:
            for dx in range(-c, c + 1):
                try_cell(mx0 + dx, py0 - c)
                try_cell(mx0 + dx, py0 + c)
            for dy in range(-c + 1, c):
                try_cell(mx0 - c, py0 + dy)
                try_cell(mx0 + c, py0 + dy)
        if best is not None:
            return best
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-yaml", required=True)
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--max-nearest-radius", type=float, default=1.5)
    args = ap.parse_args()

    map_yaml = pathlib.Path(args.map_yaml).resolve()
    cfg = parse_map_yaml(map_yaml)
    image_path = pathlib.Path(str(cfg["image"]))
    if not image_path.is_absolute():
        image_path = (map_yaml.parent / image_path).resolve()
    width, height, maxval, pixels = read_pgm(image_path)
    cls = classify(args.x, args.y, cfg, width, height, maxval, pixels)
    nearest = nearest_free(
        args.x, args.y, cfg, width, height, maxval, pixels, args.max_nearest_radius
    )

    print(f"map_yaml={map_yaml}")
    print(f"map_image={image_path}")
    print(f"status={cls['status']}")
    print(f"mx={cls.get('mx', '')}")
    print(f"py={cls.get('py', '')}")
    print(f"pixel={cls.get('pixel', '')}")
    print(f"occ_prob={cls.get('occ_prob', '')}")
    if nearest is None:
        print("nearest_free_x=")
        print("nearest_free_y=")
        print("nearest_free_dist=")
    else:
        d, nx, ny = nearest
        print(f"nearest_free_x={nx:.3f}")
        print(f"nearest_free_y={ny:.3f}")
        print(f"nearest_free_dist={d:.3f}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error={exc}", file=sys.stderr)
        raise
