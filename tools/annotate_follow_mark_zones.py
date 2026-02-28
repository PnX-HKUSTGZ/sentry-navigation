#!/usr/bin/env python3
"""
Interactive polygon annotator for follow_mark.zone_polygons on Nav2 maps.

Supports:
- Existing map YAML+PGM from repository.
- Temporary maps saved by `nav2_map_server map_saver_cli -f ...`.

Usage example:
  python3 tools/annotate_follow_mark_zones.py \
    --map-yaml src/rm_nav_bringup/map/RMUL26.yaml \
    --write-launch-params src/rm_nav_bringup/config/launch_params.yaml

You may also pass a `.pgm` path to `--map-yaml`; the tool will infer a sibling `.yaml`.
"""

from __future__ import annotations

import argparse
import datetime as dt
import pathlib
import shutil
import sys
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml

from map_goal_probe import parse_map_yaml, read_pgm

Point = Tuple[float, float]
Polygon = List[Point]


@dataclass
class MapInfo:
    map_yaml: pathlib.Path
    map_image: pathlib.Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    image_plot: np.ndarray
    image_extent: Tuple[float, float, float, float]


def _normalize_polygon(raw: object) -> Polygon:
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"polygon must be a list, got: {raw!r}")
    if len(raw) == 0:
        return []

    points: Polygon = []
    if isinstance(raw[0], (list, tuple)):
        for p in raw:
            if not isinstance(p, (list, tuple)) or len(p) != 2:
                raise ValueError(f"polygon vertex must be [x,y], got: {p!r}")
            points.append((float(p[0]), float(p[1])))
    else:
        vals = [float(v) for v in raw]
        if len(vals) % 2 != 0:
            raise ValueError(f"flat polygon length must be even, got: {len(vals)}")
        for i in range(0, len(vals), 2):
            points.append((vals[i], vals[i + 1]))

    if points and len(points) < 3:
        raise ValueError("polygon must have >= 3 points")
    return points


def _normalize_polygons(raw: object) -> List[Polygon]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError(f"zone_polygons must be a list, got: {type(raw)}")
    polygons: List[Polygon] = []
    for poly in raw:
        norm = _normalize_polygon(poly)
        if norm:
            polygons.append(norm)
    return polygons


def load_map(map_yaml_path: pathlib.Path) -> MapInfo:
    cfg = parse_map_yaml(map_yaml_path)
    image_path = pathlib.Path(str(cfg["image"]))
    if not image_path.is_absolute():
        image_path = (map_yaml_path.parent / image_path).resolve()

    width, height, maxval, pixels = read_pgm(image_path)
    image = np.array(pixels, dtype=np.float32).reshape((height, width))
    image_plot = np.flipud(image) / float(maxval)

    origin = cfg["origin"]
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError(f"invalid origin in map yaml: {origin!r}")

    origin_x = float(origin[0])
    origin_y = float(origin[1])
    origin_yaw = float(origin[2]) if len(origin) >= 3 else 0.0

    if abs(origin_yaw) > 1e-6:
        print(
            f"[WARN] map origin yaw={origin_yaw:.6f} rad. "
            "This annotator assumes yaw ~= 0 for correct overlay.",
            file=sys.stderr,
        )

    res = float(cfg["resolution"])
    x_max = origin_x + width * res
    y_max = origin_y + height * res
    extent = (origin_x, x_max, origin_y, y_max)

    return MapInfo(
        map_yaml=map_yaml_path,
        map_image=image_path,
        resolution=res,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        width=width,
        height=height,
        image_plot=image_plot,
        image_extent=extent,
    )


def polygons_to_yaml_data(polygons: Sequence[Polygon], precision: int) -> dict:
    out: List[List[List[float]]] = []
    for poly in polygons:
        out_poly: List[List[float]] = []
        for x, y in poly:
            out_poly.append([round(x, precision), round(y, precision)])
        out.append(out_poly)
    return {"follow_mark": {"zone_polygons": out}}


def load_polygons_from_launch_params(path: pathlib.Path) -> List[Polygon]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return []
    follow_mark = data.get("follow_mark", {})
    if not isinstance(follow_mark, dict):
        return []
    return _normalize_polygons(follow_mark.get("zone_polygons", []))


def write_polygons_to_launch_params(path: pathlib.Path, polygons: Sequence[Polygon], precision: int) -> None:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"launch params root must be mapping, got {type(data)}")
    follow_mark = data.get("follow_mark")
    if not isinstance(follow_mark, dict):
        follow_mark = {}
        data["follow_mark"] = follow_mark
    follow_mark["zone_polygons"] = polygons_to_yaml_data(polygons, precision)["follow_mark"]["zone_polygons"]

    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(path.suffix + f".bak_{timestamp}")
    shutil.copy2(path, backup)
    path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    print(f"[INFO] launch params updated: {path}")
    print(f"[INFO] backup saved: {backup}")


class PolygonAnnotator:
    def __init__(
        self,
        map_info: MapInfo,
        polygons: List[Polygon],
        out_yaml: pathlib.Path,
        out_png: pathlib.Path,
        precision: int,
        write_launch_params: pathlib.Path | None,
    ) -> None:
        self.map_info = map_info
        self.polygons = polygons
        self.current: Polygon = []
        self.out_yaml = out_yaml
        self.out_png = out_png
        self.precision = precision
        self.write_launch_params = write_launch_params

        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.redraw()

    def redraw(self) -> None:
        self.ax.clear()
        xmin, xmax, ymin, ymax = self.map_info.image_extent
        self.ax.imshow(
            self.map_info.image_plot,
            cmap="gray",
            origin="lower",
            extent=(xmin, xmax, ymin, ymax),
            interpolation="nearest",
            vmin=0.0,
            vmax=1.0,
        )

        for idx, poly in enumerate(self.polygons):
            xs = [p[0] for p in poly] + [poly[0][0]]
            ys = [p[1] for p in poly] + [poly[0][1]]
            self.ax.plot(xs, ys, "-", linewidth=2.0)
            cx = sum(p[0] for p in poly) / len(poly)
            cy = sum(p[1] for p in poly) / len(poly)
            self.ax.text(cx, cy, f"P{idx}", fontsize=9, color="tab:blue")

        if self.current:
            xs = [p[0] for p in self.current]
            ys = [p[1] for p in self.current]
            self.ax.plot(xs, ys, "y.-", linewidth=2.0, markersize=8)
            self.ax.scatter(xs[-1], ys[-1], c="red", s=40)

        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("map x (m)")
        self.ax.set_ylabel("map y (m)")
        self.ax.set_title(
            "Follow Mark Zone Annotator | "
            "L-click:add  R-click/Backspace:undo  Enter:commit polygon  "
            "d:drop last polygon  c:clear current  s:save  q:quit"
        )
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()

    def on_click(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        if event.button == 1:
            self.current.append((float(event.xdata), float(event.ydata)))
            print(f"[POINT] ({event.xdata:.3f}, {event.ydata:.3f})")
            self.redraw()
        elif event.button == 3:
            if self.current:
                removed = self.current.pop()
                print(f"[UNDO POINT] ({removed[0]:.3f}, {removed[1]:.3f})")
                self.redraw()

    def on_key(self, event) -> None:
        key = (event.key or "").lower()
        if key in {"enter", "return"}:
            self.commit_current_polygon()
        elif key in {"backspace", "u"}:
            if self.current:
                removed = self.current.pop()
                print(f"[UNDO POINT] ({removed[0]:.3f}, {removed[1]:.3f})")
                self.redraw()
        elif key == "c":
            self.current = []
            print("[CLEAR] current polygon cleared")
            self.redraw()
        elif key == "d":
            if self.polygons:
                self.polygons.pop()
                print("[DROP] removed last committed polygon")
                self.redraw()
        elif key == "s":
            self.save_outputs()
        elif key in {"q", "escape"}:
            plt.close(self.fig)
        elif key == "h":
            print(
                "Shortcuts: L-click add point | R-click/backspace undo point | "
                "enter commit polygon | d drop polygon | c clear current | s save | q quit"
            )

    def commit_current_polygon(self) -> None:
        if len(self.current) < 3:
            print("[WARN] current polygon has <3 points, ignored")
            return
        self.polygons.append(self.current.copy())
        print(f"[COMMIT] polygon #{len(self.polygons)-1} with {len(self.current)} points")
        self.current = []
        self.redraw()

    def save_outputs(self) -> None:
        data = polygons_to_yaml_data(self.polygons, self.precision)
        self.out_yaml.parent.mkdir(parents=True, exist_ok=True)
        self.out_yaml.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        self.out_png.parent.mkdir(parents=True, exist_ok=True)
        self.fig.savefig(self.out_png, dpi=180)
        print(f"[SAVE] yaml: {self.out_yaml}")
        print(f"[SAVE] png : {self.out_png}")
        print(f"[SAVE] polygons: {len(self.polygons)}")
        if self.write_launch_params is not None:
            write_polygons_to_launch_params(
                self.write_launch_params, self.polygons, self.precision
            )


def build_default_output_paths(map_yaml: pathlib.Path) -> Tuple[pathlib.Path, pathlib.Path]:
    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = map_yaml.stem
    base = pathlib.Path("/tmp") / f"{stem}_follow_mark_{ts}"
    return base.with_suffix(".yaml"), base.with_suffix(".png")


def resolve_map_yaml_input(map_arg: str) -> pathlib.Path:
    """
    Accept either map YAML path or map PGM path.
    If a PGM is passed, infer sibling YAML with same stem.
    """
    path = pathlib.Path(map_arg).expanduser().resolve()
    suffix = path.suffix.lower()
    if suffix == ".yaml" or suffix == ".yml":
        return path
    if suffix == ".pgm":
        candidate = path.with_suffix(".yaml")
        if candidate.exists():
            return candidate
        raise FileNotFoundError(
            f"map yaml not found for pgm: {path}. expected sibling yaml: {candidate}"
        )
    raise ValueError(
        f"--map-yaml must point to .yaml/.yml or .pgm file, got: {path}"
    )


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Interactive annotator for follow_mark zone polygons on map yaml+pgm."
    )
    ap.add_argument(
        "--map-yaml",
        required=True,
        help="Path to map .yaml/.yml or .pgm (if .pgm, sibling .yaml is inferred).",
    )
    ap.add_argument(
        "--launch-params",
        default="",
        help="Optional launch_params.yaml path, load existing follow_mark.zone_polygons as initial polygons.",
    )
    ap.add_argument(
        "--write-launch-params",
        default="",
        help="Optional launch_params.yaml path to update follow_mark.zone_polygons on save.",
    )
    ap.add_argument("--out-yaml", default="", help="Output yaml for annotated polygons.")
    ap.add_argument("--out-png", default="", help="Output png snapshot for annotated polygons.")
    ap.add_argument("--precision", type=int, default=3, help="Decimal precision for saved coordinates.")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    map_yaml = resolve_map_yaml_input(args.map_yaml)
    if not map_yaml.exists():
        raise FileNotFoundError(f"map yaml not found: {map_yaml}")

    out_yaml: pathlib.Path
    out_png: pathlib.Path
    default_out_yaml, default_out_png = build_default_output_paths(map_yaml)
    if args.out_yaml:
        out_yaml = pathlib.Path(args.out_yaml).resolve()
    else:
        out_yaml = default_out_yaml
    if args.out_png:
        out_png = pathlib.Path(args.out_png).resolve()
    else:
        out_png = default_out_png

    map_info = load_map(map_yaml)

    polygons: List[Polygon] = []
    if args.launch_params:
        launch_params = pathlib.Path(args.launch_params).resolve()
        if launch_params.exists():
            polygons = load_polygons_from_launch_params(launch_params)
            print(f"[INFO] loaded {len(polygons)} polygons from {launch_params}")
        else:
            print(f"[WARN] launch params not found: {launch_params}", file=sys.stderr)

    write_launch_params = pathlib.Path(args.write_launch_params).resolve() if args.write_launch_params else None

    print(f"[INFO] map yaml : {map_info.map_yaml}")
    print(f"[INFO] map image: {map_info.map_image}")
    print(
        f"[INFO] map size : {map_info.width}x{map_info.height} "
        f"res={map_info.resolution:.3f} origin=({map_info.origin_x:.3f},{map_info.origin_y:.3f},{map_info.origin_yaw:.3f})"
    )
    print("[INFO] Controls:")
    print("       L-click:add point | R-click/backspace:undo point | enter:commit polygon")
    print("       d:drop last polygon | c:clear current | s:save | q:quit")

    annotator = PolygonAnnotator(
        map_info=map_info,
        polygons=polygons,
        out_yaml=out_yaml,
        out_png=out_png,
        precision=args.precision,
        write_launch_params=write_launch_params,
    )
    plt.show()
    annotator.save_outputs()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
