#!/usr/bin/env python3
"""
Scan map YAMLs and PCDs to generate src/rm_nav_bringup/config/maps_index.yaml.

- Map YAML dir: src/rm_nav_bringup/map
- PCD dir:       src/rm_nav_bringup/PCD

Usage:
  python tools/update_maps_index.py \
    [--map-dir src/rm_nav_bringup/map] \
    [--pcd-dir src/rm_nav_bringup/PCD] \
    [--out-file src/rm_nav_bringup/config/maps_index.yaml]

The generated YAML structure:
worlds:
  <WORLD_NAME>:
    yaml: "<relative path to map yaml>"
    yaml_exists: <bool>
    pcd: "<relative path to pcd>"
    pcd_exists: <bool>
    status: "ok|missing-yaml|missing-pcd"
"""
from __future__ import annotations
import argparse
import os
from pathlib import Path
from typing import Dict


def rel(path: Path, root: Path) -> str:
    try:
        return str(path.relative_to(root))
    except Exception:
        return str(path)


def discover_worlds(map_dir: Path, pcd_dir: Path) -> Dict[str, dict]:
    worlds = {}
    map_files = {p.stem: p for p in map_dir.glob("*.yaml")}
    pcd_files = {p.stem: p for p in pcd_dir.glob("*.pcd")}

    names = sorted(set(map_files.keys()) | set(pcd_files.keys()))
    repo_root = Path.cwd()

    for name in names:
        map_path = map_files.get(name)
        pcd_path = pcd_files.get(name)
        yaml_path_str = rel(map_path, repo_root) if map_path else str(map_dir.joinpath(f"{name}.yaml"))
        pcd_path_str = rel(pcd_path, repo_root) if pcd_path else str(pcd_dir.joinpath(f"{name}.pcd"))
        yaml_exists = map_path is not None and map_path.exists()
        pcd_exists = pcd_path is not None and pcd_path.exists()
        if yaml_exists and pcd_exists:
            status = "ok"
        elif yaml_exists and not pcd_exists:
            status = "missing-pcd"
        elif not yaml_exists and pcd_exists:
            status = "missing-yaml"
        else:
            status = "missing-both"
        worlds[name] = {
            "yaml": yaml_path_str,
            "yaml_exists": yaml_exists,
            "pcd": pcd_path_str,
            "pcd_exists": pcd_exists,
            "status": status,
        }
    return worlds


def to_yaml(worlds: Dict[str, dict]) -> str:
    # Minimal YAML emitter to avoid external dependencies
    lines = ["worlds:"]
    for name in worlds:
        info = worlds[name]
        lines.append(f"  {name}:")
        lines.append(f"    yaml: \"{info['yaml']}\"")
        lines.append(f"    yaml_exists: {'true' if info['yaml_exists'] else 'false'}")
        lines.append(f"    pcd: \"{info['pcd']}\"")
        lines.append(f"    pcd_exists: {'true' if info['pcd_exists'] else 'false'}")
        lines.append(f"    status: \"{info['status']}\"")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", default="src/rm_nav_bringup/map", type=str)
    parser.add_argument("--pcd-dir", default="src/rm_nav_bringup/PCD", type=str)
    parser.add_argument("--out-file", default="src/rm_nav_bringup/config/maps_index.yaml", type=str)
    args = parser.parse_args()

    map_dir = Path(args.map_dir)
    pcd_dir = Path(args.pcd_dir)
    out_file = Path(args.out_file)

    if not map_dir.exists():
        raise SystemExit(f"Map dir not found: {map_dir}")
    if not pcd_dir.exists():
        raise SystemExit(f"PCD dir not found: {pcd_dir}")

    worlds = discover_worlds(map_dir, pcd_dir)

    # Ensure parent
    out_file.parent.mkdir(parents=True, exist_ok=True)

    content = to_yaml(worlds)
    out_file.write_text(content, encoding="utf-8")

    # Print a brief summary
    total = len(worlds)
    ok = sum(1 for w in worlds.values() if w["status"] == "ok")
    miss_pcd = sum(1 for w in worlds.values() if w["status"] == "missing-pcd")
    miss_yaml = sum(1 for w in worlds.values() if w["status"] == "missing-yaml")
    miss_both = sum(1 for w in worlds.values() if w["status"] == "missing-both")
    print(f"Wrote {out_file} :: total={total}, ok={ok}, missing-pcd={miss_pcd}, missing-yaml={miss_yaml}, missing-both={miss_both}")


if __name__ == "__main__":
    main()
