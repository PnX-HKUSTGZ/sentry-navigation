#!/usr/bin/env python3
"""Generate RMUL2026 wave-overlay world for bumpy-road tests.

Design goals:
- Keep original RMUL2026 mesh map intact.
- Add a configurable wave strip using dense box segments.
- Ensure exact peak-to-peak amplitude can be represented by aligning samples.
"""

from __future__ import annotations

import math
from pathlib import Path


def generate_world(
    out_file: Path,
    *,
    x_start: float,
    length: float,
    dx: float,
    y_center: float,
    width: float,
    z_base: float,
    wavelength: float,
    peak_to_peak: float,
    thickness: float,
) -> None:
    amp = peak_to_peak / 2.0
    seg_count = int(round(length / dx))
    if seg_count <= 0:
        raise ValueError("seg_count must be positive")

    parts: list[str] = []
    for i in range(seg_count):
        # Sample at left edge so peak/trough align exactly when dx divides wavelength quarter points.
        x_left = x_start + i * dx
        x_center = x_left + 0.5 * dx
        z_top = z_base + amp * math.sin(2.0 * math.pi * (x_left - x_start) / wavelength)
        z_center = z_top - thickness / 2.0
        cname = f"c_{i:03d}"
        vname = f"v_{i:03d}"
        parts.append(
            f"""
        <collision name='{cname}'>
          <pose>{x_center:.4f} {y_center:.4f} {z_center:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{dx:.4f} {width:.4f} {thickness:.4f}</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>100</mu><mu2>50</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name='{vname}'>
          <pose>{x_center:.4f} {y_center:.4f} {z_center:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{dx:.4f} {width:.4f} {thickness:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.20 0.20 0.20 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
"""
        )

    world = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <world name='default'>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name='map'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://RMUL2026_world/meshes/RMUL_2026.stl</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode><mu>100</mu><mu2>50</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://RMUL2026_world/meshes/RMUL_2026.stl</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
      <pose>0 0 0 -1.5707963 0 0</pose>
    </model>

    <!-- Wave strip for bumpy-road tests -->
    <!-- wavelength={wavelength:.3f}m, peak_to_peak={peak_to_peak:.3f}m, x=[{x_start:.3f}, {x_start+length:.3f}] -->
    <model name='wave_strip_240mm_70mm'>
      <static>true</static>
      <link name='wave_link'>
{''.join(parts)}
      </link>
    </model>

    <gravity>0 0 -9.81</gravity>
  </world>
</sdf>
"""
    out_file.write_text(world, encoding="utf-8")


def main() -> None:
    out = Path("src/rm_simulation/pb_rm_simulation/world/RMUL2026_world/RMUL2026_wave.world")
    generate_world(
        out,
        x_start=-5.20,
        length=1.44,
        dx=0.01,
        y_center=3.30,
        width=0.70,
        z_base=1.135,
        wavelength=0.24,
        peak_to_peak=0.07,
        thickness=0.006,
    )
    print(f"generated: {out}")


if __name__ == "__main__":
    main()
