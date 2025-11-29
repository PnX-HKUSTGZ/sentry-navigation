#!/usr/bin/env python3
"""
STEP -> STL conversion helper for headless FreeCAD

Usage (with FreeCADCmd):
  FreeCADCmd tools/step_to_stl_freecad.py /path/in.stp /path/out.stl --scale 0.001

If you're using the included AppImage in this repo, you can run (example):
  ./squashfs-root/usr/bin/FreeCADCmd tools/step_to_stl_freecad.py \
    /home/nyz/sentry/UTF-8__RMUC2026.stp \
    /home/nyz/sentry/sentry-navigation/src/rm_simulation/pb_rm_simulation/meshes/RMUC2026_world/meshes/RMUC_2026.stl \
    --scale 0.001

Notes:
- This script is intended to be executed using FreeCAD's Python runtime (FreeCADCmd),
  not with the system Python. It uses FreeCAD/Part/Mesh/Draft APIs which exist inside FreeCAD.
- We DO NOT run the AppImage automatically without your confirmation. I will produce the command
  for you and can run it if you want.
"""
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description="Convert STEP to STL (optionally scaled) using FreeCAD API")
    parser.add_argument('input', help='input STEP file (.stp/.step)')
    parser.add_argument('output', help='output STL file (.stl)')
    parser.add_argument('--scale', type=float, default=0.001, help='scale factor applied to geometry (default: 0.001 for mm->m)')
    parser.add_argument('--linear-deflection', type=float, default=0.5, help='mesh linear deflection for tessellation (smaller => finer mesh)')
    parser.add_argument('--angular-deflection', type=float, default=0.5, help='angular deflection for tessellation')
    args = parser.parse_args()

    # Import FreeCAD modules at runtime so the script can be imported on systems without FreeCAD
    try:
        import FreeCAD
        import Part
        import Mesh
        import Draft
    except Exception as e:
        print("ERROR: This script must be run with FreeCAD's Python (FreeCADCmd).", file=sys.stderr)
        print("Detail:", e, file=sys.stderr)
        sys.exit(2)

    infile = os.path.abspath(args.input)
    outfile = os.path.abspath(args.output)

    if not os.path.exists(infile):
        print(f"ERROR: input STEP not found: {infile}", file=sys.stderr)
        sys.exit(3)

    outdir = os.path.dirname(outfile)
    if outdir and not os.path.isdir(outdir):
        os.makedirs(outdir, exist_ok=True)

    print(f"Loading STEP: {infile}")
    try:
        shape = Part.read(infile)
    except Exception as e:
        print(f"ERROR reading STEP: {e}", file=sys.stderr)
        sys.exit(4)

    # Create a temporary document and object
    doc = FreeCAD.newDocument()
    part_obj = doc.addObject('Part::Feature', 'Imported')
    part_obj.Shape = shape

    # Apply scale by creating a scaled copy using Draft.scale
    scale = float(args.scale)
    if abs(scale - 1.0) > 1e-12:
        print(f"Scaling object by {scale} (origin at 0,0,0)")
        try:
            # Draft.scale expects a list of objects and returns scaled objects
            Draft.scale([part_obj], delta=FreeCAD.Vector(scale, scale, scale), center=FreeCAD.Vector(0, 0, 0))
        except Exception as e:
            print(f"WARNING: Draft.scale failed: {e}. Trying an alternative scaling via placement matrix.", file=sys.stderr)
            # Fallback: apply a placement matrix scaling to the shape
            m = FreeCAD.Matrix()
            m.A11 = scale
            m.A22 = scale
            m.A33 = scale
            try:
                part_obj.Shape = part_obj.Shape.transformGeometry(m)
            except Exception as e2:
                print(f"ERROR: fallback scaling failed: {e2}", file=sys.stderr)
                sys.exit(6)

    # Export mesh to STL
    try:
        print(f"Exporting STL to: {outfile}")
        Mesh.export([part_obj], outfile)
    except Exception as e:
        print(f"ERROR exporting STL: {e}", file=sys.stderr)
        sys.exit(5)

    print("Done.")


if __name__ == '__main__':
    main()
