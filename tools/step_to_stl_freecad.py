#!/usr/bin/env python3
"""
STEP -> STL conversion helper for headless FreeCAD

Usage (with FreeCADCmd):
  FreeCADCmd tools/step_to_stl_freecad.py /path/in.stp /path/out.stl --scale 0.001

Some FreeCAD builds (including the AppImage in this repo) ship `freecadcmd` which
does not reliably accept extra script arguments on the command line. A robust
headless way is to run console mode and inject `sys.argv`:

    /path/to/freecadcmd -c <<'PY'
    import sys
    sys.argv=[
        'step_to_stl_freecad.py','/path/in.stp','/path/out.stl',
        '--scale','0.001','--linear-deflection','2.0','--angular-deflection','0.8'
    ]
    p='/abs/path/to/step_to_stl_freecad.py'
    exec(compile(open(p,'r',encoding='utf-8').read(), p, 'exec'), {'__name__':'__main__'})
    sys.exit(0)
    PY

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
import time


def main():
    parser = argparse.ArgumentParser(description="Convert STEP to STL (optionally scaled) using FreeCAD API")
    parser.add_argument('input', help='input STEP file (.stp/.step)')
    parser.add_argument('output', help='output STL file (.stl)')
    parser.add_argument('--scale', type=float, default=0.001, help='scale factor applied to geometry (default: 0.001 for mm->m)')
    parser.add_argument('--linear-deflection', type=float, default=0.5, help='mesh linear deflection for tessellation (smaller => finer mesh)')
    parser.add_argument('--angular-deflection', type=float, default=0.5, help='angular deflection for tessellation')
    parser.add_argument(
        '--stage',
        choices=['all', 'read', 'mesh', 'write'],
        default='all',
        help='Run only a specific stage for profiling (default: all)',
    )
    args = parser.parse_args()

    # Ensure we see logs even if the process is interrupted by timeout
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass

    # Import FreeCAD modules at runtime so the script can be imported on systems without FreeCAD
    try:
        import FreeCAD
        import Part
        import Mesh
        import MeshPart
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

    t0 = time.time()
    print(f"Loading STEP: {infile}", flush=True)
    try:
        shape = Part.read(infile)
    except Exception as e:
        print(f"ERROR reading STEP: {e}", file=sys.stderr)
        sys.exit(4)
    print(f"Loaded STEP in {time.time() - t0:.2f}s", flush=True)

    if args.stage == 'read':
        print("Stage=read complete.", flush=True)
        return

    # Scale the shape geometry directly (fast, no Draft dependency)
    scale = float(args.scale)
    if abs(scale - 1.0) > 1e-12:
        print(f"Scaling geometry by {scale} (origin at 0,0,0)", flush=True)
        m = FreeCAD.Matrix()
        m.A11 = scale
        m.A22 = scale
        m.A33 = scale
        t_scale = time.time()
        try:
            shape = shape.transformGeometry(m)
        except Exception as e:
            print(f"ERROR: scaling failed: {e}", file=sys.stderr)
            sys.exit(6)
        print(f"Scaled geometry in {time.time() - t_scale:.2f}s", flush=True)
    # Mesh explicitly with controlled quality (avoids slow/implicit defaults)
    print(
        "Meshing with "
        f"linear_deflection={args.linear_deflection}, angular_deflection={args.angular_deflection}"
        ,
        flush=True,
    )
    t_mesh = time.time()
    try:
        mesh = MeshPart.meshFromShape(
            Shape=shape,
            LinearDeflection=float(args.linear_deflection),
            AngularDeflection=float(args.angular_deflection),
            Relative=False,
        )
    except Exception as e:
        print(f"ERROR meshing shape: {e}", file=sys.stderr)
        sys.exit(7)
    print(f"Meshed in {time.time() - t_mesh:.2f}s", flush=True)

    if args.stage == 'mesh':
        print("Stage=mesh complete.", flush=True)
        return

    # Write STL
    try:
        print(f"Writing STL to: {outfile}", flush=True)
        t_write = time.time()
        mesh.write(outfile)
        print(f"Wrote STL in {time.time() - t_write:.2f}s", flush=True)
    except Exception as e:
        print(f"ERROR exporting STL: {e}", file=sys.stderr)
        sys.exit(5)

    print(f"Done in {time.time() - t0:.2f}s", flush=True)


if __name__ == '__main__':
    main()
