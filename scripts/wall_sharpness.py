#!/usr/bin/env python3
"""Measure how SHARP a map is -- the objective "is my map good?" metric.

The idea: a wall is a flat surface. Cut the cloud into small patches, fit a
plane through each patch, and look at how far the points scatter around that
plane. That scatter IS the map error, in centimetres:

  ~5-10 cm  = the L1 sensor/Point-LIO noise floor. This is as good as it gets;
              no amount of loop closure will shrink it.
  ~20+ cm   = the same wall got mapped twice at slightly different places, i.e.
              a real pose-graph problem that loop closure CAN fix.

Use it to decide whether a map is worth re-optimising, and to compare two
pipeline settings (e.g. submap_radius 0.15 vs 1.5) with a number instead of
squinting at mm-viewer.

Usage:
    python3 wall_sharpness.py ~/maps/loc_5_map.pcd

Reference numbers measured 2026-07-22 (see CLAUDE.md):
    loc_5_map.pcd      median std  10.4 cm,  0 % of patches above 20 cm
    final_map_lc.pcd   median std   8.2 cm,  0 % of patches above 20 cm
"""
import sys
import numpy as np

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
path = sys.argv[1]

# --- Read a binary PCD by hand (no PCL/open3d needed) ------------------------
# The header is plain text and ends with the line "DATA binary"; everything
# after it is a flat array of float32, nfields values per point.
raw = open(path, 'rb').read(2048)
off = raw.find(b'DATA binary\n') + len(b'DATA binary\n')
hdr = {l.split()[0].decode(): l.split()[1:] for l in raw[:off].split(b'\n') if l.strip()}
nfields = len(hdr['FIELDS'])
npts = int(hdr['POINTS'][0])
data = np.fromfile(path, dtype=np.float32, offset=off,
                   count=npts * nfields).reshape(npts, nfields)
xyz = data[:, :3].astype(np.float64)   # we only need x,y,z
print(f"{npts} points, x[{xyz[:,0].min():.1f},{xyz[:,0].max():.1f}] "
      f"y[{xyz[:,1].min():.1f},{xyz[:,1].max():.1f}] "
      f"z[{xyz[:,2].min():.1f},{xyz[:,2].max():.1f}]")

# --- Where is the floor? -----------------------------------------------------
# Most points in an indoor lidar map are floor, so the tallest peak of the
# Z histogram is the floor level.
h, e = np.histogram(xyz[:, 2], bins=400)
floor = 0.5 * (e[h.argmax()] + e[h.argmax() + 1])
print(f"floor z ~ {floor:.2f} m")

# Keep a horizontal band well above the floor and below the ceiling: what is
# left is walls, furniture and doors -- the vertical structure we can fit.
band = xyz[(xyz[:, 2] > floor + 0.4) & (xyz[:, 2] < floor + 1.6)]
print(f"wall band: {len(band)} points")

# --- Chop the band into square XY cells; each dense cell is one wall patch ---
CELL = 2.0        # [m] patch size. Small enough that a real wall is flat inside.
MIN_PTS = 60      # a patch with fewer points is noise, not a surface
MAX_PLANARITY = 0.30  # eval0/eval1: 0 = perfectly flat, ~1 = a corner or clutter

key = np.floor(band[:, :2] / CELL).astype(np.int64)
order = np.lexsort((key[:, 1], key[:, 0]))     # sort so cells become contiguous
band, key = band[order], key[order]
starts = np.flatnonzero(np.r_[True, np.any(np.diff(key, axis=0) != 0, axis=1)])
bounds = np.r_[starts, len(band)]

results = []
for a, b in zip(bounds[:-1], bounds[1:]):
    p = band[a:b]
    if len(p) < MIN_PTS:
        continue
    c = p - p.mean(0)
    # PCA on the patch: the eigenvector of the SMALLEST eigenvalue is the
    # direction in which the points spread least = the surface normal.
    evals, evecs = np.linalg.eigh(c.T @ c / len(c))
    n = evecs[:, 0]
    if abs(n[2]) > 0.5:            # normal points up/down -> floor or ceiling
        continue
    if evals[0] / evals[1] > MAX_PLANARITY:   # not actually a flat surface
        continue
    d = c @ n                      # distance of every point from the fitted plane
    results.append((np.std(d), np.percentile(np.abs(d), 95), len(p)))

if not results:
    print("\nNo wall patches found -- try a larger CELL or a smaller MIN_PTS.")
    sys.exit(1)

r = np.array(results)
print(f"\nwall patches analysed: {len(r)}")
for q in (10, 25, 50, 75, 90):
    print(f"  p{q:<2d}  std = {np.percentile(r[:,0], q)*100:5.1f} cm   "
          f"95%-spread = {np.percentile(r[:,1], q)*100:5.1f} cm")
print(f"\n  patches with std > 10 cm: {(r[:,0]>0.10).mean()*100:.0f} %   (sensor noise level)")
print(f"  patches with std > 20 cm: {(r[:,0]>0.20).mean()*100:.0f} %   (= doubled walls, "
      f"loop closure would help)")
