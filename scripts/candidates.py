#!/usr/bin/env python3
"""Find manual loop-closure candidates from an exported keyframe trajectory.

Automatic loop closure fails on the sparse Unitree L1 lidar, so loops must be
specified by hand. This helper lists pairs of keyframes that are close in XY but
far apart in time (= the robot revisited a place). YOU confirm the real revisits
from your knowledge of the route, then copy their timestamps into the
`manual_loop_constraints:` list of your loop-closure pipeline yaml.

Usage:
    # first, in the ~/mola clean env:
    #   sm-cli export-keyframes final_map.simplemap --output kfs.tum
    python3 candidates.py kfs.tum [xy_threshold_m=2.5] [min_kf_gap=40]

Tip: for a BIGGER map with more drift, raise xy_threshold (e.g. 3-5 m).
"""
import sys
import numpy as np

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

tum = sys.argv[1]
XY_THR = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5   # [m]
MIN_GAP = int(sys.argv[3]) if len(sys.argv) > 3 else 40     # min keyframe-index gap

d = np.loadtxt(tum)                 # columns: t x y z qx qy qz qw
t, xy = d[:, 0], d[:, 1:3]

pairs = []
for i in range(len(d)):
    for j in range(i + MIN_GAP, len(d)):
        dxy = float(np.hypot(*(xy[i] - xy[j])))
        if dxy < XY_THR:
            pairs.append((dxy, i, j, t[i], t[j], t[j] - t[i]))
pairs.sort()

print(f"# candidates (dXY < {XY_THR} m, kf gap > {MIN_GAP})  -- pick ONE per real revisit")
print(f"{'dXY':>5} {'kf_i':>5} {'kf_j':>5} {'dt_s':>7}   timestamp_i          timestamp_j")
for dxy, i, j, ti, tj, dt in pairs[:60]:
    print(f"{dxy:5.2f} {i:5d} {j:5d} {dt:7.1f}   {ti:.6f}   {tj:.6f}")
if not pairs:
    print("# none found - raise xy_threshold or check the trajectory")
