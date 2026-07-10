#!/usr/bin/env python3
"""Measure the Z distribution of a map's point cloud to set the Z-clip band.

The final map keeps only a floor..ceiling Z band (FilterBoundingBox in
sm2mm_dense.yaml) to drop below-floor and high-above outliers. The right band
depends on the floor level, which differs per recording. This prints Z
percentiles and suggests a clip band.

Usage:
    # in the ~/mola clean env, first export the cloud to text:
    #   mm2txt final_map_lc.mm -l localmap        # -> creates final_map_lc_localmap.txt
    python3 floor_z.py final_map_lc_localmap.txt

Then edit `bounding_box_min[2]` / `bounding_box_max[2]` in sm2mm_dense.yaml.
"""
import sys
import numpy as np

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

# column 2 = z; skip the header row written by mm2txt
z = np.loadtxt(sys.argv[1], usecols=(2,), skiprows=1)
if z.size > 4_000_000:                      # subsample huge clouds
    z = z[:: z.size // 4_000_000 + 1]

floor = np.percentile(z, 50)                # median ~ floor level (bulk of points)
print(f"points (sampled): {len(z)}")
for q in (0, 1, 5, 50, 95, 99, 100):
    print(f"  Z p{q:>3}: {np.percentile(z, q):+7.2f} m")
z_min = round(floor - 0.6, 1)               # keep floor + margin below
z_max = round(floor + 2.4, 1)               # ~ room height above floor
print(f"\nfloor ~ {floor:+.2f} m")
print(f"suggested Z-clip band for sm2mm_dense.yaml: [{z_min}, {z_max}] m")
print(f"  bounding_box_min: [ \"-1e6\", \"-1e6\", \"{z_min}\" ]")
print(f"  bounding_box_max: [  \"1e6\",  \"1e6\",  \"{z_max}\" ]")
