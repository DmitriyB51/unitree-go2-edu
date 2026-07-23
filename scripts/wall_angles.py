#!/usr/bin/env python3
"""Is the building still RECTANGULAR? -- measures how "Manhattan" a map is.

A real building has walls in a few dominant directions, ~90 deg apart. If loop
closure warps the geometry, those directions smear out: corners stop being
square. This measures that directly:

  1. cut the cloud into small wall patches (same as wall_sharpness.py)
  2. take each patch's normal direction (its compass angle)
  3. fold every angle into 0..90 deg -- in a rectangular building EVERY wall,
     whichever way it faces, lands on the SAME folded angle
  4. report how tightly they cluster

  spread  <3 deg  : crisp right angles
  spread 3-6 deg  : normal for sparse L1
  spread  >8 deg  : geometry is warped, corners are visibly off

Usage:
    python3 wall_angles.py map.ply [map2.ply ...]     # compares several maps
"""
import sys
import numpy as np


def load(path):
    """Read a binary little-endian PLY, return the x,y,z columns."""
    with open(path, 'rb') as f:
        head, n, props = b'', 0, 0
        while True:
            line = f.readline()
            head += line
            if line.startswith(b'element vertex'):
                n = int(line.split()[2])
            elif line.startswith(b'property'):
                props += 1
            elif line.startswith(b'end_header'):
                break
        off = f.tell()
    # every property here is a float32
    return np.fromfile(path, dtype=np.float32, offset=off,
                       count=n * props).reshape(n, props)[:, :3].astype(np.float64)


def patches(xyz, cell=2.0, min_pts=60, max_planarity=0.30):
    """Yield (normal, thickness) for every flat, vertical patch of the cloud."""
    h, e = np.histogram(xyz[:, 2], bins=400)
    floor = 0.5 * (e[h.argmax()] + e[h.argmax() + 1])
    band = xyz[(xyz[:, 2] > floor + 0.4) & (xyz[:, 2] < floor + 1.6)]

    key = np.floor(band[:, :2] / cell).astype(np.int64)
    order = np.lexsort((key[:, 1], key[:, 0]))
    band, key = band[order], key[order]
    starts = np.flatnonzero(np.r_[True, np.any(np.diff(key, axis=0) != 0, axis=1)])

    for a, b in zip(starts, np.r_[starts[1:], len(band)]):
        p = band[a:b]
        if len(p) < min_pts:
            continue
        c = p - p.mean(0)
        evals, evecs = np.linalg.eigh(c.T @ c / len(c))
        n = evecs[:, 0]
        if abs(n[2]) > 0.5:                    # floor or ceiling
            continue
        if evals[0] / evals[1] > max_planarity:  # corner or clutter
            continue
        yield n, float(np.std(c @ n))


def grid_angle(ang):
    """Dominant 90-deg grid orientation + spread for a set of folded angles."""
    z = np.exp(1j * np.radians(ang * 4)).mean()
    return (np.degrees(np.angle(z)) / 4) % 90, np.degrees(np.sqrt(-2 * np.log(abs(z)))) / 4


print(f"{'map':26s} {'n':>4} {'spread':>7} {'sector twist':>13} {'thickness':>10}")
for path in sys.argv[1:]:
    xyz = load(path)
    # patch centres are needed for the per-sector check, so redo the loop here
    h, e = np.histogram(xyz[:, 2], bins=400)
    floor = 0.5 * (e[h.argmax()] + e[h.argmax() + 1])
    band = xyz[(xyz[:, 2] > floor + 0.4) & (xyz[:, 2] < floor + 1.6)]
    CELL, MIN_PTS = 1.0, 25
    key = np.floor(band[:, :2] / CELL).astype(np.int64)
    o = np.lexsort((key[:, 1], key[:, 0]))
    band, key = band[o], key[o]
    st = np.flatnonzero(np.r_[True, np.any(np.diff(key, axis=0) != 0, axis=1)])

    cen, ang, thick = [], [], []
    for a, b in zip(st, np.r_[st[1:], len(band)]):
        p = band[a:b]
        if len(p) < MIN_PTS:
            continue
        c = p - p.mean(0)
        ev, evec = np.linalg.eigh(c.T @ c / len(c))
        n = evec[:, 0]
        if abs(n[2]) > 0.5 or ev[0] / ev[1] > 0.40:
            continue
        cen.append(p.mean(0)[:2])
        ang.append(np.degrees(np.arctan2(n[1], n[0])) % 90)   # folded compass angle
        thick.append(np.std(c @ n))
    cen, ang, thick = np.array(cen), np.array(ang), np.array(thick)
    if len(ang) < 20:
        print(f"{path.split('/')[-1]:26s} only {len(ang)} patches, not enough")
        continue

    _, spread = grid_angle(ang)

    # Per-sector check: split the map into three slabs along X and compare each
    # slab's own grid orientation. If the slabs disagree, whole wings of the
    # building are rotated relative to each other -- corners stop being square
    # even though each wall on its own looks fine.
    q = np.quantile(cen[:, 0], [0, 1/3, 2/3, 1.0])
    loc = [grid_angle(ang[(cen[:, 0] >= q[k]) & (cen[:, 0] <= q[k+1])])[0]
           for k in range(3)
           if ((cen[:, 0] >= q[k]) & (cen[:, 0] <= q[k+1])).sum() >= 8]
    twist = max((abs((loc[i]-loc[j]+45) % 90 - 45)
                 for i in range(len(loc)) for j in range(i+1, len(loc))), default=0)

    print(f"{path.split('/')[-1]:26s} {len(ang):4d} {spread:6.1f}° {twist:12.1f}° "
          f"{np.median(thick)*100:8.1f} cm")
