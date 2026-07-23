#!/usr/bin/env python3
"""For each revisit candidate: build a small local submap at BOTH keyframes and
ICP them against each other, starting from the pose the map already claims.
How far ICP has to move it = the real, remaining error at that place."""
import sys, numpy as np
from scipy.spatial import cKDTree
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2 as pc2

BAG, TUM = sys.argv[1], sys.argv[2]
EVENTS = [tuple(map(int, a.split(':'))) for a in sys.argv[3].split(',')]  # "i:j,i:j"
W = 1.0          # [s] half-window of scans accumulated into one submap

def q2R(qx, qy, qz, qw):
    n = np.sqrt(qx*qx+qy*qy+qz*qz+qw*qw); qx,qy,qz,qw = qx/n,qy/n,qz/n,qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]])

def T(R, t):
    M = np.eye(4); M[:3,:3] = R; M[:3,3] = t; return M

# --- keyframe timestamps + loop-closed poses -------------------------------
tum = np.loadtxt(TUM)
kf_t = tum[:,0]
kf_T = [T(q2R(*r[4:8]), r[1:4]) for r in tum]

want = sorted({k for e in EVENTS for k in e})
windows = [(kf_t[k]-W, kf_t[k]+W) for k in want]
lo, hi = min(w[0] for w in windows)-3, max(w[1] for w in windows)+3

# --- read ONLY the windows we need: seek() straight to each one -----------
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/state_estimation',
                                               '/cloud_registered_body']))
odo, clouds = [], []
for w0, w1 in windows:
    rd.seek(int(w0 * 1e9))
    while rd.has_next():
        topic, raw, tns = rd.read_next()
        if tns * 1e-9 > w1:
            break
        if topic == '/state_estimation':
            m = deserialize_message(raw, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odo.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                        p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        else:
            m = deserialize_message(raw, PointCloud2)
            pts = pc2.read_points_numpy(m, field_names=('x','y','z'))
            clouds.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                           np.asarray(pts, dtype=np.float64)))
odo = np.array(odo); odo = odo[np.argsort(odo[:,0])]
clouds.sort(key=lambda c: c[0])
print(f"loaded {len(odo)} odom + {len(clouds)} clouds in the windows")

def raw_pose(t):
    """Nearest RAW Point-LIO pose. Locally (over ~2 s) it is accurate."""
    i = int(np.argmin(np.abs(odo[:,0]-t))); r = odo[i]
    return T(q2R(*r[4:8]), r[1:4])

def submap(kf):
    """Scans within +-W s of this keyframe, expressed in the keyframe's frame."""
    t0 = kf_t[kf]; P0inv = np.linalg.inv(raw_pose(t0)); out = []
    for tc, pts in clouds:
        if abs(tc-t0) > W or len(pts) == 0:
            continue
        rel = P0inv @ raw_pose(tc)               # body_kf <- body_scan
        out.append(pts @ rel[:3,:3].T + rel[:3,3])
    return np.vstack(out) if out else np.zeros((0,3))

def icp(src, dst, T0, iters=40, max_corr=1.0, trim=0.8, rot=True):
    """Trimmed point-to-point ICP. Returns the correction applied on top of T0."""
    tree = cKDTree(dst); Tc = T0.copy()
    for _ in range(iters):
        s = src @ Tc[:3,:3].T + Tc[:3,3]
        d, idx = tree.query(s, distance_upper_bound=max_corr)
        ok = np.isfinite(d)
        if ok.sum() < 30:
            break
        thr = np.quantile(d[ok], trim)           # drop the worst 20% of matches
        ok &= d <= thr
        A, B = s[ok], dst[idx[ok]]
        ca, cb = A.mean(0), B.mean(0)
        if rot:
            U, _, Vt = np.linalg.svd((A-ca).T @ (B-cb))
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1; R = Vt.T @ U.T
        else:
            R = np.eye(3)                    # translation-only: no rotation
        Tc = T(R, cb - R @ ca) @ Tc
    s = src @ Tc[:3,:3].T + Tc[:3,3]
    d, _ = tree.query(s, distance_upper_bound=max_corr)
    inl = np.isfinite(d)
    rmse = float(np.sqrt(np.mean(d[inl]**2))) if inl.any() else float('nan')
    return Tc, inl.mean(), rmse

def rms_to(src, dst, Tx, max_corr=1.0):
    """How far apart are the two passes when placed by the given transform?"""
    tree = cKDTree(dst)
    p = src @ Tx[:3,:3].T + Tx[:3,3]
    d, _ = tree.query(p, distance_upper_bound=max_corr)
    ok = np.isfinite(d)
    return (float(np.sqrt(np.mean(d[ok]**2))), float(np.median(d[ok])), float(ok.mean()))

print(f"\n{'i':>4} {'j':>4} {'dXY_map':>8} | {'rms0':>6} {'med0':>6} {'ovl%':>5} |"
      f" {'shift_only':>10} {'->rms':>6} | {'full_dXY':>8} {'dyaw':>6} {'->rms':>6}")
for i, j in EVENTS:
    Si, Sj = submap(i), submap(j)
    if len(Si) < 100 or len(Sj) < 100:
        print(f"{i:4d} {j:4d}  too few points"); continue
    T0 = np.linalg.inv(kf_T[i]) @ kf_T[j]      # what the closed map claims
    rms0, med0, ovl = rms_to(Sj, Si, T0)

    # (a) translation-only fit: robust in corridors, no rotation to slide into
    Tt, _, rmst = icp(Sj, Si, T0, iters=40, max_corr=1.0, rot=False)
    dt = float(np.hypot(*(Tt @ np.linalg.inv(T0))[:2,3]))

    # (b) full 6-DoF fit
    Tc, _, rmsf = icp(Sj, Si, T0, iters=40, max_corr=1.0)
    D = Tc @ np.linalg.inv(T0)
    dxy = float(np.hypot(*D[:2,3]))
    dyaw = float(np.degrees(np.arctan2(D[1,0], D[0,0])))
    dmap = float(np.hypot(*(kf_T[i][:2,3]-kf_T[j][:2,3])))
    print(f"{i:4d} {j:4d} {dmap:8.2f} | {rms0:6.3f} {med0:6.3f} {ovl*100:5.0f} |"
          f" {dt:10.2f} {rmst:6.3f} | {dxy:8.2f} {dyaw:6.1f} {rmsf:6.3f}")
