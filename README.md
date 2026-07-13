## Repo structure

```
go2_mola_pipeline/
├── README.md                       ← this runbook
├── plio2sm/                        ← the converter (C++ colcon package) — the only real code
│   ├── src/plio2sm.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── pipelines/
│   ├── sm2mm_dense.yaml            ← sm2mm map-building pipeline (lidar-only, voxel, Z-clip)
│   └── lc_manual.template.yaml     ← loop-closure pipeline template (planar + manual loops)
├── scripts/
│   ├── candidates.py               ← find manual loop-closure pairs from a trajectory
│   └── floor_z.py                  ← measure floor level to set the Z-clip band
└── env/
    ├── mola                        ← clean-env launcher (ROS+MOLA, no conda/Isaac)
    └── mola_bashrc                 ← its rcfile
```

Map data (`*.simplemap`, `*.mm`, bags…) lives in `~/maps/` and is **git-ignored** — this repo is
code only.

---

## Prerequisites / Setup

### 1. MOLA — build separately (NOT vendored in this repo)

MOLA is a large upstream framework (~28 packages); don't copy it here. Build it from source once,
in a **clean environment** (conda / Isaac Sim on PATH break the ROS build), into `~/ros2_mola_ws`.
Pin release tags compatible with the apt MRPT (2.15.18): `mola 2.9.0`, `mp2p_icp 2.10.3`,
`mola_lidar_odometry 2.2.1`, `mola_imu_preintegration 1.16.1`, `mola_state_estimation 2.4.2`,
`mola_common 0.6.1`, `mola_sm_loop_closure 1.2.2`. Then:
```bash
cd ~/ros2_mola_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
(Do NOT `apt install ros-humble-mrpt-apps-gui` — it pulls MRPT 3.0 and conflicts with 2.15.18.)

Also need **Point-LIO**: CMU `autonomy_stack_go2` (contains `point_lio_unilidar`) in `~/autonomy_stack_go2`.

### 2. Clean-env launcher

```bash
cp env/mola ~/mola && chmod +x ~/mola
cp env/mola_bashrc ~/.mola_bashrc
```
`~/mola` opens a shell with only ROS Humble + the MOLA workspace (no conda/Isaac), GUI passed
through. **Run every MOLA command below inside `~/mola`.**

### 3. Build the `plio2sm` converter (into the MOLA workspace)

```bash
ln -s ~/go2_mola_pipeline/plio2sm ~/ros2_mola_ws/src/plio2sm      # keep source in this repo
~/mola
cd ~/ros2_mola_ws && colcon build --packages-select plio2sm --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# binary: ~/ros2_mola_ws/install/plio2sm/lib/plio2sm/plio2sm
```

---

# Step 0 — Record a bag from the robot

**Make a LOOP: return to the start** (needed for loop closure). For a big map, plan several
revisits / path crossings (see "Per-map tuning").

```bash
ssh unitree@192.168.123.18          # pass: <YOUR-ROBOT-PASSWORD>   # DO NOT commit the password
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh
export ROS_DOMAIN_ID=0
ros2 topic list | grep utlidar

# record in background:
mkdir -p ~/maps && cd ~/maps
nohup ros2 bag record /utlidar/cloud /utlidar/imu > ~/maps/rec.log 2>&1 &
sleep 3; cat ~/maps/rec.log; ls -la ~/maps/     # check it started

# unplug cable -> walk the dog (make loops) -> plug cable back -> stop recording:
ssh unitree@192.168.123.18 'pkill -INT -f "ros2 bag record"'

ros2 bag info ~/maps/rosbag2_*                  # verify
# download to the workstation:
scp -r unitree@192.168.123.18:~/maps/rosbag2_YYYY_MM_DD-HH_MM_SS ~/maps/
```

# Step 1 — Run Point-LIO and record its output

Three terminals in the `autonomy_stack_go2` env (each: `conda deactivate; unset PYTHONPATH;
unset LD_LIBRARY_PATH; source /opt/ros/humble/setup.bash; source install/setup.bash`):

```bash
# T1 — Point-LIO (subscribes to /utlidar/cloud + /utlidar/imu):
cd ~/autonomy_stack_go2 && ./system_real_robot.sh

# T2 — record the SLAM output (START FIRST):
cd ~/maps && ros2 bag record -o plio_out \
  /state_estimation /cloud_registered_body /registered_scan /tf /tf_static /utlidar/imu /path

# T3 — replay the raw bag through the SLAM:
cd ~/autonomy_stack_go2 && ros2 bag play ~/maps/rosbag2_YYYY_MM_DD-HH_MM_SS/rosbag2_*.db3
#   if Point-LIO can't keep up:  ... --rate 0.5
```
Verify no gaps: `ros2 bag info plio_out` — `/cloud_registered_body` and `/state_estimation` counts
≈ number of scans. Key topics: `/state_estimation` (poses), `/cloud_registered_body` (deskewed
clouds in body frame), `/utlidar/imu` (for georef).

# Step 2 — Convert to simplemap   (in `~/mola`)

```bash
~/mola
cd ~/maps
~/ros2_mola_ws/install/plio2sm/lib/plio2sm/plio2sm \
  ~/autonomy_stack_go2/plio_out ~/maps/final_map.simplemap  0.3  0.15
#   args: <plio_out_bag> <out.simplemap> <kf_dist_m> <submap_radius_m>
#   kf_dist=0.3        distance between keyframes
#   submap_radius=0.15 accumulate ±0.15 m around each keyframe (density)
#   (Z-drift is fixed later by the planar constraint in Step 3, not in the converter.)
sm-cli info ~/maps/final_map.simplemap          # must list observations: lidar AND imu
```

# Step 3 — Loop closure: planar constraint + manual loop(s)   (in `~/mola`)

**3a. Export keyframes and find the loop pair(s):**
```bash
cd ~/maps
sm-cli export-keyframes final_map.simplemap --output kfs.tum
# start=end loop: first/last timestamp:
head -1 kfs.tum        # TS_I (start)
tail -1 kfs.tum        # TS_J (end)
# for extra revisits on a bigger map, list candidates and pick the real ones:
python3 ~/go2_mola_pipeline/scripts/candidates.py kfs.tum 2.5 40
```

**3b. Make the loop-closure pipeline from the template:**
```bash
cp ~/go2_mola_pipeline/pipelines/lc_manual.template.yaml ~/maps/lc_manual.yaml
# edit ~/maps/lc_manual.yaml -> the `manual_loop_constraints:` block:
#   replace <TS_I>/<TS_J> with your start/end timestamps.
#   add one more `- timestamp_i: .. / timestamp_j: .. / sigma_xyz: 0.15 / trust_as_inlier: true`
#   per extra revisit. (Constraint is position-only -> heading at the revisit doesn't matter.)
```

**3c. Run loop closure:**
```bash
cd ~/maps
env ASSUME_PLANAR_WORLD=true PLANAR_WORLD_SIGMA_Z=0.05 PLANAR_WORLD_SIGMA_ANG=0.02 \
    PLANAR_WORLD_ANNEALING_ROUNDS=20 \
    MOLA_DESKEW_IGNORE_NO_TIMESTAMPS=true \
    MOLA_LC_SENSOR_FILTER_MIN_RANGE=0.5 MOLA_LC_SENSOR_FILTER_MAX_RANGE=30.0 \
    MAX_LC_OPTIMIZATION_ROUNDS=25 LC_OPTIMIZE_EVERY_N=1 MIN_ICP_QUALITY=0.5 \
    INPUT_ODOMETRY_NOISE_XYZ_PER_M=0.04 INPUT_ODOMETRY_NOISE_ANG_DEG_PER_M=0.04 \
  mola-sm-lc-cli -a mola::FrameToFrameLoopClosure \
    -p ~/maps/lc_manual.yaml -i final_map.simplemap -o final_map_lc.simplemap
```
- `ASSUME_PLANAR_WORLD=true` — flat-floor constraint (z≈0/roll≈0/pitch≈0), annealed; fixes Z-drift
  INSIDE the optimization (no map distortion). Doesn't need loop closures.
- `MOLA_DESKEW_IGNORE_NO_TIMESTAMPS=true` — REQUIRED: Point-LIO clouds are already deskewed, no
  per-point timestamps.
- Manual loop(s) close the XY drift (automatic detection fails on the L1).

Check: `sm-cli info final_map_lc.simplemap | grep kf_bounding_box_span` — Z span should collapse to
~0.2–0.3 m. Re-run `candidates.py` on the new keyframes to confirm each loop pair's dXY ~ 0.

# Step 4 — Dense metric map `.mm`   (in `~/mola`)

```bash
cd ~/maps
sm2mm -i final_map_lc.simplemap -o final_map_lc.mm -p ~/go2_mola_pipeline/pipelines/sm2mm_dense.yaml
# set the Z-clip band per map: (floor level differs each recording)
mm2txt final_map_lc.mm -l localmap                 # -> final_map_lc_localmap.txt
python3 ~/go2_mola_pipeline/scripts/floor_z.py final_map_lc_localmap.txt
# edit bounding_box_min[2]/max[2] in pipelines/sm2mm_dense.yaml, then re-run sm2mm.
```
`sm2mm_dense.yaml`: generator uses **lidar only** (else it trips on the IMU observation), voxel
0.05 m, range 0.4–40 m, `FilterBoundingBox` clips Z outliers to a floor..ceiling band.

# Step 5 — Georeferencing (gravity-align Z)   (in `~/mola`)

```bash
mola-sm-georeferencing -i final_map_lc.simplemap --write-into final_map_lc.mm
```
Uses the IMU accelerometer to make Z truly vertical; writes metadata into the `.mm`.

# Step 6 — View / export   (in `~/mola`)

```bash
mm-viewer ~/maps/final_map_lc.mm
mm-viewer ~/maps/final_map_lc.mm \
  -s ~/maps/final_map_lc_lc_lc_edges.3Dscene -s ~/maps/final_map_lc_lc_path_edges.3Dscene   # overlays
mm2ply -i ~/maps/final_map_lc.mm -o ~/maps/final_map_lc.ply    # PLY for CloudCompare/Blender
# mm2las (GIS), mm2txt (text), mm2grid (2D occupancy grid)
```

---

## Per-map tuning

| What | Where | How |
|---|---|---|
| loop timestamps `TS_I/TS_J` (+extra revisits) | `~/maps/lc_manual.yaml` (Step 3b) | `candidates.py` + your route knowledge |
| Z-clip band | `pipelines/sm2mm_dense.yaml` `FilterBoundingBox` | `floor_z.py` (Step 4) |
| `kf_dist`, `submap_radius` | Step 2 | denser/sparser keyframes |
| planar strength `PLANAR_WORLD_SIGMA_Z`, `ANNEALING_ROUNDS` | Step 3c | if Z not flat enough → stronger/longer |

**Bigger maps (3–5×):** Z scales for free (planar), but XY needs **several** manual loops (record
deliberate revisits & path crossings; add one `manual_loop_constraint` per revisit — position-only,
so heading at the revisit is irrelevant). Watch RAM (multi-GB simplemaps).

## Gotchas (things that cost us time)

- Build/generate **only in the clean env** (`~/mola`). conda/Isaac Sim break `rosidl`/`catkin_pkg`.
- **`MOLA_DESKEW_IGNORE_NO_TIMESTAMPS=true`** for loop closure; use the **no-deskew** `sm2mm_dense.yaml`
  for mapping — Point-LIO clouds are pre-deskewed and lack per-point time, so the stock `*imu_mls*`
  pipelines crash.
- sm2mm generator must process **`lidar` only** (`process_sensor_labels_regex`), or it trips on IMU.
- `FilterByRange` → **`output_layer_between`** (keep inside range), NOT `output_layer_outside`.
- Z-drift is fixed by `ASSUME_PLANAR_WORLD` in loop closure, NOT in the converter (an early
  detrend-in-converter attempt distorted the map and was removed).
- Automatic loop closure does not work on the L1 (ICP fails on sparse scenes) → use planar (Z) +
  manual loops (XY).
- Never `apt install ros-humble-mrpt-apps-gui` (MRPT 3.0) — conflicts with our MRPT 2.15.18 and
  breaks MOLA.
