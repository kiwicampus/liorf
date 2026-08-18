
# Mapping guide

`liorf_mapping` builds 3D lidar-inertial maps offline from rosbags. It is not launched on the
robot by default (see the monorepo's `rover/configs/nodes_launch/*.sh` — there is no entry for
it there, same as `kiwi_slam_toolbox`): run it manually, following this guide.

## Pre-mapping

### 1. Download a rosbag

Any rosbag with the configured lidar (`pointCloudTopic`), IMU (`imuTopic`), and optionally GPS
(`gpsTopic`) topics works. Set:

```bash
export ROSBAG_PATH=/workspace/rosbags/mapping3d/<site>/<name>.mcap
```

### 2. Choose a params file

Start from `config/lio_sam_livox.yaml` (Livox + 9-axis IMU tuning) or one of the other
per-sensor configs in `config/`. Copy it if you need to change topics or sensor extrinsics for
your rig.

### 3. Fresh map vs. joining a prior session

- **Fresh map**: leave `loadSessionPath: ""` in your params file.
- **Join/extend a prior (possibly externally cleaned) session**: see "Join/clean workflow" below
  before launching.

## Mapping

### 1. Launch mapping

```bash
ros2 launch liorf_mapping run_lio_sam_livox.launch.py \
  use_sim_time:=true \
  params_file:=/path/to/your_params.yaml \
  use_wheel_odom:=false
```

Launch arguments:
- `params_file` — defaults to `config/lio_sam_livox.yaml`.
- `use_wheel_odom` — set to `true` to fuse wheel odometry instead of IMU-only preintegration
  (`liorf_mapping_wheelOdomPreintegration` instead of `liorf_mapping_imuPreintegration`).

This starts `liorf_mapping_imageProjection`, `liorf_mapping_mapOptmization`, the preintegration
node, a static `map`→`odom` transform, and RViz (`rviz/mapping.rviz`).

### 2. Play the rosbag (in another terminal)

```bash
ros2 bag play $ROSBAG_PATH --clock
```

TODO: whether `--remap tf:=tf2` is needed (as `kiwi_slam_toolbox`'s 2D workflow requires) has not
been verified for this package — `liorf_mapping` publishes its own `map`→`odom`→`base_link` chain,
so it may or may not conflict with `/tf` in the bag. Check for TF conflicts/warnings and remap if
needed.

### 3. GPS

If `gpsTopic` is publishing, GPS factors are added automatically once
`useGpsElevation`/`mappingGps*` params (see `config/lio_sam_livox.yaml`) are tuned for your site.
Toggle GPS usage at runtime:

```bash
ros2 service call /liorf_mapping/use_gps std_srvs/srv/SetBool "{data: true}"
```

## Saving the map

```bash
ros2 service call /liorf_mapping/save_map liorf_mapping/srv/SaveMap "{resolution: 0.2, destination: '/path/to/save'}"
```

`destination` is relative to `$HOME` if it doesn't already resolve; leave it empty to use
`savePCDDirectory` from the params file. This produces, under the destination directory:
- `GlobalMap.pcd`, `SurfMap.pcd`, `trajectory.pcd`, `transformations.pcd`
- `singlesession_posegraph.g2o`, `optimized_poses.txt` (KITTI format)
- `graph/` — the session's factor graph, in both the classic dump format (`dump()`) and
  `graph/factor_graph.yaml` (`dumpYAML()`) — the interchange format used by the join/clean
  workflow below.

## Join/clean workflow

This is the "join_clean_maps" feature ported from `kiwicampus/liorf`'s ROS1 fork: resume mapping
on top of a prior session, optionally after externally editing/cleaning its factor graph.

1. After a mapping run, save the map (above) — note the `graph/factor_graph.yaml` it produces.
2. Externally edit/clean `factor_graph.yaml` if needed (e.g. remove bad loop closures).
3. Set `loadSessionPath` in your params file to the directory containing that
   `factor_graph.yaml` (and its per-keyframe `NNNNNN/cloud.pcd` files).
4. Relaunch. On startup, `liorf_mapping_mapOptmization` loads the session and waits for a pose
   to relocalize into it:
   - Publish an initial guess via RViz's "2D Pose Estimate" (`/initialpose`), or
   - Wait for a GPS fix, if GPS is configured — it's used automatically as the initial guess.
5. The node relocalizes (parallel 8-way yaw-offset ICP search against the loaded map, since
   neither `/initialpose` nor GPS carries heading) and resumes mapping onto the existing graph.
6. Save again to produce an updated, joined map.

Debug/inspect a saved session without running the full pipeline:

```bash
ros2 run liorf_mapping liorf_mapping_load_liorf_session /path/to/session_dir
```

This republishes the session's prior/between/GPS/optimized paths and a downsampled concatenated
cloud (`liorf_mapping/prior_factors`, `.../between_factors`, `.../gps_factors`,
`.../optimized_trajectory`, `.../concatenated_cloud`) at 1 Hz for inspection in RViz.
