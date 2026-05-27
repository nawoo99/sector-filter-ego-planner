# Adaptive Sector Filter for LiDAR-based EGO-Planner

An adaptive horizontal sector filter for [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
running on 3D LiDAR, designed to reduce planner CPU load by restricting the
occupancy-map update to the drone's forward field of view (±60°), while
automatically recovering the full 360° view when needed.

## Motivation

EGO-Planner was originally designed for depth cameras. When driven by a 360°
3D LiDAR (e.g. Livox / Velodyne), every scan feeds points from all directions
into the occupancy map, much of which is irrelevant to forward flight. This
inflates the map and increases the cost of A* search, ESDF, and B-spline
optimization.

This work adds a **sector filter** that keeps only points within a forward
horizontal cone before they are transformed and inserted into the map, plus an
**adaptive full-view recovery** mechanism so the planner does not become blind
to obstacles outside the cone.

## How it works

### 1. Sector filter (sensor frame)
Points are filtered **in the sensor frame, before the TF transform**, so the
expensive transform and raycast only run on the reduced cloud.

```
horizontal angle = atan2(p.y, p.x)
keep if  sector_min_angle ≤ angle ≤ sector_max_angle   (default ±60°)
```

Optional stride sub-sampling and voxel down-sampling reduce the cloud further.

### 2. Adaptive full-view recovery
The sector view alone can miss obstacles to the side. Full 360° view is
re-enabled temporarily on these triggers:

| Trigger | Behavior |
|---------|----------|
| **New goal received** (>0.3 m from previous) | full view for a short hold (default 2 s) |
| **Replan failures** reach a threshold | full view held until N consecutive replan successes |

Recovery uses both a time-based hold and a **frame counter** (immune to
sim-time/clock issues). The transition is counted once per actual
`sector → full view` switch (the `adaptive_recovery_count` metric).

## Repository layout

```
ego_planner_modifications/   Changes to ZJU-FAST-Lab/ego-planner-swarm
  sector_filter.patch          git diff against upstream
  modified_files/              key modified sources for reference
sector_filter_node/          Standalone ROS 2 sector_filter node (alternative)
pipeline/                    tmux launch pipeline for PX4 SITL + Gazebo
benchmark/                   Goal-mission benchmark scripts
```

### ego_planner_modifications
Core changes live in `plan_env/grid_map.{cpp,h}` (filter + adaptive recovery)
and `plan_manage/ego_replan_fsm.{cpp,h}` (goal-triggered recovery, replan-status
publishing). Apply on top of upstream:

```bash
cd ego-planner-swarm
git apply /path/to/sector_filter.patch
```

### sector_filter_node
A standalone node that performs the same sector filtering on a `PointCloud2`
topic, usable independently of the in-planner integration.

### pipeline
`launch_drone0.sh` brings up the full SITL stack (PX4 + Gazebo 3D LiDAR + odom
remap + planner + offboard control + benchmark) in a tmux session.

```bash
./launch_drone0.sh 36_1        # ego planner, world default_36, run 1
./launch_drone0.sh 36_1 sec    # sector filter mode
./launch_drone0.sh stop        # tear down
```

Valid worlds: 6, 12, 22, 36. See `pipeline/launch_drone0_README.md`.

### benchmark
`safe_manual_goal_benchmark.sh` / `batch_20_goal_benchmark.sh` publish a fixed
multi-goal mission and record per-run CPU, RSS, mission time, success, collision
count (Gazebo contact sensor), and adaptive-recovery count.

## Benchmark results

EGO baseline vs. EGO + adaptive sector filter, same LiDAR, same worlds.

**6 pylons — sparse (n = 20 each):**

| Metric | EGO | Sector Filter |
|--------|-----|---------------|
| success | 20/20 | 20/20 |
| collisions | 0 | 0 |
| mission time (s) | 45.07 ± 1.13 | 45.21 ± 1.21 |
| avg CPU (%) | 20.03 ± 2.42 | 21.67 ± 1.13 |
| adaptive recovery | – | 4.95 ± 0.80 |

In sparse scenes the per-point filtering overhead (atan2 + voxel) outweighs the
raycast savings, so CPU is comparable. Variance is lower with the filter.

**Denser worlds (preliminary, single run):**

| World | EGO CPU | Sector CPU |
|-------|---------|------------|
| 36 pylons | 44.7% | 28.3% |
| 48 pylons | 41.8% | 33.4% |

As obstacle density rises, the filter's benefit grows: fewer points to transform
and raycast outweigh the filtering cost. Both configurations remain collision-free.

> Note: 36/48 are single runs and are indicative only; multi-run statistics are
> in progress.

## Configuration (key parameters)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sector_filter_enable` | false | enable sector filtering |
| `sector_min_angle_deg` / `sector_max_angle_deg` | ±60 | horizontal cone |
| `sector_adaptive_recovery` | true | enable full-view recovery |
| `sector_full_view_hold_sec` | 2.0 | hold time per recovery |
| `sector_use_stride` / `sector_stride` | true / 2 | point sub-sampling |
| `sector_use_voxel` / `sector_voxel_leaf` | true / 0.15 | voxel down-sample |

## Acknowledgements

Built on top of [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
(ZJU-FAST-Lab). PX4 SITL + Gazebo Harmonic for simulation.
