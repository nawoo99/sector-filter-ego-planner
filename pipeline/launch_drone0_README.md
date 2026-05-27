# launch_drone0.sh — Drone 0 Pipeline Launcher

Single-command launcher for the drone 0 simulation pipeline. Manages 8 processes in a tmux session with a built-in control panel for monitoring and restarting individual components.

## Alias

`test` is registered as an alias for `/root/commands/launch_drone0.sh` in `~/.bashrc`.

```bash
source ~/.bashrc   # apply once per terminal
```

## Usage

### Launch

```bash
test <world>_<run> [sec]
```

| Example | Description |
|---|---|
| `test 36_1` | ego planner, world 36, run 1 |
| `test 36_1 sec` | sector filter mode, world 36, run 1 |
| `test 12_2` | ego planner, world 12, run 2 |
| `test 22_1 sec` | sector filter, world 22, run 1 |

Valid worlds: `6`, `12`, `22`, `36`

Modes: `ego` (default), `sec`

### Other Commands

```bash
test stop           # kill all processes + session
test restart <N>    # kill and restart process #N
test kill <N>       # kill process #N only
test status         # show process status table
test clean          # kill leftover processes (no tmux)
```

## Process Launch Order

```
1. agent    (MicroXRCEAgent udp4)
     └─> 2. uav     (PX4 SITL + Gazebo world)
               └─> 3. lidar   (velodyne lidar bridge)
                         └─> 4. odom     (gz_odom)
                               └─> 5. planner  (gz_ego / gz_sec)
                                     └─> 6. keyboard (px4_keyboard)
                                           └─> 7. offboard (px4_offboard)
                                                 └─> 8. bench    (benchmark)
```

## Process Table

| # | Name | Command |
|---|---|---|
| 1 | agent | `px4_agent` |
| 2 | uav | `PX4_GZ_WORLD=default_<world> make px4_sitl gz_x500_lidar_3d` |
| 3 | lidar | `ros2 launch velodyne_3d_lidar_simulation px4_3d_lidar.launch.py ...` |
| 4 | odom | `gz_odom` |
| 5 | planner | `gz_ego` or `gz_sec` |
| 6 | keyboard | `px4_keyboard` |
| 7 | offboard | `px4_offboard` |
| 8 | bench | `safe_manual_goal_benchmark.sh ...` |

## Control Panel

After launch, the tmux control window opens automatically:

```
========     TMUX Commands     ========
(ctrl + b) + <number> change the window
(ctrl + b) + w  display all windows
--------    General commands   --------
  <number>   restart process
  k<number>  kill only (e.g. k3)
  q          kill entire session
---------------------------------------
  +-----+------------+------+-----+------------+------+
  |  #  | Name       | Stat |  #  | Name       | Stat |
  +-----+------------+------+-----+------------+------+
  |  1  | agent      | RUN  |  5  | planner    | RUN  |
  |  2  | uav        | RUN  |  6  | keyboard   | RUN  |
  |  3  | lidar      | RUN  |  7  | offboard   | RUN  |
  |  4  | odom       | RUN  |  8  | bench      | RUN  |
  +-----+------------+------+-----+------------+------+
>
```

| Input | Action |
|---|---|
| `5` | Kill and restart planner |
| `k8` | Kill benchmark only |
| `q` | Kill all processes + session |
| Enter | Refresh status |

## tmux Window Navigation

| Keys | Action |
|---|---|
| `Ctrl+b` then `n` | Next window |
| `Ctrl+b` then `p` | Previous window |
| `Ctrl+b` then `w` | Window list |
| `Ctrl+b` then `0` | Back to control panel |

## Benchmark Output

Results are saved to:

```
/root/share/results/<world>/ego/<world>_<run>/    # ego mode
/root/share/results/<world>/sector/<world>_<run>/ # sector mode
```

Example:
```
/root/share/results/36/ego/36_1/
/root/share/results/36/sector/36_1/
```

## Delay Settings

Startup delays are configured at the top of the script. Adjust if processes take longer on your machine:

```bash
DELAY_AFTER_AGENT=3     # MicroXRCEAgent
DELAY_AFTER_UAV=25      # PX4 + Gazebo startup
DELAY_AFTER_LIDAR=5     # lidar bridge
DELAY_AFTER_ODOM=3      # gz_odom
DELAY_AFTER_EGO=3       # gz_ego / gz_sec
DELAY_SHORT=2           # keyboard, offboard
```

## Troubleshooting

**Gazebo window doesn't open**
Check `DISPLAY` environment variable. Script sets `export DISPLAY=:0`.

**Benchmark doesn't start**
Process #8 waits until `/move_base_simple/goal` has a subscriber. If the planner (#5) didn't start correctly, the benchmark will wait indefinitely. Restart the planner first: `test restart 5`

**Processes remain after session kill**
```bash
test clean
```

**Session already exists**
```bash
test stop
test 36_1
```

**restart/kill shows "No config found"**
Config is stored at `/tmp/.drone0_config` and is cleared on reboot. Launch fresh with `test <world>_<run>`.

## Notes

- `spawn_pillars.py` in this folder is a standalone utility for spawning random obstacles into a running Gazebo session. It is **not** used in the main pipeline — pylons are already embedded in the world SDF files (`default_6`, `default_12`, `default_22`, `default_36`).
