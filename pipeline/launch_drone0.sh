#!/bin/bash
# ============================================================
# launch_drone0.sh -- drone 0 full pipeline (tmux)
#
# Usage:
#   ./launch_drone0.sh <world>_<run> [sec]
#   ./launch_drone0.sh 36_1           # ego mode, world 36, run 1
#   ./launch_drone0.sh 36_1 sec       # sector filter mode
#   ./launch_drone0.sh restart <N>    # restart process #N
#   ./launch_drone0.sh kill <N>       # kill process #N
#   ./launch_drone0.sh stop           # kill session + cleanup
#   ./launch_drone0.sh clean          # kill leftover processes
#   ./launch_drone0.sh status         # show process status
#
# Inside tmux control panel (window 0):
#   <number>   -> restart that process
#   k<number>  -> kill only (e.g. k3)
#   q          -> kill entire session
# ============================================================

SESSION="drone0"
export DISPLAY=:0
CONFIG_FILE="/tmp/.drone0_config"
PROC_COUNT=8

GOALS="4.0,11.0;19.0,5.5;6.0,-6.0;0.011,-7.683"

declare -A PROC_NAME
declare -A PROC_CMD

# --- Delay settings (seconds) ---
DELAY_BETWEEN=5     # 1~7번 프로세스 사이 공통 간격

# ============================================================
# Build process commands (requires WORLD, RUN, MODE)
# ============================================================

setup_commands() {
    local planner_cmd tag mode_dir

    if [ "$MODE" = "sec" ]; then
        planner_cmd="gz_sec"
        tag="sectorfilter_${WORLD}_${RUN}"
        mode_dir="sector"
    else
        planner_cmd="gz_ego"
        tag="egoplanner_${WORLD}_${RUN}"
        mode_dir="ego"
    fi

    PROC_NAME[1]="agent"
    PROC_CMD[1]="px4_agent"

    PROC_NAME[2]="uav"
    PROC_CMD[2]="cd /root/px4/PX4-Autopilot && PX4_GZ_WORLD=default_${WORLD} make px4_sitl gz_x500_lidar_3d"

    PROC_NAME[3]="lidar"
    PROC_CMD[3]="ws_gz && ws_custom && ros2 launch velodyne_3d_lidar_simulation px4_3d_lidar.launch.py world_name:=default_${WORLD} lidar_model_name:=x500_lidar_3d_0 lidar_sensor_name:=lidar_3d_v3"

    PROC_NAME[4]="odom"
    PROC_CMD[4]="gz_odom"

    PROC_NAME[5]="planner"
    PROC_CMD[5]="$planner_cmd"

    PROC_NAME[6]="keyboard"
    PROC_CMD[6]="px4_keyboard"

    PROC_NAME[7]="offboard"
    PROC_CMD[7]="px4_offboard"

    PROC_NAME[8]="bench"
    PROC_CMD[8]="until ros2 topic info /move_base_simple/goal | grep -Eq 'Subscription count: [1-9]'; do sleep 1; done; bash /root/safe_manual_goal_benchmark.sh --runs 1 --tag ${tag} --world default_${WORLD} --goals '${GOALS}' --planner-regex ego_planner_node --rearm-sec 6.0 --goal-repub-sec 0.5 --out-dir /root/share/results/${WORLD}/${mode_dir}/${WORLD}_${RUN}"
}

# ============================================================
# Functions
# ============================================================

print_proc_list() {
    echo ""
    echo "  +-----+-------------------------------+"
    echo "  |  #  |  Process                      |"
    echo "  +-----+-------------------------------+"
    for i in $(seq 1 $PROC_COUNT); do
        printf "  |  %d  |  %-28s |\n" "$i" "${PROC_NAME[$i]}"
    done
    echo "  +-----+-------------------------------+"
    echo ""
}

start_proc_in_window() {
    local num=$1
    local name="${PROC_NAME[$num]}"
    local cmd="${PROC_CMD[$num]}"
    if tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -q "^${name}$"; then
        tmux send-keys -t "$SESSION:$name" C-c "" 2>/dev/null
        sleep 1
        tmux send-keys -t "$SESSION:$name" C-c "" 2>/dev/null
        sleep 0.5
        tmux send-keys -t "$SESSION:$name" "$cmd" Enter
    else
        tmux new-window -t "$SESSION" -n "$name"
        tmux send-keys -t "$SESSION:$name" "$cmd" Enter
    fi
}

kill_proc_in_window() {
    local num=$1
    local name="${PROC_NAME[$num]}"
    if tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -q "^${name}$"; then
        tmux send-keys -t "$SESSION:$name" C-c "" 2>/dev/null
        sleep 0.5
        tmux send-keys -t "$SESSION:$name" C-c "" 2>/dev/null
        echo "  [x] $name killed"
    else
        echo "  [!] $name window not found"
    fi
}

get_proc_status() {
    local name="${PROC_NAME[$1]}"
    if tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -q "^${name}$"; then
        local pane_pid
        pane_pid=$(tmux list-panes -t "$SESSION:$name" -F '#{pane_pid}' 2>/dev/null | head -1)
        if [ -n "$pane_pid" ] && [ "$(pgrep -P "$pane_pid" 2>/dev/null | wc -l)" -gt 0 ]; then
            echo "RUN"
        else
            echo "IDLE"
        fi
    else
        echo "---"
    fi
}

show_status() {
    echo "  +-----+------------+------+-----+------------+------+"
    echo "  |  #  | Name       | Stat |  #  | Name       | Stat |"
    echo "  +-----+------------+------+-----+------------+------+"
    for i in 1 2 3 4; do
        local j=$((i + 4))
        local s1 s2
        s1=$(get_proc_status "$i")
        s2=$(get_proc_status "$j")
        printf "  |  %d  | %-10s | %-4s |  %d  | %-10s | %-4s |\n" \
            "$i" "${PROC_NAME[$i]}" "$s1" "$j" "${PROC_NAME[$j]}" "$s2"
    done
    echo "  +-----+------------+------+-----+------------+------+"
}

cleanup_processes() {
    echo "  Cleaning up residual processes..."

    pkill -f "MicroXRCEAgent" 2>/dev/null && echo "  [x] MicroXRCEAgent killed"

    pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -f "make px4_sitl" 2>/dev/null
    echo "  [x] PX4 processes killed"

    pkill -f "gz sim" 2>/dev/null
    pkill -f "ruby.*gz" 2>/dev/null
    pkill -f "gzserver" 2>/dev/null
    pkill -f "gzclient" 2>/dev/null
    pgrep -f "gz|gazebo" >/dev/null 2>&1 && {
        pkill -9 -f "gz sim" 2>/dev/null
        pkill -9 -f "gzserver" 2>/dev/null
    }
    echo "  [x] Gazebo processes killed"

    pkill -f "velodyne_3d_lidar_simulation" 2>/dev/null
    pkill -f "ego_planner" 2>/dev/null
    pkill -f "traj_server" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null
    pkill -f "static_transform_publisher" 2>/dev/null
    pkill -f "px4_odometry_remap" 2>/dev/null
    pkill -f "pose_to_aligned" 2>/dev/null
    pkill -f "safe_manual_goal_benchmark" 2>/dev/null
    pkill -f "goal_batch_runner" 2>/dev/null
    # benchmark의 python3 - (stdin heredoc) 프로세스 종료 (ros2 daemon 제외)
    pgrep -a python3 2>/dev/null | grep -v "ros2cli\|vscode\|jedi" | grep " -$" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
    pkill -f "keyboard.py" 2>/dev/null
    pkill -f "offboard.py" 2>/dev/null
    echo "  [x] ROS2 nodes killed"

    sleep 2

    local remaining
    remaining=$(pgrep -f "px4_sitl_default|gz sim|gzserver|ego_planner|MicroXRCEAgent" 2>/dev/null | wc -l)
    if [ "$remaining" -gt 0 ]; then
        echo "  [!] $remaining processes still alive, sending SIGKILL..."
        pkill -9 -f "px4_sitl_default|MicroXRCEAgent|gz sim|gzserver|ego_planner|traj_server|velodyne_3d_lidar_simulation|safe_manual_goal_benchmark|goal_batch_runner|keyboard.py|offboard.py" 2>/dev/null
        sleep 1
    fi

    echo "  [+] Cleanup done"
}

# ============================================================
# Subcommands
# ============================================================

case "${1:-}" in
    stop)
        echo "Killing drone0 session..."
        cleanup_processes
        tmux kill-session -t "$SESSION" 2>/dev/null
        exit 0
        ;;
    clean)
        cleanup_processes
        exit 0
        ;;
    kill)
        if [ -z "${2:-}" ] || [ "$2" -lt 1 ] || [ "$2" -gt $PROC_COUNT ] 2>/dev/null; then
            echo "Usage: $0 kill <1-$PROC_COUNT>"
            exit 1
        fi
        if [ -f "$CONFIG_FILE" ]; then source "$CONFIG_FILE"; setup_commands; fi
        kill_proc_in_window "$2"
        exit 0
        ;;
    restart)
        if [ -z "${2:-}" ] || [ "$2" -lt 1 ] || [ "$2" -gt $PROC_COUNT ] 2>/dev/null; then
            echo "Usage: $0 restart <1-$PROC_COUNT>"
            exit 1
        fi
        if [ ! -f "$CONFIG_FILE" ]; then
            echo "  [!] No config found. Launch first with: $0 <world>_<run> [sec]"
            exit 1
        fi
        source "$CONFIG_FILE"
        setup_commands
        echo "  [*] Restarting ${PROC_NAME[$2]}..."
        start_proc_in_window "$2"
        echo "  [+] ${PROC_NAME[$2]} restarted"
        exit 0
        ;;
    status)
        if [ -f "$CONFIG_FILE" ]; then source "$CONFIG_FILE"; setup_commands; fi
        show_status
        exit 0
        ;;
    "")
        echo "Usage: $0 <world>_<run> [sec]"
        echo ""
        echo "  $0 36_1          ego mode, world 36, run 1"
        echo "  $0 36_1 sec      sector filter mode"
        echo "  $0 12_2 sec      sector filter, world 12, run 2"
        echo ""
        echo "  $0 stop          kill session + cleanup"
        echo "  $0 restart <N>   restart process #N"
        echo "  $0 kill <N>      kill process #N"
        echo "  $0 status        show process status"
        echo "  $0 clean         kill leftover processes"
        exit 1
        ;;
esac

# ============================================================
# Parse argument: <world>_<run> [sec]
# ============================================================

ARG="${1}"
if ! [[ "$ARG" =~ ^[0-9]+_[0-9]+$ ]]; then
    echo "Error: argument must be <world>_<run> (e.g. 36_1)"
    echo "Valid worlds: 6, 12, 22, 36"
    exit 1
fi

WORLD="${ARG%_*}"
RUN="${ARG##*_}"
MODE="${2:-ego}"

if [ "$MODE" != "ego" ] && [ "$MODE" != "sec" ]; then
    echo "Error: mode must be 'ego' or 'sec' (default: ego)"
    exit 1
fi

setup_commands

# Save config for restart/kill/status commands
printf "WORLD=%s\nRUN=%s\nMODE=%s\n" "$WORLD" "$RUN" "$MODE" > "$CONFIG_FILE"

# ============================================================
# Main: launch all
# ============================================================

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists."
    echo "  Stop:   $0 stop"
    echo "  Attach: tmux attach -t $SESSION"
    exit 1
fi

echo "======================================="
echo "  drone 0 pipeline starting"
echo "  World : default_${WORLD}"
echo "  Run   : ${RUN}"
echo "  Mode  : ${MODE}"
echo "======================================="

cleanup_processes
echo ""
print_proc_list

tmux new-session -d -s "$SESSION" -n "control"

echo "[1/$PROC_COUNT] agent starting..."
tmux new-window -t "$SESSION" -n "agent"
tmux send-keys -t "$SESSION:agent" "${PROC_CMD[1]}" Enter
sleep "$DELAY_BETWEEN"

echo "[2/$PROC_COUNT] uav starting..."
tmux new-window -t "$SESSION" -n "uav"
tmux send-keys -t "$SESSION:uav" "${PROC_CMD[2]}" Enter
sleep "$DELAY_BETWEEN"

echo "[3/$PROC_COUNT] lidar starting..."
tmux new-window -t "$SESSION" -n "lidar"
tmux send-keys -t "$SESSION:lidar" "${PROC_CMD[3]}" Enter
sleep "$DELAY_BETWEEN"

echo "[4/$PROC_COUNT] odom starting..."
tmux new-window -t "$SESSION" -n "odom"
tmux send-keys -t "$SESSION:odom" "${PROC_CMD[4]}" Enter
sleep "$DELAY_BETWEEN"

echo "[5/$PROC_COUNT] planner starting (mode: ${MODE})..."
tmux new-window -t "$SESSION" -n "planner"
tmux send-keys -t "$SESSION:planner" "${PROC_CMD[5]}" Enter
sleep "$DELAY_BETWEEN"

echo "[6/$PROC_COUNT] keyboard starting..."
tmux new-window -t "$SESSION" -n "keyboard"
tmux send-keys -t "$SESSION:keyboard" "${PROC_CMD[6]}" Enter
sleep "$DELAY_BETWEEN"

echo "[7/$PROC_COUNT] offboard starting..."
tmux new-window -t "$SESSION" -n "offboard"
tmux send-keys -t "$SESSION:offboard" "${PROC_CMD[7]}" Enter
sleep 7

echo "  [*] keyboard: switching to offboard mode (o) - attempt 1..."
tmux send-keys -t "$SESSION:keyboard" "o" Enter
sleep 3

echo "  [*] keyboard: switching to offboard mode (o) - attempt 2..."
tmux send-keys -t "$SESSION:keyboard" "o" Enter
sleep 4

echo "  [*] keyboard: arming (t)..."
tmux send-keys -t "$SESSION:keyboard" "t" Enter
sleep 15

echo "[8/$PROC_COUNT] bench starting..."
tmux new-window -t "$SESSION" -n "bench"
tmux send-keys -t "$SESSION:bench" "${PROC_CMD[8]}" Enter

echo ""
echo "======================================="
echo "  All processes launched!"
echo "  World : default_${WORLD}"
echo "  Run   : ${RUN}"
echo "  Mode  : ${MODE}"
echo "======================================="

# --- Generate control panel script ---
CONTROL_SCRIPT_PATH="/tmp/.drone0_control.sh"
MAIN_SCRIPT_PATH="$(realpath "$0")"

cat > "$CONTROL_SCRIPT_PATH" << CTRL_EOF
#!/bin/bash
refresh() {
    clear
    echo "========     TMUX Commands     ========"
    echo "(ctrl + b) + <number> change the window"
    echo "(ctrl + b) + w  display all windows"
    echo "--------    General commands   --------"
    echo "  <number>   restart process"
    echo "  k<number>  kill only (e.g. k3)"
    echo "  q          kill entire session"
    echo "---------------------------------------"
    "$MAIN_SCRIPT_PATH" status
}
while true; do
    refresh
    read -rp "> " input
    case "\$input" in
        [1-9])
            "$MAIN_SCRIPT_PATH" restart "\$input"
            sleep 1
            ;;
        k[1-9])
            num="\${input:1}"
            "$MAIN_SCRIPT_PATH" kill "\$num"
            sleep 1
            ;;
        q)
            "$MAIN_SCRIPT_PATH" stop
            break
            ;;
        "") ;;
        *)
            echo "  [?] Enter 1-8, k1-k8, or q"
            sleep 1
            ;;
    esac
done
CTRL_EOF
chmod +x "$CONTROL_SCRIPT_PATH"

tmux send-keys -t "$SESSION:control" "bash $CONTROL_SCRIPT_PATH" Enter
tmux select-window -t "$SESSION:control"

# Auto-stop monitor: bench가 IDLE되면 자동으로 test stop
(
    sleep 30  # bench가 시작할 시간 확보
    while true; do
        if ! tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -q "^bench$"; then
            break
        fi
        bench_pane_pid=$(tmux list-panes -t "$SESSION:bench" -F '#{pane_pid}' 2>/dev/null | head -1)
        if [ -n "$bench_pane_pid" ] && [ "$(pgrep -P "$bench_pane_pid" 2>/dev/null | wc -l)" -eq 0 ]; then
            sleep 3  # 일시적 IDLE 오탐 방지
            if [ "$(pgrep -P "$bench_pane_pid" 2>/dev/null | wc -l)" -eq 0 ]; then
                echo "  [*] bench finished. stopping session..."
                "$MAIN_SCRIPT_PATH" stop
                break
            fi
        fi
        sleep 5
    done
) &
MONITOR_PID=$!

echo "Attaching to session..."
tmux attach -t "$SESSION"

kill "$MONITOR_PID" 2>/dev/null
