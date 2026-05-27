#!/usr/bin/env bash
set -euo pipefail

# Safe semi-automatic benchmark runner.
# - Executes N missions as 1-run jobs (default 20)
# - Adds health gates before each run:
#   - planner node exists
#   - /goal_pose has subscriber > 0
# - Retries failed run automatically up to MAX_RETRY
# - Aggregates per-run results to one final CSV + summary
#
# Designed for current PX4 + Gazebo + EGO planner workflow.

RUNS=20
TAG="egoplanner_6"
WORLD="default_6"
PLANNER_REGEX="ego_planner_node"
BATCH_SCRIPT="/root/batch_20_goal_benchmark.sh"
OUT_DIR="/root/benchmark_runs"

GOALS="4.0,11.0;19.0,5.5;6.0,-6.0;0.011,-7.683"
GOAL_TOPIC="/move_base_simple/goal"
ODOM_TOPIC="/odometry"
SEG_TIMEOUT=0
GOAL_TOL=0.8
HOLD_SEC=1.0
GOAL_Z=1.0
AUTO_REARM=true
REARM_SEC=10.0
GOAL_REPUB_SEC=0.5

RESET_WAIT_SEC=4.0
SUB_WAIT_SEC=20
PLANNER_WAIT_SEC=20
MAX_RETRY=1
RETRY_ON_FAIL=false
STRICT_FLIGHT_TYPE_CHECK=false

usage() {
  cat <<EOF
Usage:
  safe_manual_goal_benchmark.sh [options]

Options:
  --runs N
  --tag NAME
  --world NAME                 (default: default_6)
  --planner-regex REGEX        (default: ego_planner_node)
  --batch-script PATH          (default: /root/batch_20_goal_benchmark.sh)
  --out-dir DIR                (default: /root/benchmark_runs)

  --goals "x1,y1;...;x4,y4"
  --goal-topic TOPIC
  --odom-topic TOPIC
  --seg-timeout SEC
  --goal-tol M
  --hold-sec SEC
  --goal-z Z
  --auto-rearm BOOL
  --rearm-sec SEC
  --goal-repub-sec SEC

  --reset-wait-sec SEC
  --sub-wait-sec SEC
  --planner-wait-sec SEC
  --max-retry N
  --retry-on-fail BOOL
  --strict-flight-type-check BOOL
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runs) RUNS="$2"; shift 2 ;;
    --tag) TAG="$2"; shift 2 ;;
    --world) WORLD="$2"; shift 2 ;;
    --planner-regex) PLANNER_REGEX="$2"; shift 2 ;;
    --batch-script) BATCH_SCRIPT="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;

    --goals) GOALS="$2"; shift 2 ;;
    --goal-topic) GOAL_TOPIC="$2"; shift 2 ;;
    --odom-topic) ODOM_TOPIC="$2"; shift 2 ;;
    --seg-timeout) SEG_TIMEOUT="$2"; shift 2 ;;
    --goal-tol) GOAL_TOL="$2"; shift 2 ;;
    --hold-sec) HOLD_SEC="$2"; shift 2 ;;
    --goal-z) GOAL_Z="$2"; shift 2 ;;
    --auto-rearm) AUTO_REARM="$2"; shift 2 ;;
    --rearm-sec) REARM_SEC="$2"; shift 2 ;;
    --goal-repub-sec) GOAL_REPUB_SEC="$2"; shift 2 ;;

    --reset-wait-sec) RESET_WAIT_SEC="$2"; shift 2 ;;
    --sub-wait-sec) SUB_WAIT_SEC="$2"; shift 2 ;;
    --planner-wait-sec) PLANNER_WAIT_SEC="$2"; shift 2 ;;
    --max-retry) MAX_RETRY="$2"; shift 2 ;;
    --retry-on-fail) RETRY_ON_FAIL="$2"; shift 2 ;;
    --strict-flight-type-check) STRICT_FLIGHT_TYPE_CHECK="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ ! -x "$BATCH_SCRIPT" ]]; then
  if [[ -f "$BATCH_SCRIPT" ]]; then
    chmod +x "$BATCH_SCRIPT"
  else
    echo "[ERROR] batch script not found: $BATCH_SCRIPT" >&2
    exit 1
  fi
fi

mkdir -p "$OUT_DIR"/{results,logs}
chmod -R 777 "$OUT_DIR"
RUN_CSV="$OUT_DIR/results/${TAG}_runs.csv"
SUMMARY_CSV="$OUT_DIR/results/${TAG}_summary.csv"
SUMMARY_TXT="$OUT_DIR/results/${TAG}_summary.txt"

echo "run_idx,attempt,success,mission_time_s,failed_segment,final_dist_m,planning_latency_p95_ms,deadline_miss_rate_pct,lat_samples,avg_cpu_pct,p95_cpu_pct,avg_rss_mb,peak_rss_mb,source_tag" > "$RUN_CSV"

world_reset() {
  gz service -s "/world/${WORLD}/control" \
    --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req 'reset: {model_only: true}'
}

get_planner_node_name() {
  # ROS2 CLI compatibility:
  # - Humble: ros2 node list -a / --all
  # - Some distros may only support plain list
  local nodes
  if nodes="$(ros2 node list -a 2>/dev/null)"; then
    true
  elif nodes="$(ros2 node list --all 2>/dev/null)"; then
    true
  else
    nodes="$(ros2 node list 2>/dev/null || true)"
  fi
  echo "$nodes" | grep -E '/.*ego_planner_node' | head -n 1 || true
}

planner_alive() {
  local n
  n="$(get_planner_node_name)"
  # Fallback: if goal topic has subscribers, planner side is at least listening.
  [[ -n "$n" ]] || goal_has_subscriber
}

wait_planner_alive() {
  local deadline now
  deadline=$(( $(date +%s) + PLANNER_WAIT_SEC ))
  while true; do
    if planner_alive; then
      return 0
    fi
    now="$(date +%s)"
    if (( now >= deadline )); then
      return 1
    fi
    sleep 1
  done
}

check_flight_type_manual() {
  local node ft
  node="$(get_planner_node_name)"
  [[ -n "$node" ]] || return 2

  ft="$(ros2 param get "$node" fsm/flight_type 2>/dev/null | grep -Eo '[-]?[0-9]+' | tail -n 1 || true)"
  [[ "$ft" == "1" ]]
}

goal_has_subscriber() {
  local info
  info="$(ros2 topic info "$GOAL_TOPIC" 2>/dev/null || true)"
  echo "$info" | grep -Eq "Subscription count: [1-9]"
}

wait_goal_subscriber() {
  local deadline now
  deadline=$(( $(date +%s) + SUB_WAIT_SEC ))
  while true; do
    if goal_has_subscriber; then
      return 0
    fi
    now="$(date +%s)"
    if (( now >= deadline )); then
      return 1
    fi
    sleep 1
  done
}

echo "[INFO] tag=$TAG runs=$RUNS world=$WORLD"
echo "[INFO] planner_regex=$PLANNER_REGEX"
echo "[INFO] final run csv: $RUN_CSV"
echo

for run_idx in $(seq 1 "$RUNS"); do
  ok_run=0
  for attempt in $(seq 1 "$MAX_RETRY"); do
    echo "[RUN $(printf "%02d" "$run_idx")][TRY $(printf "%02d" "$attempt")] reset world ..."
    if ! world_reset; then
      echo "[WARN] world reset failed (world=${WORLD})"
      sleep 1
      continue
    fi
    sleep "$RESET_WAIT_SEC"

    if ! wait_planner_alive; then
      echo "[WARN] planner node not detected within ${PLANNER_WAIT_SEC}s."
      echo "       Restart planner terminal (gz_ego) and this run will retry."
      sleep 2
      continue
    fi

    planner_node_name="$(get_planner_node_name)"
    echo "[INFO] detected planner node: ${planner_node_name}"

    if ! check_flight_type_manual; then
      rc=$?
      if [[ "$rc" == "2" ]]; then
        echo "[WARN] planner node name not resolvable via ros2 node list."
        echo "       Skip flight_type check and continue with topic-level checks."
      else
        if [[ "$STRICT_FLIGHT_TYPE_CHECK" == "true" ]]; then
          echo "[WARN] fsm/flight_type is not MANUAL(=1)."
          echo "       Current planner may be waiting for /traj_start_trigger, not /goal_pose."
          echo "       Re-run gz_ego (single_run_in_sim.launch.py) and this run will retry."
          sleep 2
          continue
        else
          echo "[WARN] fsm/flight_type != 1, but strict check is disabled."
          echo "       Proceeding if ${GOAL_TOPIC} subscriber is active."
        fi
      fi
    fi

    if ! wait_goal_subscriber; then
      echo "[WARN] ${GOAL_TOPIC} subscriber still 0 after ${SUB_WAIT_SEC}s."
      echo "       Restart planner terminal (gz_ego) and this run will retry."
      sleep 2
      continue
    fi

    run_tag="${TAG}_r$(printf "%02d" "$run_idx")_try$(printf "%02d" "$attempt")"
    echo "[RUN $(printf "%02d" "$run_idx")][TRY $(printf "%02d" "$attempt")] execute mission ..."

    if ! bash "$BATCH_SCRIPT" \
      --runs 1 \
      --tag "$run_tag" \
      --goals "$GOALS" \
      --goal-topic "$GOAL_TOPIC" \
      --odom-topic "$ODOM_TOPIC" \
      --planner-regex "$PLANNER_REGEX" \
      --seg-timeout "$SEG_TIMEOUT" \
      --goal-tol "$GOAL_TOL" \
      --hold-sec "$HOLD_SEC" \
      --goal-z "$GOAL_Z" \
      --out-dir "$OUT_DIR" \
      --reset-cmd "" \
      --gz-world "$WORLD" \
      --auto-rearm "$AUTO_REARM" \
      --rearm-sec "$REARM_SEC" \
      --goal-repub-sec "$GOAL_REPUB_SEC"; then
      echo "[WARN] batch run command failed."
      sleep 1
      continue
    fi

    src_run_csv="$OUT_DIR/results/${run_tag}_runs.csv"
    src_sum_csv="$OUT_DIR/results/${run_tag}_summary.csv"
    if [[ ! -f "$src_run_csv" || ! -f "$src_sum_csv" ]]; then
      echo "[WARN] missing per-run outputs for tag=$run_tag"
      sleep 1
      continue
    fi

    data_run="$(awk -F, 'NR==2{print $0}' "$src_run_csv")"
    data_sum="$(awk -F, 'NR==2{print $0}' "$src_sum_csv")"
    if [[ -z "$data_run" || -z "$data_sum" ]]; then
      echo "[WARN] empty per-run data for tag=$run_tag"
      sleep 1
      continue
    fi

    IFS=',' read -r _ success mission failed_seg final_dist plan_p95 miss_rate lat_n _collision <<< "$data_run"
    IFS=',' read -r _ _ _ _ _ avg_cpu p95_cpu avg_rss peak_rss _ _ _ _ _ _ _ _ <<< "$data_sum"

    echo "$run_idx,$attempt,$success,$mission,$failed_seg,$final_dist,$plan_p95,$miss_rate,$lat_n,$avg_cpu,$p95_cpu,$avg_rss,$peak_rss,$run_tag" >> "$RUN_CSV"

    if [[ "$success" == "1" ]]; then
      echo "[RUN $(printf "%02d" "$run_idx")] success on try $attempt"
      ok_run=1
      break
    fi

    if [[ "$RETRY_ON_FAIL" != "true" ]]; then
      echo "[RUN $(printf "%02d" "$run_idx")] failed and retry disabled."
      ok_run=1
      break
    fi

    echo "[RUN $(printf "%02d" "$run_idx")] failed on try $attempt (failed_segment=$failed_seg). retry ..."
    sleep 1
  done

  if [[ "$ok_run" != "1" ]]; then
    echo "[RUN $(printf "%02d" "$run_idx")] exhausted retries (${MAX_RETRY})."
  fi
  echo
done

summary_header="tag,runs_target,runs_recorded,success_count,success_rate_pct,avg_mission_time_success_s,avg_cpu_pct,p95_cpu_pct_avg,avg_rss_mb,peak_rss_mb_max,planning_latency_p95_ms_avg,deadline_miss_rate_pct_avg,world,planner_regex,goals,run_csv"

runs_recorded="$(awk -F, 'NR>1{n++} END{print n+0}' "$RUN_CSV")"
success_count="$(awk -F, 'NR>1{s+=$3} END{print s+0}' "$RUN_CSV")"
success_rate="$(awk -F, 'NR>1{n++; s+=$3} END{if(n>0) printf "%.2f", (100.0*s/n); else print "0.00"}' "$RUN_CSV")"
avg_mission_success="$(awk -F, 'NR>1 && $3==1{sum+=$4; n++} END{if(n>0) printf "%.2f", (sum/n); else print "NA"}' "$RUN_CSV")"
avg_cpu="$(awk -F, 'NR>1 && $10!="NA" && $10!=""{sum+=$10; n++} END{if(n>0) printf "%.2f", (sum/n); else print "NA"}' "$RUN_CSV")"
p95_cpu_avg="$(awk -F, 'NR>1 && $11!="NA" && $11!=""{sum+=$11; n++} END{if(n>0) printf "%.2f", (sum/n); else print "NA"}' "$RUN_CSV")"
avg_rss="$(awk -F, 'NR>1 && $12!="NA" && $12!=""{sum+=$12; n++} END{if(n>0) printf "%.2f", (sum/n); else print "NA"}' "$RUN_CSV")"
peak_rss_max="$(awk -F, 'NR>1 && $13!="NA" && $13!=""{if($13>m) m=$13; found=1} END{if(found) printf "%.2f", m; else print "NA"}' "$RUN_CSV")"
plan_p95_avg="$(awk -F, 'NR>1 && $7!="NA" && $7!=""{sum+=$7; n++} END{if(n>0) printf "%.3f", (sum/n); else print "NA"}' "$RUN_CSV")"
deadline_miss_avg="$(awk -F, 'NR>1 && $8!="NA" && $8!=""{sum+=$8; n++} END{if(n>0) printf "%.3f", (sum/n); else print "NA"}' "$RUN_CSV")"

echo "$summary_header" > "$SUMMARY_CSV"
echo "$TAG,$RUNS,$runs_recorded,$success_count,$success_rate,$avg_mission_success,$avg_cpu,$p95_cpu_avg,$avg_rss,$peak_rss_max,$plan_p95_avg,$deadline_miss_avg,$WORLD,\"$PLANNER_REGEX\",\"$GOALS\",$RUN_CSV" >> "$SUMMARY_CSV"

cat > "$SUMMARY_TXT" <<EOF
tag                     : $TAG
world                   : $WORLD
runs_target             : $RUNS
runs_recorded           : $runs_recorded
success_count           : $success_count
success_rate(%)         : $success_rate
avg_mission_time_success: $avg_mission_success
avg_cpu(%)              : $avg_cpu
p95_cpu_avg(%)          : $p95_cpu_avg
avg_rss(MB)             : $avg_rss
peak_rss_max(MB)        : $peak_rss_max
planning_p95_avg(ms)    : $plan_p95_avg
deadline_miss_avg(%)    : $deadline_miss_avg
run_csv                 : $RUN_CSV
summary_csv             : $SUMMARY_CSV
EOF

echo "========== Safe Benchmark Summary =========="
echo "tag                  : $TAG"
echo "world                : $WORLD"
echo "runs_target          : $RUNS"
echo "runs_recorded        : $runs_recorded"
echo "success_count        : $success_count"
echo "success_rate(%)      : $success_rate"
echo "avg_mission_success  : $avg_mission_success"
echo "avg_cpu(%)           : $avg_cpu"
echo "p95_cpu_avg(%)       : $p95_cpu_avg"
echo "avg_rss(MB)          : $avg_rss"
echo "peak_rss_max(MB)     : $peak_rss_max"
echo "planning_p95_avg(ms) : $plan_p95_avg"
echo "deadline_miss_avg(%) : $deadline_miss_avg"
echo "run_csv              : $RUN_CSV"
echo "summary_csv          : $SUMMARY_CSV"
echo "summary_txt          : $SUMMARY_TXT"
echo "============================================"
