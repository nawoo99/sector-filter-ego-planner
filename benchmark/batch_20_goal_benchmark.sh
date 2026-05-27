#!/usr/bin/env bash
set -euo pipefail

# One-command benchmark runner for repeated goal missions.
# - Publishes 4 goals in sequence for N runs (default 20)
# - Measures planner compute load via pidstat (fallback: ps sampler)
# - Writes per-run CSV + summary CSV
#
# Assumes ROS2 graph is already running (PX4, Gazebo, planner, etc.).

RUNS=20
TAG="bench"
TAG_SET=false
GOALS="4.0,11.0;19.0,5.5;6.0,-6.0;0.011,-7.683"
GOAL_TOPIC="/move_base_simple/goal"
ODOM_TOPIC="/odometry"
PLANNER_REGEX="ego_planner_node"
SEG_TIMEOUT=0.0
GOAL_TOL=0.8
HOLD_SEC=1.0
GOAL_Z=1.0
OUT_DIR="/root/share"
RESET_CMD=""
RESET_WAIT_SEC=2.5
LATENCY_NODE_REGEX=""
COLLISION_REGEX="collision|collid|contact|crash|impact|failsafe activated"
COLLISION_DEBOUNCE_SEC=0.8
GZ_WORLD="default_6"
GZ_DRONE_MODEL="x500_lidar_3d_0"
AUTO_REARM="true"
REARM_SEC=3.0
GOAL_REPUB_SEC=1.0

usage() {
  cat <<'EOF'
Usage:
  batch_20_goal_benchmark.sh [options]

Options:
  --runs N                 Number of repeated missions (default: 20)
  --tag NAME               Experiment tag, used in output filenames
                           (if omitted, auto: egoplanner_6 / sectorfilter_22 ...)
  --goals "x1,y1;...;x4,y4"  Goal list in map frame (4 goals recommended)
  --goal-topic TOPIC       Goal topic (default: /goal_pose)
  --odom-topic TOPIC       Odom topic (default: /odometry)
  --planner-regex REGEX    pgrep regex for measured process set
                           (default: ego_planner_node)
  --seg-timeout SEC        Timeout per goal segment (default: 0 = disabled)
                           If SEC <= 0, segment timeout is not used for success/fail.
  --goal-tol M             Reach tolerance (default: 0.8)
  --hold-sec SEC           Hold time inside tolerance (default: 1.0)
  --goal-z Z               Goal z value (default: 1.0)
  --out-dir DIR            Output directory (default: /root/share)
  --reset-cmd "CMD"        Command executed before each run to reset state
                           (recommended: Gazebo world reset service)
  --reset-wait-sec SEC     Wait after reset command (default: 2.5)
  --latency-node-regex RE  Regex for /rosout logger name filter
                           (default: no filter, parse all 'time(ms)=...')
  --collision-regex RE     Regex for collision event counting from /rosout
                           (default: collision|collid|contact|crash|impact|failsafe activated)
  --collision-debounce-sec SEC
                           Min interval between collision counts (default: 0.8)
  --auto-rearm BOOL        After each reset, publish OFFBOARD+ARM repeatedly
                           (default: true)
  --rearm-sec SEC          Duration for auto OFFBOARD+ARM phase (default: 3.0)
  --goal-repub-sec SEC     Re-publish current goal every N sec while waiting
                           (default: 1.0)
  -h, --help               Show this help

Examples:
  # Baseline: plain EGO (embedded sector filter OFF)
  batch_20_goal_benchmark.sh \
    --tag ego_plain_w22 --runs 20 \
    --goals "4,0;8,2;10,-2;13,0.5" \
    --planner-regex "ego_planner_node"

  # One-process compare: EGO with embedded sector filter ON
  # (same measured process: ego_planner_node)
  batch_20_goal_benchmark.sh \
    --tag ego_embedded_sf_w22 --runs 20 \
    --goals "4,0;8,2;10,-2;13,0.5" \
    --planner-regex "ego_planner_node"

  # Deterministic reset between runs (world = default_22)
  batch_20_goal_benchmark.sh \
    --tag ego_embedded_sf_w22_reset --runs 20 \
    --goals "4,0;8,2;10,-2;13,0.5" \
    --planner-regex "ego_planner_node" \
    --reset-cmd "gz service -s /world/default_22/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'"
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runs) RUNS="$2"; shift 2 ;;
    --tag) TAG="$2"; TAG_SET=true; shift 2 ;;
    --goals) GOALS="$2"; shift 2 ;;
    --goal-topic) GOAL_TOPIC="$2"; shift 2 ;;
    --odom-topic) ODOM_TOPIC="$2"; shift 2 ;;
    --planner-regex) PLANNER_REGEX="$2"; shift 2 ;;
    --seg-timeout) SEG_TIMEOUT="$2"; shift 2 ;;
    --goal-tol) GOAL_TOL="$2"; shift 2 ;;
    --hold-sec) HOLD_SEC="$2"; shift 2 ;;
    --goal-z) GOAL_Z="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --reset-cmd) RESET_CMD="$2"; shift 2 ;;
    --reset-wait-sec) RESET_WAIT_SEC="$2"; shift 2 ;;
    --latency-node-regex) LATENCY_NODE_REGEX="$2"; shift 2 ;;
    --collision-regex) COLLISION_REGEX="$2"; shift 2 ;;
    --collision-debounce-sec) COLLISION_DEBOUNCE_SEC="$2"; shift 2 ;;
    --gz-world) GZ_WORLD="$2"; shift 2 ;;
    --gz-drone-model) GZ_DRONE_MODEL="$2"; shift 2 ;;
    --auto-rearm) AUTO_REARM="$2"; shift 2 ;;
    --rearm-sec) REARM_SEC="$2"; shift 2 ;;
    --goal-repub-sec) GOAL_REPUB_SEC="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

# Auto tag naming:
# - planner label inferred from regex (contains sector_filter_node -> sectorfilter)
# - obstacle count inferred from reset world name (/world/default_<N>/control)
if [[ "$PLANNER_REGEX" == *"sector_filter_node"* ]]; then
  PLANNER_LABEL="sectorfilter"
else
  PLANNER_LABEL="egoplanner"
fi

if [[ "$RESET_CMD" =~ /world/default_([0-9]+)/control ]]; then
  OBSTACLE_COUNT="${BASH_REMATCH[1]}"
elif [[ "$RESET_CMD" =~ /world/default/control ]]; then
  OBSTACLE_COUNT="default"
else
  OBSTACLE_COUNT="unknown"
fi

if [[ "$TAG_SET" == false && "$TAG" == "bench" ]]; then
  TAG="${PLANNER_LABEL}_${OBSTACLE_COUNT}"
fi

mkdir -p "$OUT_DIR"/{logs,results}
TS="$(date +%Y%m%d_%H%M%S)"
RUN_CSV="$OUT_DIR/results/${TAG}_runs.csv"
PID_LOG="$OUT_DIR/logs/${TAG}_pidstat.log"
PS_LOG="$OUT_DIR/logs/${TAG}_pssampler.csv"
SUMMARY_CSV="$OUT_DIR/results/summary.csv"
RUN_SUMMARY_CSV="$OUT_DIR/results/${TAG}_summary.csv"
RUN_SUMMARY_TXT="$OUT_DIR/results/${TAG}_summary.txt"

PID_LIST="$(pgrep -f "$PLANNER_REGEX" | paste -sd, - || true)"
if [[ -z "${PID_LIST:-}" ]]; then
  echo "[ERROR] No process matched --planner-regex: $PLANNER_REGEX" >&2
  exit 2
fi

echo "[INFO] tag=$TAG runs=$RUNS"
echo "[INFO] planner_label=$PLANNER_LABEL obstacle_count=$OBSTACLE_COUNT"
echo "[INFO] goals=$GOALS"
echo "[INFO] measuring pids: $PID_LIST (regex: $PLANNER_REGEX)"
echo "[INFO] run csv: $RUN_CSV"
echo "[INFO] pidstat log: $PID_LOG"
echo "[INFO] ps log: $PS_LOG"

SAMPLER_MODE=""
SAMPLER_PID=""
METRIC_LOG=""

cleanup_sampler() {
  if [[ -n "${SAMPLER_PID:-}" ]]; then
    if kill -0 "$SAMPLER_PID" >/dev/null 2>&1; then
      kill -INT "$SAMPLER_PID" >/dev/null 2>&1 || true
      for _ in $(seq 1 20); do
        if ! kill -0 "$SAMPLER_PID" >/dev/null 2>&1; then
          break
        fi
        sleep 0.1
      done
      if kill -0 "$SAMPLER_PID" >/dev/null 2>&1; then
        kill -TERM "$SAMPLER_PID" >/dev/null 2>&1 || true
        sleep 0.2
      fi
      if kill -0 "$SAMPLER_PID" >/dev/null 2>&1; then
        kill -KILL "$SAMPLER_PID" >/dev/null 2>&1 || true
      fi
    fi
    wait "$SAMPLER_PID" 2>/dev/null || true
  fi
}
trap cleanup_sampler EXIT

if command -v pidstat >/dev/null 2>&1; then
  SAMPLER_MODE="pidstat"
  METRIC_LOG="$PID_LOG"
  pidstat -h -u -r -p "$PID_LIST" 1 > "$PID_LOG" &
  SAMPLER_PID=$!
else
  SAMPLER_MODE="ps"
  METRIC_LOG="$PS_LOG"
  echo "[WARN] pidstat not found. Falling back to ps-based sampler." >&2
  echo "epoch,cpu_sum_pct,rss_sum_kb" > "$PS_LOG"
  (
    while true; do
      ts="$(date +%s.%N)"
      cur_pids="$(pgrep -f "$PLANNER_REGEX" | paste -sd, - || true)"
      if [[ -n "$cur_pids" ]]; then
        stats="$(ps -o %cpu=,rss= -p "$cur_pids" 2>/dev/null || true)"
      else
        stats=""
      fi

      if [[ -n "$stats" ]]; then
        cpu_sum="$(awk '{s+=$1} END{printf "%.6f", s+0}' <<< "$stats")"
        rss_sum="$(awk '{s+=$2} END{printf "%.0f", s+0}' <<< "$stats")"
      else
        cpu_sum="0.000000"
        rss_sum="0"
      fi
      echo "$ts,$cpu_sum,$rss_sum" >> "$PS_LOG"
      sleep 1
    done
  ) &
  SAMPLER_PID=$!
fi
sleep 1

RUNS="$RUNS" \
GOALS="$GOALS" \
GOAL_TOPIC="$GOAL_TOPIC" \
ODOM_TOPIC="$ODOM_TOPIC" \
SEG_TIMEOUT="$SEG_TIMEOUT" \
GOAL_TOL="$GOAL_TOL" \
HOLD_SEC="$HOLD_SEC" \
GOAL_Z="$GOAL_Z" \
RUN_CSV="$RUN_CSV" \
RESET_CMD="$RESET_CMD" \
RESET_WAIT_SEC="$RESET_WAIT_SEC" \
LATENCY_NODE_REGEX="$LATENCY_NODE_REGEX" \
COLLISION_REGEX="$COLLISION_REGEX" \
COLLISION_DEBOUNCE_SEC="$COLLISION_DEBOUNCE_SEC" \
GZ_WORLD="$GZ_WORLD" \
GZ_DRONE_MODEL="$GZ_DRONE_MODEL" \
AUTO_REARM="$AUTO_REARM" \
REARM_SEC="$REARM_SEC" \
GOAL_REPUB_SEC="$GOAL_REPUB_SEC" \
python3 - <<'PY'
import csv
import math
import os
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts


def parse_goals(goals_str: str):
    goals = []
    for token in goals_str.split(";"):
        token = token.strip()
        if not token:
            continue
        x_str, y_str = token.split(",")
        goals.append((float(x_str), float(y_str)))
    if len(goals) < 1:
        raise ValueError("No goals parsed from --goals")
    return goals


class MissionRunner(Node):
    def __init__(
        self,
        goal_topic: str,
        odom_topic: str,
        latency_node_regex: str,
        collision_regex: str,
        collision_debounce_sec: float,
        gz_world: str,
        gz_drone_model: str,
    ):
        super().__init__("goal_batch_runner")
        self.pub = self.create_publisher(PoseStamped, goal_topic, 10)
        # Fallback publish path used by many RViz setups; harmless if no subscriber.
        self.pub_fallback = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.kbd_pub = self.create_publisher(String, "/keyboard_cmd", 10)
        self.odom = None
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.lat_pat = re.compile(r"time\(ms\)=([0-9]*\.?[0-9]+)")
        self.lat_name_pat = re.compile(latency_node_regex) if latency_node_regex else None
        self.coll_pat = re.compile(collision_regex, re.IGNORECASE) if collision_regex else None
        self.coll_debounce_sec = max(0.0, float(collision_debounce_sec))
        self.run_active = False
        self.run_lat_ms = []
        self.run_collision_count = 0
        self.last_collision_t = -1e9
        self.crashed = False          # 고도 기반 추락 감지 플래그
        self.crash_z_threshold = 0.15 # 이 고도 이하면 추락으로 판정
        self.crash_enable_after = 0.0 # 이 시각 이후부터 크래시 감지 활성화 (이륙 유예)
        self.adaptive_recovery_count = 0  # sector → full view 전환 횟수
        self.adapt_pat = re.compile(r"Sector full-view recovery", re.IGNORECASE)
        self.rosout_sub = self.create_subscription(Log, "/rosout", self.rosout_cb, 200)

        # --- Gazebo contact sensor bridge ---
        self._bridge_proc = None
        contact_gz_topic = (
            f"/world/{gz_world}/model/{gz_drone_model}"
            f"/link/base_link/sensor/base_contact/contacts"
        )
        contact_ros_topic = "/drone_contact"
        bridge_mapping = (
            f"{contact_gz_topic}"
            f"@ros_gz_interfaces/msg/Contacts"
            f"[gz.msgs.Contacts"
        )
        try:
            self._bridge_proc = subprocess.Popen(
                ["ros2", "run", "ros_gz_bridge", "parameter_bridge", bridge_mapping],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # 브리지가 연결될 때까지 잠시 대기
            time.sleep(1.5)
            self.create_subscription(Contacts, contact_ros_topic, self.contact_cb, 50)
            self.get_logger().info(
                f"[collision] gz contact bridge started (gz_topic={contact_gz_topic})"
            )
        except Exception as e:
            self.get_logger().warn(f"[collision] gz bridge failed to start: {e}. Falling back to /rosout only.")

    def destroy_node(self):
        if self._bridge_proc and self._bridge_proc.poll() is None:
            self._bridge_proc.terminate()
            try:
                self._bridge_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._bridge_proc.kill()
        super().destroy_node()

    def contact_cb(self, msg: Contacts):
        if not self.run_active:
            return
        if len(msg.contacts) == 0:
            return
        now = time.monotonic()
        if (now - self.last_collision_t) >= self.coll_debounce_sec:
            self.run_collision_count += 1
            self.last_collision_t = now
            c = msg.contacts[0]
            self.get_logger().warn(
                f"[collision] contact detected! "
                f"bodies=({c.collision1.name} <-> {c.collision2.name}) "
                f"total_count={self.run_collision_count}"
            )

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def rosout_cb(self, msg: Log):
        if not self.run_active:
            return
        # adaptive recovery 전환 횟수 감지
        if self.adapt_pat.search(msg.msg):
            self.adaptive_recovery_count += 1
        if self.lat_name_pat and (not self.lat_name_pat.search(msg.name)):
            return
        matches = self.lat_pat.findall(msg.msg)
        if not matches:
            return
        for m in matches:
            try:
                self.run_lat_ms.append(float(m))
            except ValueError:
                pass

    def publish_goal(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        # publish a few times for reliability
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            self.pub_fallback.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_keyboard_cmd(self, cmd: str, repeats: int = 3):
        msg = String()
        msg.data = cmd
        for _ in range(max(1, repeats)):
            self.kbd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def ensure_offboard_arm(self, duration_sec: float):
        # Step 1: DISARM first to clear any crash/failsafe state
        disarm_end = time.monotonic() + 2.0
        while time.monotonic() < disarm_end:
            self.send_keyboard_cmd("DISARM", repeats=1)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Step 2: OFFBOARD + ARM for the requested duration
        end_t = time.monotonic() + max(0.5, duration_sec)
        while time.monotonic() < end_t:
            self.send_keyboard_cmd("OFFBOARD", repeats=1)
            self.send_keyboard_cmd("ARM", repeats=1)
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_reached(
        self,
        x: float,
        y: float,
        z: float,
        tol: float,
        hold_sec: float,
        timeout_sec: float,
        goal_repub_sec: float,
    ):
        t0 = time.monotonic()
        inside_since = None
        last_goal_pub = t0
        use_timeout = timeout_sec > 0.0

        while True:
            now = time.monotonic()
            if use_timeout and (now - t0 > timeout_sec):
                return False, now - t0, float("inf")

            if now - last_goal_pub >= max(0.1, goal_repub_sec):
                self.publish_goal(x, y, z)
                last_goal_pub = now

            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is None:
                continue

            px = self.odom.pose.pose.position.x
            py = self.odom.pose.pose.position.y
            d = math.hypot(px - x, py - y)

            if d <= tol:
                if inside_since is None:
                    inside_since = now
                elif now - inside_since >= hold_sec:
                    return True, now - t0, d
            else:
                inside_since = None


def main():
    runs = int(os.environ["RUNS"])
    goals = parse_goals(os.environ["GOALS"])
    goal_topic = os.environ["GOAL_TOPIC"]
    odom_topic = os.environ["ODOM_TOPIC"]
    seg_timeout = float(os.environ["SEG_TIMEOUT"])
    tol = float(os.environ["GOAL_TOL"])
    hold = float(os.environ["HOLD_SEC"])
    goal_z = float(os.environ["GOAL_Z"])
    run_csv = os.environ["RUN_CSV"]
    reset_cmd = os.environ.get("RESET_CMD", "").strip()
    reset_wait_sec = float(os.environ.get("RESET_WAIT_SEC", "2.5"))
    latency_node_regex = os.environ.get("LATENCY_NODE_REGEX", "")
    collision_regex = os.environ.get("COLLISION_REGEX", "")
    collision_debounce_sec = float(os.environ.get("COLLISION_DEBOUNCE_SEC", "0.8"))
    gz_world = os.environ.get("GZ_WORLD", "default_6")
    gz_drone_model = os.environ.get("GZ_DRONE_MODEL", "x500_lidar_3d_0")
    auto_rearm = os.environ.get("AUTO_REARM", "true").strip().lower() in ("1", "true", "yes", "y")
    rearm_sec = float(os.environ.get("REARM_SEC", "3.0"))
    goal_repub_sec = float(os.environ.get("GOAL_REPUB_SEC", "1.0"))

    rclpy.init()
    node = MissionRunner(
        goal_topic,
        odom_topic,
        latency_node_regex,
        collision_regex,
        collision_debounce_sec,
        gz_world,
        gz_drone_model,
    )

    # wait odometry: 첫 수신 + 연속 안정화 확인 (2초간 끊기지 않고 수신)
    ODOM_STABLE_SEC = 2.0   # 연속 수신 확인 시간
    ODOM_TIMEOUT_SEC = 20.0 # 전체 대기 타임아웃
    ODOM_MAX_GAP_SEC = 0.5  # 이 시간 이상 끊기면 안정화 리셋

    t_wait = time.monotonic()
    stable_since = None
    last_odom_t = None

    while time.monotonic() - t_wait < ODOM_TIMEOUT_SEC:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()

        if node.odom is not None:
            if last_odom_t is None:
                # 첫 odom 수신
                last_odom_t = now
                stable_since = now
                print("[INFO] odom first received, waiting for stability...")
            else:
                gap = now - last_odom_t
                last_odom_t = now
                if gap > ODOM_MAX_GAP_SEC:
                    # 끊겼으면 안정화 리셋
                    stable_since = now
                    print(f"[WARN] odom gap {gap:.2f}s > {ODOM_MAX_GAP_SEC}s, reset stability timer")
                elif now - stable_since >= ODOM_STABLE_SEC:
                    print(f"[INFO] odom stable for {ODOM_STABLE_SEC}s, starting mission")
                    break

    if node.odom is None or (stable_since is None) or (time.monotonic() - stable_since < ODOM_STABLE_SEC):
        raise RuntimeError("Odometry not stable before mission start")

    with open(run_csv, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow([
            "run_idx",
            "success",
            "mission_time_s",
            "failed_segment",
            "final_dist_m",
            "planning_latency_p95_ms",
            "deadline_miss_rate_pct",
            "lat_samples",
            "collision_count",
            "adaptive_recovery_count",
        ])

        for run_idx in range(1, runs + 1):
            if reset_cmd:
                print(f"[RUN {run_idx:02d}] reset_cmd ...")
                subprocess.run(reset_cmd, shell=True, check=True)

                # Let sim/estimator settle and ensure odom stream is alive after reset.
                t_settle_end = time.monotonic() + reset_wait_sec
                while time.monotonic() < t_settle_end:
                    rclpy.spin_once(node, timeout_sec=0.05)

            if auto_rearm:
                print(f"[RUN {run_idx:02d}] auto OFFBOARD+ARM for {rearm_sec:.1f}s ...")
                node.ensure_offboard_arm(rearm_sec)

            # rearm 후 odom 안정화 확인
            print(f"[RUN {run_idx:02d}] waiting for odom stability ({ODOM_STABLE_SEC}s)...")
            stable_since = None
            last_odom_t = None
            t_stab = time.monotonic()
            while time.monotonic() - t_stab < ODOM_TIMEOUT_SEC:
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if node.odom is not None:
                    if last_odom_t is None:
                        last_odom_t = now
                        stable_since = now
                    else:
                        gap = now - last_odom_t
                        last_odom_t = now
                        if gap > ODOM_MAX_GAP_SEC:
                            stable_since = now
                            print(f"[RUN {run_idx:02d}] odom gap {gap:.2f}s, reset stability")
                        elif now - stable_since >= ODOM_STABLE_SEC:
                            print(f"[RUN {run_idx:02d}] odom stable, starting")
                            break

            run_start = time.monotonic()
            success = 1
            failed_segment = 0
            final_dist = 0.0
            node.run_lat_ms = []
            node.run_collision_count = 0
            node.last_collision_t = -1e9
            node.crashed = False
            node.adaptive_recovery_count = 0
            # rearm 완료 후 2초 유예: 이륙 전 지상 상태에서 크래시 오감지 방지
            node.crash_enable_after = time.monotonic() + rearm_sec + 2.0
            node.run_active = True

            for seg_idx, (gx, gy) in enumerate(goals, start=1):
                goal_subs = node.pub.get_subscription_count()
                fallback_subs = node.pub_fallback.get_subscription_count()
                if node.odom is not None:
                    ox = node.odom.pose.pose.position.x
                    oy = node.odom.pose.pose.position.y
                    odom_info = f"odom=({ox:.2f},{oy:.2f})"
                else:
                    odom_info = "odom=(NA,NA)"

                print(
                    f"[RUN {run_idx:02d}][SEG {seg_idx:02d}] "
                    f"goal=({gx:.3f},{gy:.3f},{goal_z:.2f}) "
                    f"subs({goal_topic}={goal_subs},/move_base_simple/goal={fallback_subs}) "
                    f"{odom_info}"
                )
                if goal_subs == 0 and fallback_subs == 0:
                    print(
                        f"[RUN {run_idx:02d}][SEG {seg_idx:02d}] WARN: no goal subscribers now; "
                        "planner may not receive this segment goal."
                    )
                node.publish_goal(gx, gy, goal_z)
                ok, seg_wait_s, dist = node.wait_reached(gx, gy, goal_z, tol, hold, seg_timeout, goal_repub_sec)
                if math.isfinite(dist):
                    dist_msg = f"{dist:.3f}"
                else:
                    dist_msg = "inf(timeout)"
                print(
                    f"[RUN {run_idx:02d}][SEG {seg_idx:02d}] "
                    f"reached={1 if ok else 0} wait_s={seg_wait_s:.2f} dist={dist_msg}"
                )
                final_dist = dist if math.isfinite(dist) else final_dist
                if not ok:
                    success = 0
                    failed_segment = seg_idx
                    break

            mission_time = time.monotonic() - run_start
            node.run_active = False

            lat = sorted(node.run_lat_ms)
            if lat:
                idx = max(0, int(math.ceil(0.95 * len(lat))) - 1)
                p95_ms = lat[idx]
                miss_rate = 100.0 * sum(1 for v in lat if v > 50.0) / len(lat)
                p95_out = f"{p95_ms:.3f}"
                miss_out = f"{miss_rate:.3f}"
                n_out = len(lat)
            else:
                p95_out = "NA"
                miss_out = "NA"
                n_out = 0
            collision_out = int(node.run_collision_count)
            adaptive_out = int(node.adaptive_recovery_count)

            wr.writerow([
                run_idx,
                success,
                f"{mission_time:.3f}",
                failed_segment,
                f"{final_dist:.3f}",
                p95_out,
                miss_out,
                n_out,
                collision_out,
                adaptive_out,
            ])
            # Ensure result row is persisted immediately.
            f.flush()
            os.fsync(f.fileno())
            print(
                f"[RUN {run_idx:02d}] success={success} time={mission_time:.2f}s "
                f"failed_segment={failed_segment} lat_p95_ms={p95_out} "
                f"miss_rate={miss_out}% n={n_out} collisions={collision_out} "
                f"adaptive_recovery={adaptive_out}",
                flush=True,
            )

            # settle before next run (skip on final run for immediate exit)
            if run_idx < runs:
                end_pause = time.monotonic() + 1.0
                while time.monotonic() < end_pause:
                    rclpy.spin_once(node, timeout_sec=0.1)

    # Best-effort cleanup only; avoid blocking shutdown paths.
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
PY

cleanup_sampler
trap - EXIT

SUCCESS_RATE="$(awk -F, 'NR>1{n++; s+=$2} END{if(n>0) printf "%.2f", (100.0*s/n); else print "0.00"}' "$RUN_CSV")"
AVG_MISSION_TIME="$(awk -F, 'NR>1 && $2==1{sum+=$3; n++} END{if(n>0) printf "%.2f", (sum/n); else print "NA"}' "$RUN_CSV")"

if [[ "$SAMPLER_MODE" == "pidstat" ]]; then
  read -r AVG_CPU P95_CPU AVG_RSS_MB PEAK_RSS_MB AVG_MEM_PCT < <(
  awk '
  function isnum(x){ return (x ~ /^-?[0-9]+(\.[0-9]+)?$/) }
  BEGIN{mode=""; ncpu=0; srss=0; nrss=0; maxrss=0; smem=0; nmem=0}
  {
    if ($0 ~ /%CPU/ && $0 ~ /Command/) {
      mode="cpu";
      cpu_idx=0;
      for(i=1;i<=NF;i++) if($i=="%CPU") cpu_idx=i;
      next;
    }
    if ($0 ~ /RSS/ && $0 ~ /%MEM/ && $0 ~ /Command/) {
      mode="mem";
      rss_idx=0; mem_idx=0;
      for(i=1;i<=NF;i++) { if($i=="RSS") rss_idx=i; if($i=="%MEM") mem_idx=i; }
      next;
    }
    if ($1=="Linux" || $1=="Average:" || $0 ~ /^[[:space:]]*$/) next;
    if (mode=="cpu" && cpu_idx>0 && cpu_idx<=NF && isnum($cpu_idx)) {
      ncpu++; cpu[ncpu]=$cpu_idx+0; scpu+=cpu[ncpu];
    } else if (mode=="mem" && rss_idx>0 && rss_idx<=NF && isnum($rss_idx)) {
      rss=$rss_idx+0;
      nrss++; srss+=rss; if (rss>maxrss) maxrss=rss;
      if (mem_idx>0 && mem_idx<=NF && isnum($mem_idx)) { smem+=($mem_idx+0); nmem++; }
    }
  }
  END{
    avg_cpu = (ncpu>0)? scpu/ncpu : -1;
    if (ncpu>0) {
      for (i=1; i<=ncpu; i++) {
        for (j=i+1; j<=ncpu; j++) {
          if (cpu[i] > cpu[j]) { t=cpu[i]; cpu[i]=cpu[j]; cpu[j]=t; }
        }
      }
      idx = int(0.95*ncpu); if (idx<1) idx=1;
      p95_cpu = cpu[idx];
    } else p95_cpu=-1;
    avg_rss_mb = (nrss>0)? (srss/nrss)/1024.0 : -1;
    peak_rss_mb = (nrss>0)? maxrss/1024.0 : -1;
    avg_mem = (nmem>0)? smem/nmem : -1;
    printf "%.2f %.2f %.1f %.1f %.2f\n", avg_cpu, p95_cpu, avg_rss_mb, peak_rss_mb, avg_mem;
  }
  ' "$PID_LOG"
  )
else
  read -r AVG_CPU P95_CPU AVG_RSS_MB PEAK_RSS_MB < <(
  awk -F, '
  NR==1 { next }
  {
    c=$2+0; r=$3+0;
    n++; cpu[n]=c; sc+=c;
    nr++; sr+=r; if (r>mx) mx=r;
  }
  END{
    if (n>0) {
      for (i=1; i<=n; i++) {
        for (j=i+1; j<=n; j++) {
          if (cpu[i] > cpu[j]) { t=cpu[i]; cpu[i]=cpu[j]; cpu[j]=t; }
        }
      }
      idx = int(0.95*n); if (idx<1) idx=1;
      avg_cpu = sc/n;
      p95_cpu = cpu[idx];
    } else {
      avg_cpu = -1; p95_cpu = -1;
    }
    avg_rss_mb = (nr>0)? (sr/nr)/1024.0 : -1;
    peak_rss_mb = (nr>0)? mx/1024.0 : -1;
    printf "%.2f %.2f %.1f %.1f\n", avg_cpu, p95_cpu, avg_rss_mb, peak_rss_mb;
  }
  ' "$PS_LOG"
  )
  AVG_MEM_PCT="NA"
fi

AVG_PLAN_P95_MS="$(awk -F, 'NR>1 && $6!="NA"{sum+=$6; n++} END{if(n>0) printf "%.3f", (sum/n); else print "NA"}' "$RUN_CSV")"
AVG_DEADLINE_MISS_RATE="$(awk -F, 'NR>1 && $7!="NA"{sum+=$7; n++} END{if(n>0) printf "%.3f", (sum/n); else print "NA"}' "$RUN_CSV")"
AVG_COLLISION_COUNT="$(awk -F, 'NR>1{sum+=$9; n++} END{if(n>0) printf "%.2f", (sum/n); else print "NA"}' "$RUN_CSV")"
TOTAL_COLLISION_COUNT="$(awk -F, 'NR>1{sum+=$9} END{printf "%d", sum+0}' "$RUN_CSV")"

SUMMARY_HEADER="timestamp,tag,runs,success_rate_pct,avg_mission_time_s,avg_cpu_pct,p95_cpu_pct,avg_rss_mb,peak_rss_mb,avg_mem_pct,planning_latency_p95_ms_avg,deadline_miss_rate_pct_avg,avg_collision_count,total_collision_count,run_csv,pidstat_log,planner_regex,goals"
SUMMARY_ROW="$TS,$TAG,$RUNS,$SUCCESS_RATE,$AVG_MISSION_TIME,$AVG_CPU,$P95_CPU,$AVG_RSS_MB,$PEAK_RSS_MB,$AVG_MEM_PCT,$AVG_PLAN_P95_MS,$AVG_DEADLINE_MISS_RATE,$AVG_COLLISION_COUNT,$TOTAL_COLLISION_COUNT,$RUN_CSV,$METRIC_LOG,\"$PLANNER_REGEX\",\"$GOALS\""

if [[ ! -f "$SUMMARY_CSV" ]]; then
  echo "$SUMMARY_HEADER" > "$SUMMARY_CSV"
else
  EXISTING_HEADER="$(head -n 1 "$SUMMARY_CSV" || true)"
  if [[ "$EXISTING_HEADER" != "$SUMMARY_HEADER" ]]; then
    SUMMARY_CSV="${OUT_DIR}/results/summary_v2.csv"
    if [[ ! -f "$SUMMARY_CSV" ]]; then
      echo "$SUMMARY_HEADER" > "$SUMMARY_CSV"
    fi
    echo "[WARN] summary.csv header mismatch. Writing to: $SUMMARY_CSV"
  fi
fi
echo "$SUMMARY_ROW" >> "$SUMMARY_CSV"

# Per-experiment summary file (single row, easier to share/archive)
echo "$SUMMARY_HEADER" > "$RUN_SUMMARY_CSV"
echo "$SUMMARY_ROW" >> "$RUN_SUMMARY_CSV"

cat > "$RUN_SUMMARY_TXT" <<EOF
tag                : $TAG
runs               : $RUNS
success_rate(%)    : $SUCCESS_RATE
avg_mission_time(s): $AVG_MISSION_TIME
avg_cpu(%)         : $AVG_CPU
p95_cpu(%)         : $P95_CPU
avg_mem(%)         : $AVG_MEM_PCT
avg_rss(MB)        : $AVG_RSS_MB
peak_rss(MB)       : $PEAK_RSS_MB
planning_p95(ms)   : $AVG_PLAN_P95_MS
deadline_miss(%)   : $AVG_DEADLINE_MISS_RATE
avg_collision_cnt  : $AVG_COLLISION_COUNT
total_collisions   : $TOTAL_COLLISION_COUNT
run_csv            : $RUN_CSV
sampler_mode       : $SAMPLER_MODE
metric_log         : $METRIC_LOG
summary_csv        : $SUMMARY_CSV
EOF

echo
echo "========== Benchmark Summary =========="
echo "tag                : $TAG"
echo "runs               : $RUNS"
echo "success_rate(%)    : $SUCCESS_RATE"
echo "avg_mission_time(s): $AVG_MISSION_TIME"
echo "avg_cpu(%)         : $AVG_CPU"
echo "p95_cpu(%)         : $P95_CPU"
echo "avg_mem(%)         : $AVG_MEM_PCT"
echo "avg_rss(MB)        : $AVG_RSS_MB"
echo "peak_rss(MB)       : $PEAK_RSS_MB"
echo "planning_p95(ms)   : $AVG_PLAN_P95_MS"
echo "deadline_miss(%)   : $AVG_DEADLINE_MISS_RATE"
echo "avg_collision_cnt  : $AVG_COLLISION_COUNT"
echo "total_collisions   : $TOTAL_COLLISION_COUNT"
echo "run_csv            : $RUN_CSV"
echo "sampler_mode       : $SAMPLER_MODE"
echo "metric_log         : $METRIC_LOG"
echo "summary_csv        : $SUMMARY_CSV"
echo "run_summary_csv    : $RUN_SUMMARY_CSV"
echo "run_summary_txt    : $RUN_SUMMARY_TXT"
echo "======================================="
