#!/usr/bin/env bash
# Run cave-exploration experiments from ONE terminal.
#
# For every configuration (and repeat) this script:
#   1. starts roscore and the cave world (WORLD_CMD)            terminal 1
#   2. spawns the A1 and waits for its LiDAR and IMU             terminal 2
#   3. IMU filter, Velodyne time repair and LIO-SAM              terminals 3-5
#   4. fusion, mode selector, voxel map, OctoMap, frontier
#      detector, metrics logger, rosbag, planner, follower
#      and mission manager                                       terminals 6-13
#   5. waits until the robot is back at the start (or the 40 min cap),
#      then shuts everything down and moves to the next run.
# All output goes to log files; this terminal shows one status line per minute.
#
#   ./run_experiments.sh                         # all 4 configurations once
#   ./run_experiments.sh -c ADAPTIVE             # one configuration
#   ./run_experiments.sh -c "RGB_LIDAR ADAPTIVE" -n 3 --rviz
#   Ctrl-C stops the current run cleanly and exits.
# Run it with:  bash run_experiments.sh ...   (or chmod +x it once).

set -uo pipefail
# Job control: background stages get their own process group and, unlike
# plain '&' jobs, do NOT ignore SIGINT - so roslaunch shuts down cleanly and
# the metrics logger still writes its summary when a run is stopped.
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF="$SCRIPT_DIR/../config/experiment.conf"
RVIZ=false

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
  cat <<EOF
Options:
  -c, --configs "A B"   configurations (default from experiment.conf)
  -n, --repeats N       repeats per configuration
  -t, --max-time S      mission cap in sim seconds (default 2400 = 40 min)
  -s, --speed V         max linear speed m/s (default 0.35)
      --auto-threshold  calibrate adaptive thresholds at the start of each run
      --rviz            open RViz during runs
      --no-bag          do not record a rosbag
      --config FILE     use another experiment.conf
  -h, --help
EOF
}

ARGS=("$@")
for ((i = 0; i < ${#ARGS[@]}; i++)); do
  [[ "${ARGS[$i]}" == "--config" ]] && CONF="${ARGS[$((i + 1))]}"
done
# shellcheck source=/dev/null
# tr strips Windows line endings in case the file was edited on Windows.
source <(tr -d '\r' < "$CONF") || { echo "Cannot read $CONF"; exit 1; }
[[ -f "$CONF" ]] || { echo "Cannot read $CONF"; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--configs) CONFIGS="$2"; shift 2 ;;
    -n|--repeats) REPEATS="$2"; shift 2 ;;
    -t|--max-time) MAX_RUN_SECONDS="$2"; shift 2 ;;
    -s|--speed) MAX_LINEAR_SPEED="$2"; shift 2 ;;
    --auto-threshold) THRESHOLD_MODE=auto; shift ;;
    --rviz) RVIZ=true; shift ;;
    --no-bag) RECORD_BAG=false; shift ;;
    --config) shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

# ---------------------------------------------------------------- environment
# ROS setup files reference unset variables, so relax 'set -u' while sourcing.
set +u
# Keep whatever this terminal already has (e.g. the workspace that holds the
# cave world from ~/.bashrc): only source ROS itself if nothing is sourced yet,
# then ADD the experiment workspaces on top with --extend.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
fi
for ws in "${WORKSPACES[@]}"; do
  # shellcheck source=/dev/null
  if [[ -f "$ws" ]]; then source "$ws" --extend; else echo "WARNING: workspace not found: $ws"; fi
done
set -u
for pkg in cave_evaluation cave_exploration cave_multimodal_fusion cave_sensor_robot; do
  rospack find "$pkg" >/dev/null 2>&1 || { echo "ROS package '$pkg' not found - check WORKSPACES in $CONF and run catkin_make"; exit 1; }
done
for f in "$(rospack find cave_exploration)/scripts/mission_manager.py" \
         "$(rospack find cave_multimodal_fusion)/scripts/multimodal_mode_selector_experiment.py"; do
  [[ -x "$f" ]] || { echo "Not executable: $f  (run: chmod +x '$f')"; exit 1; }
  if grep -q $'\r' "$f"; then echo "Windows line endings in $f  (run: sed -i 's/\\r\$//' '$f')"; exit 1; fi
done
# LIO_SAM_LAUNCH may use rospack, so re-read only that line now ROS is sourced
# (re-sourcing the whole file would undo the command-line options).
eval "$(tr -d '\r' < "$CONF" | grep -E '^LIO_SAM_LAUNCH=' | tail -n 1)"

if [[ -z "${WORLD_CMD// }" ]]; then
  echo "Set WORLD_CMD in $CONF to the command that opens the cave world."; exit 1
fi
# If WORLD_CMD is 'roslaunch <package> <file>', check both can be found now
# rather than failing silently in world.log.
read -r -a _world <<< "$WORLD_CMD"
if [[ "${_world[0]:-}" == "roslaunch" && -n "${_world[2]:-}" && "${_world[1]}" != *.launch ]]; then
  if ! _world_dir="$(rospack find "${_world[1]}" 2>/dev/null)"; then
    echo "Cannot find ROS package '${_world[1]}' used by WORLD_CMD."
    echo "In a terminal where '$WORLD_CMD' works, run:  rospack find ${_world[1]}"
    echo "then add <that workspace>/devel/setup.bash to WORKSPACES in $CONF,"
    echo "or start this script from that terminal."
    exit 1
  fi
  if [[ -z "$(find -L "$_world_dir" -name "${_world[2]}" -print -quit 2>/dev/null)" ]]; then
    echo "Package '${_world[1]}' ($_world_dir) has no launch file '${_world[2]}'."; exit 1
  fi
fi
if [[ ! -f "$LIO_SAM_LAUNCH" ]]; then
  echo "LIO-SAM launch file not found: '$LIO_SAM_LAUNCH' (set LIO_SAM_LAUNCH in $CONF)"; exit 1
fi
mkdir -p "$RESULTS_DIR" "$LOG_DIR" "$BAG_DIR"

PIDS=()
log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

start_bg() {  # start_bg <logfile> <command...>  (own process group)
  local logfile="$1"; shift
  bash -c "$*" >"$logfile" 2>&1 &
  PIDS+=($!)
  disown $!
}

wait_for() {  # wait_for <seconds> <description> <test command>
  local timeout="$1" what="$2"; shift 2
  local start=$SECONDS
  until eval "$*" >/dev/null 2>&1; do
    if (( SECONDS - start > timeout )); then log "TIMEOUT waiting for $what"; return 1; fi
    sleep 2
  done
  log "ready: $what ($((SECONDS - start)) s)"
}

topic_alive() { timeout 15 rostopic echo -n 1 --noarr "$1" >/dev/null 2>&1; }

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -INT -- "-$pid" 2>/dev/null
  done
  sleep 8
  for pid in "${PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -KILL -- "-$pid" 2>/dev/null
  done
  # Exact process names only, so nothing else that mentions them is touched.
  pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
  pkill -f -- "rosmaster --core" 2>/dev/null
  PIDS=()
  sleep 3
}

ABORT=false
trap 'log "Interrupted - shutting down"; ABORT=true; cleanup; exit 130' INT TERM

next_run_id() {  # first free runNN for this configuration
  local cfg_lower n
  cfg_lower="$(echo "$1" | tr '[:upper:]' '[:lower:]')"
  cond_lower="$(echo "$CONDITION" | tr '[:upper:]' '[:lower:]')"
  for n in $(seq 1 99); do
    local id; id=$(printf 'run%02d' "$n")
    [[ -e "$RESULTS_DIR/${cfg_lower}_${cond_lower}_${id}.csv" ]] || { echo "$id"; return; }
  done
}

run_one() {
  local cfg="$1" run_id="$2" dir="$3"
  cleanup
  log "=== $cfg $run_id  (logs: $dir)"

  start_bg "$dir/roscore.log" "roscore"
  wait_for 30 "roscore" "rostopic list" || return 1
  rosparam set use_sim_time true

  start_bg "$dir/world.log" "$WORLD_CMD"
  wait_for 180 "Gazebo world" "rosservice list | grep -q /gazebo/spawn_urdf_model" || return 1

  start_bg "$dir/a1.log" "roslaunch cave_sensor_robot spawn_cave_a1.launch"
  wait_for 180 "A1 LiDAR" "topic_alive /velodyne_points" || return 1
  wait_for 60 "A1 IMU" "topic_alive /a1/trunk_imu" || return 1
  sleep 15  # let CHAMP stand the robot up before LIO-SAM starts

  start_bg "$dir/localisation.log" \
    "roslaunch cave_evaluation localisation.launch lio_sam_launch:='$LIO_SAM_LAUNCH'"
  wait_for 120 "LIO-SAM odometry" "topic_alive /lio_sam/mapping/odometry" || return 1

  start_bg "$dir/pipeline.log" \
    "roslaunch cave_evaluation pipeline.launch configuration:=$cfg condition:=$CONDITION \
     run_id:=$run_id output_directory:='$RESULTS_DIR' max_run_seconds:=$MAX_RUN_SECONDS \
     max_linear_speed:=$MAX_LINEAR_SPEED threshold_mode:=$THRESHOLD_MODE rviz:=$RVIZ \
     cmd_vel_topic:=$CMD_VEL_TOPIC record_bag:=$RECORD_BAG bag_directory:='$BAG_DIR'"
  local pipeline_pid="${PIDS[-1]}"

  local csv
  csv="$RESULTS_DIR/$(echo "${cfg}_${CONDITION}_${run_id}" | tr '[:upper:]' '[:lower:]').csv"
  local wall_start=$SECONDS last_report=0
  while kill -0 "$pipeline_pid" 2>/dev/null; do
    sleep 5
    if (( SECONDS - last_report >= 60 )) && [[ -f "$csv" ]]; then
      last_report=$SECONDS
      python3 - "$csv" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
if rows:
    r = rows[-1]
    print("    sim %5.1f min | %-9s | %-13s | %6.1f m travelled | %5s m from start | goals %s | switches %s" % (
        float(r["elapsed_s"]) / 60, r["mission_phase"], r["mode"], float(r["distance_m"]),
        "%.1f" % float(r["distance_to_start"]) if r["distance_to_start"] != "nan" else "?",
        r["goals_reached"], r["mode_switches"]))
PY
    fi
    if (( SECONDS - wall_start > WALL_TIMEOUT_SECONDS )); then
      log "Wall-clock safety timeout reached; stopping run"; break
    fi
  done
  log "run finished; event log:"
  grep "MISSION:" "$dir/pipeline.log" | sed 's/^/    /' || true
  cleanup
  local summary="${csv%.csv}_summary.json"
  if [[ -f "$summary" ]]; then
    python3 - "$summary" <<'PY'
import json, sys
s = json.load(open(sys.argv[1]))
print("    RESULT: %s | %.1f min | %.0f m | goals %s | plan fails %s/%s | switches %s | returned %s" % (
    s["mission_phase"], s["duration_s"] / 60, s["distance_m"], s["goals_reached"],
    s["planner_failures"], s["planner_failures"] + s["plans_succeeded"],
    s["mode_switches"], s["returned_to_start"]))
PY
  else
    log "WARNING: no summary written ($summary) - see $dir/pipeline.log"
  fi
}

log "Bags: $RECORD_BAG ($BAG_DIR) | cmd_vel: $CMD_VEL_TOPIC"
log "Configurations: $CONFIGS | repeats: $REPEATS | cap: $((MAX_RUN_SECONDS / 60)) min sim time | speed: $MAX_LINEAR_SPEED m/s"
for ((rep = 1; rep <= REPEATS; rep++)); do
  for cfg in $CONFIGS; do
    $ABORT && break 2
    run_id="$(next_run_id "$cfg")"
    dir="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_${cfg}_${run_id}"
    mkdir -p "$dir"
    run_one "$cfg" "$run_id" "$dir" || { log "Run failed during start-up - see $dir"; cleanup; }
  done
done
log "All runs complete. Results in $RESULTS_DIR"
