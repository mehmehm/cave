#!/usr/bin/env bash
# Restore the seven third-party packages that are empty in this repo, at the
# exact versions the repo recorded, and report what is installed where.
#
#   cd ~/cave/cave && bash restore_packages.sh
#
# Safe to run more than once. It ONLY fills folders that are empty or missing;
# a folder that already has files is never changed. It does not edit any
# config, does not build, and does not commit or push.

set -uo pipefail

REPO="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel 2>/dev/null)" || {
  echo "Run this from inside your cave git repo (e.g. ~/cave/cave)."; exit 1; }
cd "$REPO" || exit 1

# path | upstream URL | exact commit recorded by this repo
PACKAGES=(
  "catkin_ws/src/gazebo_cave_world|https://github.com/LTU-RAI/gazebo_cave_world.git|62ed2f33a65e39486a78aae42b5ab1ddbc954dfb"
  "a1_ws/src/LIO-SAM|https://github.com/TixiaoShan/LIO-SAM.git|0be1fbe6275fb8366d5b800af4fc8c76a885c869"
  "a1_ws/src/champ|https://github.com/chvmp/champ.git|7f7d91724dc9cae43d5c3868821f4f8b79877d1a"
  "a1_ws/src/champ_robots|https://github.com/chvmp/robots.git|83d59811d262d52c16fe8b6da8f269ae312e1c5f"
  "a1_ws/src/champ_teleop|https://github.com/chvmp/champ_teleop.git|c5983c2dba5408af76bad3fd23f60e7814ef9617"
  "a1_ws/src/unitree_ros|https://github.com/chvmp/unitree_ros.git|817e824cf5cd69c56719d4bafc48872ffb2c9361"
  "a1_ws/src/yocs_velocity_smoother|https://github.com/chvmp/yocs_velocity_smoother.git|5950b007ed5243e07a4710b22b5c11445cab0f9a"
)

is_empty_dir() { [[ ! -e "$1" ]] || { [[ -d "$1" ]] && [[ -z "$(ls -A "$1" 2>/dev/null)" ]]; }; }

echo "Repository: $REPO"
echo

# ------------------------------------------------------------ 1. .gitmodules
if [[ ! -f .gitmodules ]]; then
  echo "Writing .gitmodules (tells git where each package comes from)"
  for entry in "${PACKAGES[@]}"; do
    IFS='|' read -r path url _sha <<< "$entry"
    git config -f .gitmodules "submodule.$path.path" "$path"
    git config -f .gitmodules "submodule.$path.url" "$url"
  done
fi
git submodule sync -q

# ------------------------------------------------- 2. fill empty folders only
echo "== Packages in this repo"
FAILED=0
for entry in "${PACKAGES[@]}"; do
  IFS='|' read -r path url sha <<< "$entry"
  recorded="$(git ls-tree HEAD "$path" | awk '$2 == "commit" {print $3}')"
  if [[ -n "$recorded" && "$recorded" != "$sha" ]]; then
    echo "  SKIP     $path: repo records $recorded, expected $sha - not touching it"
    continue
  fi
  if is_empty_dir "$path"; then
    printf '  RESTORE  %-36s from %s ... ' "$path" "$url"
    if git submodule update --init --depth 1 -q -- "$path" 2>/dev/null \
       || git submodule update --init -q -- "$path" 2>/dev/null; then
      echo "done ($(git -C "$path" rev-parse --short HEAD))"
    else
      echo "FAILED (check your internet connection and run again)"; FAILED=1
    fi
  else
    if git -C "$path" rev-parse --git-dir >/dev/null 2>&1 \
       && [[ "$(git -C "$path" rev-parse --show-toplevel 2>/dev/null)" == "$REPO/$path" ]]; then
      echo "  OK       $path (has files, at $(git -C "$path" rev-parse --short HEAD))"
    else
      echo "  OK       $path (has files; left unchanged)"
    fi
  fi
done
echo

# ------------------------------------------- 3. which workspaces exist where
echo "== Workspaces"
for ws in "$HOME/catkin_ws" "$HOME/a1_ws" "$REPO/catkin_ws" "$REPO/a1_ws"; do
  if [[ -e "$ws" ]]; then
    real="$(readlink -f "$ws")"
    built="not built"; [[ -f "$real/devel/setup.bash" ]] && built="built"
    echo "  $ws -> $real ($built)"
    for pkg in gazebo_cave_world LIO-SAM champ cave_sensor_robot cave_evaluation; do
      [[ -d "$real/src/$pkg" ]] || continue
      if is_empty_dir "$real/src/$pkg"; then state="EMPTY"; else state="has files"; fi
      echo "      src/$pkg: $state"
    done
  else
    echo "  $ws: does not exist"
  fi
done
echo

# ------------------------------------- 4. find your A1 LIO-SAM configuration
echo "== LIO-SAM configuration set up for the A1"
echo "   (looking for params files that use /a1/imu/lio_filtered or velodyne_points_timed)"
mapfile -t A1_PARAMS < <(find "$HOME" -xdev \( -name .git -o -name build -o -name devel -o -name .cache -o -name .ros \) -prune \
  -o -type f -name '*.yaml' -size -500k -print 2>/dev/null \
  | xargs -r grep -l -E 'lio_filtered|velodyne_points_timed' 2>/dev/null)
if [[ ${#A1_PARAMS[@]} -eq 0 ]]; then
  echo "  NONE FOUND. LIO-SAM will use its default topics, which do not match the A1."
else
  for f in "${A1_PARAMS[@]}"; do
    echo "  $f"
    grep -E '^\s*(pointCloudTopic|imuTopic|odomTopic|lidarFrame|baselinkFrame|odometryFrame|mapFrame|sensor|N_SCAN|Horizon_SCAN)\s*:' "$f" | sed 's/^/      /'
  done
fi
lio="$REPO/a1_ws/src/LIO-SAM/config/params.yaml"
if [[ -f "$lio" ]] && ! grep -q -E 'lio_filtered|velodyne_points_timed' "$lio"; then
  echo "  NOTE: $lio is the original LIO-SAM file (not set up for the A1)."
fi
echo

if (( FAILED )); then
  echo "Some packages could not be downloaded; fix the connection and run this again."
  exit 1
fi
echo "Done. Nothing was built, committed or pushed. Send this whole output to Claude."
