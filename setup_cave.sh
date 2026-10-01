#!/usr/bin/env bash
# One-step setup: make ~/cave/cave complete and build it.
#
#   cd ~/cave/cave && bash setup_cave.sh
#
#  1. downloads every package (and CHAMP's nested libchamp) at the versions the
#     repo records
#  2. uses the third-party package versions from your previous working
#     workspace (~/a1_ws) where it has them, so behaviour matches your pilot runs
#  3. applies LIO-SAM's documented Ubuntu 20.04 / Noetic fix if needed
#  4. copies your A1 LIO-SAM settings into cave_evaluation/config/
#  5. installs the final experiment files and settings
#  6. builds catkin_ws and a1_ws from scratch and checks the result
# Your previous workspace is only read, never changed. Files replaced in the
# repo are backed up first. Nothing is committed or pushed.

set -uo pipefail

REPO="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel 2>/dev/null)" || {
  echo "Run this from inside your cave repo (e.g. ~/cave/cave)."; exit 1; }
cd "$REPO" || exit 1
[[ -f /opt/ros/noetic/setup.bash ]] || { echo "ROS Noetic not found in /opt/ros/noetic."; exit 1; }
BACKUP="$REPO/.setup_backup/$(date +%Y%m%d_%H%M%S)"
LOGS="$REPO/.setup_logs"; mkdir -p "$LOGS"
for ign in .setup_logs/ .setup_backup/; do
  if ! grep -q -x -F "$ign" .gitignore 2>/dev/null; then
    [[ -s .gitignore && -n "$(tail -c 1 .gitignore)" ]] && echo >> .gitignore
    echo "$ign" >> .gitignore
  fi
done
ORIGINAL_COMMIT=4a41ceb
step() { echo; echo "== $*"; }
fail() { echo; echo "STOPPED: $*"; echo "Send this whole output to Claude."; exit 1; }
is_empty_dir() { [[ -d "$1" ]] && [[ -z "$(ls -A "$1" 2>/dev/null)" ]]; }
same_text() { cmp -s <(tr -d '\r' < "$1") <(tr -d '\r' < "$2"); }
backup_file() { mkdir -p "$BACKUP/$(dirname "$2")" && cp -a "$1" "$BACKUP/$2"; }
overlay_dir() {  # copy <src>'s files over <dst>; never deletes anything in <dst>
  local src="$1" dst="$2"
  (cd "$src" && tar --exclude=.git -cf - .) | (cd "$dst" && tar -xf -)
}

echo "Repository: $REPO"

# ------------------------------------------------------------- 1. packages
step "1. Packages at the versions the repo records"
if [[ ! -f .gitmodules ]]; then
  while IFS='|' read -r path url; do
    git config -f .gitmodules "submodule.$path.path" "$path"
    git config -f .gitmodules "submodule.$path.url" "$url"
  done <<'LIST'
catkin_ws/src/gazebo_cave_world|https://github.com/LTU-RAI/gazebo_cave_world.git
a1_ws/src/LIO-SAM|https://github.com/TixiaoShan/LIO-SAM.git
a1_ws/src/champ|https://github.com/chvmp/champ.git
a1_ws/src/champ_robots|https://github.com/chvmp/robots.git
a1_ws/src/champ_teleop|https://github.com/chvmp/champ_teleop.git
a1_ws/src/unitree_ros|https://github.com/chvmp/unitree_ros.git
a1_ws/src/yocs_velocity_smoother|https://github.com/chvmp/yocs_velocity_smoother.git
LIST
fi
git submodule sync -q --recursive
# Reset every downloaded package to its recorded version first (restores any
# file a previous run changed or removed); step 2 then re-applies your versions.
git submodule update --init --recursive >"$LOGS/submodules.log" 2>&1
git submodule foreach -q --recursive 'git checkout -q -f HEAD -- . 2>/dev/null; true' >>"$LOGS/submodules.log" 2>&1
git submodule update --init --recursive --force >>"$LOGS/submodules.log" 2>&1 \
  || { tail -n 20 "$LOGS/submodules.log"; fail "could not download the packages (internet connection?)"; }
for p in catkin_ws/src/gazebo_cave_world a1_ws/src/LIO-SAM a1_ws/src/champ a1_ws/src/champ/champ/include/champ \
         a1_ws/src/champ_robots a1_ws/src/champ_teleop a1_ws/src/unitree_ros a1_ws/src/yocs_velocity_smoother; do
  is_empty_dir "$p" && fail "$p is still empty"
  echo "  OK  $p"
done

# ------------------------------- 2. versions from your previous workspace
step "2. Third-party packages from your previous working workspace"
OLD="$(readlink -f "$HOME/a1_ws" 2>/dev/null || true)"
if [[ -z "$OLD" || ! -d "$OLD/src" || "$OLD" == "$REPO/a1_ws" ]]; then
  echo "  (no separate previous workspace found; using the downloaded versions)"
else
  echo "  Previous workspace: $OLD (read only)"
  for name in LIO-SAM champ champ_robots champ_teleop unitree_ros yocs_velocity_smoother; do
    src="$OLD/src/$name"; dst="a1_ws/src/$name"
    if [[ ! -d "$src" ]] || is_empty_dir "$src"; then
      echo "  downloaded version  $name (previous copy is empty or missing)"
    elif diff -rq --exclude=.git "$src" "$dst" >/dev/null 2>&1; then
      echo "  same                $name"
    else
      n="$(diff -rq --exclude=.git "$src" "$dst" 2>/dev/null | wc -l)"
      mkdir -p "$BACKUP" && echo "$name: $n differences" >> "$BACKUP/third_party_differences.txt"
      diff -rq --exclude=.git "$src" "$dst" >> "$BACKUP/third_party_differences.txt" 2>&1
      overlay_dir "$src" "$dst" || fail "could not copy $name"
      echo "  your files added    $name ($n differences from the download; list in $BACKUP/third_party_differences.txt)"
    fi
  done
fi

[[ -f a1_ws/src/champ/champ/include/champ/utils/urdf_loader.h ]] \
  || fail "CHAMP's libchamp headers are missing (a1_ws/src/champ/champ/include/champ)"

# -------------------------------------------------- 3. LIO-SAM Noetic fix
step "3. LIO-SAM fix for Ubuntu 20.04 / Noetic (LIO-SAM issue #206)"
U=a1_ws/src/LIO-SAM/include/utility.h; CM=a1_ws/src/LIO-SAM/CMakeLists.txt
if grep -q '^#include <opencv/cv.h>' "$U"; then
  backup_file "$U" "$U"
  sed -i '/^#include <opencv\/cv.h>/d' "$U"
  grep -q '^#include <opencv2/opencv.hpp>' "$U" || \
    sed -i 's|^#include <pcl_conversions/pcl_conversions.h>.*|&\n#include <opencv2/opencv.hpp>|' "$U"
  echo "  utility.h: opencv/cv.h -> opencv2/opencv.hpp (after the PCL headers)"
else
  echo "  utility.h: already fixed"
fi
grep -q '^#include <opencv2/opencv.hpp>' "$U" || grep -q 'opencv2' "$U" || fail "could not fix $U"
if grep -q 'std=c++11' "$CM"; then
  backup_file "$CM" "$CM"; sed -i 's/-std=c++11/-std=c++14/' "$CM"; echo "  CMakeLists.txt: C++11 -> C++14"
else
  echo "  CMakeLists.txt: already C++14 or later"
fi

# ------------------------------------------- 4. A1 LIO-SAM settings file
step "4. A1 settings for LIO-SAM"
TARGET=a1_ws/src/cave_evaluation/config/lio_sam_cave_a1.yaml
mkdir -p "$(dirname "$TARGET")"
pick=""
for f in a1_ws/src/LIO-SAM/config/*.yaml; do
  [[ -f "$f" ]] && grep -q 'lio_filtered' "$f" && { pick="$f"; break; }
done
if [[ -z "$pick" ]]; then
  pick="$(find "$HOME" -xdev \( -name .git -o -name build -o -name devel -o -name .setup_backup \) -prune -o \
      -type f -path '*LIO-SAM/config/*.yaml' -print0 2>/dev/null \
    | xargs -0 -r grep -l 'lio_filtered' 2>/dev/null | xargs -r -d '\n' ls -t 2>/dev/null | head -n 1)"
fi
if [[ -n "$pick" ]]; then
  if [[ -f "$TARGET" ]] && same_text "$pick" "$TARGET"; then
    echo "  already in place ($TARGET)"
  else
    [[ -f "$TARGET" ]] && backup_file "$TARGET" "$TARGET"
    tr -d '\r' < "$pick" > "$TARGET"
    echo "  copied $pick"
    echo "      -> $TARGET"
  fi
elif [[ -f "$TARGET" ]]; then
  echo "  using existing $TARGET"
else
  fail "no LIO-SAM settings for the A1 found anywhere (a .yaml containing /a1/imu/lio_filtered)"
fi
grep -q '^lio_sam:' "$TARGET" || fail "$TARGET does not start with 'lio_sam:' - not a LIO-SAM settings file"
grep -E '^\s*(pointCloudTopic|imuTopic|lidarFrame|baselinkFrame|odometryFrame)\s*:' "$TARGET" | sed 's/^ */      /'

# ------------------------------------------------ 5. experiment files
step "5. Experiment files"
PAYLOAD="$(mktemp -d)"; trap 'rm -rf "$PAYLOAD"' EXIT
sed -n '/^__PAYLOAD_BELOW__$/,$p' "${BASH_SOURCE[0]}" | tail -n +2 | tr -d '\r' | base64 -d | tar -xz -C "$PAYLOAD" \
  || fail "this script file is damaged; download it again"
known_version() {  # 0 if <file> is your original or any version Claude gave you
  local rel="$1" sum; sum="$(tr -d '\r' < "$2" | sha256sum | cut -d' ' -f1)"
  grep -q -x -F "$rel $sum" "$PAYLOAD/.known_versions"
}
while IFS= read -r rel; do
  new="$PAYLOAD/$rel"; dst="$REPO/$rel"
  if [[ -f "$dst" ]] && cmp -s "$new" "$dst"; then :
  elif [[ -f "$dst" ]] && [[ "$rel" != */experiment.conf ]] && ! known_version "$rel" "$dst"; then
    backup_file "$new" "proposed/$rel"
    echo "  KEPT YOURS  ${rel#a1_ws/src/} (your own edit; new version in $BACKUP/proposed/$rel)"
  else
    [[ -f "$dst" ]] && backup_file "$dst" "$rel"
    mkdir -p "$(dirname "$dst")"; cp "$new" "$dst"
    echo "  installed   ${rel#a1_ws/src/}"
  fi
  case "$rel" in *.sh|*.py) chmod +x "$dst" ;; esac
done < <(cd "$PAYLOAD" && find . -type f ! -name .known_versions | sed 's|^\./||' | sort)
chmod +x a1_ws/src/*/scripts/*.py catkin_ws/src/cave_sensor_robot/scripts/*.py 2>/dev/null
# Windows line endings break Python/bash scripts.
for f in a1_ws/src/cave_*/scripts/*.py a1_ws/src/cave_*/scripts/*.sh catkin_ws/src/cave_sensor_robot/scripts/*.py; do
  [[ -f "$f" ]] && grep -q $'\r' "$f" && { backup_file "$f" "$f"; sed -i 's/\r$//' "$f"; echo "  line endings fixed: $f"; }
done
echo "  (other files already up to date)"

# ------------------------------------------------------------- 6. build
step "6. Build (from scratch, clean ROS environment)"
CLEAN_ENV=(env -i HOME="$HOME" USER="${USER:-}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8)
JOBS="$(nproc 2>/dev/null || echo 4)"; (( JOBS > 8 )) && JOBS=8
build_ws() {  # build_ws <workspace>
  local ws="$1"
  rm -rf "$REPO/$ws/build" "$REPO/$ws/devel"
  echo "  building $ws ... (log: $LOGS/build_$ws.log)"
  if "${CLEAN_ENV[@]}" bash -c "source /opt/ros/noetic/setup.bash && cd '$REPO/$ws' && catkin_make -j$JOBS" \
       >"$LOGS/build_$ws.log" 2>&1; then
    echo "  OK $ws"
  else
    echo "  ---- first errors in $ws ----"
    grep -n -E 'error|Error|CMake Error|Could not find' "$LOGS/build_$ws.log" | grep -v -E 'Werror|-Wno-error' | head -n 25
    echo "  ---- end of log ----"; tail -n 15 "$LOGS/build_$ws.log"
    fail "$ws did not build (full log: $LOGS/build_$ws.log)"
  fi
}
build_ws catkin_ws
build_ws a1_ws

# ------------------------------------------------------------- 7. check
step "7. Check"
"${CLEAN_ENV[@]}" bash -c "
  source /opt/ros/noetic/setup.bash
  source '$REPO/catkin_ws/devel/setup.bash'
  source '$REPO/a1_ws/devel/setup.bash' --extend
  ok=1
  for pkg in gazebo_cave_world cave_sensor_robot champ_config champ_base a1_config lio_sam cave_evaluation cave_exploration cave_multimodal_fusion; do
    where=\$(rospack find \$pkg 2>/dev/null) || { echo \"  MISSING \$pkg\"; ok=0; continue; }
    case \"\$where\" in '$REPO'/*) echo \"  OK \$pkg\";; *) echo \"  WRONG COPY \$pkg: \$where\"; ok=0;; esac
  done
  for exe in lio_sam_imuPreintegration lio_sam_imageProjection lio_sam_featureExtraction lio_sam_mapOptmization; do
    [[ -x '$REPO/a1_ws/devel/lib/lio_sam/'\$exe ]] || { echo \"  MISSING program \$exe\"; ok=0; }
  done
  [[ \$ok == 1 ]]
" || fail "the build finished but something is missing (see above)"

cat <<NEXT

READY. Run a 10-minute test (one terminal; close any Gazebo/roscore first):

  cd "$REPO/a1_ws/src/cave_evaluation/scripts"
  bash run_experiments.sh -t 600 -c ADAPTIVE --rviz

Results, bags and logs go to $REPO/a1_ws/experiments/.
To use these workspaces in a normal terminal too (e.g. for rosrun), replace the
catkin_ws/a1_ws lines in ~/.bashrc with:
  source $REPO/catkin_ws/devel/setup.bash
  source $REPO/a1_ws/devel/setup.bash --extend
NEXT
[[ -d "$BACKUP" ]] && echo "Backups of replaced files: $BACKUP"
exit 0
__PAYLOAD_BELOW__
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
