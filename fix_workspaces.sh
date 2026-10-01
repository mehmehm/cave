#!/usr/bin/env bash
# Make your working workspaces (~/catkin_ws and ~/a1_ws) complete and up to date.
#
#   cd ~/cave/cave && bash fix_workspaces.sh
#
# Run restore_packages.sh first. This script:
#   1. fills EMPTY package folders in ~/catkin_ws/src and ~/a1_ws/src from the
#      restored copies in this repo (same versions). Folders with files are
#      never touched.
#   2. installs the final experiment files (runner, mission manager, mode
#      selector, ...) into ~/a1_ws/src and into this repo. A file you changed
#      yourself is never overwritten; anything replaced is backed up first.
#   3. finds your A1 LIO-SAM settings and points the runner at the launch file
#      that uses them (copying them back from your backup only if missing).
#   4. reports how your ~/.bashrc loads workspaces.
# It never deletes anything, and does not build, commit or push.
# Safe to run more than once.

set -uo pipefail

REPO="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel 2>/dev/null)" || {
  echo "Run this from inside your cave git repo (e.g. ~/cave/cave)."; exit 1; }
CATKIN="$(readlink -f "$HOME/catkin_ws" 2>/dev/null || true)"
A1="$(readlink -f "$HOME/a1_ws" 2>/dev/null || true)"
for d in "$CATKIN" "$A1"; do
  [[ -n "$d" && -d "$d/src" ]] || { echo "Cannot find ~/catkin_ws/src and ~/a1_ws/src."; exit 1; }
done

ORIGINAL_COMMIT=4a41ceb   # your first commit: the files before any of Claude's edits
BACKUP="$HOME/cave_fix_backup_$(date +%Y%m%d_%H%M%S)"
WARN=0

is_empty_dir() { [[ -d "$1" ]] && [[ -z "$(ls -A "$1" 2>/dev/null)" ]]; }
same_text() { cmp -s <(tr -d '\r' < "$1") <(tr -d '\r' < "$2"); }
backup_file() {  # backup_file <file> <path inside the backup folder>
  mkdir -p "$BACKUP/$(dirname "$2")" && cp -a "$1" "$BACKUP/$2"; }

echo "Repository:  $REPO"
echo "~/catkin_ws: $CATKIN"
echo "~/a1_ws:     $A1"
echo

git -C "$REPO" cat-file -e "$ORIGINAL_COMMIT^{commit}" 2>/dev/null || {
  echo "Commit $ORIGINAL_COMMIT (your first commit) is not in $REPO; cannot tell your own edits apart. Stopping."
  exit 1; }

# ------------------------------------------------ unpack the final file set
PAYLOAD_DIR="$(mktemp -d)"
trap 'rm -rf "$PAYLOAD_DIR"' EXIT
sed -n '/^__PAYLOAD_BELOW__$/,$p' "${BASH_SOURCE[0]}" | tail -n +2 | tr -d '\r' | base64 -d \
  | tar -xz -C "$PAYLOAD_DIR" || { echo "This script file is damaged. Download it again."; exit 1; }

# ------------------------------------------- 1. fill empty package folders
echo "== 1. Third-party packages in your workspaces"
PKGS=("catkin_ws|gazebo_cave_world" "a1_ws|LIO-SAM" "a1_ws|champ" "a1_ws|champ_robots"
      "a1_ws|champ_teleop" "a1_ws|unitree_ros" "a1_ws|yocs_velocity_smoother")
for entry in "${PKGS[@]}"; do
  IFS='|' read -r ws name <<< "$entry"
  src="$REPO/$ws/src/$name"
  if [[ "$ws" == catkin_ws ]]; then dst="$CATKIN/src/$name"; else dst="$A1/src/$name"; fi
  label="~/$ws/src/$name"
  if [[ "$(readlink -f "$dst" 2>/dev/null)" == "$(readlink -f "$src" 2>/dev/null)" ]]; then
    echo "  OK           $label (is the repo copy)"
  elif [[ ! -e "$dst" ]]; then
    echo "  NOT PRESENT  $label (left as is)"
  elif is_empty_dir "$dst"; then
    if is_empty_dir "$src" || [[ ! -e "$src" ]]; then
      echo "  CANNOT FILL  $label: the repo copy is empty too - run restore_packages.sh first"
      WARN=1; continue
    fi
    if (cd "$src" && tar --exclude=.git -cf - .) | (cd "$dst" && tar -xf -); then
      echo "  FILLED       $label (version $(git -C "$src" rev-parse --short HEAD 2>/dev/null))"
    else
      echo "  FAILED       $label"; WARN=1
    fi
  else
    echo "  OK           $label (has files; unchanged)"
  fi
done
echo

# ----------------------------------------------- 2. install experiment files
echo "== 2. Experiment files"
install_file() {  # install_file <workspace label> <destination file> <path in repo>
  local label="$1" dst="$2" rel="$3"
  local new="$PAYLOAD_DIR/$rel" orig="$PAYLOAD_DIR.orig"
  local has_original=0
  if git -C "$REPO" cat-file -e "$ORIGINAL_COMMIT:$rel" 2>/dev/null; then
    git -C "$REPO" show "$ORIGINAL_COMMIT:$rel" > "$orig"; has_original=1
  fi
  if [[ ! -e "$dst" ]]; then
    mkdir -p "$(dirname "$dst")" && cp "$new" "$dst"
    echo "  ADDED      $label: ${rel#a1_ws/src/}"
  elif [[ "$rel" == */experiment.conf ]]; then
    merge_conf "$label" "$dst" "$new" "$rel"
  elif same_text "$dst" "$new"; then
    if grep -q $'\r' "$dst"; then
      backup_file "$dst" "$label/$rel"; cp "$new" "$dst"
      echo "  FIXED      $label: ${rel#a1_ws/src/} (Windows line endings)"
    fi
  elif (( has_original )) && ! same_text "$dst" "$orig"; then
    backup_file "$new" "proposed/$label/$rel"
    echo "  KEPT YOURS $label: ${rel#a1_ws/src/}"
    echo "             (you edited it; the new version is in $BACKUP/proposed/$label/$rel)"
    WARN=1
  else
    backup_file "$dst" "$label/$rel"; cp "$new" "$dst"
    echo "  UPDATED    $label: ${rel#a1_ws/src/} (old copy backed up)"
  fi
  case "$rel" in *.sh|*.py) chmod +x "$dst" ;; esac
  rm -f "$orig"
}
merge_conf() {  # your settings file: keep every value you set, add only what is missing
  local label="$1" dst="$2" new="$3" rel="$4" added=() key line tmp
  tmp="$(mktemp)"; tr -d '\r' < "$dst" > "$tmp"
  while IFS= read -r line; do
    [[ "$line" =~ ^([A-Z_]+)= ]] || continue
    key="${BASH_REMATCH[1]}"
    if ! grep -q -E "^${key}=" "$tmp"; then
      printf '%s\n' "$line" >> "$tmp"; added+=("$key")
    elif [[ "$key" == WORLD_CMD ]] && grep -q -E '^WORLD_CMD=""\s*$' "$tmp"; then
      sed -i "s|^WORLD_CMD=\"\".*|${line//|/\\|}|" "$tmp"; added+=("WORLD_CMD (was empty)")
    fi
  done < "$new"
  if (( ${#added[@]} )) || ! cmp -s "$tmp" "$dst"; then
    backup_file "$dst" "$label/$rel"; cat "$tmp" > "$dst"
    if (( ${#added[@]} )); then
      echo "  MERGED     $label: ${rel#a1_ws/src/} (kept your values; added ${added[*]})"
    else
      echo "  FIXED      $label: ${rel#a1_ws/src/} (Windows line endings)"
    fi
  fi
  rm -f "$tmp"
}
mapfile -t FILES < <(cd "$PAYLOAD_DIR" && find . -type f | sed 's|^\./||' | sort)
for rel in "${FILES[@]}"; do
  install_file "workspace" "$A1/${rel#a1_ws/}" "$rel"
  if [[ "$(readlink -f "$A1")" != "$(readlink -f "$REPO/a1_ws")" ]]; then
    install_file "repo" "$REPO/$rel" "$rel"
  fi
done
echo "  (files not listed were already up to date)"
echo

# ------------------------------------------------- 3. LIO-SAM A1 settings
echo "== 3. LIO-SAM settings for the A1"
LIO="$A1/src/LIO-SAM"
LIO_LAUNCH_LINE=""
find_a1_yaml() { grep -l -E 'lio_filtered' "$1"/config/*.yaml 2>/dev/null | head -n 1; }
A1_YAML="$(find_a1_yaml "$LIO")"
if [[ -z "$A1_YAML" ]]; then
  # Not in the live LIO-SAM: look for the newest copy elsewhere under $HOME.
  BACKUP_YAML="$(find "$HOME" -xdev \( -name .git -o -name build -o -name devel \) -prune -o \
      -type f -path '*LIO-SAM/config/*.yaml' -print0 2>/dev/null \
    | xargs -0 -r grep -l -E 'lio_filtered' 2>/dev/null \
    | grep -v -F "$LIO/" | xargs -r -d '\n' ls -t 2>/dev/null | head -n 1)"
  if [[ -n "$BACKUP_YAML" ]]; then
    A1_YAML="$LIO/config/$(basename "$BACKUP_YAML")"
    if [[ -e "$A1_YAML" ]]; then
      echo "  $A1_YAML exists but is not set up for the A1; not overwriting it."
      echo "  Your A1 settings are in: $BACKUP_YAML"
      A1_YAML=""; WARN=1
    else
      cp "$BACKUP_YAML" "$A1_YAML"
      echo "  COPIED     your A1 settings from $BACKUP_YAML"
      echo "             into $A1_YAML"
      # Bring back launch files from the same backup that use them (never overwrite).
      for bl in "$(dirname "$(dirname "$BACKUP_YAML")")"/launch/*.launch; do
        [[ -f "$bl" ]] && grep -q -F "$(basename "$A1_YAML")" "$bl" || continue
        if [[ ! -e "$LIO/launch/$(basename "$bl")" ]]; then
          cp "$bl" "$LIO/launch/"; echo "  COPIED     launch file $(basename "$bl") from your backup"
        fi
      done
    fi
  fi
fi
if [[ -z "$A1_YAML" ]]; then
  echo "  NOT FOUND: no LIO-SAM settings for the A1 anywhere. Send this output to Claude."
  WARN=1
else
  echo "  Settings:  $A1_YAML"
  grep -E '^\s*(pointCloudTopic|imuTopic|lidarFrame|baselinkFrame|odometryFrame)\s*:' "$A1_YAML" | sed 's/^ */             /'
  name="$(basename "$A1_YAML")"
  USES=""
  # A top-level LIO-SAM launch that loads the file directly, or includes a
  # file from launch/include/ that does. Prefer run.launch.
  loaders=("config/$name")
  for inc in "$LIO"/launch/include/*.launch; do
    [[ -f "$inc" ]] && grep -q -F "config/$name" "$inc" && loaders+=("include/$(basename "$inc")")
  done
  for l in "$LIO"/launch/*.launch; do
    [[ -f "$l" ]] || continue
    for pat in "${loaders[@]}"; do
      if grep -q -F "$pat" "$l"; then USES="$l"; break; fi
    done
    [[ -n "$USES" && "$(basename "$USES")" == run.launch ]] && break
  done
  if [[ -n "$USES" ]]; then
    echo "  Launch:    $USES (loads these settings)"
    LIO_LAUNCH_LINE="LIO_SAM_LAUNCH=\"\$(rospack find lio_sam 2>/dev/null)/launch/$(basename "$USES")\""
  else
    echo "  No LIO-SAM launch file loads $name; using cave_evaluation/launch/lio_sam_a1.launch,"
    echo "  which loads it and starts the four LIO-SAM nodes."
    LIO_LAUNCH_LINE="LIO_SAM_LAUNCH=\"\$(rospack find cave_evaluation 2>/dev/null)/launch/lio_sam_a1.launch\""
    for f in "$A1/src/cave_evaluation/launch/lio_sam_a1.launch" "$REPO/a1_ws/src/cave_evaluation/launch/lio_sam_a1.launch"; do
      mkdir -p "$(dirname "$f")"
      cat > "$f" <<LAUNCH
<launch>
  <!-- LIO-SAM with the A1 settings ($name). Written by fix_workspaces.sh. -->
  <rosparam file="\$(find lio_sam)/config/$name" command="load"/>
  <include file="\$(find lio_sam)/launch/include/module_loam.launch"/>
</launch>
LAUNCH
    done
  fi
  for conf in "$A1/src/cave_evaluation/config/experiment.conf" "$REPO/a1_ws/src/cave_evaluation/config/experiment.conf"; do
    [[ -f "$conf" ]] || continue
    if ! grep -q -x -F "$LIO_LAUNCH_LINE" <(tr -d '\r' < "$conf"); then
      if [[ "$conf" == "$A1"/* ]]; then backup_file "$conf" "workspace/lio_conf/experiment.conf"
      else backup_file "$conf" "repo/lio_conf/experiment.conf"; fi
      tmp="$(mktemp)"
      tr -d '\r' < "$conf" | awk -v line="$LIO_LAUNCH_LINE" '/^LIO_SAM_LAUNCH=/{print line; next} {print}' > "$tmp" && cat "$tmp" > "$conf"
      rm -f "$tmp"
      echo "  SET        LIO_SAM_LAUNCH in $conf"
    fi
  done
fi
echo

# ----------------------------------------------------------- 4. ~/.bashrc
echo "== 4. Workspaces loaded by ~/.bashrc"
if grep -n -E 'setup\.(bash|sh)' "$HOME/.bashrc" 2>/dev/null | grep -v '^\s*#' | sed 's/^/  /'; then
  if grep -E 'setup\.(bash|sh)' "$HOME/.bashrc" | grep -v '^\s*#' | grep -q -F "cave/cave"; then
    echo "  WARNING: ~/.bashrc loads the repo copy in ~/cave/cave as well as ~/a1_ws."
    echo "  Both contain the same packages; remove the ~/cave/cave lines so only one copy is used."
    WARN=1
  fi
else
  echo "  (none)"
fi
echo

[[ -d "$BACKUP" ]] && echo "Backups of everything replaced: $BACKUP" && echo
cat <<'NEXT'
== Next: build and run (in a NEW terminal)
  cd ~/catkin_ws && catkin_make
  cd ~/a1_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash && source ~/a1_ws/devel/setup.bash --extend
  rospack find gazebo_cave_world && rospack find cave_evaluation && rospack find lio_sam
  cd ~/a1_ws/src/cave_evaluation/scripts && bash run_experiments.sh -t 600 -c ADAPTIVE --rviz
NEXT
(( WARN )) && echo && echo "Some items above need attention - send this whole output to Claude."
exit 0
__PAYLOAD_BELOW__
H4sIAAAAAAACA+xce1PbSLbP3/oUvWRqsTP4jSFDhlQRcBJ2AXMxSXZubZUsrLatQZa8ahlwKjWf
fX/ndEuWZDvv172FqzB2P06fPu9z1O2pc2eP/PDK8e2xdFwZPfj6rzpeOzs7/B+v4n90bj9otJvt
3e1GY7e1jfbGdrvZejB68B1eMxU7EZaMwjB+37gP9Rc393/k1W6KQTiZyCDeH+7utH4DM9rtRn27
0arLodNsN+vthhw0GgN32Nxt7rQf1+vWg/vX/5uX07BvVe3brkH6sLvbXq//+FzQ/2aj3njQvtf/
78R/FQ1qPxf/d3br9/z/rvwfODfSljeOP3NiLwxqP5T/O/VW+57/P5T/F52Do9NOdeJ+pfhvew3/
G4327hL/d1tN9N/z/5u/HoqLWRB4wUjIu6mMPIoElRhG4USEgRSxjCZe4PiW1Y9mgZ0ZU1Xjvojk
1HcGUol5OItEo5WOV1XxPIyEdAZjxJfB0BvNIhYr4cV7lvVO9GI5Fe/Em7ETo0kQE7AuGhy8W+8q
lUr6h9EN9EShGoSRFL+KPj76ziwA7JHzVl6FNsvubRj5rlh8rOoxfcxN0CJAANdEUwYIT1EyUGFk
R+FVCHSmzm2goTqNBM6WiMcyELeORxTC7vq1G+mH7jyQ9jT0QJO+cAIXzU6jFoNc17Y3mem2Rlvo
OQAhDhoiDmnL6Mjg1mTcWmg6Pn0lhp6Pni3x2iwhYtCdCO54kSj11cAJbGJG6Ip6tdEvb4mT426l
d3D6JIeh74W2cia1iTOdgsu10A0nMo7mWaoo0aq0efFttA5nCnzaEpPQlUJJXw7iEHjchHfSFwCz
JbpoOaUPEJMg9mQkXBmbYQTcGyjhh6MRoQ8qXzmjLQE5CQJqGIa+H95K2gPRaTBxbVCxX2YyTTxF
i2OZwMH0HI47FcjXO8s6jrN8mAEBn6mqOecpceUMrgWLldJyJUqG8Nv1CoDNYgmWT8tblgKDFfcc
9l5vCTWbTJxozpgw0mo8Awx5I6N5PCYdccPbgLuNvNLUQN7FAtyuWtbDh6IbyApzSsl4NrWsS4wA
K7HMBJZGXEnhBTwNInoNMRtI0h3hDOKZ4/tzcTXzILnieMjN/M3q/1VjM93H91hcS2gOQRhBbyAP
IUHEEJJWfoOgDsLpnMdI14ulazCAiIYpMLb51Wq1z9Rh+RX9N92Lf/bODw47vT5D1apbW6h9lVr6
RF1At/J7YGTLIEO/379y1NgauCKzmPVQnID4d8L3IM0ycEFQBW2Wd3Iwi50rX4orYlkpkNIFzt7Q
oD1wQM4bzxFvvAAMUGVLobviiU1V+3f0S622KYq+Sw0ibxqr2rLNWhq6eovi35YovvTMu6kfRvlV
jNTaRmqr07k1GEN9xK93n47ZR8JPSSv+/nfMjK896r+W9FXBGIMnLgTXr7EcVpkf4ItlHY4l1OPw
5cHp+Sb0YwpigyMTmGUWmFQ9HXWtbchsStaqXm21xaSmWPrr1d9E5Lg1VbUgqH3gYaho/o2gmvxW
nTsTv0+6oER/4tzZxHsnIo0PB148t+/6Iows7nKC0czP9r3tC0ZlS2MrBr431SpHFQvg8QRIeEpS
E7wVhNhRWKai+tatF4+Fo6frPUIsD/0Qg51gLl6w16ApiVMh+R2D/iKcwrQMvUjFT3gpzQPoe6jX
nmg1N05znazX1rDdorFiBe8rsdip10VlIA6ODs4vj193SOIeikZqsWIJ88GIrQWy+gW1Oz46uLC7
Zyd/bImLF89s/r4lLl92Lk4PTpKv6cJhAOEhz70e2wBuasVCLbJH0gGzw6E2m3nnvx5gZt+VSnTj
vdUAb50Y/hl2Dsbo4rX3dj2AShBWYLOLGKlrb2p8EHADq8nqaEUgy5z63mlE/ptDHpj2eKa0lXKE
oT5JvQMIaubHugvLa6v/EOLwhvAkB5HgafX1NvosUIobBUtln5qz5kYPKw29OzLUERm7Pnlt8tT9
LYusX+QYvaQldARRho+AWI5hD03LlpZXHwtWlDPE5iIIYaTYspLftqh/gg14sE3Yc8ajU4/vQfzh
/IGFjmbAOx+WBN+vtDOBk6IxFBhYJQiSGaB4fjQByH90LheNo0jOYfIhXJUw8OflRbygZ4jBLIpA
AWsUYiq8ECwAFomi8PYJ8fzP2YR0LtT7Ykeu3dQtuf+Fy6e6tRJjxDVlvUcda2AbTjzWK5nQiEjy
Jwcq8y1mKfXRRsxn3oMmuOVNYGsRxjLnNBNp1ICMiNKs5OhWe/7LUFsOoA1ijWkkwhiHejm4nnpT
SWKzt7AYkEr0ChZ2fqu44pcSWqcUvwy9wC06j3Jtlexoae7O4imCFcB/JFIXn1GSmpZdVftda+RT
e0ICZwOFs7PqQN2Y2NU2QVD1TxUG/XXAoE5rIKGHZgUUPIaMkwlPloEgRAQQCpae2ikshlLrw/uW
+iaKD0eIaPoUh+tPcA+O7ykmSdKW0Je/ly0L9lkEs8kVZI1MUAwtljqridOAzPEjSA6LdiQR7IXg
kh/GRsvRH1FwBxWyQrzdRl4cy6AqDuPIrxwuPEIixsRriId0IOtPOPicQmQ9SFQxuLTIGhF0hdjV
Fwlki5wpi4wYOh4iXsfkRZWZUVHjjAJIqEq0YkT+j+Ix0NElkj+B2wuvQXWL23nIJgfk2nVxyqWX
cUOpSGAePepw2AFJffRobxHWk2IqvCdKePAoieMRHiCWRXggqzz/AlFGFJj5bGcKkTzSK1flNJl8
sW5h/SelBtelR+wQJaesEx5fDjmoD0LYnYj3anRfKhndEIc4WgSGIxmzFcDkGJDACXjMbbLgW3pu
wW4A6tDBTm4dsk0IBkpXZVontVJYY+J4hBXFQTt1obKbxaI1cQkcQfNk1yZ7ZNFI7AsRmjZfFQf8
mbkydlSwSViOQoM0zMZ2nZAl36mQOPo6X2YqeMp4LywKovUvj0873VeXfXD0TN6SgJHZnU0CtSf6
0nemsOK2IsVgjBBmpR/n9NH1KPscSDtGdkj8gKPpJ2HmFMjJflWQg0yk9h+97plwXLCwH5nd5+YO
yYvaS2DtiTErCWyoE+XJoEUUeTdkGXnzHlEDW0A2J13LIeKJTCTMEqKl93BMXkJpKjrDIaw5hYNQ
NQ8GS9tl9gFalWcBC/iB60zZd8VjyM049F2j3uEtZc+Nx2Al/rfAAOivuIq80TgOJMLJ0i2wQVeb
RzTrW+DZ7dgb6FVOj//VOWIzrS0FuASfIJEFIQ6tOLM4rKQrUkalYlgcQebrCjuD8gkdvGYsEwI8
FjUOoVIH84i5kUvJSShu+BNiYKSMXWNtFaVRFc8FoT2sRJqhNXWL0kes2E8g2NMoHGCTYIM9QXbp
RiS3jMtf24hGJ+x9EXtAxCYqpS15STKXAOWGmm9Ef9A8Bael9krGtxIqp3lBchAAEvEk2VBGEUEa
aGCSZmwJQ/fHujBA5pTNRBz68M4UoXpk02BFImIJjHcifDxeWy6K0ZIADiY6RZVDohWikqBlpB5W
s++a0BXaxNJiEn82JUUt1X1bGBcn5owDKy6F8IZ0JYmykZ+//mtyuR9T/99t1Zv39f+fgf+FwsjX
r//Xm03IRp7/zWartXtf//8u9f+ejGMuxVGQs5xak9sVf1B5X/umJGXeM0kRR8DkAHX5HfGLrq6Y
+EFHrFyw5aK6AjRTA4dNl/5QlD61EF+uWm+6FydH9uHp0f7GJz0L2Eg3k2SEZm4prQS0AR6dNjrt
k4NXZ4cv9zcKaZmpqYvm05orb2rBzPfLNQ2I6nrZtd4kFVJ2pbowt0VEolguYhopmZaGc3Vhfqgi
nr06PjliXwVo2TqfDi+dmNMbRxeVVb7gW7Uuuj2717l8db5fC6dxDbuoBaFEXJAtCy6KvvuljV9e
dk87NbMQLEOxiLghzBBtOJa6y7Rryr5UIlalASIqeGMKQ8QIQVhAqXGmjMf1FAQUUopKZSz9KThw
2D17fvyit7+xqF0tSlf5ylVaOdqwLjrnnYPL3n6D5h8dXx53z/Y5RLNOD/5lX7w6AzGop7ffpPgm
UydaH3qbOg9DODk+6wCb3nmnc7RPoYp1+fKi03vZhTCedo86+7qIk4ELlaIo0IKo2q87J/Zl9/z4
cD/z4MOMS+qxSfETIkLBXImfqy2eXrXK2ONh9+LIfnbwYj+OkNQWK16m2GXQXoLQJAi9VyeXPfvo
+GI/x8wV5YINCwu9fyTVAjask+4HhlG2v2G9OTgBEXT2kjKjUX+sufFQUOEKu6cMih4AcL7oTTg4
E1xPVH54689FqSXG5fuDaN/S/xuD9sPiv937+O9n4H++2sZtXzn+Q9pd5H+z1Wjv3Md/3+H1u2bp
U+Sqv/+tUhE9rig2C36DTwiU9/hgAnlF0GwyXX1EgbuSh6fmuAI5eRNwVTk4cIaU7ZvHcEmhDCEh
VREQSSAOYIScaMTFzv0NE3LZJrQSrhw68E8UmmVDslVhWK0AKnNyIgOnXm2YkVQfmJixM4V41JvY
tLENQbqBRnK7NJYGc517ej3a32D1WTxdsfUxig0Rz6eYo0lF50HslEiqOp1vpI+Z9YIrx22YSjrI
EI42QNT/zLxIup+LyZ2dHluhNVZiURjzfgye8vQc4XJENnT7pURMyPSUDcVrhLvehRcM/JmrK/TJ
jDzvedLvtURs7z34N7b/iycrn2X7P+L8Xx2Gfzn/b9zn/z/O/rf2sqeeuCJsDpBRGmvOeG1xxaB7
1skcHeRUydiSoguhA1zlZfu/lRr/hZdIHEHqR/IlgoWUioJ45s8b7O2npwqosOG5e/v4X28kYKlC
Yebx46mreVKUSB8VFZ9jwdt4asyVe71JXadIzpxxCZgfJunzZpxV0mGzJ9lSsD6/FnnxAs7vOcT1
A1Gk8vqzRv4pPaZlKuUe0qZPsjPEogzQU/phtbvCneYWy3jBNJWuPRVGIFIKvssk4O8KKfi7zBmT
lavprWRW4rR8yTfrjWaGMbuWhmlnZLvwQfwwPxcPyOBGUA5afk9WWwRI549obSUJV5WBR4WClcPN
SSY+XJSLI1rtpfHpMyCbnuBkRnO9wAwnYq8pA2QOji5VBASf2MhUFMqr+K27bKTV3iCzfmbaMi9u
vLdZVKHCcuUgc/hrOSj7qOMLSfxC27/Qkrtcs9DFxJTfumAm7zwVr9isflJqA1IGpSRUyo3EkM8Q
Ii54rAJFn9JQKgsMJIBFG5U2c3q3Wa7yAbVSWfwqNu1NvCeDtLqsG6C1ZLNcTmj3SSErEfpcRgM5
ZQNayhrox+WEnMVAbMHQpchyEXSPrmzzfBDq4UI70pptEiMuCMaib/N5pxTN5NhTEhcaJD4boUwP
KZ6dPvjMyOA6/EKMcGJ+NMo6mwtic3w0Yex7NT43O99Z/ga75aNdNlz2x+z0xlMzpPhv9XNOyIL9
NkW4Uq9u11fsrzAH9nAxp1Ft1Zf2lEjeqQkj2GclB9Q8n2zdogb/2xeJYQgW09ZN3pDPA9cDzJw3
SEEBo8gBQfhUnT2aOZGbAZfsqbOYuZUGDLRBfVTCHPanc+gLXWvU9b2A1kdsdRVmySEVOzl6/yXb
pBJ1EE7Cmcp2rxWYJW+Zk+5C7yrtWPafSxCy3atAFDxaXjuzfeX1omiuKizkrtH4IsFb6JptQuS1
JCwEYJ9mXDLxVHGibl81KQmtcjN046rhyyFWbmKxez2VE5+eded7QtExV+aPWhwlE1N/pnJRtx6R
+niucXjDFPvU0Zc3dOlDh75JqUP3byQE4D7bNKYVj2xhI20EdAXjV/HfbotKV+jlctFCubZoJPhl
OvlYuDNQQ/yNtKAWD+mPTkTF3qA4pnCFaKmBHblbnEWuVh9UrfFBVTtybrGIcb7FnuLsdZeClvCn
KyVayFc49wTh7Ch9RNh0rQQ3TUOPGh9nWtmqz4l9aH7mmNZ7+uzFma8PAaQ9/of8GrzRUmeyeTPg
Y2BBZTyOcQYcaq0FmR/3Icg6ikHw9CHwq895rQQPSajpoIEP4WVbPeVd+dLO9K6EkL24Yg6J2uYo
/fKA5JaJGbEaJW0DanwcsdDG4lFoWzp8WISaeeia+VxTkzCMx7kyasbEIENJjQsnRMa08OdMolQ0
Ha7YXEAwGVJ58+erXa6v/3Gy9qPuf+/Wm/fP/34C/heS9W9x/7u+u9NsLNV/G63Gff33O7zOnUD6
ag/mqyIOfUepPbZYtSNPwYrPtWk+g53bE5mm/Fg6BJ8Zl37tpMJDx8W4gMTtvXkwOIWd3RP19HuP
j0btiY0N63U2t0TCyIVXwtCsuUHGNkFmjyHk8XkReUnApDHKNHQCOvzr7gkqiZi21xTZ5loOpe+L
nvcWrUmx+Bw+TeqOQ3KCe2KnnowO4dgwcqf+JHnbNl0HPtzXHt2eNA0XcigjSceUn0eM2+/P+cwQ
f3u6YjMXVF8mavm5LR00PmlDDEUcSX0ej0rj7AP1lQB30ZxHO1lDc2TlUti773GsnvYuGJ3fCVL/
3BYu8/fTRGlxd638SZu7pCxhLxOz6HSdAFHo86eOlibp8swv0RuMJaGxaE+ZtWMajiiqfibHyP0y
CxZ3FV3L6CCKnHlud6crLtp90q404OXNFcs8SfT0PzM5k0ZomysQPaeY/NAPZ24zL0n6Vl/u5l+J
Q3Nzk+8LmPGerEAzATIQKOR+yFf26PnC46R3TBGkb1BELG3ae/HcR8N5FhJtWZTOPaIFUstWrnlS
JobWk8YjOXDmbKES4wPNDhVnzXlk/vXH/64gbNu0vUJImlHcHAVyyvNxPLiAnPFNySKpF7qU0jrb
lBC7kDOuJ/FxEMtALdKXPJ29Qu9HULu5itrNn4Pa61TzeXLB64v1MZfTFKB+WB9TNmq0Ds09Qn0d
NgHHDxP5khQ/k4ySi3ZfYiJNxkQLJewbO1OyBXT5Nu/Vmu02HNo2vFp9tXPA1GEsTmQwisdZ46nb
LxzXmymWikS3X9IdqMX4VrZ5MbzxeBXNnHico9l55qrv5xMkl7suIBkKYOvNOr21E3k8oQvgRjme
eb5/FTpRWl3gzjeeq3dX31lNtWezIQKBlAyrxLebr8qkTtPcYBYl87D88yThAzWgf0o53aN9L+tt
cgMry7yDYAQvnunaqTYf52QrTZANSllRywsbyVqjWU/7CpRblrhGu9CVEbpW2lUQu5QxS5JXb6f8
v3Eij/dTxH5djHNMJbe8aad78vpK+aewiQGlzFpV7kv4SWZ0GkaxeOlRVLroOQupuER2+YLuTObA
f8g4Le/jMn89/vP38oEi5dfZTxGxF/wTt6LLQa5JGp45g+tRhFjeTcSvBcnjPx2+H+lHuNDo0TjO
LJdzRckzS93DP99w4cSEBkmwiV+QO1qkeqG/MmEhr0xVvMQSegi+jgPzcwvdK1J4tT4ADW/k4YIp
+c4elwBXdDwPBzO1dtqpdNQsotXy3HztyVuzA+OtEs3I5YNjL3LP4QnD4Lkp9OX9LzcuUphLJ6K7
24agV07qF49MSQ/Kn4QU5148YP3dTbT0D+cWnKs2kpTrOR1Y1jEL2+6M5zqTTgRMvWkGMvRd49GD
J4DA/GXpnxkSL6Q2iHtL2SbiYn3FemEHjL1H9kdrvZRaYn5r8zdi5wndYz8KB9eLOdx+QSNNBxP5
p6n/JL8+9GPqf+3m9n3976fg//KVwK/L/+Z2s95aOv/farTu63/f4/7n32ozFdWuvKBGZ574QqC+
wEfyUMn+CMPSD0TS0c/ksW7Veoh5/JuPy7/7JEr6NAT9NlRZxOP/tvft72kcWaL3Z/6KStsaNTYg
kCwnQ4TvOomT8a5f13ZmNlej5WtBIzFGQGiwhBPnb7/nUe+qbkDxzuTba77Egu6qU+/zqvMYF9Ix
tAt1hOi0lFu9CselXAIsx9JU+3zW7Vsdc61MoA5b0s3UNio18RDxL8nZ9By9GaKwDgnWUWtzIEbb
ZjUKizwnCNyD1m1CK1JVMifbOcKiqhoLrLjdx7XdJXjHrc2BF02UlzQ0hG2obrFPcGmExavZ+1xH
fzIxFu+AODBRkYUuZlwEpoI9Xr/mzaUXkuNj+VG9TFgI3rVCtA62DqeGXooZdOGBu8ULCptWDsyL
7Ga7iE5zL0paFZDEGOFqY12Ox8YBx6jyNiGKaJLzG1jKljzw4yX5GXeFKIuy1mq10OQZFlXHN4RK
OHAM+1gAD9dczcgkGwMYAdh/n52Tf/JiNunS9pD8NgUlKihiD3QPzhGuurylFlhmjt1rwCabjN/h
rMLmhv2z/6d98Y/ZedGAbSJevHwrxhdTRBdvnv7w9MVb4GGLmWUqznuLNpQ1Zul/7p4iKxITxsnE
wKEy8A0pXThaEmItmNE5hu+gsV7Vam++ff301Vvp+ZoOhgL+HY4XeDEMX3/55vGbv/TfvPzx9bdP
TttnH5N6gvEZ59fDekIuzlDJQDhotUqCLiS11399+n97zDjWVgVMXloXv2DcE4yECRNz2Dhsz/eh
yXYifqWn+8XBf90Rf/+l3ej8/ePBwX4NA1guxcnJk5ff1yw5qDlowNbhdguRPBbfJMTiOns7ldas
jPe93qH+oTlFKCr63wvj/kW/8cC5WxxqLLHGVXbD4VLfcA2FpwBToJc8+v5KOzLTB/Ld7kmcQo0X
CIpDWf5VnywALdiKTEa5xOiVGgjaaSvFiR+pR5gIPSKLRQ6yUZwVpkeDUwEMtY0RhoajQHLDFQbZ
4rhEqrQfuRD29pTCkKGhEu49QvC6OE+k+P7pM8YkKwpqOaOwWd7K4NxcNqRXfQ0X/mOt9vj1D+To
/29JvYY0MU3HMJsgJY3Fibj7yx18f/pvZx/hwf379frX0B+Ac3qK+5ne3R3DTha9nkhUZxJxdoYb
W25pWQwB3xedeh2K14aA5uDoFdCTyYBCkHIwhJ4OoYAHEw/iAqN8ylCvbqxYoiAc75PQPYXUkQEP
YM/IOnA4Of7pSQrw0BDj74t9GFpyF7uX1MWvv4pfRD64nInkWyCZNNXZUPDrrwknis7X4mMNxtwc
qXo4xK1r1q4vsX8A4O4d0byA7QbV5UzSEJK7nUSMecc0B7/qA1gXOvLB3UMAWVyOQW48FF9/zUWn
v+pTVhcq3EG86PJXc7zqwg+AEK9T/CrPEVdw4h3Ea3hHpy68iAj4WtXSdciqRhBOQ5k3eM8nAgeo
Yx0Q7gsKSruaoFeXv8pIEoKQpVyatnp/ry5X8cfpuynSh5m8E4VV+dqp0uEqeZENeAvDLm3+zg/s
5/djYO/wmCLpffmGg0XLsIQLfUMMSALIDOkgz+ENhShc5BPAa/tMa/cF7zPa8HBEmC7dXwFU1N/C
W0Bh78nhyOaKVODDSzg+ad66aHkBqTlCHaE6dCu6Y7PhHB3sgIJ9LAb1rsDQniriMA5FOlCNR4jE
OCZqId8PxTpfNpgAT4F9+Y5DVBvnMSv2CRt/cpCuJoghS0ACrRpAxUP5AXERxjb57umbt69fdpsf
EzpfSw66VY1mhOptcleHR0lqozEhw2vCMgDdBEVBXJjIs7sJsuwfIo3rwvTJtIhP1XBgi00wYgvt
xL89fv3i6YsfutYyIH4ZIbsE+xLqfQ37g/cgLz71d/7ugtGi6xXnW3qXWC6HAXfkOJ1wN8ldaCUR
JtiNOHz0p46NDHHdsTyqefex9L7pPHBlPF1mRrHHhDPZSJ5kTR3YxsGjNFwc54iXJRYe1bZnL4+Y
ndgRvaOAInbdGtwWfiTQhCGVzRtoY+TTjBezpRXtHFZ1hP6DK8A7mp3evzvarztTQJvqAnC+aP4s
7hItQ9ByY8nNU0ItTQNWxPS/L/5OMdP9ptTuwnjRdvgjYKTWxGTIKZNoqEmkj04/4QtqGyPkERLQ
Rx7ApVBYYSjGNIBbFAWfrQCpwDaZ2RGBmgSMkXIBogU5UMGixeg5sLs8O0/E/n95gZv2MYEASCLI
IneA5Xbwh9YnHBwIF3/wpL6BU6bLmC27dHrKY49FwmrpqUXccgcj+lvQCrFvBJUTeXYeiROclEf7
DXlk4ECix+sUIyfxYYIJBliw3S8Jq2ccKhYntoCa0+UEXQaFDpzbqtEqNWHeMiGDYp2c4NzpviRy
UmBGuAAIKoRQkcHTfSS5pTm1Sh1yKXhsnnWQMfyiJ+4pb1xrUqGVL2QX0Io+CKrlQbFjayUaiHC5
L6roIh8LyD67Rp6vzcS3EgvIUxTsNFG8xijAAECX3WdU3BB0gFyU6PTVAklnMhvCJNO+0Lj8URAo
C/dRBCM2bGizhZQxLI2ZChWamUhsalC82QQeZE2JcKez70rzGXw1849KAymk6gWFaW9SIHRANSsA
5qyBvZaqg6/i055a7dSJy5jOlJ81nfp9u9F966RQ5+E/7v0XTEndQx05qV4sN2rCIp77HgToIFJQ
D8upFag75/bqHYwBZgWZBRM2C4OgyYBX+FWGyAL88uopsNZpvQZHDwV0Dis/Evune8WZ2Cv+Pt0n
BQEKlvf3/tLde97de1NHEPcSEhs4aO45V0ZuQ/4WJwCRcAM6ihPmabVaj1AX4ytOUKilmD1CVukl
xNUSi1yTuh1UJEGb4lFyV5ZKmKj/CUrgIO730rtfIKjhuMAm7n6BoiNq/vpAjlX31G9xIoV06J5l
TAi/KGuB7PEj3bMlR02mnhGT6goXuhwNv3dXyizwmJWOkhzcC3gSSYBp96epkPVQL0TH6JFqWNTr
kn6i5jCR8choOBTvAEZ0F7sFXZIufEwfEXQxQc76kIPf5tRVAEEMdZdrwf5PU6/tOsgoQIBgDsml
qZ9htH2aRtWlzjGiGHrL2xppFslC2WLB8mIwWgBHmq3VXKqDiCUcDyUPiwsJ3CviaT0ziBTw5Rx9
wVhqf4eqryYp0Jog0/Mr6+ybofLYv/p9Df3H02fPNrZ0Rzy5yQZLvbU55DlyG8R+KNGCGGhChsiD
kaqK4imjtDNbcaRdAUwyNXwjLj5QzPCF3ebX9uvBZIxyiNsl+X5EnYY1uuJYxSh9LnK//woDqMk6
IpXLNy9fv5Xqu+Uim4t92jR07b9YzVF/0SR15VKpwGEeuRLLx3KVFWY6au8LXK+3T14/r9VQOd5n
Tz51MDlU52iRU/jxFy9kBIFxEajh+KANRhd9TpaCz/Qvcl4nFIvb71fUz+yfdldzYHq7Z/v4nYrB
9zrSIEQBQUUdiXELANjLKWFiwNA/w/b/85/rekNxT8cgNo2HvbupQq0wvr324RAR6zSp662Xuxj7
4O4velQf+/BDdxV/jYcfMbaGz6/fhe2pMACz4rQ7YUFxtuGrPHV6Dhmh8VIQShPM6RzR7PAaKozR
A/7qLtQRd7k84HIMkQg4BKlmgs5IGvsn+OxAXo61KARTIn8hZI2Gj9rmOX1jdDIZF8sEB6aQmZLw
MGoAkkI7ZgBbIYSta54ycblHq/nOV9C+DOpFxbkTeObGg5y6obn1n8UBx2s94BCvq8VwRMLVxO1p
2BFOBJFYzOmWcWPDvj7u8G0gALPQcmBO68+dBvKQYTx9/qMPwc4A51dnxAD4Hg7qBGafsxtxOjhz
n7aaK69Yxd7wHWlkRvyEGErWTcrD5kSi+nlxtrq9gG9yJ/CwbTgvZTzoT0OZkeGmNbbTeWwxmg1B
gOiYaedo+K1xklIKqOhA6iz67s04GRY22fed3KGmp2BVkH1fdlnS1qx68SCghKtDJaMqeIr6UgXW
cXHHEdnxZS3HaKymlaiuBzOOSXKutLKMxdRU9ueEwpi0N0nMMZiueI/4rHjf83CswvmIaxGv6nnG
Hzy1HzeQAcLDuqXrbDLpu1wgcPkFkLscTQR7bc7GAPw+0+g2MRxmBIlL6SUl4QN4HOUULejiUQ9P
OLBuwLsotZ4kE5YoJJweWcwqfubr5eVsegSAZdWTk/1XP+3XxlfUAjwCfmZd1BaovukRikzhYeu7
8WD5GnjKfJGiciGFMq1scfEeBKx6vY7yEdZgc7QFVMRfuEz0gEhjmtBQx1di77jVGdH9+69ir/nn
gv50jujvQ3rFkTgmkxzzUe4dF0JmpGC2+VeZTWEPaxTXaICX469E7IlUK9VGk1m2TBeniU4/kpzV
xQHMIEjQp4mTXCQ542cYGgW+6qrahfcK6jY06GQPeomthQWVry+2hZMSfYUqiWSaTRPmF5P/nRjY
UING15eZO6yu9dVYAXi99uonqiOlAH/bmI0KUkYsyLEWOdSeQR7gb1CryUECZOBjJQ3I3nzNN87y
yhCYkXN48c50xBFBMG8QRydD1TJ5SMOLLp4mornJ86dv3sBh7CZRLGuujQ/IXXofcbS0+7R5FxLM
+HYcEQTs1j08tB+doGSJqwyXr7yTYx8OXcI9IBzgDE8IHg/8BZ3NhpFDUbN2PSOlLu/YPWv3t9q4
250NjWYzMt/SXnHg73GhyBRMjb3ji8iOhmcmWwjsOt77hbut6YG/4xRI5TeP3VktcOM14o/FffW8
gGkfDCgHkgXI274EJcjbY7Z1rozZeVeqq4jpzJhBcI4qEG3lkzpKLCBehBuJlTjAJBOwbzLiaS0i
lCqqU4fJlXQMSjgULOHK3zpWCF1W0Dz94Q2tCl2CEmi6BUVg2Rx+p6lPinEhAItfSYMC4nJ/ZaMA
KB+Q46uDIpE343hseij845eTnrpxpd/W5TgWRh5D6ZCgi5rY3CU5DkkInV1pba9lhNSS3pBGjC6S
OuvzWHSQaqYDrTX6ae9qb9jf+8ve87039b4mtYq6UlVLa4WqPt0imhpxG/BHBYThMiz30Ky/lgnI
YMtLcwWVhcxa88QSSrVkRP8QDDTSoqDtmHAHmNy8JV7LtGo4SbYu7XMI1z+S/a/lCfXJ2riF/ffD
w6PP+X/+xev/yRIA3Mr+v/3Z/v+Psf6VweE+RfznTjuM/3x8/KD92f7/nxz/2Q7QNw9jxW4MrxDG
6R3OrkI4pUohv7qOcYj8ckl/7A17noMcriqFoWpz2dMtoTnlA2joJrwFECwW1C2NxmuF4tVBClnK
6BLX2jyfoeehbdizNE7hfqrkItfxs1HBK+WVLt9DYP7U5Wwh88jOFuMLugXXqV+bj3Sq1+sxMoGR
QLsW1E2RdjcGWFbyOA1augtcZoshWQKnim2vh52AguOr1VWf56Av08K6DRB8gpxNrrN1Id6hAuhR
j7PDOrljI9GECa7OO2lB7rTC0NB4i4MXOMBr91kaj0yNDrZOmenFSzn77PZOdsqoC263HhyLRTY8
KFo6MPTjjrjIxmhtczU2AfZSwOLSclz+wUL0T2udXU3qHDA5I7dLzhNG0RyLyKLuGtsay2fTi9Uk
WuHPbTVe6+hIcdY+hzK4ZBifILGCsVn7rR4eyHKoXDuCTeoJehjkRbFTC+xKsrHnstzOA6gAb2MW
p4nqUUTi2jlFrLQhVgsq1F3Els5LF+KVMMlCisEiz6fb5AuxKYUbaFS/MFFK7YpRGrF53StB3WIF
YvC229+lNXfcZjE4twzU64AoQa8upGghA1AGSSZyKjDoExoNsVOGTAUIWJnw79fity/be+hUwdZ0
OheQ1y3Zkh89mCxD2q0vxT2pK6Z46T5K26/X44MFgrhcYZyLtRXM+qto0emM95M/tQ/b0eIhFXHj
8Hqv492L0RY3t05QIJJix0sUZJ94fa755GfISfQlXoqce+d9GFFXnv7ovpxHj6p+Hh//rZFEKW4I
iUZ9xwPpwrGL1Es2QoQL9fZCUCIOymZB3d7oF2WILs9NVFxd+bBk61KMuAWFR9Fl2zrGu38u4JQx
G57nfiXNLwVzgvPXR38r05mWzRUa1hBdfgBOHCvwjPEVioZ0XNYqFMqRbSrGxTKfDtb9eQ5MmHX2
Db8T4lXCd3B2siliYjPGI5L+KlEp8N8T9IXs0zpNKCLNxjlSTerKNFZ1sxBWv92Jpx4p/+HIiXfe
73Lib71Xb33mdw0Q71dnr1V3U3aO/V1p9mTJlpzMZu+yyzwbhmvVbn2loWk47dZxuys4JjHIXphS
iOhkxX4PaAoKDnY3LeBHAFzLeHwLNkXw05mYL/IBR3KMtqU24K3i9scAeUJDAMl5HwelioCgM7WO
wMOS8w7IOe+Pp0ixBjnCn9hz9mdLTOTJengMk5Vfk3vwbN5ExwQk00V8gjAzLRHfyEJ/WX2oByDO
nceZGUAaKS5bYxNDU0UjtsSIfKS8wgFG+Zx77/+T+58NmUY+Wfzno8Ojh77+98GDw+PP+t8/gP43
SGe1nRq4VA8cwqtSB+/GvYQ7NZLa1C9TwcX86wUX3dtNuggdfDQKxlGMb4QVUaPvIMRspUj3c5bN
0bghKgBsZvp3k2YUEz6YrNCKv1+MP5g2j6JVtPnONYV1s8h6XDrIKfYgJZrBuDY8bD8TW4WiRVYr
5+2D1bAairKDxxsqoiUH1ekP8smk2DAfutoix+Qgy23FvHNgu96hhePuFYZ+pqTOYdvbHw/billb
YGgX9BYZifNsKLncOE8r57xMljr2d6BqYjmeor5qXpRtSVfluPWKKGsZebK2nCevFlkSh3opf8Jg
Crsiez8bY2QuWkg09YmORzbAOt7l4DLsWImgrLfKeIqRntk2zqt81NJ3BJ85zD8S//fJAkDexv7j
y8POZ/uPP8b6+2rgT7z+Dx982T4O4j92HnyO//gviP8oTbJrtSRJHt8TFxw2WVkCWHmIg+iQs8Fg
NQcqtxYXi/GwBfVryoj7Ms/mP6sfVxhfXX5Hri3DaBW66HR1NV9jRsDpXD1CT7V1rUYOCRcyJm//
qrgoWvCPkIUwmP6bZXY1z4cN8X9WwEgupujgSLWm2fugwkvVXcwN0xAq1npDUIh56f0wDKq9WaIt
BL9fjlpLlcNAxkmTpfLVBM21oRBmi5N9aQjznd9RMYaFIYXzgQNkmP+MjngDDKwsHsPgFhzsfsFe
HyBAiT7QVeC/+imG3KmbIOX4s6VlEnIPgTlsXeTLPlHnNPnNSDINkRy48lvdBWSElAgkJ8la8psl
6SDcUtlOV/PaMgLF5rYs4QPb2iDHlLVoFNCbW7S02NEWydqnrCH7bmhzU85lU7wx586ptNmIrLa5
9ZiAF+2EK9tZfXA74cpo0D66aoSb0hPlGuKwXfeHY91GARzpERRAci6tGqT49iFFbqo0QGc6Quix
S64Gygf18lXQV1xVvbYuwhiev1nzxXg2HA/6sqSKjbhlv0uq40RX9D287ZILuKG1yC1ZQ/D9WHlb
FXdk246y8p6tQaHjN441fs22dQ+qL+tKt8od8VgUVxhddjBD/30gt9l5Nh3Opip00hxFtdkKIyWv
lvyT4jHnmJV0srYgwRFCB8PZAujbbCEjNhfXMv12MR5ykDUZrfkym17gc8zDxdqYln/uSm5LYU5g
uClIl43tJqf83rXhROWEifHRCHnK9Qzb0HoGD1JvCScZBh1BQgZFX7CDXPgaKVHVe0rCE3vPgjkJ
zDB9PdH2TignpOn7xZodvxlohA4huSMpjIy5ktJ2PVKWp41DOMX7rQuV9t0qM6JMFRsKAZKbTVZL
HkHpVAzy8Xt0K9s4Dvt6vgILOrf4DXEcokEqkb3PxhNKHtcTb2XMAp+uz1fnukuvVucTdI5cuBvU
YwKYAUReLV/lpB3sdRpiglqP3luTb6SMum/boE3eG5Kr3L1Rl0xv23aEuDdcBtruRylNRw6ysJp8
szpHifE82qZmNxs+561fDySyLG0+ZEq3bt6wpTarb17t1jZbA23bduksm7cbm4+1vxM7GRowRdk5
26CwXt5oydjjndPbuyZ2/ATw4vNkOsr8DXpsttrigOiSz30hjel4Q0Ncs3BQ10Kipe+k3jllyPWG
VUH3xloerjWZXaC+M0UJmgICKvmZLPPzoTW1XoXn6MG6h3ySe2pKK/wwQ7NxU8OyGSurgvjNrmJZ
4JRVeS0xBuvS7coxY7VSKJwnYDqasMZgITNu7bUOR1caoMW4l0Fy9/ZzZveEYrbIpEhMZKIvCTyC
kSq4xIZ/9FDYtnEUbStMxVBgrGBL8qZ4tZpX6YbNOtyJrG/acJDR725E8jhBK7EjdfvGAmIsYbSA
i8ww9ML+i5f9H14+frbv9eATNV3OgLSms2uLO9zAPtjFxqOQGRwXFFQQ2aBugMpYqvXrtMgJZy4z
5wWVpvm1NV3VZQtg1zTTltZK0aXd+iUFEGlRrf542JpQcPU0OUjqUQA905ldqoZPcPpMfymbBxyu
1uV6PlumMOrWjWjilLVuGjgJrbX8ua6LE0ra191qgNbejhVno+pa7RYgIpz+/R5HKhLi3/DojAfA
RFzOhubokhNPf51dp+FO/tlaZ0RhLS6NfzA7trPi09kCOBuQ5xY57qmfYbLu0b/34d81fV/T9w/0
/QN9v6bvJp8ihd+24JyITt7sHLrzKt3GYMJr3qOoAjM9/RkXDJrHfz7gP9dn9dPDM3OuOeznctZH
HbB7rhvy5Y36srbmh6ejZJKCMwFzrA5bZNatsSCheZ/3b6C0bBw3GkO9Ccutdbm1KWdkapDJMSxx
jzcz/EqhWYuVsF7Cd/clhU2hjkgo9+zu3Rf+s7VXE7vWjFUMwa39xXSxBSpuqI8gfs0WqerYgTv1
RvKr1xub66+3q29RU9wguFFovv2dQi9v5N9/wj6xpNySUUQWMuVewhq0W8d1WoGS0mtdel1Z+vds
sOhaq61u7xM1gKZwHqwbsZprszVVvQDUOljfOIIcT1HpBHunAP4d7UysdYW1xknCp/rZJVnbALbA
1Io07uJdq8Bcsv6g2xia5QZwHBcl30l8tIZHDMXsu/PVGDGUST3fR7ilLAiuGrQ9nbeyIsNE0qnN
3TTEkCzP4DUci87DOu4Y7KG7DM6eUsNyHlLHI+ImZacgNbPaQui12q6LP8lfJ72Ygt0AoFu5MRES
rh0rbskvxHezFZBU8vJezMeTmCbBVsYfbDw7lhoHY3XZbcE42lHiZGbA9JJFCOD4rNGpr3Bs5muL
6fN3kS7HO8mEL5stxJCCpy8wuWzatLvXcDuLSW3czlLtm9vWlhMyREI/xBM2RDIP/zxya95zfsZ5
JPRbGE9tbZheMQrI379pK41tQzSHN/XSch0sB2iGZk5NYFNUVFk7oNfl5RRotTj8F4FHKi0pOyz3
24wBJummtGjHKtqpLLq2oK4J6rq0qAV13eGi4TL6W/M0ukq69a4G3jAD7epxBJXPxK9mD8dh69F0
dWcbZuK6el5C2DXr+uAt5kgDIRVTlSzGmDoRU/5xthy8cxCA21cXl5j/TQZ4p3ACQ8LhRcuCRJlo
TFzT0Wy25CDzTBD4VT5Hp9Ep6hiw1XOM3I6x5IyFr8HYsAwWXvyT+C2Y9Fit0657JrtnAOd7ncbb
L+2c4O7G4ljCrrGxtAvfFPdpm1XNEDF1/2i9VBRMU1f8Lr1LsAkL5SjhNiTIUlQr3p3iz9POGT8/
bZ+dRZEz0WvDn1AMVyDl/CUg6Hgfr+4RHLwrCYjGnh2v7xGUOYCejvGaCsnU6VmIjENUrtBwCQIu
QeEbasn5zM6LFBAjajtkcZxJerq2nnZLVaAabUdfqsEiEyBnmbCamer7UdRZvtoKor3k6hmvu/4V
LH58HVrZfJ5Ph2kakDG7ubqvZjEAwkbkhkFKYYo1xLt83ZtkV+fDDLBPftWlf6Gb9dPOWQiDAmb6
pwohh8eJAjIST8gIOHXUxI3YUW64u9/8HOYFRjwgJQM9tDZPkiSv5elGJ3fY4rjtNIrc56h5cEwA
Scq+0VtS6qBhlTV/LleGCip7te2eykQEHptPVWzcRKVoC/A3WH7DwEWxgHugSTMPq0bmSymDOzOt
kqExcWy/0KuPPnqgp85DfXFuaUXcYaeqq8hCWBOPm0LcuycOndL3VflOUL7jlzcdnyKPAnRpUYTN
A8Jq1xsibaovwAB15N9mxxOgsTS97PBLrqa/OOWtqwUOc0yz666ClJ3oDcjA80k+Wnpaz40TSP26
2Wr25AyuN05dqBpEHtfvyUl8iUuRQYrDrUffxvaJ/6gWYdsbklqYFQ5bn6KmEqdZId51nFfUMLCr
XCmqHjXloGF5JjYw9DEoVcddN1G/HWAbKUwBiU9vznYHBHQAqF+bqAzM8heBmFfRKLcZw1HYnZuz
W1BTOdOtbDhMzfwExfgsSYJmlSsnI3GVx2W+WsAGHA9S5sjwoFirMUQ1EjIJksmCI6XOnSmztst0
dJnOWaD7QcmLd3MdTyjriX5eLFM07oJ6aHQEBBnJqSxmSOCcsk7iYWbLj1ApEvQeic8G0yCU6i3l
skd14rYtmPQqZs8C8odjgWIBsy4XwlsXumuIQYRp3nQlHYG2LoOG+O8RKffKiWSpamtHBREbf5Fa
6kO+mBVp6kCsW3qp89ls0r+lrpNiIGewVbI1Uqdz+HYOewsw1ofxPI0u35aX/NG6p53umTd5sJ/m
hVQqAG109VGwrXE7m8U/v2lm1MdmVsIMRz8HwtZW1wMKMTZCAfenRIBYZAM5sWzZNMbw91QjKDuA
bg7WSjnt3pro85YhrVFQ7/Ho6jtaUWRrFwZNTZQikcZ0ENGiDowaNY5xaQlPB8jn35z5t6l3xGOK
OsdmjXiW0BicDQ5BeMMrXrQGI4wx/qDMHPE6VZkuGh2ARlJ0ywuN+so+I/kxK9aoUNSVlbCIERTM
KOcifF3vSPvWAJr1XKwPG+IRiui84oLbDdS4sWvfyNqOMu5+VBmnZ+l03e6i7uem3b3pkOKIV2rd
bsJw1p0mimY38OMG3sO/Zz5B0ZAMnSBfIEkYQuGCaUTDVOwh2rMm2Ed8FghPtX/RpzRGjORGq8kk
xHGkdR9ZuI6O3cMHlj0QsHzTJd3WlELBbWBr8Y8Og/rr29YfTGZFPtwJUfsTEJPHhEtWlcR1yijZ
MB3WmtTR7qltYKhvQLXMsjsG3u3a1qIP2zM3zJHCX24xfKqLtXnWIsU6VMzlXLSoFL5wKzfLajc3
VY+LWXT4rTk5KTOHd1FDH/DgakEbB624G0pAIy8oNKyYg5CWUguhNoQ2zGkJextFNnaVmCmLNQBj
wmDzcCRQoe0H7pOwUbKmws1F5UIFC0+YhPKFlN/jyFIZr9OBRFquzif3vl5dae1WWldVGo/sxk6A
kGG+QgPppEwM8Ti2hjV90fJybU1jDauZ+HhwEypdGc1aPTrnLTTtXxR5Gr5W6JkMla0Olsi2DWJC
aDN+KjFXEsiUGIRphGmYGqbhFuKn3NPTaglws0AavL0jXrHHhBiOs4sZRjwGJIummQOZfPE8X17n
eL1xPROzc9jMg0luXWf8cUTbErqfUxSHGetKyq2oSmW2iuMkeS/LDk6zR4bnUGtGmZZiwl/kgo9N
oN4jDbMxJ9oe6I173x9fbEkMpEc9TUBvuVW86sisKOiR8yrxmCl8U1ZobRVaRxZxPFuMyavGDOa+
8Ai70ixJ2h5AsUjNqrhkWgN0UAFvGNgNeeTroXIjwH+aCYQC2XiqdAW+moBvGGzew9UZfFLRP+7W
8sVGQ8b9g/36NvAsefmLzQLz9jqJ5ifUSDTL9RHEKB5toZFQFh+nVQIp03gkLEQZMJ1SbB3PLEON
YX4jb3BYgJ7k05TvI+vuRU5XxJlXKnw6PnNtRyrLYqNnpOmKLxepab483jwpxWo0Gt/Ia0wJuHvm
SJvjhmJ9YDby6eoqRyeDlGt68x5R2PJZkWyAIgd0G1dCBmLdlLDHYWnW3aCagfpzOm5GbsikAhxQ
FmlPUENQq7jifCQ6gvwS+W4TfnZrG7ioEuopCSf+SfW4M2ZkvanIbs7q2zcDSH1Ksb0aoq90LCy4
OrhJToqtbZVLpAFolGA5B/TReygvzLHQpWk2Wu3NG0tdn6uKp11YGETz1KMqjXIxvppPxqN1HwMG
UATIFLvmolhi2SXSxBHgoaNSyJnFbYDxtWVoSI2wYdUpvnJuATWDy6lEI96BdDLo7Nt3+rob3lLa
YNIoLyzPNKvIzQPUaZ9FkaZdqROr1PEqBZJYZJA+42PefNGLlA/3q5lXJQB43XJ7EZ1n/b22AWoz
ckNgilqaf/YU7OdX8+WanGAkQVcks4HJKIvZ1Fo1KROiJ1Fad5769BaVofJrtFyBznBVThuOL2dL
dpa3UZkDpi7FfmeYOC/rJd8/fvrsyXddkcA5kwOK+BddZxiG/XIxWy4nOaoJGgK9uMiBC8UDzosn
PaAkGJsxQm8oor9yElkfVjGVSZK8Wc7YQEoFVmbjkeKdidTLwQpxB5BP+JSDAwf2AMobC9fGeBqm
9aBAZJliHiOIPILl82FsXEJdAfkW1RJ+3+DMqtdRvfF3Rbhzg4m29nmIwT3GNWInRcSAWCOJRrkM
yLblrhS3tCtXIQ5smakcj0oeykOkHq8iEab1RGFMt9rar9aJVvN4B9nj+z2b11X3j1H7djlJXDHE
QBbucdfElkJsZwCK1xniIX4e2eIlMkFJxdJ9bXSFxCZzjnjNO7uODRW8M9sJ1sLFBoYwcJVx2Uu7
YW8PyDMWP/6qgBwimkPbQw7L+Tep2o9mU1HLlWZT0Q9y0wf8LO28+8BuntCWrxg0K6lvyPiQ/prm
ufypBhbyv+wlQnsYCMX0MJUQmmoAEuaNfuJduuQT3dtHMR3QjJdyRn5F0X41t+qX5ZrE3bJ8mmK9
KiJ+igw0mO6fyYAoEp2K43rQP467ibuWlhsbADLht9J7P3v2ZrzdsJI2GMQf9doGgs8V67dFl87e
8TZPFI+Fau5g1SR2LVnOKFsaA9KpAlLOpjK7AxOQvHr2+MWLJ9/RchS9Xz7KAfV+6bYORx+vLNwJ
L5MWRytL/SlI7aWpNySQqP57N8aLX9dLfLk9XksWNrTB9bSX1KFPKtwtHYUlBva8c6+yubsxZVAI
rxg+rvn6l4ijb1DGDv0SOpOGxaXvcolXs1PedlyOuDPXbP5FDd4SBvVIrWe2lq0bCd8TMMbHijG+
zjhs7IhUXvOGBt9QcD2HYd8pF0hqFe8oeTAzZjRJgSJNPWv1FuYzzwdpXTk32fFjIk7jEbaxhDFo
iP3pTKmknayL2MR+dGAG/WZzvvvF7jpGQzrqj+m6roZBBpSX9y6+2BQ4QFVUa7BVTZotVXNrt3G5
Lqa7vP5G/2p1yH9lWtxudQI0WrZcQcGE27waFxS2GLvR2//l4z71jr9hZ+hbHDMG69KwRtawhhLe
c1QpGPxTkMvkaBTRxJcW+GrtXsxKzF4PROK2cyLdxjqAtzwMZqjJePo+m6AcqGL2NMmLkIKmV4/I
9dehtkpcLwN/XPZpWBjnXVcdHcPujSgyb8V42O2LOo6tLnLfqW8xpf1N9Ok6SmOzxeBya5fMWFTD
nTwzOXtsz4VmOxqFZj84Hw2noxY/x64UVQCdQUR9K+yJ90O5mEYjk4d3gtx+afSOJSzFUtP9Eunt
HkMJtIQWx7i8acbXebmOvVhrclUdLLBb4o7oq1xt2zg+2TGLXteWt7YhJB0Qru2C27nMjXNRQ7sp
ylpsj+41QgoRPNBn5f3XJHdAbhC3GTvTJDtgYXWvqabQu+fUk1dm+Oa6N8vK0fA8QTy/HXUHCvhZ
+YzajI2MrRS5PFCA0D6sXhqmrlSp+vrJj29Qqap4JDZMXuRXALQQmBomqVfPucJrzmb2DNHU41WB
a6GdtHTAp8B38o74frzAhMfLJW4s42alvalm0u305xX8hBVXmtbWLqgjUE65l03hyS3DbRH/Mfe0
N8qtPL397bnwkiVNwUpMOEhrMRnnIhsAv12IDLh3eIp2QuNC3kWx/60FRHuitWBOp0OeSglUzY/r
u4YewqZWVJvqYeAohSh3xvs906moSAVWKN9lbpTJYJMoaUqNp9QWzlJ3811Breyqv7GBGY2+1ul5
9K7ukn82nEahxwUbaHYBE10kW1joxCIpSR91hcB2pp6lEXartDB2o8ZhpprFC+p1Suqtq28ESzp8
Syr+L9gDZu2xU8PZaIS7VqmJkjgcChts8CRpClLKPq9q1qslp/K1Lrd12WJG45Uj7hHb7eY/NCov
R52bd1F8B1WxVJU7xwRZdG9Ed2GzYDglOLYbxxBa2bvZFIOFj7KTKwGdbI4A+Yc4sG5ES3VQ4cxS
bO98UXZml5fZVBXf4XD6sS/jp3KrsJmf5EyqKCg6MkqJlFamHyo7gVHVvHzl3cPUa9toQwNgqMsO
D9brgNsUq6nWbH4NhwLPkzlr4eomGnFzd3sphjdtUJDTelJK9soJ4rqce3yD4VzEbDpZA6NYrAYD
mNTRaiIdr5i5lFw/EYVBpsQSMV4WDgfJflyWAS9GcKfoMZmgDCp4eyQRiUEsg/Vg4kehv5XktNlj
LnItW9asvABIt9fp1DfGYNeoYbtA7JuUOUqkKpP5fA0NZR2gbAFsXKTtxzCCJIw2K6zwPdkcGEfY
oXmBy8wDtiBxpGBcPmAQEMujqFXkaM8yFSTkM/R/rEB+OM9HuMcy2B/v0Uzm+nI8uLSAQUtQI2eh
Q+XrRICDyxlec0s7GIxuBIRgDPMi/f2lPB6NQiQRWFPSAhgJINjsfPY+p21wTyG2exjeAEbCmX1a
tS2E7NBYoVarjTFND2bD6/dRJEj6fRSO+/2ESQyjFErjg8nv0sTJtiWVrHb2H3lZwPWK+XgKD/57
839djYuCU/9NYZi3yABWnf/r8PDwwVGQ/+vB8YPP+b/+dfm/kiTB27jm+ewmHzpXX3I38J2rClLF
6hTWvRH6q9XeIHrQ3jRo3eYas83YzI3Sid1TEdK7jJWCJL2i2dyQ36nZfCS8jboDKCsidrNZBsp7
Bk8UKPmGwDTdzyPhHOgdAFn9iQGqvbrMCsB4qcRFsEywKi6YORahe/K/PX769umLH/jG1dwBr3O8
Qnryn6+evXxNr/UicdbUDNDzPCsIj0uSjYk50bQCpZXXT97++PoFVXTXn5hUJOpyZ6gA+OMp8D+Z
qfrkO0JlFBeencFZublC9kqlTcVErhr817J4sZyRX/3bp8+fvPzxLXPYN/3FyiRFIk4Kei7JDPA7
RJguYey12ltN0bCxgnA/vgEiBPv18YufZKswFgyRSXLXPX2r+6gXtNZU/cZAdgv0rAFIi9x7Ki4W
s2seKW0FHtciw8u8JtJnHbGGe5Pqn/dgLhfLFbIVawpZQGCLec52kfdKzhgG51vkmFAOui0DsdNg
pzMWqGX/qTepfdBHQJRoWwHPMMFYIX8jK4vL1XKIwffQTIhKdHuY05iap1Td+Q2e/GyEAopa5QO5
TC3xjDK7AndQkwjk5xXGEJKJkYGSE6BrmHOYutVUpApJ4IYdDwoB3DecmbrIcdIxcTFwqK3a48lE
MAeAW/b1yzciBc5nxZkG6vSqVbNzElanIeScg1unHKxIMyhPWnlGwe/Raf3oUGXJqKmMf8/5DD9n
HLFlzr/KVH3bZ+er+9nb3HTGW2UciVTaJk1fWdO7pTuJVrtdypPy2YwNzEbgVRlUfn/algipSMIU
PS6OKs+55JXEjHAPMGN0vSQ/m4fVdszM5tbmxsK2bBRXmTTPFKMUYofHPiQLc5bDMYUoQdtXQb5A
F2FWQPJKAriHFVPJmzHfNZdfSXXs+2H5ZBqiunFCTdryeCpCKQJtSGloF5N5Bn37wYCiADiMhhGB
FpYFmHgt4wPll8D0TbKqNbeL6VRjt8g5R4wWGmNKPisJTCQX0CibZtM1keWRnkpjG1dXYdWTt/Fh
/jXmX3yIm8E6qDkK2cOfODLJOyQlh2LMeyoCSrIRFcNQ2AA9NrCpJI4HOHlSEEHW4NZ4ArTqJFzx
RFK3SelWnviqNK9bbBdt014SYfOTnZtDBtPETNy9WV0VrRhxcaALmpupmFey3r1Fe1SvZJhtd5ye
sbGfo+xW6eCO6xugRjged7PVquOhufU35Trb0IcdM7BFK5d1oWTXxs2/dZHABDyaaM3L3GaTjwZF
1zM52DDzWtSoPE0k8yykhN1FgCRO7LXaI4zpzHoLxb6AOEcvkrIZinFTjSqOSE7SHdH8PR+hVqCo
zhBWXFiCgNYqFxdbmfLOZdnQAnL7RFyKEimiM0dl+Ly1rkezalnkUJlNoJTnG4Hbht3tVrvkSs6h
rdXtx+lqZb4wUwW5BtzGaaLVJciMEyjnHoYXoLdXJGJPpKon0gm0bvk2xM97dE03LoBP0qF6dOol
owJE1oyiZGIDN0zsUqT3JYnkbtl/zXDglkQM4sW3vt60YmiVKavSOBXfEo3yF+FhoraKlTyPbSZZ
7nm1mV265TpFbPTMSv1OBERICLHqwEQg0AeBF5q+N1DhlI0nvSRxfZu/hV2RL1iFZjaAH8Rcj9aM
vbudcwUzFPzT2gjL/AbRUbLX6ozEXtF8tFcIeSBl4YbVljeIEt6e/u5C+nyq5zI/0SrY7zK3dKBq
T9+8efryhfQ+p7LmPKpJ8NRCLs4llBd39CnNBaeze6f+uVNeLQa2wdlWzzSfiAxnWf+0ElhZ0ZVQ
imivWbxLptk0iURF1uZkpiG2AfOaCLztonGzsPZhrPbhmRMJwOYG5FFRE2ENn+yItEeZpae4ZxTB
TiJbo+sgt8zjaBToCsYEZflj1BZTw1aPSRrQkQvsFFhMUeIexdGoAb1gbiyPc7vGxtAB0XDOvcjC
VdVYR2ocxmpYbqyta84i7M8uOftZ3om380kMsbzSG5RF/PEIzAb0x1FVtEUllXJPYdyTYVygtL7M
08hOrRYdNTaTAiCjMw2mXvsdVM4oGV11AR8tPc7yQehgdJGQX6tlfzZSrKB1vRPVnzZVX8IgSrPp
hsSsAZehbMqjVSj0qN+buMavtH4pvna6YrPZvaimc7uEr/ZMokl5NQ8V6Hyo3vlqeJEvEx8cLR+6
eigusRCY5CLZCN6ojORcltew+Ht9q4n8PfcQuHmXjPWQs7hS26HH0uM2obn3RBpOgCENDQVy85Ex
3Yxnk1Pn4qQX1wBXrI6vs9soIpVP45PvcBb9uTukueP+jK92nD8zV+qENGOrHomrGufMyyUeg+uI
MNarBr7BU+bxt2+f/vVJssuyxpGAi5JK8PPOC+gvnryvja0d7HtkpDV2rRoQxll3NoKCGyEsdwRe
MA/Rt1IlO1LXwcp8QJk6FJ41GUxUBJwTUonsBYwdW6t2q1VUwmQ8hH5adp+BCxlZl01EYCMid8BF
ELl9vVEWP4/tyMYXU0T4sutpoqx8WGgyK9qi2ST2fmfLNs/sRbLq7j33J7Zu+/zZ0f7vajWBvTQb
olv7irTun6CNavs//u7a/3W+fHDY/l/Hn+3//vXrPyGDnYN/7vofth9+efh5/f9A62+9gH9R+JiQ
gRlKI/kCyM902eKi8al5+PBB2fofHT380lv/o86DzuFn+99/wueEF+0RkN2TbHEhkJ73ksXFeX98
BQRZWh2htiODHdBLDvDVIMNAdAdcAhi25MCrPyIHr8FkthqGEOhSV+4uhIY2/FewrybjIdq3UhCs
ACKHp6oGhlmOYA8bCAjii2ZTPP7u8Stkvxvi9Q/f9J89/e7x64Z4+5cnr58/fqZ+gsxK3/ovXzz7
STSbfvtzDL0HnCBtf6sHCrbd3vMc+FNoStAMifMFmndO86IQabvZqbfEt9lkfI53muQLdMVM7ngC
zODzp//55Du8qjSs/VU+HAO839qtzuGXDfHlMXC80JsBHDlMvYHPj1ri5WQo4NvxQbt12BZTTB4h
iuvxEk1gW+F4YKrfmQzw1ngARuerYPonOIKy8kftcAOgLrAgywDqg1VBAceJGo1vMJoq7BaaAg2/
YI+Mr0W2Ws4wNDj58F/xZJnpVDNkZqMQMxw4sf4UAmAgZxoZT6XGkfbEMMmRidF98Bea+hoMNALf
qoN2X0EVFOH46MD/VuFDLouFyZB2/u4C4cewshGVKX1Qsg12nq9NLdmRklqJFxGzlxSDRZ5P7UCz
jtnuI3pxQlZVZRjkfTZZwfO7KU6E97bOc+SBiCARB0jwPg7GxRwOBPtVSWXv2LvVnZdxAP45cwC4
L+MAgpPnQPDelkxkcBjdefRex4H458IB4b6MA4ieFAdKpEQclHeCHCDOO1n95AAP1KPayYGidp8F
v235P+UF9s/l/zud9pef+f8/0vobStKXrhh9dsXY7A9Yvf5HneN24P/38Mujo8/8/7/U/+81YODF
EJ2H8mxJDum4Q5rEa83JQ0jmxzOugdJFZzkT377568G/v3n5omX73AyK9+rrP4rZNOaKMyu2csrZ
wtUmnxbA/4ROOyAbfIssw+EWLjlPp7ZnDpV/Py5WmKmUqZRf8zkQ9HzxeLHI1tqR5/WK/Nmf0+Rs
6cczmK2mSyf2lbQPWNJdvG2lYV8NRs2rZRwVx26aUnkOh/Lumuin1QX6ra3m+W0Y+dO+avUhRC6u
rXFRHsLIwOA5ASobG/dL3TCo58pmhG4JMdOQ/VYNrmxKPJDyuQtSG1TwWw3SmEStrq6yxdpfSWky
8Itr00xTkHSt+XCNTpIrEB7Ve56XA3vyVF/5F/WQMnV5QMYahpoJr0B2owvIcekCH/XefaKRPm/f
4hmhfB7jty+f/fj8xRuMwWBMvorxFd1G9NFECQY7HY0vpG2vfDAcqx94jzVGn52EWErTgcSIeFQO
pIWf8dSRa06i9AX6kamHJSliaV/lPLYr+G+sXpPkA9JEWFmLUvPFDGNgELtvWwonUu3AgogMBkD9
dvUR8OD97Caf9HkHWAAwbiKGwXFeWxdvV/hL3oH1MUzGapE7PcB3RZ+CdORD8oJKyH+3L11g1RRL
Bt+pfEc8Hg5RD4GhFYzLt7ysKXw/b7pcNbdnibyf5OWmq7X+jfm6dsZh+QnoyyDpu0AAz2rbYkdr
V0Vd84J9p7U09QAU78cyMGa3kmbGr89bOFrZ7G741u4kfoIKmdZnRkHHZwXFNWpR7IghCLaLdINr
oA8EG/rtgNk5w6sVB7BVgKezLtetu0to9Sp7lwOMIvXhYRyLMYZAf+f7jizzK7KtLPr0H1lVhha9
zhKoa8OGN+vec56yskDzwD70ZeYhNV3/gKMV6Tl18b5IsIq/YhJZ7wipL6u1kHFJbuUjRlQDLZs9
gkC4ryuSH1/8x4uXf3vh+SDYqLDrGDp65WwkWVnQR58boXo4cSvgu9QpQ75bVArQcmUdG792Rdt/
G8HCTqmPHhMhuV1/QVFF0XW5vpSjt+IbMokIAzvdkuD9XsL3SQhgdH4bJWStJE6cN7dRkhryww6B
DV8bAhrjmTncsTQSjXLOPskNW/AIb1jAIcPha+qDslGxnC09D0iHdpdAwTLRUWir1/KBOsSYnT1f
PEniZcjOldIdndVc0x1U4/cV/9cQlrkK8NNtsUIFvTgYELKkmE950RIY2YjCcsBetYOGcakl1LvK
YGAS/YqhJCgFnKPBZAVvfsg+5OeSL2mu5iLNWxctD9rrH75pPht/9/i1vGRR0K6zQvx2/GVbYFiJ
6QVdHGQUvwwF2HrLywqPco+0aimMrUx9g/PSSS/0XpL2LZM8n6ftVqfK7bc8/ZQmiqMxBdmfzXMp
/ChKCYfwOsH0RdcYWgSdM9zK14vxktJAQYXWd+PB8m/0IHUgN8RonE+GiLwKdueTjH8UGP9h++60
Hu9razTBtMT2UAbZJFuwIj5CIvmWz2geDhyi6CDPDRVdKuki2g1VQ7q5ATHHWw+IXBR3b9mXENhW
KD8EWE6Dy0nDViAjFHozRfnopHSSPqyKejp7pQWb7apI67HjZfnGhvEPGabW9PC5Y8ja7TWopd70
s8VF0ZuSq6HjhVwWTrG6Q+HcsTysPZ0NEYi75LYjLPNWTUYvzhu2lqwR4eyJKn/inlxl8wOXf2Dl
W9j8GNPD68bdJfFYkE/Xs6iEfrseRkF9sp5aCnxqAZfT0kpGentFb+PreXiLrpRGFbJ87cNelPvd
H956PuwwP4qXk5Fy4i7xDtf339gZj+Or6o0u+om7UxY7wvJr/NQnvCSMg+UWubnFLUMIKIUCMSP9
xeza7yveMCsGzjKVLg0oYBShXiAyIb0NktKqwEl22XUBF9rhz0rrvGEGtWvVsfUWtu7ZpVl+/lSk
Ubv4YbOS4hSrnekbACerVASB8IRwpRZeK7gVdF9tvPiJO0ppkT5RNx1C6/Zz25yDLIvZ8KNOEkZw
83OL08Mvel6xEm8PVzx0bleiMiL+KZ1R1kedqWJ6Vlyqf6tpqRDt45125fv77jJTWjTMNGw/5LRo
9mK61K203+rGDY5ZSpnUqF5LpgyV+Q3xEbKgqjV+YgWcrxr9HcxSw/HcOYk3Zm3FZN+DwYoiFc5I
/FyKCQb8kB2SXG75asXYCVw8+lZ+COL16FTQd2sGoxFHgvmz9AtqdioCiqjI4T2R2uGzrdRr5vuH
+k4+ta5apzR3D42O8mVzVzibSAjBT5ZN9dZWvU5JvUhG3+EHq95hSb3Ds0g+iXyuEqMUPy8wyzb6
h2M+4yF5iq/x2wf89qEecYR6ejHFAKgwZyDBFXxrgm6GS05rjqHg4eVs3sTEPiso+Y/V1bxoRQNm
YFdOyDW6wpHOUrzd71Gd6jgweq3i6xRXY8mJLEF1kYIm4bjD3twelflqMwffm8AMDmvz+1tjBZzK
nlxCw4I7t+q4EP4a6AAMwWn6JDEY/BMWRFNQLzbGY/DPXAmkjh2bwWfwb7UkJrkuTT03iDT4Mk2+
f/z0GV4VluzkQK0cED/KKF4KXiZ8roRvK6UJvInb4wsUtxr+HfEYNbgqnrLCs+iW6aXJbfnTVp7P
NRgp+lr2Xz95/O1fnJtX2zcSWy1Vpe8KLHKqyYOV1OENO0qBW79eEULJaP8rGLPwCiCOSyQqQ4lm
O2sTyxyji+FwHo7wurZUWd3wbVXsa/Ru5GI3rCAvzLveTa9/r8iX41373rcRvReN8KZVN6ROcevN
WeV1qVPLfnW28fLUqeq/PtvmOjVo3Ctxtu0da7QrG4CVK34daGXFzra9lo1AC0r5wOKGNd0KOSIO
QOo1uxGhovKG2GW0bUZ5q6vjzXy6B8Yy+On6jJRXNLAG6sZpS6SabSjUjVIMr5JrStSNoLbIyTWm
Rt2IkOpVMEZEhKSOCEndMu6Uf+KkSVI3ztuXiA1kVldlFiGtm7olXMjtoIaGUv420NxcPbD3s82p
uhFK5pj3KXqi9WO3iWK0mOks2DZdqrgWxNeOMq7iWpB4FVLK+ZSuqk8mNNJtw5Y5BFreE/sXkrci
jjsRyC2JJG+blfKZgaIqvF1Y7Pdi0xDI5QeoXhHyKK7qOdCLNB6Z8CZ4R26sWWtbJJHbGbvviGN/
B569Na69Fb69Fc7dGWuYXUmMZj4MsJQnDutYNhj5JQQDZ9tK56mBRdalDOk5Q2CpGOpOoFoaEZij
lVkbBrV+ie5iNuXiUkoBXxL9SNl3NWRxuqm2NG7qojqo/NHt10cXzRmbDlv/T3YddUzMpp4i/nQx
IRoqtoarq3kqyzScwg3o4BDmpYeX3zOY93f5uvAsPaWpv4ukKflzLKt5gMbLSxCMdJvLFoWBi+w9
OgqXXofsHIym1J1JinYllu//4+LTbO3/tZ2L8S38vx48aHfC+A/t9vFn/69/qf/XG1pgtJ1rCClM
cpQEMqQbzICqLygBMF5msDO2bTjSqtVc/2h2m6GwBsoaXx5/I6Y3LygmArSpTJwE7zOdce5yXQAf
mRfjolZ7PMzmSO2tuAGYEUzHUsAoCio7PcdVwD0uYyPkN1I9lgn0uAlDNcxGACw7n62WgkIqpNvE
YWiITjt83D4m6iOjmc2gDSR+FK4B2CGCboU+eJfPl5wYTB4xHIMTrQLnvMbJw7CVhYxlQEOX4QQw
994EGHRMhzUw0SYoH1u2JP3wYnxOiUMZQ1ojf9Tz/cpFSmEe+B7M9xeXD0TzkQmv4YM8EZ6zuyCQ
na82gnQGDmC/sVIKLq9nqHnMrF0hzjlhGIzxXZ7LkG/qugm3IUzTa8qHxhNYqH3JmcYuZxR9bi2u
MXfb4BLo1LSu9hAWhxaAXlzOhrD/L9YtlFXeyMRyrhN8t4exK1TGO7MG0MMZ7BBrvVUEkJozY3ZM
iwIHPOTjtjG0RYabAWClaid0aepBiDlyd2yDVxlePHRfkOGrXkrN3wxyimnCae1ID9SQyOBiAcwD
dAf2Nh7b6zEFyna3rISgjvXWUKygLKSmnVxn64IihnAtQDj5lEIm6/pOojfY56vBMvQqVQ/GmA1Q
Oai+P1Rfp8A5rZHDms4j/qeD931YqiEgDPnu2/ff0O+G/vZksYBDWuWL+vSKLB9sYzv+8T3a2+6S
Me6vj2GW+s9ffvcEvfF+Md5OwCjqdaTwi/aS4AMzu8nHWu2O1b5AzTNGF8FDyLMxpJ+pNZwDqzxs
QMq9W8D26b96+fTF2++fPnn2Xf+7tz+94n4RUuh0YUrRRO6rhjik7yv+caRedB42xAP9Bn5RvWP1
Gkf+UL/GX1/Sr5Galq/Mz4cPGjUY18sf37768W2f+kP+ign5qJFzxQf8R28i6X0hHea0BwaJ9UNM
XZ2cKWg0Lgz/P2/RxKSnKQsApi+u24fTC7yQqpGn2zwbvANpClpNFxjV+wJDrTTEufGo5RJ4WY6m
DlCoLk5OROdhXfzKj6gKPfwKn+Gjc+31Ki8G+By0VlOEliYjZKT5ET94Cg+4oXodL71l785XY0Bo
OkTTaqlsbc8vyHD8/SHeRE7W3wI+XDzP5kYdANOQLTBTaHp4DCtKc9RTy13HxNWX2Vy+7NQbBOrb
l89ewgZ91f/3J28JkFfuqN7KCppsvfx1b4qgX6dd2Fpn1hzJZ50zPUXyibzcl1PEQMIWWu/H+XVq
LStMjj5JP76FVmPTVPvh9ZOf+ogxe9aWSO317nzVhl7Jf+pqR6iINv0MLTYxmYM0wSf5ppDzTx76
QFjwmcN4cVlkfID8ZHKRVwvkdMgCVAwXM7LMFC+yF+Jm/aHFCrcnP68wDbfM2koKFFbdHLYQZ0qV
StpqtWDnvBvPoTdTKTEKYCSIPUNmiU2dXhEj2ZzkGN4ejQs4MBMDaakR0F94+26FBg6/UM9bLHXT
d2YO6Bua3RQXLR4cS8n5lBgykPYeJSrvx7jon48v5BsOHH3CzYwwm/myAB52NCLbB3RhaSg3Fvug
WpNtpdWhXvRkb9mKzDZTR9jqUl6jhBAPnvIYFXI9q7em+fX5epnPFugxwR23XTS5swoy1+aHXIiR
soWGjCojoSEonYi9hQClyS6jx5ycGFNPNgrv5DdEkai/GH9ANRFnC8L9gaYdXO8j90aZUWERMtAS
92QiImQ1alKrIEv1RDu4v4RhfMiB0qZthTLo33rNyu2ymF332SYmbMj0ywKdXcszCCT1fDUawUSr
HDFOKw3uWU+aQOmQ0ZSMPiM3aUzfqwEzrI2wQ6TnWgvq+Wk4wzNmr4iquuUjPascKf9pLWe4ywr0
ta0eMbLNDGQGjCibg6k6mJaxrs9LdjMu+ILcJqeukYkqEz1Turk/UXs6nAQM4xQrnjkUDJ9iaUWb
CJ0hniQslbLPUYMfGyT5Cg3uMk19mWpTGeTu4MVqOltcZNOxL78q9ETQKQGGfie1QPRG5rKAAvzF
eSP56471kJeQTRmpGy3aEkBybaASh9uhFQyrJfmMB2T7OMxvbLax9f2zl4/fIhtkeZWNSIakkgrH
5cDSoXSepw5bUpeO+KYnDkLtoQ2jzILDr80uhJf2HLcUwrAKm4Mb1r5nz4/bPjBeZF+FtMZ6wyme
iM8oBpgY4GI1WxVMNHkTmE1v7yOqbfIsg6z2RgraW0Znkax/T/P6/p3XVv7gExQGl31L5Is6qVIh
yyShopSMx5kP8vH7fOj4DqZW7os77HRJDHu+eJ+hCmWylv6Z+ZSyvLM2RErhADujYOgsV7u+lQUI
jWT/wwNpibeX8GO+kH6iGQufpHhR0RvzETSHBC0jiTYrXFdNUcwm2KHzfJCpyJBaFYL5zP+xKpbK
7r4VesoiQ+AKOp5bMCoiSiLssN7DvPTSa7sKLTRtXGzK0+tFEIzHoqi3ODFfvbWCabEdJ9VNstc0
6uXhFFuCn3c3mwE+E39FzQYJohGXcw/iFU7qOcjeaP036qrI75Q4PrXaqZf5uJm5NUqe8sy7XlxE
TGaE2qB6bDm2geeHSSSAR3ayYX0n78FU+THcHm2YTj8so54+EMWypXIk9gYZpOH1lF58ae003GkQ
oYhkLvZjOoLcaw3WT4DtqKa227hewEdkGSkCamzXyjAewVx77cpdm0pIGCJntZz5hozhZHtg1Fzv
E5h9VErvI6D9ILpLREO2ZXrtWLRKmce7bJJptS3N74bNb0oC4KPjMDE2b7GtAPpFsasRiM6gsqv5
JPcc+cNSRCliMQMspXIvvsl6aseESel1gM4NKelNkE8/dbgVPTYSfceLleMHo4WNF49jHVlaK77s
5obCkLUNsWXA60jTdnjazW07cW4blR6+tTL0TTd722dmLvdYLktBXXZ2qOFdE0N7zfcrE0SXtWz4
r9u17ERuiaa/LmvZYupu17QbJmantj1D1du1H4Q/2H38bE5669EH0WpuNQm/oxNl4RZ264hnXXur
npRH8NmpK45F7u/pSRjeYdfFUdTkdx5Lw4CVOVrviqwYv8e7ZWNh726nJIl6tfO0RbYa6saIgwzQ
i8G5j2tR3SOdw8W9e+Lwwc7+2hb58kZgh384r5pDvxMPy124Tc52bWDC3q9a5NvoYf1SSTJU0bYS
8iK0lwBwt5K2KrAkc7OF5GXqiUAz5YYswwnkj0ZJxIHflQEiBTxJolG6Xnq8b43U4Y3Xi8PuHW3e
pNZ5KDfBejqFXcybr6tc1+2A/dX1aIvoenaE/tIlpGOjK1I9Jzq/5UUuN3400fhyse6G50dxqKy4
gaNzAbVIc/j+kO9XhnmB+Qz6+XQwQ41NLzm/WHzluflcoL5x9ZW89hq8X9KlVzrmU6nvr/rf/PD6
8IfXj38Ka0NVCcS6ZdKXhQfi8Bi4djehnqMfYl4Z6qDJSoqgPItl1IMtADTKkuNpqssXy6EszgYn
x5gH2MuHuiooyrPfCtVjA13o458E/z6B338GidMF4eqpNJzBZDwPNRDt1jFmIrYGeJ+6hqpANYz7
bB1zT3auEQHSDh92/Id+2sUqE/JKBV2Ye0Rs1tdZvyprVejv/NhfJeyrzlpoHkfsMD3GU1eynntN
OacVEchsuZzk4ZIexlYDTRCsieshphSynV4Ebbq7PnxndbMsnCAad82XInWMMfiI5vzVqBfIjhYq
dOMjxtTrZsTH6FFAA4It+j5fUJJENAI31qnLRQrQ1I77N/ToGw/YZsikB4chSLk3fuVvqasrDQSq
GlG8KZvqBTG6l/kVUUd0+ObrCHVS7bjflGW8wYipHjj2cknrLvzUgur43Ia0J6ZZd5i+aArONHQ1
ouS8QGVhLz0YsU0Tf2UdWW/Psr4aQmE3z6xKyOuSzkaUeG8g2FXKvYavNbHUSHVronQBSd6sg2zm
KkkSID1knmlhJ7zliaityHpzKnMoqdlvmfP9mpayoNsVMZsO3DRMC7Q2m2aTlhA/0gUBAXNU8mRn
ZxQzy8K5WWD2TTFnInU081aIxcQKeqljnOvZirqcOzz6lLyRShPd+iCNcivq1F6qCAOoNkzbajIM
81imdFO3+TEsjR7SOJZmSR/syJKlWs7odJmrO9nORMeLDLuI0DvtCpyYJt+YsVsAxGWGZqYzMcqv
hQT2tajK7ZzgBnIMKAvipoFTKaEO1Sx29JRatCFIAm1UoXSXyDeIZRNjXepLEzOXwkpbS837GKVs
qiHE1MUeJ6VMMzfD8XW/HqA74j/whj0TKl+CZSt7kc3RKHk6Q4X7IJuifp7v3EYTjt4WeO9zv5pK
CoIt3z4O+acriokODB5wbilXuU9VQv8XfGosUbFik6Aii4q/7stftVstO4C0GqjQYDu4ZAdxKTgK
1ia20JeRGh8AUi87DYktTgIBS4F0qfOYNDbLkxsky+oDHxB3mhJNmFZzoL95P5PSMVFJSaNshkx4
SsESCob3w+PZqtD3UUoJHHc+0kQxCszKNmLdJKMxvPFNwTHglb4MYzKeTFZwIBhjnWeFykJISRun
cFq4oeCem43L2YSZytG60ruWK/5AIwUcx4lOWj50L7qh6ji7mM6g0KBBBnRa6IGTKDAuP5QBFnPN
bbAgWwwwks8l9G2ytnM4wCkeL/liDGdtuGC3iJyJ7jAfjJFTbZWQrpPNN6D+zXlJMAvv+twJaOJ6
GMR2aLcaYrzNkuv8EM3fEX9x3QS6uGsyadxv+whwRGeMkCTI8hft45zp29T4ZnsCxY9YAHTWdo91
jMUC4lMTudV3DR+wGe9AMp/SkbaRlv13iSi1eZ7di/XbjsUyVL/FNKp4eQ5e8VzD38uIY+GkYVwU
kN6aj/YKV1B9MLLPND8Ikbcf9oPKhcX2Snylt4zGEkWdjXLv6/LXVcJ1pYCtRUcPx4fu2ps2k3vf
pyU9Dj9QxXSiDYp487enb7/9ixS4ZR0/UGFcP+iEVsc/yFeNeGc5FgxkqejaJFcbIzMR4rSoYQTz
LfwN5KQmZSFjgoQO0YDZ5PQMvHRPd6Y1HKMBKDovpGz9a58eWb6E0c8XiwptDytDvifPP7aSVK3z
xCiNCNsQyXdVW8Mn4lVqOkctF9fXVagmS5R1kasXSx1XrqtzEJJfz7PDi+inHUsI2BwjT/FcaiMY
Dsx+G5JAu7XKABnuGMriY+jBIsRHEfOOUDIo0atF0cwh7a8XM4H9mC5NRu2uDqotsGUSFo2Rjm59
A1raOLPx2bVCXsq0IzGHjdN/6vk/q7u90qQS1XrS58I1M1ZjI0hQkAtZ8aeSM+IUjNzltG3VcPvk
14ItYvWoG+xc97Y6uHzQRbydF793D6o7xepVOoBoX/wNUdqqsy98hqtXUq+2ycgTLai0sWiEc4qK
ZFsLY1UKEYsvi3TMlrL+SjpAMp7XLuFdgb5yINeAkMSXeOx3TBT0Cr2A8kWTdgV5BlnQ0JNtLR07
Yb9CB1bTCUvVwJSTDZNQfnkoYfFeZA90Ali3hSIUqn47bjc77faNGGWFtstE7IFkKvBeY/3nXDkq
tTxTLNYQsX+KtauVSsg2f3eN73Xmp3JE4NEFblDHt5aHzfU6kmUIgWAh5XAWFNCujGfeplYFbP9G
QdFdS1jsnsOoB7uGLFDkjRetTWxM3N9TVfzMxT72m2htMxgHRMc7q7GhORViZN8IM3q0rkBVOmLn
iLtEEh0veOfE70HhxTSbIv2Yrq5SD63anUbvjWmvTeHtza2My0953otf1bdcAfsOh3q80+Qf7jr5
HmKqxkVqjqvOHnsm1fxaEuXa5BDXAX/2p7MpgktVWddaNmZMFUiPTG1ijeE9u02KXTrI0rd9EoO8
TsrGSMtGnrsTe42xyxNPt33J7Zhe4elotzHA1j2RRoQevGswsdDiFqG6G/pppR1l5UX09pfQm2wW
dS3vXZWlodOSu76b7AOD5iqrh1Z99iTG9tcmWzxd33lTZlJVwl0Hd/T6EhUlRpBt6JI0uLCndeGv
erL5Z1IVURSKdEZ7e4YppzqFxPh4N2vGQmWD1IIlyowIX9KobaPlKNVuVGo1OvL8lKxcIy5muY9t
tOW+sJfTNgDbPa5VWZgmKVHa3m7/42JZff58/nz+fP58/nz+fP58/nz+fP58/nz+fP6Uf/4fqf+U
rwDgAQA=
