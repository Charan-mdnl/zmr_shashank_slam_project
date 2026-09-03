#!/bin/bash
# Autonomous point-to-point navigation over SLAM, fully headless.
#
# Drives the waypoint list in zmr_control/config/waypoints.yaml, saves the map
# built during THAT run, and renders map + driven path to a PNG.
#
#   ./scripts/demo_navigate.sh [seconds] [output_dir]
#
# Output defaults to <workspace>/results/ so nothing lands in /tmp,
# where it would be lost on reboot.
DUR=${1:-420}
ZMR_DEFAULT_OUT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/results"
OUT=${2:-$ZMR_DEFAULT_OUT}
ZMR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ZMR/scripts/setup_env.sh"
bash "$ZMR/scripts/stop_all.sh"
mkdir -p "$OUT"

echo "== ZMR autonomous navigation demo (${DUR}s, headless, no GUI) =="
ros2 launch zmr_control navigate_slam.launch.py \
    record_path:="$OUT/path.csv" > "$OUT/run.log" 2>&1 &
LAUNCH=$!

# map_saver joins the same session and waits to be asked
sleep 10
ros2 run zmr_tools map_saver --ros-args \
    -p filename:="$OUT/map" -p use_sim_time:=true > "$OUT/save.log" 2>&1 &
SAVER=$!

# let the run proceed, then capture the map before tearing anything down
sleep "$((DUR - 20))"
echo
echo "== saving the map built during this run =="
ros2 service call /map_saver/save std_srvs/srv/Trigger > "$OUT/save_call.log" 2>&1
grep -oE "saved [^\"']*" "$OUT/save.log" | tail -1

kill $LAUNCH $SAVER 2>/dev/null
bash "$ZMR/scripts/stop_all.sh"

echo
echo "== waypoint results =="
grep -oE "\[waypoint_navigator\]: (reached|waypoint [0-9]+:|all waypoints).*" \
    "$OUT/run.log" || echo "(none logged)"

if [ -f "$OUT/map.yaml" ]; then
  echo
  ros2 run zmr_tools render_map "$OUT/map.yaml" "$OUT/result.png" \
      "$OUT/path.csv" "ZMR autonomous run" && echo "wrote $OUT/result.png"
fi

python3 - "$OUT/path.csv" <<'PY'
import csv, math, sys
try:
    rows=[r for r in csv.DictReader(open(sys.argv[1])) if r['gt_x'] not in ('nan','')]
except Exception:
    sys.exit(0)
if not rows: sys.exit(0)
errs=[math.dist((float(r['x']),float(r['y'])),(float(r['gt_x']),float(r['gt_y']))) for r in rows]
d=0.0; prev=None
for r in rows:
    p=(float(r['gt_x']),float(r['gt_y']))
    if prev: d+=math.dist(p,prev)
    prev=p
print(f"\n== metrics ==\ndistance travelled : {d:.1f} m")
print(f"SLAM pose error    : mean {sum(errs)/len(errs):.3f} m, max {max(errs):.3f} m")
PY
echo
echo "== done: artifacts in $OUT =="
