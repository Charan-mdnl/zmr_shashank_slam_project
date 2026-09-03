#!/bin/bash
# Stop every ZMR node and verify nothing survives.
#
# Two traps this avoids:
#  * Linux truncates a process's `comm` to 15 chars, so `pkill -x` on a longer
#    executable name silently matches nothing.
#  * `pkill -f` matches ANY command line containing the pattern - including the
#    shell that invoked this script, if the caller happened to mention it. So
#    we resolve PIDs ourselves and exclude this process and all its ancestors.

PATTERN='sim_node|slam_toolbox|robot_state_publisher|lifecycle_manager|obstacle_avoidance|waypoint_navigator|teleop_key|path_recorder|map_saver|render_map|zmr_sim|zmr_control|zmr_tools|navigate_slam|teleop_slam'

ancestors() {                      # this PID and every parent up to init
  local p=$$
  while [ -n "$p" ] && [ "$p" -gt 1 ]; do
    echo "$p"
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  done
}

targets() {
  local skip
  skip=$(ancestors | tr '\n' '|')
  pgrep -f "$PATTERN" 2>/dev/null | while read -r pid; do
    case "|$skip" in *"|$pid|"*) continue ;; esac
    echo "$pid"
  done
}

for sig in TERM TERM KILL; do
  pids=$(targets)
  [ -z "$pids" ] && break
  kill -"$sig" $pids 2>/dev/null
  sleep 1
done

left=$(targets)
if [ -n "$left" ]; then
  echo "WARNING: ZMR processes still running after stop_all:"
  ps -o pid=,args= -p $(echo "$left" | tr '\n' ',' | sed 's/,$//') 2>/dev/null
  exit 1
fi
exit 0
