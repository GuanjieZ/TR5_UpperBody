#!/usr/bin/env bash
set -euo pipefail

SESSION="rps_hand_cam"
TMUX_BIN="tmux -L $SESSION"

ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/niic/ros2_ws/install/setup.bash}"
WS_SETUP_2="${WS_SETUP_2:-/home/niic/upper_body_demo/install/setup.bash}"


if [ -n "${TMUX:-}" ]; then
  unset TMUX
fi


ENV_CMD=""
[ -f "$ROS_SETUP" ] && ENV_CMD+="source \"$ROS_SETUP\"; "
[ -f "$WS_SETUP" ]  && ENV_CMD+="source \"$WS_SETUP\"; "
[ -f "$WS_SETUP_2" ]  && ENV_CMD+="source \"$WS_SETUP_2\"; "

$TMUX_BIN has-session -t "$SESSION" 2>/dev/null && $TMUX_BIN kill-session -t "$SESSION"


$TMUX_BIN new-session -d -s "$SESSION" -n rps_hand
$TMUX_BIN send-keys -t "$SESSION:rps_hand" "${ENV_CMD} ros2 run dex_hand rps_hand" C-m


$TMUX_BIN new-window -t "$SESSION:" -n rps_cam
$TMUX_BIN send-keys -t "$SESSION:rps_cam" "${ENV_CMD} ros2 run dex_hand rps_cam" C-m


$TMUX_BIN select-window -t "$SESSION:rps_hand"
exec $TMUX_BIN attach -t "$SESSION"

