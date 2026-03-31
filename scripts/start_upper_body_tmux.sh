#!/usr/bin/env bash
set -euo pipefail

SESSION="upper_body"
TMUX_BIN="tmux -L $SESSION"      
AUTO_ENABLE="${AUTO_ENABLE:-false}"
ENABLE_DELAY="${ENABLE_DELAY:-2}"


if [ -n "${TMUX:-}" ]; then
  unset TMUX
fi


ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/niic/ros2_ws/install/setup.bash}"
WS_SETUP_2="${WS_SETUP_2:-/home/niic/upper_body_demo/install/setup.bash}"
ENV_CMD=""
[ -f "$ROS_SETUP" ] && ENV_CMD+="source \"$ROS_SETUP\"; "
[ -f "$WS_SETUP" ]  && ENV_CMD+="source \"$WS_SETUP\"; "

[ -f "$WS_SETUP_2" ]  && ENV_CMD+="source \"$WS_SETUP_2\"; "


$TMUX_BIN has-session -t "$SESSION" 2>/dev/null && $TMUX_BIN kill-session -t "$SESSION"


$TMUX_BIN new-session -d -s "$SESSION" -n upper_body_demo


$TMUX_BIN new-window  -t "$SESSION:" -n enable


$TMUX_BIN new-window  -t "$SESSION:" -n dex_hand


$TMUX_BIN send-keys -t "$SESSION:upper_body_demo" "${ENV_CMD} ros2 run upper_body_demo teleop" C-m
$TMUX_BIN send-keys -t "$SESSION:enable"          "${ENV_CMD} ros2 run upper_body_demo enable" C-m
$TMUX_BIN send-keys -t "$SESSION:dex_hand"        "${ENV_CMD} ros2 run dex_hand five_one_trigger" C-m


if [ "$AUTO_ENABLE" = "true" ]; then
  $TMUX_BIN send-keys -t "$SESSION:enable" "sleep ${ENABLE_DELAY}" C-m
  $TMUX_BIN send-keys -t "$SESSION:enable" -l "l"
  # $TMUX_BIN send-keys -t "$SESSION:enable" C-m
fi


$TMUX_BIN select-window -t "$SESSION:upper_body_demo"
exec $TMUX_BIN attach -t "$SESSION"

