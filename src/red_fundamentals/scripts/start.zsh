#!/bin/zsh
SESSION_NAME="ros2_cluster"

WORKSPACE_SETUP="source install/setup.zsh && export ROS_DOMAIN_ID=11 && source /opt/ros/humble/setup.zsh"

COMMANDS=(
    "ros2 run red_fundamentals conversion"
    "ros2 run red_fundamentals movement 1 6 20 50 2.4 1 15 -1.3 4 4"
    "ros2 run red_fundamentals line_detection 30 0.25 0.02 100"
    "ros2 launch red_fundamentals lidar_tf_launch.py"
    "ros2 run map_and_messages publish_map.py"
    "echo 'Commands here'"
    "echo 'Commands here 2'"
    "echo 'Commands here 3'"
)

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Session '$SESSION_NAME' already exists."
    echo "Attaching to existing session..."
    sleep 1
    tmux attach-session -t "$SESSION_NAME"
    exit 0
fi

echo "Creating new tmux session: $SESSION_NAME"

tmux new-session -d -s "$SESSION_NAME"

for i in {1..${#COMMANDS[@]}}; do
    pane_index=$((i - 1))
    
    if [ "$pane_index" -gt 0 ]; then
        tmux split-window -t "$SESSION_NAME"
    fi

    tmux send-keys -t "$SESSION_NAME" "$WORKSPACE_SETUP" C-m
    tmux send-keys -t "$SESSION_NAME" "clear" C-m
    tmux send-keys -t "$SESSION_NAME" "${COMMANDS[$i]}" C-m

    tmux select-layout -t "$SESSION_NAME" tiled
done

tmux attach-session -t "$SESSION_NAME"
