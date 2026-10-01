#!/bin/bash
SESSION_NAME="ros2_cluster"

WORKSPACE_SETUP="source install/setup.bash && export ROS_DOMAIN_ID=11 && source /opt/ros/jazzy/setup.bash"

COMMANDS=(
    #"ros2 run create_fundamentals driver.py"
    #"ros2 launch create_fundamentals laser.launch.py serial_port:=/dev/ttyACM0"
    "ros2 run red_fundamentals conversion"
    "ros2 run red_fundamentals movement 1 6 20 50 2.4 1 15 -1.3 4 4"
    "ros2 run red_fundamentals line_detection 30 0.25 0.02 100"
    "ros2 launch red_fundamentals lidar_tf_launch.py"
    "rm gridfast.log ; ros2 run red_fundamentals grid_fast > gridfast.log 2>&1"
    "rm localize.log ; ros2 run red_fundamentals localize > localize.log 2>&1"
    "rm align.log ; ros2 run red_fundamentals align > align.log 2>&1"
    "ros2 run map_and_messages publish_map.py"
    "echo 'Commands here'"
    "watch -n 1 ros2 node list" # Useful utility pane
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

for i in "${!COMMANDS[@]}"; do
    if [ "$i" -gt 0 ]; then
        tmux split-window -t "$SESSION_NAME"
    fi

    tmux send-keys -t "$SESSION_NAME" "$WORKSPACE_SETUP" C-m
    tmux send-keys -t "$SESSION_NAME" "clear" C-m
    tmux send-keys -t "$SESSION_NAME" "${COMMANDS[$i]}" C-m

    tmux select-layout -t "$SESSION_NAME" tiled
done

tmux attach-session -t "$SESSION_NAME"
