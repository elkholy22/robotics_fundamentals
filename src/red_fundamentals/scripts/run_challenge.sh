#!/usr/bin/env bash
set -e

WS="$HOME/uni/Robotics/red_ws"
GOLD="${1:-$WS/src/red_fundamentals/data/gold.txt}"
PICKUP="${2:-$WS/src/red_fundamentals/data/pickup.txt}"

echo "Workspace : $WS"
echo "Gold file : $GOLD"
echo "Pickup    : $PICKUP"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=11
source "$WS/install/setup.bash"

ros2 launch red_fundamentals challenge.launch.py \
    gold_file:="$GOLD" \
    pickup_file:="$PICKUP"