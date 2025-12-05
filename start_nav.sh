#!/bin/bash

# --- CONFIGURATION ---
MAP_PATH="/home/trash/my_room_map.yaml"
# ---------------------

# 1. Run the Base Layer first
./start_robot.sh

sleep 3

echo "📍 Starting Navigation Suite..."

# 2. Nav2 Bringup
gnome-terminal --tab --title="Nav2 System" -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 launch nav2_bringup bringup_launch.py map:=$MAP_PATH autostart:=True; exec bash"

# 3. RViz2
gnome-terminal --tab --title="RViz Visualization" -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run rviz2 rviz2; exec bash"

echo "✅ Navigation Ready!"
echo "👉 1. Go to RViz."
echo "👉 2. Perform '2D Pose Estimate'."
echo "👉 3. In a new terminal, run: python3 ~/robot_ws/navigate_points.py"
