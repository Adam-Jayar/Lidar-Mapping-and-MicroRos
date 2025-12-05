#!/bin/bash

# 1. Run the Base Layer first
./start_robot.sh

sleep 3

echo "🗺️ Starting Mapping Suite..."

# 2. SLAM Toolbox
gnome-terminal --tab --title="SLAM Toolbox" -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 launch slam_toolbox online_async_launch.py; exec bash"

# 3. RViz2
gnome-terminal --tab --title="RViz Visualization" -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run rviz2 rviz2; exec bash"

# 4. Teleop (Keyboard)
gnome-terminal --tab --title="Teleop Control" -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"

echo "✅ Mapping Ready! Use the Teleop tab to drive."
