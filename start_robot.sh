#!/bin/bash
echo "🚀 Starting Robot Base Layer..."

# 1. Micro-ROS Agent (Remember to press EN button!)
gnome-terminal --tab --title="Micro-ROS (Motors)" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/microros_ws/install/local_setup.bash; ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200; exec bash"

sleep 2

# 2. Lidar Sensor
gnome-terminal --tab --title="Lidar Sensor" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/robot_ws/install/setup.bash; ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB1 frame_id:=laser; exec bash"

sleep 2

# 3. Lidar Odometry (RF2O)
gnome-terminal --tab --title="Lidar Odometry" -- bash -c "source /opt/ros/jazzy/setup.bash; source ~/robot_ws/install/setup.bash; ros2 launch rf2o_laser_odometry rf2o_launch.py; exec bash"

# 4. Static Transforms (Glue & Bridge)
gnome-terminal --tab --title="TF Transforms" -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser & ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id base_footprint; exec bash"

echo "✅ Base Layer Started. Check the Micro-ROS tab and press the EN button on ESP32!"
