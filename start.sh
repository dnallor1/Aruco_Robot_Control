#!/bin/bash

# Source ROS 2 Humble setup files
echo "Sourcing ROS 2 Humble setup files..."
source /opt/ros/humble/setup.bash

# Set the TurtleBot3 model (change to waffle or waffle_pi if needed)
export TURTLEBOT3_MODEL=burger

# Point to the robot model path for Gazebo simulation
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/

# Launch Gazebo with the empty world (or other simulated environments like house)
echo "Launching TurtleBot3 Gazebo simulation in an empty world..."
ros2 launch turtlebot3_gazebo empty_world.launch.py &

# Launch the custom camera node for ArUco marker detection (replace with your custom node)
echo "Launching custom camera node..."
ros2 run camera_subscriber camera_node &

echo "Startup complete."
