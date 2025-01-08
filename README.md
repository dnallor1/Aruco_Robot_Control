# ArUco Marker Robot Control with ROS2

This project demonstrates a ROS2-based robot control system. The robot's movement is controlled based on ArUco markers detected in the camera feed. It integrates a USB camera driver, nodes for marker detection, and logic for sending movement commands to the robot.

## Features

- **Camera Integration**: Uses the `usb_cam` package to capture the video feed from a laptop or external camera.
- **ArUco Marker Detection**: Detects markers in the video feed and extracts their IDs and positions.
- **Robot Control**: Moves the robot based on the detected ArUco marker's position (see `camera_subscriber.py`).
- **Turtlesim Simulation**: Provides a simple robot simulation environment for testing purposes.
- **Gazebo Simulation**: Simulates the robot's environment for testing and visualization.

## Team

- **Ruman Mondal** (Email: rumam.mondal@student.put.poznan.pl)
- **Theodore Rolland** (Email: rolland.theodore@student.put.poznan.pl)

## Repository

GitHub: [dnallor1/Aruco_Robot_Control](https://github.com/dnallor1/Aruco_Robot_Control)

## Requirements

- turtlesim: For testing robot movements in a simple simulated environment.
- Gazebo: For advanced robot simulations and testing.
- `usb_cam` ROS2 package: To capture video feed from a USB camera.
- OpenCV: For ArUco marker detection in the camera feed.

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/dnallor1/Aruco_Robot_Control.git
   cd Aruco_Robot_Control
   ```

2. Build the ROS2 workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

3. Install dependencies:
   ```bash
   sudo apt install ros-<ros2-distro>-usb-cam
   sudo apt install ros-<ros2-distro>-gazebo-ros
   sudo apt install python3-opencv
   sudo apt install ros-<ros2-distro>-turtlesim
   ```

## Usage

### 1. Launch the Camera Driver
Start the camera driver to capture the video feed:
```bash
ros2 launch usb_cam usb_cam.launch.py
```

### 2. Run the Custom Node
Run the node to process the video feed and detect ArUco markers:
```bash
ros2 run aruco_marker_robot_control camera_node
```

### 3. Launch the Robot in Gazebo
Start the Gazebo simulation environment:
```bash
ros2 launch aruco_marker_robot_control robot_sim.launch.py
```

### 4. Run turtlesim for Simulation
Run the turtlesim simulation::
```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

### 5. Run All Nodes with a Shell Script
Use the `start.sh` script to launch all nodes together:
```bash
./start.sh
```
Ensure the script is executable:
```bash
chmod +x start.sh
```
