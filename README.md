# ArUco Marker Robot Control with ROS2

This project demonstrates a ROS2-based robot control system where the robot's movement is controlled based on ArUco markers detected in the camera feed. It integrates a USB camera driver, nodes for marker detection, and logic for sending movement commands to the robot. The system also supports simulation with TurtleBot3 in Gazebo.

## Features

- **Camera Integration**: Uses the `usb_cam` package to capture the video feed from a laptop or external camera.
- **ArUco Marker Detection**: Detects markers in the video feed and extracts their IDs and positions.
- **Robot Control**: Moves the robot based on the detected ArUco marker's position (see `camera_subscriber.py`).
- **Gazebo Simulation**: Provides a simulated environment for TurtleBot3 robot control.
- **Custom Camera Node**: Processes video feed to detect ArUco markers and control robot movement in real-time.

## Team

- **Ruman Mondal** (Email: rumam.mondal@student.put.poznan.pl)
- **Theodore Rolland** (Email: rolland.theodore@student.put.poznan.pl)

## Repository

GitHub: [dnallor1/Aruco_Robot_Control](https://github.com/dnallor1/Aruco_Robot_Control)

## Requirements

- TurtleBot3:: For simulation and robot control.
- Gazebo: For the TurtleBot3 simulation environment.
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
   sudo apt install python3-opencv
   sudo apt install ros-<ros2-distro>-turtlebot3-gazebo
   ```

## Usage

### 1. Launch the TurtleBot3 Gazebo Simulation
Launch the Gazebo simulation environment with the TurtleBot3 robot:
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. Launch the Camera Driver
Start the camera driver to capture the video feed:
```bash
ros2 launch usb_cam usb_cam.launch.py
```

### 3. Run the Custom Node
Run the custom node to process the video feed, detect ArUco markers, and control the robot's movement:
```bash
ros2 run camera_subscriber camera_node
```

### 4. Run All Nodes with a Shell Script
Use the `start.sh` script to launch all necessary nodes (Gazebo and camera) together:
```bash
./start.sh
```

Ensure the script is executable:
```bash
chmod +x start.sh
```
## Video Demonstration
A recorded video demonstrating how the robot is controlled based on ArUco marker detection is available. Watch it below to see the system in action: Watch the demonstration video on YouTube
