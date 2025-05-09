# Autonomous Search and Rescue Robot 

This project simulates an **Autonomous Search and Rescue Robot** using ROS (Robot Operating System), integrating **face detection**, **intelligent control**, and **mapping capabilities** to navigate an environment, locate individuals, and display real-time maps.

## Project Overview

The system is designed for autonomous operation in search and rescue scenarios. It utilizes a TurtleBot3 in a simulated Gazebo environment to:
- Navigate autonomously using SLAM and path planning
- Detect human faces via integrated computer vision
- Display a dynamically built map of the environment
- Respond to hand gestures for manual control

## Project Objectives

- Apply ROS concepts and packages in a practical simulation
- Explore and use Gazebo and RViz platforms
- Integrate computer vision techniques (face and hand detection)
- Simulate intelligent autonomous robot behavior

## Technologies Used

- **ROS Noetic** on Ubuntu 20.04
- **Gazebo** for simulation
- **RViz** for visualization
- **OpenCV** and `face-recognition` for face detection
- **MediaPipe** for hand gesture recognition
- **GMapping** for SLAM
- **Python** & **Blender** for 3D frame creation

## Setup Instructions

### Prerequisites

Install the following:

- ROS Noetic
- TurtleBot3 packages
- OpenCV (`pip install opencv-python`)
- face-recognition (`pip install face-recognition`)
- mediapipe (`pip install mediapipe`)
- gtts (`pip install gtts`)
- Blender (for mesh generation)
- SLAM tools:  
  ```bash
  sudo apt-get install ros-noetic-openslam-gmapping

### Workspace Setup

```bash
# Clone the repository
git clone https://github.com/yourusername/Autonomous-Search-and-Rescue-Robot.git
cd Autonomous-Search-and-Rescue-Robot

# Initialize your ROS workspace
mkdir -p ~/catkin_ws/src
cp -r * ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Export TurtleBot3 model
export TURTLEBOT3_MODEL=burger
```

### Launch Simulation

```bash
# Launch Gazebo simulation
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Launch SLAM
roslaunch turtlebot3_slam turtlebot3_slam.launch

# Launch face recognition
roslaunch face_recognition face_recognition.launch

# Launch hand gesture control
roslaunch hand_control hand_control.launch
