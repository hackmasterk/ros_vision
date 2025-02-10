# ROS2 Hand Gesture Control for Create3 Robots

This repository contains a ROS2-based project that allows you to control a Create3 robot using hand gestures. The system uses a camera to capture hand gestures, processes them using MediaPipe, and translates them into movement commands for the robot. This README provides an overview of the project, its features, and instructions on how to set it up and use it.

---

## Table of Contents
1. [Overview](#overview)
2. [Features](#features)
3. [Prerequisites](#prerequisites)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Customization](#customization)
7. [Contributing](#contributing)
8. [License](#license)

---

## Overview

This project leverages ROS2 (Robot Operating System 2) to create a hand gesture-based control system for robots. It uses the following components:
- **MediaPipe**: For hand gesture recognition.
- **OpenCV**: For image processing and capturing video frames.
- **ROS2**: For communication between nodes (e.g., image processing, gesture recognition, and robot control).
- **Create3 Robot**: The target robot platform for movement control.

The system works as follows:
1. A camera captures video frames of hand gestures.
2. The `image_publisher` node publishes these frames to a ROS2 topic.
3. The `hands` node processes the frames using MediaPipe to detect hand landmarks and determine gestures.
4. Based on the detected gestures, the `move` node sends movement commands (e.g., move forward, rotate) to the Create3 robot.

---

## Features

### 1. **Hand Gesture Recognition**
   - Detects hand landmarks (e.g., thumb, index finger, middle finger) using MediaPipe.
   - Recognizes specific gestures (e.g., thumb and index finger together) to trigger actions.

### 2. **Robot Control**
   - **Move Forward**: The robot moves straight.
   - **Rotate**: The robot rotates in place.
   - **Draw a Circle**: The robot moves in a circular path.
   - **Draw a Polygon**: The robot moves in a polygonal path.
   - **Stop**: The robot stops all movement.

### 3. **Teleportation**
   - Teleports the robot to a predefined location (e.g., center of the workspace).

### 4. **Canvas Clearing**
   - Clears the robot's workspace (if supported by the simulation environment).

### 5. **Real-Time Feedback**
   - Logs messages to the console for debugging and monitoring.

---

## Prerequisites

Before using this project, ensure you have the following installed:

1. **ROS2 (Humble or Foxy)**: Follow the official [ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html).
2. **Python 3.8+**: Required for running the ROS2 nodes.
3. **OpenCV**: Install via pip:
   ```bash
   pip install opencv-python
   ```
4. **MediaPipe**: Install via pip:
   ```bash
   pip install mediapipe
   ```
5. **Create3 Simulation**: Set up the Create3 simulation environment. Refer to the [Create3 documentation](https://github.com/iRobotEducation/create3_sim).

---

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/hackmasterk/ros_vision.git
   cd ros_vision
   ```

2. **Build the ROS2 Workspace**:
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **Install Dependencies**:
   Ensure all Python dependencies are installed:
   ```bash
   pip install -r requirements.txt
   ```

---

## Usage

### 1. **Launch the Simulation**
   Start the Create3 simulation environment:
   ```bash
   ros2 launch create3_sim create3_house.launch.py
   ```

### 2. **Run the Image Publisher**
   Start the `image_publisher` node to capture and publish video frames:
   ```bash
   ros2 run motion image_publisher
   ```

### 3. **Run the Hand Gesture Recognition**
   Start the `hands` node to process video frames and detect gestures:
   ```bash
   ros2 run motion hands
   ```

### 4. **Run the Movement Controller**
   Start the `move` node to control the robot based on detected gestures:
   ```bash
   ros2 run motion move
   ```

### 5. **Perform Gestures**
   Use the following gestures to control the robot:
   - **Thumb and Index Finger Together**: Draw a circle.
   - **Thumb and Middle Finger Together**: Draw a polygon.
   - **Thumb and Ring Finger Together**: Rotate.
   - **Thumb and Pinky Finger Together**: Clear the canvas.
   - **All Fingers Together**: Teleport to the center.
   - **Thumb Away from Other Fingers**: Move forward.
   - **Back of Hand Facing Camera**: Stop.

---


### Key Files:
- **`hands.py`**: Processes video frames, detects hand landmarks, and publishes gesture data. (from aisd_vision package)
- **`image_publisher.py`**: Captures video frames from the camera and publishes them to a ROS2 topic. (from aisd_vision package)
- **`move.py`**: Subscribes to gesture data and sends movement commands to the robot. (from aisd_motion)

---

## Customization

### 1. **Modify Gesture Actions**
   Edit the `get_action_by_hand` function in `move.py` to change the actions associated with specific gestures.

### 2. **Adjust Movement Parameters**
   Modify the `Twist` messages in `move.py` to change the robot's speed, rotation angle, or movement duration.

### 3. **Add New Gestures**
   Extend the `get_action_by_hand` function to recognize additional gestures and trigger new actions.

---

## Contributing

Contributions are welcome! If you'd like to contribute, please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Submit a pull request with a detailed description of your changes.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments
- **MediaPipe**: For providing an excellent hand gesture recognition solution.
- **ROS2 Community**: For their extensive documentation and support.
- **iRobot**: For the Create3 robot platform and simulation environment.

---

For any questions or issues, please open an issue on GitHub or contact the maintainers. Happy coding! 🚀
