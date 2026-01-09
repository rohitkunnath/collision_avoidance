# Panda Robot Arm Project

## Structure
The workspace contains the following packages:
- **panda_bringup**: Main launch files and demo scripts.
- **panda_controller**: configurations for ros2_control.
- **panda_description**: URDF descriptions and Gazebo plugins.
- **panda_moveit**: MoveIt 2 configuration and config files.
- **pymoveit2**: A helper library for easier MoveIt 2 Python interactions.

## Prerequisites
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Gazebo
- MoveIt 2
- Gazebo Fortress

## Installation & Build

1.  **Clone the repository** (if you haven't already):
    ```bash
    cd ~/arm2_ws/src
    # git clone <repo_url> .
    ```

2.  **Install Dependencies**:
    Navigate to the workspace root and install required dependencies.
    ```bash
    cd ~/arm2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the Workspace**:
    ```bash
    colcon build
    ```

4.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```

## Usage

###  Launch the Simulation
This command launches Gazebo, spawns the Panda robot, and starts MoveIt 2 and RViz.
```bash
ros2 launch panda_bringup demo.launch.py
```

