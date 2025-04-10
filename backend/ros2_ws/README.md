The ROS2 workspace for LIO-SAM and Gazebo simulation.

## Installation

### LIO-SAM

See src/LIO-SAM/README.md for installation instructions.

### Gazebo

Source video is [here](https://www.youtube.com/watch?v=NNR9RUNz5Pg).

Run the following commands in the terminal:

```bash
sudo apt install gazebo
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros-pkgs
```

Run `gazebo` to check if Gazebo is installed correctly.

Paste the models from `backend/models/` into `~/.gazebo/models/`.

Finally, run `colcon build` in the `backend/ros2_ws/` directory.

## Running

In one terminal, run:

```bash
source /opt/ros/humble/setup.bash
ros2 launch robot_gazebo robot_sim.launch.py
```

In another terminal, run:

```bash
source /opt/ros/humble/setup.bash
ros2 launch lio_sam run.launch.py
```
