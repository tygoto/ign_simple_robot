# ign_simple_robot

## Install

```bash
sudo apt install ignition-fortress ros-galactic-ros-ign

mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/tygoto/ign_simple_robot.git
cd ..
colcon build --symlink-install
source /opt/ros/galactic/setup.bash
. ./install/local_setup.bash
```

## Usage

```bash
ros2 launch ign_simple_robot robot.launch.py
```