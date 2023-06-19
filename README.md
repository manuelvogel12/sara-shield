# SaRA-shield
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This package provides safety for human-robot interaction using reachability analysis.
We use [SaRA](https://github.com/Sven-Schepp/SaRA) to calculate the reachable sets of humans and robots.
The SaRA shield additionally provides the necessary trajectory control to stop the robot before any collision with the human could occur.

# Installation
### Clone the repo with submodules
```
git clone --recurse-submodules git@github.com:JakobThumm/sara-shield.git
```
### Install the shield [C++ only]
The installation requires `gcc`, `c++>=17`, and `Eigen3` version 3.4 (download it here: https://eigen.tuxfamily.org/index.php?title=Main_Page).
Set the path to your eigen3 installation to this env variable, e.g.,
```
export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"
```
Install gtest
```
sudo apt-get install libgtest-dev
```
```
cd safety_shield
mkdir build && cd build
cmake ..
make -j 4
```
### Install the shield [With Python bindings]
```
pip install -r requirements.txt
python setup.py install
```
### Run the python binding tests
```
pytest safety_shield/tests
```

# Run Safety-shield with Xbot2
Start Ros with
```
roscore
```
Run the concert gazebo project with
```
cd ~/concert_ws
source ./src/safe_rl_manipulators/src/catkin_ws/devel/setup.bash
source setup.bash

mon launch concert_gazebo concert.launch realsense:=true velodyne:=true
```
**After** Gazebo runs, start Xbot2-Gui with
```
xbot2-gui
```
This should open a large window with status "Running" in the top left corner. If it opens a small window with the status "Inactive" instead, close and rerun the command above.

**Open Rviz** for visualization (with the command ```rviz```) and **start the xbot plugin** in the xbot2 gui. The visualization topics of sara-shield are named ```/human_joint_marker_array``` and ```/robot_joint_marker_array``` and can be visualized by adding them in rviz.

## Notes
1. Sara-shield should always be built in *Release* mode, since the timesteps can take too long otherwise, resulting in crashes (```safety limit violation detected ...```).
2. The plugin is not supposed to be paused and restarted. Instead it is recommended to stop gazebo and start it again. (Rviz and Xbot2-gui can stay open)
