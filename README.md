# Brunhilde workspace

This is a ros2 iron workspace for Brunhilde, the HCR Lab's quadruped based on the ODRI Solo8 robot.

This workspace is supposed to be used with ros2 iron.

## Dependencies
The original project repositories use the package manager (treep)[https://gitlab.is.tue.mpg.de/amd-clmc/treep]. We try to avoid using it, to reduce dependencies and most importantly to run the project inside a docker container without the need of a personal ssh key to clone repositories.

### Third Party Packages
The following packages are needed:
- (odri_control_interface)[https://github.com/open-dynamic-robot-initiative/odri_control_interface]
- (master_board_sdk)[https://github.com/open-dynamic-robot-initiative/master-board/tree/master/sdk/master_board_sdk]

The following packages are needed as dependencies, but might also be availabe through the ubuntu repositories:
- (Eigen)[https://eigen.tuxfamily.org/index.php?title=Main_Page] (sudo apt install libeigen3-dev)
- (Pybind11)[https://github.com/pybind/pybind11] (sudo apt install pybind11-dev)
- (Doxygen)[https://www.doxygen.nl/index.html] (sudo apt install doxygen)
- (eigenpy)[https://github.com/stack-of-tasks/eigenpy] (sudo apt install ros-iron-eigenpy)
- (yaml-cpp)[https://github.com/jbeder/yaml-cpp] (sudo apt install libyaml-cpp-dev)
- (boost)[https://www.boost.org/] (sudo apt install libboost-all-dev)

## Installation
with all dependencies installed, clone the repository and build the workspace with colcon:
```colcon build```

