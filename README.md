# Brunhilde workspace
This is a ros2 iron workspace for Brunhilde, the HCR Lab's quadruped based on the ODRI Solo8 robot.

## Dependencies
The original project repositories use the package manager [treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep). To make it easier to run our workspace inside a docker container we built the project without treep.

### Third Party Packages
The following packages are needed:
- [odri_control_interface](https://github.com/open-dynamic-robot-initiative/odri_control_interface)
- [master_board_sdk](https://github.com/open-dynamic-robot-initiative/master-board/tree/master/sdk/master_board_sdk)
The master_board_sdk is a subfolder of the [master board repository](https://github.com/open-dynamic-robot-initiative/master-board)

The following packages are needed as dependencies and are availabe through the ubuntu repositories:
- [boost](https://www.boost.org/) (sudo apt install libboost-all-dev)
- [Doxygen](https://www.doxygen.nl/index.html) (sudo apt install doxygen)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (sudo apt install libeigen3-dev)
- [eigenpy](https://github.com/stack-of-tasks/eigenpy) (sudo apt install ros-iron-eigenpy)
- [Pybind11](https://github.com/pybind/pybind11) (sudo apt install pybind11-dev)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (sudo apt install libyaml-cpp-dev)

To install them all at once just run:
```sudo apt install libboost-all-dev doxygen libeigen3-dev ros-iron-eigenpy pybind11-dev libyaml-cpp-dev```


## Installation
with all dependencies installed, clone the repository and build the workspace with colcon:
```colcon build```

