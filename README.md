# rcll_status_board

This package provides ROS nodes to visualize the RoboCup Logistics League (RCLL) gameplay.

## Getting Started

### Prerequisites

The following software is required to run this package:

ROS Indigo (including OpenCV 2.x): (e.g. sudo apt-get install ros-indigo-desktop-full), see [ROS Indigo Install Ubuntu](http://wiki.ros.org/indigo/Installation/Ubuntu) for installation process

Installed RCLL refbox (see [RCLL refbox installation](https://trac.fawkesrobotics.org/wiki/RCLLRefBox/Install))

The following ROS packages are required to run this package:

- [rcll_vis_msgs](https://github.com/ethflo/rcll_vis_msgs)
- [protobuf_comm](https://github.com/ethflo/protobuf_comm)
- [llsf_msgs](https://github.com/ethflo/llsf_msgs)

### Installing

Install the required software, create a catkin workspace (see [create ROS catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)) and download the required ROS packages into the workspaces src folder.

Adjust the llsf_msgs and protobuf_comm packages to your system. Therefore set the REFBOX_ROOT_DIR path variable in the CMakeLists.txt file to your refbox installation path.
```
(in CMakeLists.txt)
...
set(REFBOX_ROOT_DIR /home/username/llsf-refbox)
...
```
Restart your system. For some reasons the refbox header files are just found by ROS build process after a restart of the system.

Build your catkin workspace

```
cd ~/catkin_ws
catkin_make
```

Start the status boards, the argument 'side' is required and expects '0' for side cyan and '1' for side magenta, while the arguments 'show_field' and 'show_team' are optional and expect 'true' or 'false'
```
roslaunch rcll_status_board llsf_interface.launch side:=1 show_field:=true show_team:=false
```

## Versioning

For the versions available, see the [github repository](https://github.com/ethflo/rcll_status_board).

## Authors

* **Florian Eith** - *Initial work* - [Florian Eith](https://github.com/ethflo)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

* Thanks to the developers of the RCLL Refbox
* Inspirated by a concept of the RCLL Team Lueneburg 2016


