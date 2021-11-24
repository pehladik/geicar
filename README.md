# GeiBike Project

The GeiBike project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous vehicule, base on a 3-wheels bike, in order to carry out different missions. Several projects are exposed on the [official website](https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiBike. The present code as well as the documentation is the result of the combination of the various projects carried out by:

* Matis Delcourt
* Nicolas Fercoq
* Marcos Frances
* Mathilde Ibled
* Yiannis Manzo
* Yohan Simard

The platform is (or was) developped and maintained by :

* LOMBARD Emmanuel
* MARTIN José
* BOUBACAR ALZOUMA Abdou Rahamane
* DI MERCURIO Sébastien

The projects are (or were) surpervised by:

* CHANTHERY Elodie
* AURIOL Guillaume

## Quick User Guide

### Requirements

- Linux or WSL
- Any C++ compiler (`sudo apt install build-essential`)
- CMake

### Visualizer

#### Building

Clone the project and cd into it.

```bash
mkdir build
cd build
cmake ..
cmake --build . --target radar_visualizer
function_3-Radar/radar_visualizer <path/to/dump_file_or_simplyCAN_device>
```

Add the `-dump <path>` option to dump the raw radar messages to a file in order to replay them.

#### Commands

You can pan and zoom using the mouse (left click and scroll).

You can toggle displaying the speed vectors, distances and warning regions with the keys S, M and W respectively.

You can change the configuration of the radar with the keys D (max distance), O (objects or clusters), P (power) and R (RCS
threshold).

### Ros node

#### Building

Clone the project and cd into it.

```bash
mkdir build
cd build
cmake ..
cmake --build . --target radar_ros_node

# In separate terminals:
roscore
roslaunch rosbridge_server rosbridge_websocket.launch
devel/lib/radar_ros/radar_ros_node <path/to/dump_file_or_simplyCAN_device>
```

Add the `-dump <path>` option to dump the raw radar messages to a file in order to replay them.

