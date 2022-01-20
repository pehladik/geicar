# GeiBike Project

The GeiBike project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous vehicule, base on a 3-wheels bike, in order to carry out different missions. Several projects are exposed on the [official website](https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiBike. The present code as well as the documentation is the result of the combination of the various projects carried out by:

- Muttley team (2021-2022):
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

## Resources

The slides for the sprint reviews and the posters can be found at the following links:

- [Muttley team (2021-2022)](https://drive.google.com/drive/folders/1gfQWTaghTNbTpblPRNZOpv6hvGePmNHc?usp=sharing)

## Quick User Guide

### Requirements

- Linux (or WSL, but with limited functionalities and more complex setup)
- Any C++ compiler (`sudo apt install build-essential`)
- CMake
- Ros 1 (tested on ROS Noetic)

### Building

Run these commands in the geiflix folder:

```bash
mkdir build
cd build
cmake -DVISUALIZER=True ..
make -j 8
```

You can specify what you want to build by appending the name of a target to the last command. 

For ROS nodes, the targets names are the name of their subdirectory + `_node`. For example, the `ros_visualizer` directory contains a node named `ros_visualizer_node`. You can build it using `make -j 8 ros_visualizer_node`

For other executables, the names are not really unified (TODO 😅). Check in the corresponding `CMakeLists.txt` file, or just build everything. 

### Running

#### ROS nodes

The first thing to do is to source the setup file:

```bash
source devel/setup.bash
```

(replace "bash" with your shell if you are using another one)

Then, you can run each node one by one:

In another terminal, start `roscore`. Then in your previous terminal, you can try:

```bash
rosrun ros_dashboard ros_dashboard.py
rosrun radar radar_node some_dump_file.txt
# etc.
```

But you can also use the convenient launch files to run them all in the same command, already configured properly:

```bash
cd ../launch_files
./sudo_roslaunch.sh main.launch
```

#### Non-ROS executables

For other executables, you will need to launch them manually.

The actual executable file is located in the build folder, at the same relative path as the `CMakeLists.txt` file in the project. For example, the radar_visualizer executable is defined in the [`function_3-Radar/visualizer/CMakeLists.txt`](function_3-Radar/visualizer/CMakeLists.txt) file. The path to the executable file is then `build/function_3-Radar/visualizer/radar_visualizer`.

### Troubleshooting

#### WSL

You will not be able to build nor run the visualizer in WSL without installing some graphical libraries on Ubuntu and setting up an X11 server on Windows. You will find tutorials to do so on Internet.

However, we strongly recommend using a computer running on linux, and more precisely the last Ubuntu version supported by ROS.

### Subsequent work and improvements to do

Here is listed ideas to improve the current project. Other ideas specific to a part of the project are listed in the subdirectory's README files. 

#### Catkin

We did not know about ROS at the beginning of the project, so we began with a regular CMake setup. We then added some ROS nodes, and thought it would be simpler to continue using plain CMake. It could be possible and preferable to build ROS **and** non-ROS package with catkin.

Additionnally, we created one package per node, but it is quite useless. It is possible to create several nodes inside one package.

#### Real tricycle

When the radar will be mounted on the real tricycle, you should send the speed and the steering angle so that it can deduce the speed of the objects and better track them.
