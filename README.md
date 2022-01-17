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
- Ros 1 (tested on ROS Noetic)

### Building

Run these commands in the geiflix folder:

```bash
mkdir build
cd build
cmake -DVISUALIZER=True ..
make -j 8
```

You can specify what you want to build with the option `--target [target_name]` of the last command.

### Running

The first thing to do is to source the setup file:

```bash
source devel/setup.bash
```

(replace "bash" with your shell if you are using another one)

Then, you can run each node one by one:

In another terminal, start `roscore`. Then in your previous terminal, you can try:

```bash
rosrun ros_dashboard ros_dashboard.py
rosrun radar radar_node dump_file.txt
# etc.
```

But you can also use the convenient launch files to run them all in the same command, already configured properly:

```bash
cd ../launch_files
./sudo_roslaunch.sh main.launch
```


### Old instructions

#### Building

Clone the project and cd into it.

```bash
mkdir build
cd build
cmake ..
cmake --build . --target radar_node
```

#### Running

In a separate terminal, run the ROS server:

```bash
roscore
```

Then, launch the program that sends the radar messages to the ROS topic `radar_frames`:

```bash
# in the build folder
source devel/setup.bash
devel/lib/radar/radar_node ../function_3-Radar/radar_dump.txt
```

(You can change the argument to be another dump file or even the path to the actual simplyCAN converter to use the real radar)

Add the `-dump <path>` option to dump the raw radar messages to a file in order to replay them.

You can now echo the messages published to the topic with this command:

```bash
# in the build folder
source devel/setup.bash
rostopic echo radar_frames
```

### Visualizer (using ROS)

#### Building

Clone the project and cd into it.

```bash
mkdir build
cd build
cmake -DVISUALIZER=True ..
cmake --build . --target ros_visualizer_node
cmake --build . --target radar_node
```

#### Running

In a separate terminal, run the ROS server:

```bash
roscore
```

Launch the visualizer:

```bash
# in the build folder
source devel/setup.bash
devel/lib/ros_visualizer/ros_visualizer_node
```

Then, in another terminal, launch `radar_node` (the program that publishes the radar messages):

```bash
# in the build folder
source devel/setup.bash
devel/lib/radar/radar_node ../function_3-Radar/radar_dump.txt
```

(You can change the argument to be another dump file or even the path to the actual simplyCAN converter to use the real radar)

Add the `-dump <path>` option to dump the raw radar messages to a file in order to replay them.

#### Commands

You can pan and zoom using the mouse (left click and scroll).

You can toggle displaying the speed vectors, distances and warning regions with the keys S, M and W respectively.

You can change the configuration of the radar with the keys D (max distance), O (objects or clusters), P (power) and R (RCS
threshold).

### Visualizer (not using ROS)

#### Building

Clone the project and cd into it.

```bash
mkdir build
cd build
cmake -DVISUALIZER=True ..
cmake --build . --target radar_visualizer
function_3-Radar/radar_visualizer ../function_3-Radar/radar_dump.txt
```

(You can change the argument to be another dump file or even the path to the actual simplyCAN converter to use the real radar)

Add the `-dump <path>` option to dump the raw radar messages to a file in order to replay them.

#### Commands

You can pan and zoom using the mouse (left click and scroll).

You can toggle displaying the speed vectors, distances and warning regions with the keys S, M and W respectively.

You can change the configuration of the radar with the keys D (max distance), O (objects or clusters), P (power) and R (RCS
threshold).
