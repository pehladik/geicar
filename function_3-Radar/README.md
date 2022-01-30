# Radar

The radar is a [ARS 408 from Continental Labs](https://conti-engineering.com/components/ars-408/). It sends its data on a CAN bus. You will need to connect it to a CAN to USB converter (SimplyCAN from IXXAT) to receive the messages on the computer.

## Troubleshooting

The connection between the CAN cable and the radar is a bit loose. You might need to manipulate the cable to find a position where it works. Or re-solder the cable.

## Building and running

Follow the instructions on [the main README](../README.md).

All executables that requires access to the CAN converter must be run with `sudo`, and expect one argument: the path to the device. On my computer, it is `/dev/ttyACM0` for the first I connect, then `/dev/ttyACM1` for the second, etc.

Additionnally, you can add `-dump <path>` to dump the messages to a file at `<path>`. You will then be able to replay these messages by replacing the first argument from `/dev/ttyACMx` to `<path>`.

TODO: get rid of this functionnality and use `rosbag` instead.

## Components

### driver

The `driver` subdirectory contains a library that sends and receives the messages from the radar.

Target name: `radar_driver`

TODO: split this library in two: one to manage the CAN to USB converter to receive the messages, and one to actually parse the data.

### radar_msgs

This directory contains the C++ representation of the messages sent by and to the radar. It can convert messages from and to CAN frames.

Target name: `radar_msgs`

### radar_ros_msgs

This directory contains the ROS representation of the messages sent by and to the radar. It can convert messages from and to C++ object.

### ros_radar

The `radar` subdirectory contains the code for a ROS package that fetches the radar messages using the driver library, and sends
them to a ROS topic (`/radar_frames`).

TODO: 

- change topic to `/radar/frames`
- add topic (or services) to configure the radar
- change name to ros_radar for consistency with other ROS nodes

### visualizer

The `visualizer` subdirectory contains the code for an executable that uses the driver library to display the obstacles on the screen.

You can pan and zoom using the mouse (left click and scroll).

You can toggle displaying the speed vectors, distances and warning regions with the keys S, M and W respectively.

You can change the configuration of the radar with the keys D (max distance), O (objects or clusters), P (power) and R (RCS threshold).

Target name: `radar_visualizer`

### ros_visualizer

The ros_visualizer executable is similar to the visualizer, except that it uses the ROS messages instead of directly receiving the radar data from the CAN converter. This allows using the radar in several nodes at the same time.

It also shows the measure from the ultrasonic sensor.

The commands are the same as the visualizer, except you cannot yet send configuration to the radar (TODO 😉), so D, O, P and R are ignored.

You can change the size of the warning region by right-clicking. It is set as a ROS parameter, and this size is used by the emergency stop node.

TODO: move the visualizers in another directory, since it is not linked only to the radar anymore.

### Tests

There are some unit tests for the radar driver, using the [Catch library](https://github.com/catchorg/Catch2).

Target name: `radar_test`
