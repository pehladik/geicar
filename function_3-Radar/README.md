# Radar

## Driver

The `driver` subdirectory contains the code for a library that receives and parses the messages from the radar.

TODO: split this library in two: one to manage the CAN to USB converter to receive the messages, and one to actually parse the
data.

## Radar

The `radar` subdirectory contains the code for a ROS package that fetches the radar messages using the driver library, and sends
them to a ROS topic (`/radar_frames`).

## Visualizer

The `visualizer` subdirectory contains the code for an executable that uses the driver library to display the obstacle on the
screen.

TODO: Subscribe to the ROS topic to get the messages.

## Logger

The `logger` subdirectory contains the code for an executable that just logs all the messages received by the radar.

