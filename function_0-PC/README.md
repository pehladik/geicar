# Embedded laptop

This directory contains the code that will run on the embedded laptop.

## Components

### ros_dashboard

This node allows us to easily send commands to the STM32 (through ROS services -> ros_stm32_can_node -> CAN bus).

### ros_emergency_stop

This node sends an emergency stop command when an obstacle is detected in a zone, which size is configurable with ROS param:

- Longitudinal size: `/trigger_dist_long` 
- Lateral size: `/trigger_dist_lat`

These values can be set using the ros_visualizer (right click).
