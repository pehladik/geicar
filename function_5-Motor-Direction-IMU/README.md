# Actuators and sensors

This directory contains the code linked to the sensors and the actuators connected to the STM32.

## Components

### STM32TricycleProject

This directory contains the project running on the STM32. Check the README inside.

### ros_stm32_can

This ROS node receive the CAN messages from the STM32 and transmit them on ROS topics:

- `/stm32/init` (bool): one message is transmitted when the STM32 has started and is ready to accept messages.
- `/stm32/ultrasound` (float): the measure from the ultrasonic sensor (in meters)

It also advertises ROS services to allow you sending commands to the STM32:

- `/stm32/emergency_stop` (bool): activate or deactivate emergency stop 
- `/stm32/brakes` (bool): activate or deactivate the brakes 
- `/stm32/motor` (int): the power of the motor (from 0 to 100)
- `/stm32/steering` (int): the steering angle (from 0 to 120)

### stm32_ros_msgs

This package contains the ROS config files for the services (i.e. the commands to the tricycle)
