# Overview of the Nucleo software architecture

## Connexion

The Nucleo board controls the vehicle's engines and steering, as well as the battery level. The ports used to connect the board are described in the diagram of the [electronic board](../../hardware/electronic/electronic_overview.md)

### Sensors
| Port | Configuration | Remap | Description                     |
|------|---------------|-------|---------------------------------|
| PA0  | Analog Input  | ADC1 IN0 |Battery Level             |
| PA1  | Analog Input  | ADC1 IN1 | Steering wheel angle     |
| PB10 | AF Output PP  | TIM2 CH3 | Position left motor |
| PB8 | AF Output PP  | TIM4 CH3 | Position right motor   |

### Left wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC10 | Output PP 10MHz| -        | Enable left motor         |
| PA8  | AF Output PP  | TIM1 CH1 |  Control left motor (IN1) |
| PA7  | AF Output PP  | TIM1 CH1N|  Control left motor (IN2) |
| PA4  | Analog Input  | ADC1 IN4 | Current left motor        |

### Right wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC11 | Output PP 10MHz| -        | Enable right motor         |
| PA9  | AF Output PP  | TIM1 CH2 |  Control right motor (IN1) |
| PB0  | AF Output PP  | TIM1 CH2N|  Control right motor (IN2) |
| PA5  | Analog Input  | ADC1 IN5 | Current right motor        |

### Steering wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC12 | Output PP 10MHz| -       | Enable steering motor         |
| PA10 | AF Output PP  | TIM1 CH3 |  Control steering motor (IN1) |
| PB1  | AF Output PP  | TIM1 CH3N|  Control steering motor (IN2) |
| PC0  | Analog Input  | ADC1 IN10 | Current steering motor      |

### CAN bus
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PA11 | AF Output PP  | CAN RX | CAN Rx bus                  |
| PA12 | AF Output PP  | CAN TX | CN TX bus                   |

The full description of the port configuration is provided in the file [electronic board](../../../nucleo/voiture-elec.txt).

## How to manage sensors and actuators

All voltage measurements (battery, steering angle, current left rear motor, current right rear motor and current front motor)  are made in continuous mode by the ADC. The measurements are stored in a circular buffer in the DMA.

### Battery level

The battery level is obtained by calling the function `u8 get_battery_level(void)`. 

### Steering angle

The steering wheel angle is obtained by calling the function `s8 get_steering_angle(void)`. The value returned an estimate of the steering wheel angle in degrees.

The steering wheel angle is set by calling the `void set_steering_angle(int angle)` function. The value `angle` must be between -45 and 45.

### Motor Commands

All engines are speed controlled, even the steering wheel engine. 

The command of a motor is encoded by 8-bit data. The bit 7 is used to enadble or disable the motor and bits 6-0 are used to code the value of the command. The values are between 0 and 100 with 50 to stop the motor. To avoid problems some software thresholds were implemented.

**Warning:** All motors operate in open loop.

The motor control of the wheels is done by calling the function `wheels_set_speed()`. This function is periodically called in the main file.


## How to use the CAN bus

Sending a message on the CAN bus is done by calling the function `void CAN_Send (unint8_t* data, unint32_t id)`. 

The reception of a message is done in the callback function of the CAN interruption `HAL_CAN_RxCpltCallback` in file `can.c`. Motor controls (wheel and steering) are shared via global variables. These variables are updated on receipt of the CAN message. The actuators are updated periodically in the main file.

## Scheduling

The position of the wheels is updated by interruption on the fronts.

The speed is calculated under interruption at each edge of the position sensor. The clock used to calculate the speed is the timer. This timer is reset at each interruption and 

The speed of the rear wheel and steering wheel position are updated under interruption when orders are received on te CAN bus.

The sending of information on wheel position and speed, steering wheel position and battery level is done under interruption of the Systick.



