# CAN Bus

## Overview
The CAN network provides communication between the various ECUs in the system. The code available is based on a set of pre-defined messages with their own identifier and payload.

![CAN Bus](./figures/hw_archi.jpg)

## CAN configuration


The CAN bus is configured with a bitrate of 400 Kbit/s.

### Clock Tree

* f_APB1 = 24 MHz

### CAN Configuration 

CubeMX :

* Prescaler = 6 (for 4MhZ)
* Time quantum (tq): tq = 1 / (f_APB1/prescaler) =  250 ns
* Bit Segment 1 (BS1) = 7 tq
* Bit Segment 2 (BS2) = 2 tq
* Synchronization Jump Width (SJW) = 1 tq

baudrate = 1 / [tq * (SJW + BS1 + BS2) ]  = 400 kBits/s

### How to use the PICAN 2
On the Raspberry Pi the CAN uses the shield **PICAN 2**. This shield is designed by **CopperhillTech**. The procedure to configure the PICAN 2 on a Raspbian is explained on the website of [CopperhillTech](https://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/). Samples of C and python code are available on the web site as well as a set of programs to test the configuration (for example, *candump* monitors the traffic).

./candump can0.

Main steps are :

* Edit the file `/boot/config.txt` by:

~~~~
sudo nano /boot/config.txt
~~~~

* Add these 3 lines to the end of file:

~~~~
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=spi-bcm2835-overlay
~~~~

* Reboot the Raspberry Pi:

~~~~
sudo reboot
~~~~

* Initialize the CAN interface by entering:

~~~~
sudo /sbin/ip link set can0 up type can bitrate 500000
~~~~



## Messages

### From Nucleo
|Name                | Type   |Size (B)|Period |Destination|
|--------------------|--------|--------|-------|-----------|
|Steering wheel Angle|s8      |1       |100ms  |Raspi      |
|Pose wheels L+R     |s32     |8       |100ms  |Raspi      |
|Speed wheels L+R    |s32     |8       |100ms  |Raspi      |
|Battery level       |u8      |1       |500ms  |Raspi      |

***Data format***

* Steering Wheel Angle [s8 : 0] = direction sensor [...,...]  
* Position [s32 : 0] = left position (mm)   
* Position [s32 : 1] = right position (mm)   
* Speed Wheels [float : 0] = left Speed (m/s)   
* Speed Wheels [float : 1] = right Speed (m/s)   
* Battery [uint8 : 0] = battery level [...,...] 


### From Discovery
|Name                | Type   | Size (B) |Period |Destination|
|--------------------|--------|----------|-------|-----------|
|Ultrasonic U1   	 |u16     |6         |100ms  |Raspi      |
|Ultrasonic U2       |u16     |6         |100ms  |Raspi      |
|Orientation         |u24     |8         |50ms   |Raspi      |

***Data format***

* Ultrasonic U1 [u16 : 0] = front ultrasonic
* Ultrasonic U1 [u16 : 1] = back right ultrasonic
* Ultrasonic U1 [u16 : 2] = back left ultrasonic
* Ultrasonic U2 [u16 : 0] = back ultrasonic
* Ultrasonic U2 [u16 : 1] = front right ultrasonic
* Ultrasonic U2 [u16 : 2] = front left ultrasonic
* Orientation [u24 : 0] = Pitch in radian with Fixed-point s:1:22
* Orientation [u24 : 1] = Yaw in radian with Fixed-point s:1:22
* Orientation [u24 : 2] = Roll in radian with Fixed-point s:1:22

### From Raspeberry Pi
|Name                | Type   |Size (B)|Period |Destination|
|--------------------|--------|--------|-------|-----------|
|Motors order        |s16     |4       |50ms   |Nucleo     |
|Steering wheel order|u8      |1       |50ms   |Nucleo     |

***Data format***

* motors order[int16 : 0] = motor left    : [-100 , 100] (mm/s)   
* motors order[int16 : 1] = motor right   : [-100 , 100] (mm/s)   
* direction order[int8 : 0] = direction motor  : [-100 , 100] (%)   


## CAN Messages

|Name                |Class ID |SubClass ID|Priority |ID    |
|--------------------|---------|-----------|---------|------|
|                    |3bits    |4bits      |4bits    |11bits|
|Steering wheel Angle|0x1      |0x0        |0x1      |0x101 |
|pose wheels L+R     |0x1      |0x0        |0x0      |0x100 |
|speed wheels L+R    |0x1      |0x0        |0x2      |0x102 |
|Ultrasonic U1       |0x0      |0x0        |0x0      |0x000 |
|Ultrasonic U2       |0x0      |0x0        |0x1      |0x001 |
|Orientation         |0x0      |0x0        |0x2      |0x002 |
|Motors order        |0x0      |0x1        |0x0      |0x010 |
|Steering wheel order|0x0      |0x1        |0x1      |0x011 |
|Battery level       |0x2      |0x0        |0x0      |0x200 |

IDs are defined in the file can_definition.h
