# Overview of the software architecture

### Embedded code
Each ECU has its own embedded code. The proposed codes were developed with **CubeMx** and **Keil** for the Nucleo and the Discovery boards. The code on the Raspberry is based on the distribution **Raspbian**.

### Networks
Communications between ECUs are provided through a **CAN bus**. Communication to external systems can be done via **Bluetooth** or **WiFi**. A Bluetooth library is available as well as a server based on **nodesjs**.