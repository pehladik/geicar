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

### Building

Clone the project and cd into it. Then run the following commands:

```bash
mkdir build
cd build
cmake ..
cmake --build . --target radar_visualizer
function_3-Radar/radar_visualizer
```
