# Communication bluetooth et CAN sur RasPi 3.0

# Pré-requis

## Installer bluetooth bibliothèque :

```
sudo apt-get install libbluetooth-dev
```

## Initialiser interface de CAN
Initialiser CAN interface en suivant l'instruction suivant:

https://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/


# Usage
Pour établir la communication bluetooth:

```
cd ~/bluetooth/src/
make
./main
```