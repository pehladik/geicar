#!/bin/bash

prefix=$(echo "$CMAKE_PREFIX_PATH" | awk -F':' '{ print $1}')
sudo -E bash -c "source $prefix/setup.bash && roslaunch $*"