#!/bin/bash

# musis udelat idk
source /opt/ros/humble/setup.bash

#  Vytvoření build složky, pokud neexistuje
mkdir -p build

#  Vstup do složky
cd build

#  Samotná kompilace
make
