#!/bin/bash -eu
sudo rm -rf build
mkdir build
cd build
cmake ..
make
./a_star