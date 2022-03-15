#!/bin/bash -eu

SCRIPT_DIR=$(cd $(dirname $0); pwd)
cd $SCRIPT_DIR

sudo rm -rf build
mkdir build
cd build
cmake ..
make

./dwa