#!/usr/bin/env bash

sudo apt-get update
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference 
mkdir build
cd build
cmake ../ -D BUILD_INTERACTIVE="NO"
make -j$(nproc)
sudo make install
sudo ldconfig