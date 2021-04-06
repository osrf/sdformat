#!/bin/sh -l

set -x

BUILD_DIR=`pwd`

# Install
make install

# Compile examples
cd ../examples
mkdir build;
cd build;
cmake ..;
make;
./simple ../simple.sdf;

cd $BUILD_DIR
