#!/usr/bin/env bash
# clone igl
git clone https://github.com/libigl/libigl.git 3rdparty/libigl
cd 3rdparty/libigl/external/nanogui
git submodule update --init --recursive .
cd ../../../..
mkdir build
cd build

if [[ $OSTYPE == darwin* ]];
then
    export PATH="/Applications/CMake.app/Contents/bin":${PATH}
fi

cmake ..
make -j8
./iglFramework ../3rdparty/libigl/tutorial/shared/bunny.off
