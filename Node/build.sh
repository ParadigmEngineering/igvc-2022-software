#!/bin/bash
if [ ! -d "build" ]; then
    echo "Making build directory..."
    mkdir build
fi
pushd build
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Debug ..
make
mv compile_commands.json ..
popd
