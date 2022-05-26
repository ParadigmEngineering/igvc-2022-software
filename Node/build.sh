#!/bin/bash
if [ ! -d "Build" ]; then
    echo "Making build directory..."
    mkdir Build
fi
pushd Build
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Debug ..
make
mv compile_commands.json ..
popd
