#!/bin/bash
# Install all ROS packages in the provided ros-requirements file.

file=$1
file_contents=$(cat ${file})

# noglob protects against packages names with glob characters (unlikely)
# https://unix.stackexchange.com/questions/628527/split-string-on-newline-and-write-it-into-array-using-read
set -o noglob         
IFS=$'\n' 
packages=($file_contents)
set +o noglob

echo "Installing ROS packages from: ${file}"
for i in "${packages[@]}"
do
   echo "Installing package: ${i}"
   sudo apt-get install ${i}
done
