#!/bin/bash

# Must be executed from this directory
REPO_DIR=$(pwd) 
REPO_ROS_DIR="${REPO_DIR}/ROS"
echo "Repo dir: ${REPO_ROS_DIR}"

# Setup catkin workspace and create symlink between
# packaged in this repo and the src folder in the workspace.
rm -r ~/catkin_ws 
mkdir ~/catkin_ws
cd ~/catkin_ws
ln -s ${REPO_ROS_DIR} src 
catkin_make

# This is a little dicey, might already be there
# Might want to check if it is already there 
COMMAND="source ~/catkin_ws/devel/setup.bash"

if ! grep -xq "$COMMAND" ~/.bashrc
then 
    echo "Currently not sourcing catkin setup in bashrc..."
    echo "Appending following to bashrc:"
    echo "'source ~/catkin_ws/devel/setup.bash'"
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi
