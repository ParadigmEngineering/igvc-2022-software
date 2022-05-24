#!/bin/bash

# Setup ROS workspace

# Must be executed from this directory
REPO_DIR=$(pwd) 
REPO_ROS_DIR="${REPO_DIR}/ROS"
echo "-- Repo dir: ${REPO_ROS_DIR}"

# Setup catkin workspace and create symlink between
# packaged in this repo and the src folder in the workspace.
WS_DIR=~/catkin_ws
echo "-- Creating folder to house catkin workspace: ${WS_DIR}"

rm -r $WS_DIR 
mkdir $WS_DIR
cd $WS_DIR

echo "-- Creating symlink: ${WS_DIR}/src -> repo/ROS"
ln -s ${REPO_ROS_DIR} src 

echo "-- Creating catkin workspace and building packages..."
catkin_make
source devel/setup.bash

echo "-- Installing ROS package dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo "-- Installing ROS workspace and pmake alias..."
COMMAND="source ${WS_DIR}/devel/setup.bash"
ALIAS="alias pmake='cd ${WS_DIR} && catkin_make'"

if ! grep -xq "$COMMAND" ~/.bashrc
then 
    echo "-- Currently not sourcing catkin setup script in bashrc..."
    echo "---- Appending following to bashrc:"
    echo "---- ${COMMAND}"
    echo $COMMAND >> ~/.bashrc
fi

if ! grep -xq "$ALIAS" ~/.bashrc
then 
    echo "-- Adding pmake alias to bashrc for building paradigm ROS packages..."
    echo "${ALIAS}" >> ~/.bashrc
fi
