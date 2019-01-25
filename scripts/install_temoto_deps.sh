#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

ROS_VERSION="kinetic"

# Usage: find_install_from_source <package_name> <git_repo_uri> 
find_install_from_source () {
  PACKAGE_NAME=$1
  PACKAGE_PATH=$2

  # Look for the package
  rospack find $PACKAGE_NAME &> /dev/null

  # Get the package if it was not found
  if [[ $? = 0 ]]; then
    echo -e $GREEN$BOLD"*" $PACKAGE_NAME $RESET$GREEN"package is already installed."$RESET
  else
    # Clone the rviz_plugin_manager package
    echo -e $RESET$GREEN"Cloning the" $PACKAGE_NAME "package to"$BOLD $CW_DIR $RESET
    git clone $PACKAGE_PATH
  fi 
}

# Usage: find_install_from_apt <package_name>
find_install_from_apt () {
  PACKAGE_NAME=$1
  
  # Look for the package
  SEARCH_RESULT=$(dpkg --list | grep $PACKAGE_NAME | awk -F: '{print $1}' | cut -d' ' -f3)

  if [[ $SEARCH_RESULT = $PACKAGE_NAME ]]; then
    echo -e $GREEN$BOLD"*" $PACKAGE_NAME $RESET$GREEN"package is already installed."$RESET
  else
    # Clone the rviz_plugin_manager package
    echo -e $RESET$GREEN"Installing the" $PACKAGE_NAME $RESET
    sudo apt install $PACKAGE_NAME
  fi 
}

# Go back to the catkin_workspace/src folder
P1=$(awk -F: '{print $1}' <<< "$ROS_PACKAGE_PATH")
P2=$(awk -F: '{print $2}' <<< "$ROS_PACKAGE_PATH")
CW_DIR="Uninitialized directory"

# Find the catkin_workspace directory
if [[ $P1 = *"opt"* ]]; then
  CW_DIR=$P2
else
  CW_DIR=$P1
fi

if [[ -z $CW_DIR ]]; then
  echo -e $RED$BOLD"Could not find the catkin workspace, have you sourced it? Exiting."$RESET
  exit
fi

# Save the currend directory and go to the catcin_ws/src directory
PREV_DIR=$(pwd)
cd $CW_DIR

# Check if the rviz_plugin_manager package exists
find_install_from_source rviz_plugin_manager https://github.com/UTNuclearRoboticsPublic/rviz_plugin_manager

# Check if the human_msgs package exists
find_install_from_source human_msgs https://github.com/ut-ims-robotics/human_msgs

# Check if the file_template_parser package exists
find_install_from_source file_template_parser https://github.com/ut-ims-robotics/file_template_parser

# Check if the moveit_ros_planning_interface  package exists
find_install_from_apt ros-$ROS_VERSION-moveit-ros-planning-interface

# Check if the moveit_ros_planning_interface  package exists
find_install_from_apt ros-$ROS_VERSION-tf2-geometry-msgs

# Install Intel TBB
find_install_from_apt libtbb2
find_install_from_apt libtbb-dev

# Install TinyXML
find_install_from_apt libtinyxml-dev
find_install_from_apt libtinyxml2-2v5
find_install_from_apt libtinyxml2-dev
find_install_from_apt libtinyxml2.6.2v5

cd $PREV_DIR
echo -e $NL"Dependencies are installed, you are good to go."
