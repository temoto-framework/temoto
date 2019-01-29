#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

ROS_VERSION="kinetic"

# Usage: update_from_source <package_name> 
update_from_source () {
  PACKAGE_NAME=$1
  PACKAGE_PATH=$(rospack find $PACKAGE_NAME)

  # Update if found
  if [[ ! -z $PACKAGE_PATH ]]; then
    echo -e $RESET$GREEN"* Updating the" $BOLD$PACKAGE_NAME$RESET$GREEN "package" $RESET
    cd $PACKAGE_PATH
    git pull
  else
    echo -e $RESET$RED"* Could not find the" $BOLD$PACKAGE_NAME$RESET$RED "package"$RESET
  fi 
}

PREV_DIR=$(pwd)

# Open the subsystems file
TEMOTO_SUBSYS_FILE=$(rospack find temoto)/scripts/temoto_subsystems.txt
SUBSYSTEM_NAMES=$(cat  $TEMOTO_SUBSYS_FILE |tr "\n" " ")

for subsys_name in $SUBSYSTEM_NAMES 
do
  update_from_source $subsys_name
done

cd $PREV_DIR

echo -e $NL"All TeMoto packages updated, compile your workspace."
