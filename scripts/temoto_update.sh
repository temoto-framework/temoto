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
    #cd $PACKAGE_PATH
    #git pull
  else
    echo -e $RESET$RED"* Could not find the" $BOLD$PACKAGE_NAME$RESET$RED "package"$RESET
  fi 
}

PREV_DIR=$(pwd)

update_from_source temoto
update_from_source temoto_core
update_from_source temoto_er_manager
update_from_source temoto_nlp
update_from_source temoto_action_assistant
update_from_source temoto_sensor_manager

cd $PREV_DIR

echo -e $NL"All TeMoto packages updated, compile your workspace."
