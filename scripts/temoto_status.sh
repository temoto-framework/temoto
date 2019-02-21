#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

ROS_VERSION="kinetic"
PREV_DIR=$(pwd)

# Open the subsystems file
TEMOTO_SUBSYS_FILE=$(rospack find temoto)/scripts/temoto_subsystems.txt
SUBSYSTEM_NAMES=$(cat  $TEMOTO_SUBSYS_FILE |tr "\n" " ")

for subsys_name in $SUBSYSTEM_NAMES 
do
  echo -e $NL$GREEN"Checking the status of"$BOLD $subsys_name $RESET
  cd $(rospack find $subsys_name) && git status
done

cd $PREV_DIR
