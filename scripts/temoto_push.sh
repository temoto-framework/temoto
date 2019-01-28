#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

ROS_VERSION="kinetic"

# Usage: git_push <package_name> <commit_message> 
git_push () {
  PACKAGE_NAME=$1
  PACKAGE_PATH=$(rospack find $PACKAGE_NAME)

  cd $PACKAGE_PATH

  # Push if found
  if [[ ! -z $PACKAGE_PATH ]]; then
    echo -e $BOLD"* $PACKAGE_NAME package:"$RESET;
    if [ -n "$(git status --porcelain)" ]; then
      echo -e $GREEN"  - Found changes, commiting with message: $2" $RESET;
      git add -A
      git commit -m "$2"
      git push
    else
      echo -e $YELLOW"  - No changes."$RESET;
    fi
  else
    echo -e $RESET$RED"* Could not find the" $BOLD$PACKAGE_NAME$RESET$RED "package"$RESET
  fi 
}

PREV_DIR=$(pwd)

echo $1

git_push temoto "$1"
git_push temoto_core "$1"
git_push temoto_er_manager "$1"
git_push temoto_nlp "$1"
git_push temoto_action_assistant "$1"
git_push temoto_sensor_manager "$1"

cd $PREV_DIR

echo -e $NL"Done."
