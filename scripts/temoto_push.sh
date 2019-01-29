#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

# Usage: git_push <package_name> <commit_message> 
git_push () {
  PACKAGE_NAME=$1
  PACKAGE_PATH=$(rospack find $PACKAGE_NAME)

  cd $PACKAGE_PATH

  # Proceed if the package was found
  if [[ ! -z $PACKAGE_PATH ]]; then
    echo -e -n $BOLD$GREEN"* $PACKAGE_NAME package: "$RESET;

    # Check if there are any changes in the repo
    if [ -n "$(git status --porcelain)" ]; then
      echo -e $GREEN"Found changes" $RESET;

      # List the tracked files
      echo -e $GREEN"  Modified files:" $RESET;
      MOD_FILES=$(git ls-files -m)
      for file in $MOD_FILES
      do
        echo "    $file"
      done

      # List the untracked files
      echo -e $GREEN"  Untracked files:"$RESET
      UNTRACKED_FILES=$(git ls-files . --exclude-standard --others)
      for file in $UNTRACKED_FILES
      do
        echo "    $file"
      done

      # Ask if the user is sure
      echo -e -n $NL$YELLOW$BOLD"  Add all and push? (y/n)" $RESET
      read -p " " -n 1 -r
      echo

      # If the user replied "y" or "Y"
      if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -n
      else
        echo -e $YELLOW"  Doing nothing."$RESET $NL;
        return
      fi

      # Ask about the default message
      echo -e $YELLOW"  Default commit message: $2" $RESET
      echo -e -n $YELLOW$BOLD"  Use the default message? (y/n)" $RESET
      read -p " " -n 1 -r
      echo

      # If the user replied "y" or "Y"
      CM=""
      if [[ $REPLY =~ ^[Yy]$ ]]; then
        CM=$2
      else
        read -p "  Please enter the commit message: "  msg
        CM=$msg
      fi

      # Do the git magic
      git add -A
      git commit -m "$CM"
      git push
      
    else
      echo -e $YELLOW"No changes."$RESET;
    fi
  else
    echo -e $RESET$RED"* Could not find the" $BOLD$PACKAGE_NAME$RESET$RED "package"$RESET
  fi 
}

COMMIT_MESSAGE=""

# Check if commit message was provided
if [ -z "$1" ]; then
  read -p "Please enter the commit message: "  MSG
  COMMIT_MESSAGE=$MSG
else
  COMMIT_MESSAGE=$1
fi

# Save the current directory
PREV_DIR=$(pwd)

# Open the subsystems file
TEMOTO_SUBSYS_FILE=$(rospack find temoto)/scripts/temoto_subsystems.txt
SUBSYSTEM_NAMES=$(cat  $TEMOTO_SUBSYS_FILE |tr "\n" " ")

for subsys_name in $SUBSYSTEM_NAMES 
do
  git_push $subsys_name "$COMMIT_MESSAGE"
done

cd $PREV_DIR
echo -e $NL"Done."
