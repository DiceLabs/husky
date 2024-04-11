#!/bin/bash

INSTALL_DIR="$HOME/ROBOT_LIB"

mkdir -p "$INSTALL_DIR"
# Define the source directory and the destination directory
destination_dir=$INSTALL_DIR
# Use the find command to locate all Python files in the source directory and its subdirectories
find . -type f -name "*.py" -exec cp {} "$destination_dir" \;
find . -type f -name "*.sh" -exec cp {} "$destination_dir" \;
find . -type f -name "*.json" -exec cp {} "$destination_dir" \;
find . -type f -name "*.pt" -exec cp {} "$destination_dir" \;

sudo chmod +x $INSTALL_DIR/*
echo "Copied all python/bash/json files to $destination_dir and gave permission"