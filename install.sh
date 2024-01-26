#!/bin/bash

INSTALL_DIR="$HOME/HUSKY_LIBRARY"

mkdir -p "$INSTALL_DIR"
# Define the source directory and the destination directory
destination_dir=$INSTALL_DIR
# Use the find command to locate all Python files in the source directory and its subdirectories
find . -type f -name "*.py" -exec cp {} "$destination_dir" \;
find . -type f -name "*.sh" -exec cp {} "$destination_dir" \;

echo "Copied all python files to $destination_dir"