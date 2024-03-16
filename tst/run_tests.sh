#!/bin/bash

BIN_DIR="$HOME/ROBOT_LIB"

find "$BIN_DIR" -type f -name 'test*.py' | while read filename; do
    echo "Executing $filename"
    python3 "$filename"
done

