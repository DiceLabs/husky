#!/bin/bash

BIN_DIR="/home/dicelabs/HUSKY_LIBRARY"

find "$BIN_DIR" -type f -name 'test*.py' | while read filename; do
    echo "Executing $filename"
    python3 "$filename"
done

