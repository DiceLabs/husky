#!/bin/bash

source term.sh
REPLACE_TEMP="$SSH_HUKSY sudo rm -r $HOME/catkin_ws/temp"
create_terminal "Replace Temp Dir" "$REPLACE_TEMP"

scp -r . administrator@146.244.98.51:/home/administrator/catkin_ws/temp


